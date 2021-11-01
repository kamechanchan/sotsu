#!/usr/bin/env python3

from logging import info
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/options'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/__init__.py'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../rikuken_original/'))
from models.networks import YOLOv3
# from options.test_options import TestOptions
# from models import create_model
# from dnn_test import estimation
from cv2 import BRISK
from numpy.core.fromnumeric import trace
import torch
import rospy
from estimator.srv import input_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from torchvision import transforms 
import numpy as np
import cv2
from torchvision import ops as ops
from PIL import ImageDraw, ImageFont
from matplotlib import pyplot as plt
import yaml
import PIL
from color_cloud_bridge.msg import yolo_bridge
from std_msgs.msg import Float32MultiArray


def postprocess(outputs, conf_threshold, iou_threshold, pad_infos):
    decoded = []
    for output, *pad_info in zip(outputs, *pad_infos):
        # 矩形の形式を変換する。 (YOLO format -> Pascal VOC format)
        output[:, :4] = yolo_to_pascalvoc(output[:, :4])

        # フィルタリングする。
        output = filter_boxes(output, conf_threshold, iou_threshold)

        # letterbox 処理を行う前の座標に変換する。
        if len(output):
            output[:, :4] = decode_bboxes(output[:, :4], pad_info)

        # デコードする。
        decoded.append(output)

    return decoded

def yolo_to_pascalvoc(bboxes):
    cx, cy, w, h = torch.chunk(bboxes, 4, dim=1)
    # print("majikami")
    # print(cx.shape)
    # print(cy.shape)
    # print(w.shape)
    # print(h.shape)

    x1, y1 = cx - w / 2, cy - h / 2
    x2, y2 = cx + w / 2, cy + h / 2

    bboxes = torch.cat((x1, y1, x2, y2), dim=1)
    # print(bboxes.shape)

    return bboxes

def filter_boxes(output, conf_threshold, iou_threshold):
    # 閾値以上の箇所を探す。
    keep_rows, keep_cols = (
        (output[:, 5:] * output[:, 4:5] >= conf_threshold).nonzero().T
    )
    if not keep_rows.nelement():
        return []

    conf_filtered = torch.cat(
        (
            output[keep_rows, :5],
            output[keep_rows, 5 + keep_cols].unsqueeze(1),
            keep_cols.float().unsqueeze(1),
        ),
        1,
    )

    # Non Maximum Suppression を適用する。
    nms_filtered = []
    detected_classes = conf_filtered[:, 6].unique()
    for c in detected_classes:
        detections_class = conf_filtered[conf_filtered[:, 6] == c]
        keep_indices = ops.nms(
            detections_class[:, :4],
            detections_class[:, 4] * detections_class[:, 5],
            iou_threshold,
        )
        detections_class = detections_class[keep_indices]

        nms_filtered.append(detections_class)

    nms_filtered = torch.cat(nms_filtered)

    return nms_filtered

def decode_bboxes(bboxes, info_img):
    # print(np.array(info_img).shape)
    scale_x, scale_y, dx, dy = info_img

    bboxes -= torch.stack([dx, dy, dx, dy])
    bboxes /= torch.stack([scale_x, scale_y, scale_x, scale_y])

    return bboxes

def get_text_color(color):
    value = color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114

    return "black" if value > 128 else "white"


class Get_data:
    def __init__(self):
        self.img_path = "/home/ericlab/Desktop/ishiyama/zatsumuyou/ishi.jpg"
        rospy.init_node("client", anonymous=True)
        rospy.Subscriber("/photoneo_center/sensor/image_color", Image, self.callback)
        # self.pub = rospy.Publisher("ishiyama_pub_data", Image, queue_size=10)
        self.data = None
        rospy.wait_for_service("ishiyama_input_data")
        try:
            tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
            out_data = tem_data(self.data)
            self.input = self.data_transformation(out_data.out_img)
            # To_Yolo(self.input, self.img_path)
            # self.pub.publish(out_data.out_img)
            # print(out_data.out_img)
        except rospy.ServiceException:
            print("service call failed")


        rospy.spin()

    def callback(self, data):
        # rospy.wait_for_service("ishiyama_input_data")
        # try:
        #     tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
        #     self.data = None
        #     out_data = tem_data(self.data)
        #     self.input = self.data_transformation(out_data.out_img)
        To_Yolo(self.input, self.img_path)
            # self.pub.publish(out_data.out_img)
            # print(out_data.out_img)
        # except rospy.ServiceException:
        #     print("service call failed")
        # self.data = data
        print("data_sub")
    
    def data_transformation(self, data):
        try:
            bridge = CvBridge()
            out_data = bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite(self.img_path, out_data)
            # out_data = transforms.ToTensor()(out_data)
            return out_data
        except CvBridgeError as e:
            print(e)
            return e


class To_Yolo:
    def __init__(self, input, img_path):
        # self.in_pub = rospy.Publisher("ishiyama_input", Image, queue_size=10)
        self.out_pub = rospy.Publisher("ishiyama_YOLO", yolo_bridge, queue_size=10)
        self.yolo_out_pub = rospy.Publisher("YOLO_output", Image, queue_size=10)
        self.img_size = 416
        self.gpu_id = 0
        # self.opt = TestOptions().parse()
        self.conf_threshold = 0.5
        self.nms_threshold = 0.45
        # self.class_names = "HV8"
        self.img_path = img_path
        self.font_path = "/home/ericlab/Desktop/ishiyama/Yolo_saikou/yolov3/utils/font/ipag.ttc"
        self.save_path = "/home/ericlab/Desktop/ishiyama/zatsumuyou/output.jpg"
        self.config_path = "/home/ericlab/Desktop/ishiyama/Yolo_saikou/config/yolov3_denso.yaml"
        self.load_path = "/home/ericlab/Desktop/ishiyama/Yolo_saikou/weights/yolo_simulator.pth"
        self.con_path = "/home/ericlab/Desktop/ishiyama/Yolo_saikou/config/"
        self.arch = "YOLO"
        self.input = input
        with open(self.config_path) as f:
            self.config = yaml.safe_load(f)
        # print(self.config["model"]["class_names"])
        class_path = os.path.join(self.con_path, self.config["model"]["class_names"])
        with open(class_path) as f:
            
            self.class_names = [x.strip() for x in f.read().splitlines()]
        
        self.device_set()
        self.data_for_yolo()
        self.est_net()
        self.result()

        # rospy.spin()
    
    def data_for_yolo(self, jitter=0, random_placing=False):
        # bridge = CvBridge()
        # in_img = bridge.cv2_to_imgmsg(self.input)
        # self.in_pub.publish(in_img)
        
        org_h, org_w, _ = self.input.shape

        if jitter:
            dw = jitter * org_w
            dh = jitter * org_h
            new_aspect = (org_w + np.random.uniform(low=-dw, high=dw)) / (
                org_h + np.random.uniform(low=-dh, high=dh)
            )
        else:
            new_aspect = org_w / org_h

        if new_aspect < 1:
            new_w = int(self.img_size * new_aspect)
            new_h = self.img_size
        else:
            new_w = self.img_size
            new_h = int(self.img_size / new_aspect)

        if random_placing:
            dx = int(np.random.uniform(self.img_size - new_w))
            dy = int(np.random.uniform(self.img_size - new_h))
        else:
            dx = (self.img_size - new_w) // 2
            dy = (self.img_size - new_h) // 2

        img = cv2.resize(self.input, (new_w, new_h))
        pad_img = np.full((self.img_size, self.img_size, 3), 127, dtype=np.uint8)
        pad_img[dy : dy + new_h, dx : dx + new_w, :] = img
        pad_img = transforms.ToTensor()(pad_img)
        pad_img = pad_img.unsqueeze(0)
        self.pad_img = pad_img.to(self.device)

        scale_x = np.float32(new_w / org_w)
        scale_y = np.float32(new_h / org_h)
        pad_info = (scale_x, scale_y, dx, dy)
        pad_infos = []
        for x in pad_info:
            y = torch.from_numpy(np.array([x], dtype=np.float32))
            pad_infos.append(y)
        # pad_info = np.array(pad_info)
        # pad_info = torch.from_numpy(pad_info.astype(np.float32))
        # pad_info = [torch.from_numpy(x.astype(np.float32)) for x in np.array(pad_info)]
        self.pad_info = [x.to(self.device) for x in pad_infos]

    def device_set(self):
        if self.gpu_id >=0 and torch.cuda.is_available():
            self.device = torch.device("cuda", self.gpu_id)
        else:
            self.device = torch.device("cpu")

    def est_net(self):
        self.create_model()
        # self.model = model.net.to(self.device).eval()
        # print(self.pad_img.shape)
        with torch.no_grad():
            # print(self.pad_img)
            outputs = self.model(self.pad_img)
            self.outputs = postprocess(outputs, self.conf_threshold, self.nms_threshold, self.pad_info)
            # self.detections = self.output_to_dict(outputs, self.class_names) 

    def create_model(self):
        if self.arch =="YOLO":
            self.model = YOLOv3(self.config["model"])
            state = torch.load(self.load_path, self.device)
            self.model.load_state_dict(state["model_state_dict"])
            print(f"state_dict format weights file loaded. {self.load_path}")
            self.model = self.model.to(self.device).eval()

    def result(self):
        # self.detections = self.output_to_dict(self.outputs, self.class_names) 
        # img = PIL.Image.open(self.img_path)

        img = cv2.cvtColor(self.input, cv2.COLOR_BGR2RGB)
        img = PIL.Image.fromarray(img)
        msg_data = yolo_bridge()
        # detection = []
        for x in self.outputs:
            cluster_num = x.shape[0]
            # box_coor = np.zeros((cluster_num, 4), dtype=np.Float32)
            # box_coor = []
            # box_final_coor = []
            for i, (x1, y1, x2, y2, obj_conf, class_conf, label) in enumerate(x):
                box = {
                    "confidence": float(obj_conf * class_conf),
                    "class_id": int(label),
                    "class_name": self.class_names[int(label)],
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                }

                x1 = int(np.clip(box["x1"], 0, img.size[0] - 1))
                y1 = int(np.clip(box["y1"], 0, img.size[1] - 1))
                x2 = int(np.clip(box["x2"], 0, img.size[0] - 1))
                y2 = int(np.clip(box["y2"], 0, img.size[1] - 1))
                caption = box["class_name"]

                draw = ImageDraw.Draw(img, mode="RGBA")
                # 色を作成する。
                cmap = plt.cm.get_cmap("hsv", len(self.class_names))
                # フォントを作成する。
                fontsize = max(3, int(0.01 * min(img.size)))
                font = ImageFont.truetype(self.font_path, size=fontsize)

                color = tuple(cmap(box["class_id"], bytes=True))
                # 矩形を描画する。
                draw.rectangle((x1, y1, x2, y2), outline=color, width=3)
                # ラベルを描画する。
                text_size = draw.textsize(caption, font=font)
                text_origin = np.array([x1, y1])
                text_color = get_text_color(color)

                draw.rectangle(
                    [tuple(text_origin), tuple(text_origin + text_size - 1)], fill=color
                )
                draw.text(text_origin, caption, fill=text_color, font=font)

                img_3 = np.array(img)
                # print(type(img_3))
                img_3 = cv2.cvtColor(img_3, cv2.COLOR_RGB2BGR)
                # cv2.imshow("img_3", img_3)
                # if cv2.waitKey(1) & 0xff == ord("q"):
                #     break
                # box_coor = [x1, y1, x2, y2]
                
                # np.append(box_coor,x1)
                # np.append(box_coor,y1)
                # np.append(box_coor,x2)
                # np.append(box_coor,y2)
                
                box_coor = []
                box_coor.append(x1)
                box_coor.append(y1)
                box_coor.append(x2)
                box_coor.append(y2)
                
                box_final = Float32MultiArray(data=box_coor)
                msg_data.out_data += [box_final]
                # print(box_final_coor)
                
                # for j, name in enumerate(box_coor):
                #     box_final_coor[i][j] = name
                
                # box_final_coor[0].append(x1)
                # print("ifajjf;")
                # print(x1)
                # print(i)
                # box_final_coor[i][0] = x1
                # box_final_coor[i][1] = y1
                # box_final_coor[i][2] 

                # detection.append(bbox)
            # for box in self.detection:
        #     print(
        #         f"{box['class_name']} {box['confidence']:.0%} "
        #         f"({box['x1']:.0f}, {box['y1']:.0f}, {box['x2']:.0f}, {box['y2']:.0f})"
        #     )
        # 検出結果を画像に描画して、保存する。
        
        # self.draw_boxes(img, self.detection)
        # img.save(self.save_path)
        # print(box_final_coor)
        bridge = CvBridge()
        msg_data.input_img = bridge.cv2_to_imgmsg(self.input)
        msg_data.output_img = bridge.cv2_to_imgmsg(img_3)
        # print(len(box_coor))
        # box_coor = np.array(box_coor).reshape(cluster_num, 4)
        # box_final_coor = box_coor.tolist()
        # print(type(box_final_coor))
        # print(np.array(box_final_coor).shape)
        # print(box)
        # for i in range(cluster_num):
        #     tem_data = Float32MultiArray(data=box_final_coor(i))
        #     msg_data.out_data += [tem_data]
        # msg_data.out_data = Float32MultiArray(data=box_final_coor)
        # print(msg_data)
        print(np.array(msg_data.out_data).shape)
        print(type(msg_data.out_data))
        print(msg_data.out_data)
        self.out_pub.publish(msg_data)

        self.yolo_out_pub.publish(msg_data.output_img)

        # rospy.spin()

        # print("ishiy")

if __name__ == "__main__":
    Get_data()
    # rospy.spin()