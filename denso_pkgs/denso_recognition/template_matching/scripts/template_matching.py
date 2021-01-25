#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import cv2
import tools
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

g_sub = False  # 画像をサブスクライブしたか否か
g_img = None   # サブスクライブした画像
g_fId = None
bridge = CvBridge()


def callback(data):
    try:
        global g_img, g_sub, g_fId
        g_fId = data.header.frame_id
        g_img = bridge.imgmsg_to_cv2(data, "bgr8")
        g_sub = True
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":

    """初期化"""
    rospy.init_node("template_matching")
    sub = rospy.Subscriber(
        rospy.get_name() + "/inputIm",
        sensor_msgs.msg.Image,
        callback)
    pub = rospy.Publisher(
        rospy.get_name() + "/outputIm",
        sensor_msgs.msg.Image,
        queue_size=1)

    """パラメタ取得"""
    DEBUG = rospy.get_param(
        rospy.get_name() + "/debug",
                  False)  # デバッグ
    TEMPLATE_1 = rospy.get_param(
        rospy.get_name() + "/template_image_1",
        None)  # テンプレート画像1
    TEMPLATE_2 = rospy.get_param(
        rospy.get_name() + "/template_image_2",
        None)  # テンプレート画像2
    TEMPLATE_3 = rospy.get_param(
        rospy.get_name() + "/template_image_3",
        None)  # テンプレート画像3
    MATCHNG_TH = float(
        rospy.get_param(
            rospy.get_name(
            ) + "/template_mathing_th",
            0.50))  # パターンマッチングで使用する閾値
    CROP_AREA_X1 = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/crop_area/x1",
            800))  # 入力画像から切り出す領域
    CROP_AREA_Y1 = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/crop_area/y1",
            700))  # 入力画像から切り出す領域
    CROP_AREA_X2 = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/crop_area/x2",
            1300))  # 入力画像から切り出す領域
    CROP_AREA_Y2 = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/crop_area/y2",
            1000))  # 入力画像から切り出す領域
    CLOP_XY = {
        "y1": CROP_AREA_Y1,
        "x1": CROP_AREA_X1,
        "y2": CROP_AREA_Y2,
        "x2": CROP_AREA_X2,
    }
    CORRECTION_GAMMA = float(
        rospy.get_param(
            rospy.get_name(
            ) + "/correction/gamma",
            2.5))  # 画像補正，ガンマ補正のパラメタ
    CORRECTION_CONTRAST = float(
        rospy.get_param(
            rospy.get_name(
            ) + "/correction/contrast",
            15))  # 画像補正，コントラスト補正のパラメタ
    CORRECTION_NOISE_H = float(
        rospy.get_param(
            rospy.get_name(
            ) + "/correction/noise_h",
            100))  # 画像補正，ノイズ除去のパラメタ
    ERROR = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/bb_error",
            15))  # 同じBBだとみなす大きさ
    BOX_TH = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/bb_num_th",
            5))  # マッチング結果として採用するために必要な投票の票数
    RESULT_CIRCLE_RADIUS = int(
        rospy.get_param(
            rospy.get_name(
            ) + "/result_circle_radius",
            15))  # 画像出力時に描画する円の半径


    if (TEMPLATE_1 is None) and (TEMPLATE_2 is None) and (TEMPLATE_3 is None):
        rospy.logerr("Please set template image parameters.")
        sys.exit(0)

    """テンプレート画像読み込み"""
    pt1 = cv2.imread(TEMPLATE_1, cv2.IMREAD_GRAYSCALE)
    pt2 = cv2.imread(TEMPLATE_2, cv2.IMREAD_GRAYSCALE)
    pt3 = cv2.imread(TEMPLATE_3, cv2.IMREAD_GRAYSCALE)

    rate = rospy.Rate(10)
    img = None
    debugIm = None

    while not rospy.is_shutdown():
        if not g_sub:
            continue
        g_sub = False
        fId = g_fId
        debugIm = g_img.copy()
        img = cv2.cvtColor(debugIm, cv2.COLOR_BGR2GRAY)

        """結果出力用キャンバスの準備"""
        imH, imW = img.shape[:3]
        resultIm = np.zeros((imH, imW, 3), np.uint8)

        """画像の加工（補正，切り出し）"""
        img = img[CLOP_XY["y1"]: CLOP_XY["y2"],
                  CLOP_XY["x1"]: CLOP_XY["x2"]
                  ]  # h, w
        if DEBUG:
            debugIm = debugIm[
                CLOP_XY["y1"]: CLOP_XY["y2"],
                CLOP_XY["x1"]: CLOP_XY["x2"]
                ]  # h, w
        img = tools.adjust_objects_image(
            img,
            CORRECTION_GAMMA,
            CORRECTION_CONTRAST,
            CORRECTION_NOISE_H)

        """テンプレートマッチング"""
        point_list = []    # 投票箱
        point_flag = False  # 投票したか否か，フラグ
        for ptn in [pt1, pt2, pt3]:
            if ptn is None:
                continue
            h, w = ptn.shape[:3]
            res = cv2.matchTemplate(img, ptn, cv2.TM_CCOEFF_NORMED)
            threshold = MATCHNG_TH
            loc = np.where(res >= threshold)
            for pt in zip(*loc[::-1]):
                if DEBUG:
                    cv2.rectangle(
                        debugIm,
                        pt,
                        (pt[0] + w,
                         pt[1] + h),
                        (0,
                         0,
                         255),
                        2)  # マッチング結果をそのまま描画

                """投票の実施"""
                point_flag = False
                for idx, _point in enumerate(point_list):
                    if _point[1][0] - ERROR < pt[0] < _point[1][0] + ERROR and _point[1][1] - ERROR < pt[1] < _point[1][1] + ERROR:
                        point_list[idx][0] += 1
                        point_flag = True
                if point_flag == False:
                    point_list.append([1, [pt[0], pt[1]]])

        """結果出力用キャンバスへ書き込み"""
        for _point in point_list:
            if _point > BOX_TH:
                cv2.circle(
                    resultIm,
                    (
                        int(CLOP_XY["x1"] + _point[1][0] + int(w / 2)),
                        int(CLOP_XY["y1"] + _point[1][1] + int(h / 2))
                    ),
                    RESULT_CIRCLE_RADIUS,
                    (0, 0, 255),
                    thickness=-1
                )

        """デバッグ用，画像表示"""
        if DEBUG:
            cv2.imshow(
                'debug',
                cv2.resize(
                    debugIm,
                    None,
                    fx=3,
                    fy=3))  # 縮小して表示
            cv2.waitKey(1)

        """結果のパブリッシュ"""
        pubIm = bridge.cv2_to_imgmsg(resultIm, "bgr8")
        pubIm.header.frame_id = fId
        pubIm.header.stamp = rospy.Time.now()
        pub.publish(pubIm)

    """ウィンドウ削除"""
    cv2.destroyAllWindows()

