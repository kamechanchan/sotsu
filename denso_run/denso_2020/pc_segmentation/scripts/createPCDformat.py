#!/usr/bin/env python3

import json
import os
import glob

view_x = 0
view_y = 0
view_z = 0
view_qw = 1
view_qx = 0
view_qy = 0
view_qz = 0

data_format = "# .PCD v0.7 - Point Cloud Data file format"
version = 0.7
data_fields = "x y z rgb"
data_size = 4
data_type ="F"
data_count = 1
data_cls = "ascii"

color_r = 255
color_g = 0
color_b = 0

points_color = color_r * 256 * 256 + color_g * 256 + color_b

def makingHeader(p_size, file_name):
    print("# .PCD v0.7 - Point Cloud Data file format", file=file_name)
    print("VERSION", version, file=file_name)
    print("FIELDS", data_fields, file=file_name)
    print("SIZE", data_size, data_size, data_size, data_size, file=file_name)
    print("TYPE", data_type, data_type, data_type, data_type, file=file_name)
    print("COUNT", data_count, data_count, data_count, data_count, file=file_name)
    print("WIDTH", p_size, file=file_name)
    print("HEIGHT", 1, file=file_name)
    print("VIEWPOINT", view_x, view_y, view_z, view_qw, view_qx, view_qy, view_qz, file=file_name)
    print("POINTS", p_size, file=file_name)
    print("DATA", data_cls, file=file_name)


def writingPCD(list, o_file):
    if (os.path.exists(o_file)):
        os.remove(o_file)
    pcdout = open(o_file, 'wt')
    makingHeader(int(len(list)/3), pcdout)
    # for value in list:
    #     print(value, file=pcdout)
    for i in range(0, int(len(list)/3)):
        print(list[i*3], list[i*3+1], list[i*3+2], points_color, file=pcdout)
    pcdout.close()

if __name__ == '__main__':
    files = glob.glob("*.json")
    for file in files:
        json_open = open(file, 'r')
        json_load = json.load(json_open)
        list = json_load['vertices'][0]['values']
        output_file_name = file[0:-5] + ".pcd"
        writingPCD(list, output_file_name)
