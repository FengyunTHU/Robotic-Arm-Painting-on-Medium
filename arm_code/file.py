# version: Python3
import time, threading
import os
import csv

## 在下面俩个文件夹中都保存了csv
path1 = "/dobot/userdata/user_project/process/trajectory/"
path2 = "/dobot/userdata/project/process/trajectory/"

head = 'j1,j2,j3,j4,j5,j6,x,y,z,Rx,Ry,Rz,user,tool,ecoKey'
x01 = [-36.016766,-7.087997,86.004951,-11.515045,-91.159744,-17.262268,-252.135269,84.525162,358.172546,-158.617340,-7.577857,69.581093,0,0,0]
x02 = [-36.016766,-7.087997,86.004951,-11.515045,-91.159744,-17.262268,-252.135269,84.525162,358.172546,-158.617340,-7.577857,69.581093,0,0,0]
x03 = [-36.016766,-7.087997,86.004951,-11.515045,-91.159744,-17.262268,-252.135269,84.525162,358.172546,-158.617340,-7.577857,69.581093,0,0,0]
x04 = [-36.016766,-7.087997,86.004951,-11.515045,-91.159744,-17.262268,-252.135269,84.525162,358.172546,-158.617340,-7.577857,69.581093,0,0,0]

file = path1 + "005.csv"

try:
    with open(file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # 写入表头（按逗号分割 head 字符串）
        writer.writerow(head.split(','))
        # 写入数据行
        writer.writerow(x01)
        writer.writerow(x02)
        writer.writerow(x03)
        writer.writerow(x04)
    print(f"成功写入 CSV 文件：{file}")
except Exception as e:
    print(f"写入 CSV 文件失败：{e}")