#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import csv
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget

class ThrustCalibration:

    def __init__(self):

        # 加载参数
        self.time_average_interval = float(rospy.get_param('~time_interval', 1.0))  # 时间间隔
        self.vbat_min = float(rospy.get_param('~min_battery_voltage', 13.2))  # 最小电池电压
        self.mass_kg = float(rospy.get_param('~mass_kg', 1.0))  # 质量

        # 存储电池电压和推力指令的测量值
        self.volt_buf = np.array([])  # 电池电压缓存
        self.volt_records = np.array([])  # 电池电压记录
        self.cmd_buf = np.array([])  # 推力指令缓存
        self.cmd_records = np.array([])  # 推力指令记录
        self.last_records_t = rospy.get_rostime()  # 上次记录时间

        # 一些标志
        self.count_start_trigger_received = 0  # 触发信号接收计数

        # 订阅
        self.bat_sub = rospy.Subscriber("/mavros/battery", BatteryState, self.battery_voltage_cb)  # 订阅电池状态信息
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self.thrust_commands_cb)  # 订阅姿态控制指令
        rospy.Subscriber("/traj_start_trigger", PoseStamped, self.start_trigger_cb)  # 订阅起始触发信号

    # 处理电池电压信息的回调函数
    def battery_voltage_cb(self, msg):

        if (self.count_start_trigger_received == 0):  # 如果还未接收到触发信号，退出
            return

        self.volt_buf = np.append(self.volt_buf, np.array([msg.voltage]), axis=0)  # 将电池电压信息加入缓存

        # 记录滤波后的数据
        cur_t = rospy.get_rostime()
        if ((cur_t - self.last_records_t).to_sec() > self.time_average_interval):
            self.last_records_t = cur_t
            self.volt_records = np.append(self.volt_records, np.array([np.mean(self.volt_buf)]), axis=0)  # 记录平均电压
            self.volt_buf = np.array([])  # 清空电压缓存
            self.cmd_records = np.append(self.cmd_records, np.array([np.mean(self.cmd_buf)]), axis=0)  # 记录平均推力指令
            self.cmd_buf = np.array([])  # 清空推力指令缓存
            print ("volt=", self.volt_records[-1], " thr=", self.cmd_records[-1])

        # 计算校准参数并保存数据
        if ((self.count_start_trigger_received >= 2) or (self.volt_records.size != 0 and self.volt_records[-1] < self.vbat_min)):
            self.cal_and_save_data()
            self.bat_sub.unregister()  # 停止订阅电池状态信息

    # 处理推力指令信息的回调函数
    def thrust_commands_cb(self, msg):
        if (self.count_start_trigger_received == 0):  # 如果还未接收到触发信号，退出
            return
        self.cmd_buf = np.append(self.cmd_buf, np.array([msg.thrust]), axis=0)  # 将推力指令信息加入缓存
        print (msg.thrust)

    # 处理起始触发信号的回调函数
    def start_trigger_cb(self, msg):
        self.count_start_trigger_received += 1  # 触发信号计数加一
        if(self.count_start_trigger_received == 1):
            rospy.loginfo("Start recording.")  # 开始记录
        if(self.count_start_trigger_received > 1):
            rospy.loginfo("Stop recording.")  # 停止记录

    # 计算校准参数并保存数据
    def cal_and_save_data(self):
        rospy.loginfo("Data storing.")  # 记录数据

        data_stack = np.vstack((self.cmd_records, self.volt_records))  # 堆叠推力指令和电压数据
        data = [tuple(x) for x in data_stack.tolist()]  # 转换为元组列表

        rospack = rospkg.RosPack()
        filedir = rospack.get_path('px4ctrl')+'/thrust_calibrate_scrips/data.csv'  # 获取数据存储路径
        f = open(filedir, 'a')  # 打开CSV文件
        writer = csv.writer(f)  # 创建CSV写入对象
        writer.writerow(((time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())),
                         'mass(kg):', self.mass_kg, 'commands', 'voltage'))  # 写入标题
        writer.writerows(data)  # 写入数据
        f.close()  # 关闭文件

        rospy.loginfo("Stored to " + filedir)  # 记录数据存储位置

if __name__ == '__main__':

    try:
        rospy.init_node("thrust_calibration")  # 初始化ROS节点

        thrust_calibration = ThrustCalibration()  # 创建ThrustCalibration对象
        rospy.loginfo("Waiting for trigger.")  # 等待触发信号

        rospy.spin()  # 进入循环等待触发信号

    except rospy.ROSInterruptException:
        pass
