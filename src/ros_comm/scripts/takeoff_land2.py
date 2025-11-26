#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的位置控制
# 效果说明: 无人机起飞后悬停30秒,然后降落
# 备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
from math import fabs
from turtle import position
import ros
import rospy
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState
from std_msgs.msg import String

# 创建无人机相关数据变量
uav_control_state_sv = UAVControlState()
uav_command_pv = UAVCommand()
uav_state_sv = UAVState()

# 创建状态更新发布者
status_pub = None

# 无人机状态回调函数
def uavStateCb(msg):
    global uav_state_sv
    uav_state_sv = msg

# 无人机控制状态回调函数
def uavControlStateCb(msg):
    global uav_control_state_sv
    uav_control_state_sv = msg

# 无人机命令回调函数
def command_callback(msg):
    global cmd_pub_flag
    if msg.data.lower() == "takeoff" and not cmd_pub_flag:
        rospy.loginfo("Received takeoff command")
        cmd_pub_flag = True

def main():
    # ROS初始化,设定节点名
    rospy.init_node('takeoff_land_py', anonymous=True)
    global cmd_pub_flag, status_pub

    # 创建命令发布标志位,命令发布则为false
    cmd_pub_flag = False
    # 是否已发布起飞命令
    send_takeoff_cmd = False
    # 创建无人机控制命令发布者
    UavCommandPb = rospy.Publisher("/uav1/prometheus/command", UAVCommand, queue_size=10)
    # 创建状态更新发布者
    status_pub = rospy.Publisher('/uav_status_update', String, queue_size=10)
    # 创建无人机控制状态命令订阅者
    rospy.Subscriber("/uav1/prometheus/control_state", UAVControlState, uavControlStateCb)
    # 创建无人机状态命令订阅者
    rospy.Subscriber("/uav1/prometheus/state", UAVState, uavStateCb)
    # 订阅 /drone_command 话题
    rospy.Subscriber('/drone_command', String, command_callback)

    # 记录起飞初始高度
    init_height = 0.0

    # 循环频率设置为10HZ
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 检测无人机是否处于[COMMAND_CONTROL]模式
        if uav_control_state_sv.control_state == UAVControlState.COMMAND_CONTROL:
            if cmd_pub_flag:
                # 时间戳
                uav_command_pv.header.stamp = rospy.Time.now()
                # 坐标系
                uav_command_pv.header.frame_id = 'ENU'
                # Init_Pos_Hover初始位置悬停
                uav_command_pv.Agent_CMD = 1
                # 发布的命令ID,每发一次,该ID加1
                uav_command_pv.Command_ID = 1
                # 发布起飞命令
                UavCommandPb.publish(uav_command_pv)
                rospy.loginfo("Takeoff command published")
                # 记录起飞高度
                init_height = fabs(uav_state_sv.position[2])
                send_takeoff_cmd = True
            # 检查起飞是否完成
            if send_takeoff_cmd == True:
                now_height = fabs(uav_state_sv.position[2])
                rospy.loginfo("now_height - init_height", now_height - init_height)
                if now_height - init_height >= 2.0
                # 发布起飞成功状态
                    rospy.loginfo("UAV takeoff successfully")
                    status_pub.publish("UAV takeoff successfully")
                    rospy.sleep(30)
                    # 时间戳
                    uav_command_pv.header.stamp = rospy.Time.now()
                    # 坐标系
                    uav_command_pv.header.frame_id = "ENU"
                    # Land降落
                    uav_command_pv.Agent_CMD = 3
                    # 发布的命令ID加1
                    uav_command_pv.Command_ID += 1
                    # 发布降落命令
                    UavCommandPb.publish(uav_command_pv)
                    rospy.loginfo("UAV Land")
                    rospy.loginfo("[takeoff & land tutorial_demo completed]")
                    rospy.signal_shutdown("shutdown time")
                else:
                    rospy.loginfo("UAV height : %f [m]", uav_state_sv.position[2])
                    rospy.sleep(1)
        else:
            if cmd_pub_flag:
                rospy.logwarn("Waiting for UAV to enter [COMMAND_CONTROL] MODE")
            else:
                rospy.loginfo("No takeoff command received, waiting...")
            rospy.sleep(2)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass