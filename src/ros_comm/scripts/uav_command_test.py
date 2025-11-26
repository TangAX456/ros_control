import rospy
from std_msgs.msg import String
import random

def command_callback(msg):
    """
    监听中控的命令，并根据命令内容发送回应
    """
    rospy.loginfo(f"Received command: {msg.data}")
    if msg.data == "起飞":
        response = "OK 起飞"
    else:
        response = f"Unknown command: {msg.data}"

    # 发布回应到 /uav_status_update 话题
    status_pub.publish(response)
    rospy.loginfo(f"Sent status update: {response}")

def publish_gps_data():
    """
    模拟发布 GPS 数据到 /gps 话题
    """
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        latitude = random.uniform(-90, 90)  # 模拟纬度
        longitude = random.uniform(-180, 180)  # 模拟经度
        altitude = random.uniform(0, 1000)  # 模拟高度
        gps_data = f"{latitude},{longitude},{altitude}"
        gps_pub.publish(gps_data)
        rate.sleep()

def main():
    rospy.init_node('uav_command_test', anonymous=True)

    # 订阅 /drone_command 话题
    rospy.Subscriber('/drone_command', String, command_callback)

    # 发布到 /uav_status_update 话题
    global status_pub
    status_pub = rospy.Publisher('/uav_status_update', String, queue_size=10)

    # 发布到 /gps 话题
    global gps_pub
    gps_pub = rospy.Publisher('/gps', String, queue_size=10)

    rospy.loginfo("uav_command_test node started, waiting for commands and publishing GPS data...")

    # 启动 GPS 数据发布
    try:
        publish_gps_data()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
