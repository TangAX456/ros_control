import rospy
import roslibpy
import time

def message_callback(message):
    rospy.loginfo(f"Received: {message['data']}")

def main():
    rospy.init_node('simple_subscriber', anonymous=True)
    
    # 从参数获取连接信息
    bridge_host = '192.168.1.11'        #无人机ip，因为要读取无人机的/chat话题
    bridge_port = 9090
    
    rospy.loginfo(f"Connecting to rosbridge at {bridge_host}:{bridge_port}")
    
    try:
        # 连接到机器A的rosbridge
        client = roslibpy.Ros(host=bridge_host, port=bridge_port)
        client.run()
        
        rospy.loginfo("Connected to rosbridge server")
        
        # 订阅话题
        listener = roslibpy.Topic(client, '/chat', 'std_msgs/String')
        listener.subscribe(message_callback)
        
        rospy.loginfo("Subscribed to /chat, waiting for messages...")
        
        # 保持节点运行
        while not rospy.is_shutdown():
            time.sleep(0.1)
            
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'client' in locals():
            client.terminate()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass