#######################################################################
#本文件用于模拟贡嘎1号机器人的上层文件的消息接收
#编写人：蒋沛言
#时间：2025-3-26
#######################################################################
import rclpy
from rclpy.node import Node 
from my_interface.msg import MotorFeedback   #自定义消息类型
import time

class subNode(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.time1 = time.perf_counter()
        self.pub = self.create_subscription(
                    MotorFeedback, 'motorFbk_topic', self.subCallBack, 10)
    
    def subCallBack(self, msg):
        t = time.perf_counter() - self.time1
        self.time1 = time.perf_counter()
        print(f'订阅节点接收到最新电机状态：\n{msg}')
        print(f'时间间隔：{t}')


def main(args=None):
    rclpy.init(args=args)
    subnode = subNode('subnode')
    try:
        rclpy.spin(subnode)
    except KeyboardInterrupt:
        print(f"\n用户终止进程")
    finally:
        subnode.destroy_node()