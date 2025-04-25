#######################################################################
#本文件用于模拟贡嘎1号机器人的上层文件的消息发布
#编写人：蒋沛言
#时间：2025-3-26
#######################################################################
import rclpy
from rclpy.node import Node 
from my_interface.msg import MotorDrive   #自定义消息类型

class pubNode(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.pub = self.create_publisher(MotorDrive, 'MotorDrive_topic', 10)
        self.msg = MotorDrive()
        self.timer = self.create_timer(1/100, self.timer_callback)
        self.i = 0

    def timer_callback(self): 
        self.msg.joint = ['lift']*25
        self.msg.joint[0] = 'bottom'
        self.msg.joint[1] = 'arm'
        self.msg.mode = [0]*25
        # self.msg.mode[0] = 2
        # self.msg.mode[1] = 2
        self.msg.pos = [100000]*25
        self.msg.vel = [100000]*25
        if self.i < 300:
            self.msg.vel[0] = 10000
            self.msg.vel[1] = 10000
        else:
            self.msg.vel[0] = 0
            self.msg.vel[1] = 0
        self.msg.tq = [100000]*25
        if self.i < 999999:
            self.i += 1
            self.pub.publish(self.msg)
        print(f'发布电机命令{self.msg}')


def main(args=None):
    rclpy.init(args=args)
    pubnode = pubNode('pubNode')
    try:
        rclpy.spin(pubnode)
    except KeyboardInterrupt:
        print(f"\n用户终止进程")
    finally:
        pubnode.destroy_node()