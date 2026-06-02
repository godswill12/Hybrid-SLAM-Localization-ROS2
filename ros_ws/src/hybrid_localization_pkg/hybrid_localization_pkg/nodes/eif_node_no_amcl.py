#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw/2)
    q.z = math.sin(yaw/2)
    return q

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class EIF(Node):
    def __init__(self):
        super().__init__('eif_no_amcl')

        P = np.eye(3)*0.5
        self.Omega = np.linalg.inv(P)
        self.xi = np.zeros((3,1))

        self.Q = np.diag([0.1,0.1,0.05])

        self.last_time=None

        self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        self.pub = self.create_publisher(Odometry,'/eif_pose',10)

    def mean(self):
        return np.linalg.solve(self.Omega,self.xi)

    def cov(self):
        return np.linalg.inv(self.Omega)

    def odom_cb(self,msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        if self.last_time is None:
            self.last_time=t
            return

        dt = t-self.last_time
        self.last_time=t
        if dt<=0: return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        x = self.mean()
        P = self.cov()

        th = x[2,0]

        x_pred = np.array([
            [x[0,0] + v*math.cos(th)*dt],
            [x[1,0] + v*math.sin(th)*dt],
            [wrap(x[2,0] + w*dt)]
        ])

        F = np.array([
            [1,0,-v*math.sin(th)*dt],
            [0,1, v*math.cos(th)*dt],
            [0,0,1]
        ])

        P = F@P@F.T + self.Q

        self.Omega = np.linalg.inv(P)
        self.xi = self.Omega @ x_pred

        self.publish(v,w)

    def publish(self,v,w):
        x = self.mean()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id='map'

        msg.pose.pose.position.x=float(x[0])
        msg.pose.pose.position.y=float(x[1])
        msg.pose.pose.orientation=yaw_to_quaternion(float(x[2]))

        msg.twist.twist.linear.x=v
        msg.twist.twist.angular.z=w

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(EIF())
    rclpy.shutdown()

if __name__=='__main__':
    main()