#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math
import csv
import os
from datetime import datetime

def quat_to_euler(q):
    x,y,z,w = q
    t0 = 2*(w*x + y*z)
    t1 = 1 - 2*(x*x + y*y)
    roll = math.atan2(t0,t1)
    t2 = 2*(w*y - z*x)
    t2 = max(-1,min(1,t2))
    pitch = math.asin(t2)
    t3 = 2*(w*z + x*y)
    t4 = 1 - 2*(y*y + z*z)
    yaw = math.atan2(t3,t4)
    return roll, pitch, yaw

class ImuDebug(Node):
    def __init__(self):
        super().__init__('imu_debug_logger')
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 100)
        # Archivo CSV
        t = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join('/tmp', f'imu_debug_{t}.csv')
        self.file = open(self.csv_path, 'w', newline='')
        self.csvw = csv.writer(self.file)
        self.csvw.writerow([
            'ros_time_secs','ros_time_nsecs','topic_recv_time','frame_id',
            'qx','qy','qz','qw','q_norm',
            'roll','pitch','yaw',
            'ax','ay','az',
            'gx','gy','gz',
            'orient_cov_0','orient_cov_1','orient_cov_2','orient_cov_3','orient_cov_4','orient_cov_5',
            'angvel_cov_0','linacc_cov_0','msg_seq'
        ])
        self.prev_stamp = None
        self.get_logger().info(f'Logging IMU to: {self.csv_path}')

    def cb(self, msg: Imu):
        now = self.get_clock().now().to_msg()
        rx_time = f"{now.sec}.{now.nanosec:09d}"
        stamp = msg.header.stamp
        q = msg.orientation
        qv = np.array([q.x,q.y,q.z,q.w],dtype=float)
        q_norm = np.linalg.norm(qv)
        roll,pitch,yaw = quat_to_euler(qv)
        ax,ay,az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx,gy,gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        oc = msg.orientation_covariance if hasattr(msg, 'orientation_covariance') else [None]*9
        gc = msg.angular_velocity_covariance if hasattr(msg, 'angular_velocity_covariance') else [None]*9
        lac = msg.linear_acceleration_covariance if hasattr(msg, 'linear_acceleration_covariance') else [None]*9

        row = [
            stamp.sec, stamp.nanosec, rx_time, msg.header.frame_id,
            q.x, q.y, q.z, q.w, q_norm,
            roll, pitch, yaw,
            ax, ay, az,
            gx, gy, gz,
            oc[0], oc[1], oc[2], oc[3], oc[4], oc[5],
            gc[0] if len(gc)>0 else None,
            lac[0] if len(lac)>0 else None,
            getattr(msg, 'header', {}).get('seq', 0)
        ]
        self.csvw.writerow(row)
        # Flush occasionally
        if self.csvw:
            self.file.flush()

        # Log pequeño resumen en consola cada 1s aproximadamente
        if self.prev_stamp is None or (stamp.sec != self.prev_stamp):
            self.get_logger().info(f"t={stamp.sec}.{stamp.nanosec:09d} q_norm={q_norm:.4f} euler=[{roll:.3f},{pitch:.3f},{yaw:.3f}] frame={msg.header.frame_id}")
        self.prev_stamp = stamp.sec

    def destroy_node(self):
        try:
            self.file.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print(f'CSV saved to: {node.csv_path}')

if __name__ == '__main__':
    main()
