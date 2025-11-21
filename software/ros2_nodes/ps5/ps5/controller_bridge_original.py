#! /usr/local/bin python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from scipy.ndimage import gaussian_filter1d
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import time
import matplotlib.pyplot as plt
from pydualsense import pydualsense
import numpy as np
import re

class ControllerBridge(Node):
    def __init__(self):
        super().__init__('controller_bridge')
        self.imu = Imu()
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        freq = 10  # Hz
        self.timer = self.create_timer(1 / freq, self.timer_callback)

        # Conexión con el mando
        self.controller = pydualsense()
        self.controller.init()

        # Inicializar variables
        self.accx = self.accy = self.accz = 0.0
        self.gx = self.gy = self.gz = 0.0

        # Buffer para guardar lecturas del giroscopio (últimas N muestras)
        self.buffer_size = 50
        self.gyro_data = np.zeros((0, 3))  # columnas: gx, gy, gz
        self.gyro_data = np.zeros((0, 3))  # columnas: gx, gy, gz
        
    def timer_callback(self):

        # ajuste de frames
        self.accx = float(self.controller.state.accelerometer.X)
        self.accy = float(self.controller.state.accelerometer.Y)
        self.accz = float(self.controller.state.accelerometer.Z)
        acc = np.array([self.accx, self.accy, self.accz])
        acc = acc / np.linalg.norm(acc) * 9.94
        self.accx, self.accy, self.accz = acc
        # self.get_logger().info(f'acc: {self.accx:.3f}, {self.accy:.3f}, {self.accz:.3f} ')
        
        # === LECTURA DEL CONTROLADOR ===
        self.gx = float(np.deg2rad(self.controller.state.gyro.Roll * 0.1))
        self.gy = float(np.deg2rad(self.controller.state.gyro.Pitch * 0.1))
        self.gz = float(np.deg2rad(self.controller.state.gyro.Yaw * 0.1))
        # self.get_logger().info(f'gyro: {self.gx:.3f}, {self.gy:.3f}, {self.gz:.3f} ')
        
        new_row = np.array([[self.gx, self.gy, self.gz]])
        self.gyro_data = np.vstack([self.gyro_data, new_row])
        
        if len(self.gyro_data) > self.buffer_size:
            self.gyro_data = self.gyro_data[-self.buffer_size:]
            
        # === SUAVIZAR DATOS ===
        if len(self.gyro_data) >= 5:  # aplicar filtro solo con suficientes datos
            gx_f = gaussian_filter1d(self.gyro_data[:, 0], sigma=2)
            gy_f = gaussian_filter1d(self.gyro_data[:, 1], sigma=2)
            gz_f = gaussian_filter1d(self.gyro_data[:, 2], sigma=2)

            gx_s, gy_s, gz_s = gx_f[-1], gy_f[-1], gz_f[-1]
        else:
            gx_s, gy_s, gz_s = self.gx, self.gy, self.gz
            
        #self.imu.header.stamp = self.get_clock().now().to_msg()
        #self.imu.header.frame_id = 'imu'
        
        self.imu.angular_velocity.x = gx_s
        self.imu.angular_velocity.y = gy_s
        self.imu.angular_velocity.z = gz_s
        
        self.imu.linear_acceleration.x = self.accx
        self.imu.linear_acceleration.y = self.accy
        self.imu.linear_acceleration.z = self.accz
        
        #self.imu.orientation.w = 1.0
        
        #self.imu_pub.publish(self.imu)
        self.get_logger().info(f"{self.accx},{self.accy},{self.accz}\n")
        #self.get_logger().info(f"{gx_s:.6f},{gy_s:.6f},{gz_s:.6f}")
        
def main(args = None):
    rclpy.init(args = args)
    
    controller_bridge = ControllerBridge()
    
    rclpy.spin(controller_bridge)
    
    controller_bridge.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
    