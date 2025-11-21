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
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 15)

        freq = 15  # Hz
        self.timer = self.create_timer(1 / freq, self.timer_callback)

        # Conexión con el mando
        self.controller = pydualsense()
        self.controller.init()

        # Inicializar variables
        self.accx = self.accy = self.accz = 0.0
        self.gx = self.gy = self.gz = 0.0

        # Calibración
        self.calibrating = True
        self.calibration_samples = []
        self.calibration_duration = 2.0  # segundos
        self.start_time = self.get_clock().now()

        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_az = 0.0
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0

        # Buffer del giroscopio
        self.buffer_size = 15
        self.gyro_data = np.zeros((0, 3))

    def timer_callback(self):
        now = self.get_clock().now()
        
        # Lectura del Control
        self.accx = float(self.controller.state.accelerometer.X)
        self.accy = float(self.controller.state.accelerometer.Y)
        self.accz = float(self.controller.state.accelerometer.Z)

        acc = np.array([self.accx, self.accy, self.accz])
        acc = acc / np.linalg.norm(acc) * 9.81
        self.accx, self.accy, self.accz = acc

        self.gx = float(np.deg2rad(self.controller.state.gyro.Roll * 0.1))
        self.gy = float(np.deg2rad(self.controller.state.gyro.Pitch * 0.1))
        self.gz = float(np.deg2rad(self.controller.state.gyro.Yaw * 0.1))

        # Calibración (2s)
        if self.calibrating:
            elapsed = (now - self.start_time).nanoseconds / 1e9

            # Guardar muestras
            self.calibration_samples.append([
                self.accx, self.accy, self.accz,
                self.gx, self.gy, self.gz
            ])

            if elapsed < self.calibration_duration:
                self.get_logger().info("Calibrando... NO MUEVAS EL CONTROL")
                return

            # Terminar calibración
            samples = np.array(self.calibration_samples)
            acc_mean = samples[:, 0:3].mean(axis=0)
            gyro_mean = samples[:, 3:6].mean(axis=0)

            # Bias esperado: acc = [0, 9.81, 0]
            self.bias_ax = acc_mean[0] - 0.0
            self.bias_ay = acc_mean[1] - 9.81
            self.bias_az = acc_mean[2] - 0.0

            # Gyro debe ser 0
            self.bias_gx = gyro_mean[0]
            self.bias_gy = gyro_mean[1]
            self.bias_gz = gyro_mean[2]

            self.calibrating = False
            self.get_logger().info(">>> Calibración completa <<<")
            self.get_logger().info(f"Bias acc: {self.bias_ax:.4f}, {self.bias_ay:.4f}, {self.bias_az:.4f}")
            self.get_logger().info(f"Bias gyro: {self.bias_gx:.6f}, {self.bias_gy:.6f}, {self.bias_gz:.6f}")
            return

        # Calibración
        acc_x = self.accx - self.bias_ax
        acc_y = self.accy - self.bias_ay
        acc_z = self.accz - self.bias_az

        gx_c = self.gx - self.bias_gx
        gy_c = self.gy - self.bias_gy
        gz_c = self.gz - self.bias_gz

        # Suavizado del Giroscopio
        new_row = np.array([[gx_c, gy_c, gz_c]])
        self.gyro_data = np.vstack([self.gyro_data, new_row])

        if len(self.gyro_data) > self.buffer_size:
            self.gyro_data = self.gyro_data[-self.buffer_size:]

        if len(self.gyro_data) >= 5:
            gx_s = gaussian_filter1d(self.gyro_data[:, 0], sigma=2)[-1]
            gy_s = gaussian_filter1d(self.gyro_data[:, 1], sigma=2)[-1]
            gz_s = gaussian_filter1d(self.gyro_data[:, 2], sigma=2)[-1]
        else:
            gx_s, gy_s, gz_s = gx_c, gy_c, gz_c

        # Publicar
        
        self.imu.header.stamp = self.get_clock().now().to_msg()
        self.imu.header.frame_id = 'imu'
        
        self.imu.angular_velocity.x = gx_s
        self.imu.angular_velocity.y = gy_s
        self.imu.angular_velocity.z = gz_s

        self.imu.linear_acceleration.x = acc_x
        self.imu.linear_acceleration.y = acc_y
        self.imu.linear_acceleration.z = acc_z
        
        self.imu.orientation.w = 1.0
        
        self.imu_pub.publish(self.imu)

        # Debug
        self.get_logger().info(f"Accelerometer: {acc_x:.3f}, {acc_y:.3f}, {acc_z:.3f}")
        self.get_logger().info(f"gyroscope: {gx_s:.3f}, {gy_s:.3f}, {gz_s:.3f}")
        
        
def main(args = None):
    rclpy.init(args = args)
    controller_bridge = ControllerBridge()
    
    rclpy.spin(controller_bridge)
    
    controller_bridge.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
    