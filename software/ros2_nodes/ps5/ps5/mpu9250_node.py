import rclpy
from rclpy.node import Node
import serial
import time

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

import numpy as np

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')

        self.port = '/dev/ttyACM2'   
        self.baud = 115200
        self.ser = None

        self.connect_serial()

        self.timer = self.create_timer(0.1, self.read_serial)

        self.imu = Imu()
        self.mag = MagneticField()

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        #self.controller_sub = self.create_subscription(Vector3, 'spm/steps', self.controller_callback)

    #def controller_callback(self, msg: Vector3):
        #m1 = msg.x
        #m2 = msg.y
        #m3 = msg.z

        #send2arduino(m1,m2,m3)

    def connect_serial(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                self.get_logger().info(f"Conectado a {self.port}")
            except serial.SerialException as e:
                self.get_logger().warn(f"No se pudo abrir {self.port}: {e}")
                time.sleep(2)

    def read_serial(self):
        if self.ser is None:
            self.connect_serial()
            return

        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return  

            parts = line.split(';')
            if len(parts) < 3:
                self.get_logger().warn(f"Línea incompleta ignorada: {line}")
                return

            try:
                accel_data = [float(v) for v in parts[0].replace('ACCEL:', '').split(',')]
                gyro_data  = [float(v) for v in parts[1].replace('GYRO:', '').split(',')]
                mag_data   = [float(v) for v in parts[2].replace('MAG:', '').split(',')]

                accx = accel_data[0]
                accy = accel_data[1]
                accz = accel_data[2]

                gx = np.deg2rad(gyro_data[0])
                gy = np.deg2rad(gyro_data[1])
                gz = np.deg2rad(gyro_data[2])

                mx = mag_data[0]
                my = mag_data[1]
                mz = mag_data[2]

                header = Header()
                header.frame_id = 'imu'
                header.stamp = self.get_clock().now().to_msg()

                self.imu.header = header
                self.imu.angular_velocity.x = gx
                self.imu.angular_velocity.y = gy
                self.imu.angular_velocity.z = gz
                self.imu.linear_acceleration.x = accx
                self.imu.linear_acceleration.y = accy
                self.imu.linear_acceleration.z = accz

                self.mag.header = header
                self.mag.magnetic_field.x = mx
                self.mag.magnetic_field.y = my
                self.mag.magnetic_field.z = mz

                self.imu_pub.publish(self.imu)
                self.mag_pub.publish(self.mag)
                
            except ValueError:
                self.get_logger().warn(f"No se pudo convertir a float: {line}")
                return

        
            self.get_logger().info(f"ACCEL: {accel_data} | GYRO: {gyro_data} | MAG: {mag_data}")

        except serial.SerialException as e:
            self.get_logger().warn(f"Error de serial: {e}")
            self.ser.close()
            self.ser = None  

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

