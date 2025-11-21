import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        # Cambia este puerto según tu sistema:
        # En Linux: /dev/ttyACM0  |  En Windows: COM3
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                parts = line.split(';')
                accel = [float(x) for x in parts[0].replace('ACCEL:', '').split(',')]
                gyro = [float(x) for x in parts[1].replace('GYRO:', '').split(',')]
                mag = [float(x) for x in parts[2].replace('MAG:', '').split(',')]

                msg = Imu()
                msg.linear_acceleration.x = accel[0]
                msg.linear_acceleration.y = accel[1]
                msg.linear_acceleration.z = accel[2]
                msg.angular_velocity.x = gyro[0]
                msg.angular_velocity.y = gyro[1]
                msg.angular_velocity.z = gyro[2]
                self.publisher_.publish(msg)

                self.get_logger().info(f"Accel: {accel}, Gyro: {gyro}")
            except Exception as e:
                self.get_logger().warn(f"Error parsing line: {line}, {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
