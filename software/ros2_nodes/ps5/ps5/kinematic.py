#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
import time
import serial   # <--- NUEVO

def angles_to_steps(q, steps_per_rev, microsteps, gear_ratio):
    steps_per_rad = (steps_per_rev * microsteps * gear_ratio) / (2 * math.pi)
    return [int(angle * steps_per_rad) for angle in q]

class MadgwickIKNode(Node):
    def __init__(self):
        super().__init__('madgwick_inverse_kinematics')

        self.sub = self.create_subscription(
            Imu, '/imu/data', self.cb, 10)

        # ROS publisher opcional, ya no depende del Arduino
        self.pub = self.create_publisher(
            Int32MultiArray, '/spm/steps', 10)

        # --------------------------
        # Serial hacia Arduino
        # --------------------------
        self.serial = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
        time.sleep(2)
        self.get_logger().info("Serial conectado a Arduino")

        # Filtro suave
        self.alpha_joints = 0.10

        # Estado interno
        self.joint_filtered = np.zeros(3)

        # Ganancia que controla "cuánto del ángulo usar"
        self.GAIN = 0.2

        # Parámetros del motor
        self.steps_per_rev = 200
        self.microsteps = 16
        self.gear_ratio = 1.0

        # ---- Calibración ----
        self.calibrating = True
        self.calibration_samples = []
        self.calibration_duration = 2.0
        self.calibration_start_time = time.time()
        self.offset = np.zeros(3)

        self.get_logger().info("Calibrando IMU... Mantén inmóvil 2 segundos.")

    def cb(self, msg: Imu):
        qw = msg.orientation.w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z

        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(2*(qw*qy - qz*qx))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        q_arr = np.array([roll, pitch, yaw])

        # ------------------------------
        #   CALIBRACIÓN
        # ------------------------------
        if self.calibrating:
            self.calibration_samples.append(q_arr)

            if time.time() - self.calibration_start_time >= self.calibration_duration:
                self.offset = np.mean(self.calibration_samples, axis=0)
                self.calibrating = False
                self.get_logger().info(f"Calibración completada. Offset = {self.offset}")
            else:
                return

        q_arr = q_arr - self.offset

        # ------------------------------
        #   UNWRAP
        # ------------------------------
        diff = q_arr - self.joint_filtered
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        q_arr = self.joint_filtered + diff

        # Filtro suave
        q_smooth = (
            self.alpha_joints * q_arr +
            (1 - self.alpha_joints) * self.joint_filtered
        )
        self.joint_filtered = q_smooth

        # Ganancia
        q_gain = self.joint_filtered * self.GAIN

        # Pasos
        abs_steps = angles_to_steps(
            q_gain, self.steps_per_rev, self.microsteps, self.gear_ratio
        )

        # Enviar por Serial (formato: 120,-50,300\n)
        serial_msg = f"{abs_steps[0]},{abs_steps[1]},{abs_steps[2]}\n"
        self.serial.write(serial_msg.encode())

        # Publicación opcional
        ros_msg = Int32MultiArray()
        ros_msg.data = abs_steps
        self.pub.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MadgwickIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
