#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
import serial

import numpy as np
import math
import time

# =======================
#   HERRAMIENTAS
# =======================

def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rotc():
    return np.array([[0,1,0],[0,0,-1],[-1,0,0]])

def euler_xyz_to_R(roll,pitch,yaw):
    return rotx(roll) @ roty(pitch) @ rotz(yaw) @ rotc()

def normalize(v):
    v = np.array(v,dtype=float)
    n = np.linalg.norm(v)
    return v/n if n != 0 else v

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

    return roll,pitch,yaw

def angles_from_euler(roll,pitch,yaw,p_list,u_list,s_list):
    R = euler_xyz_to_R(roll,pitch,yaw)
    q_list=[]
    for i in range(3):
        p = p_list[i]
        u = normalize(u_list[i])
        s = s_list[i]
        r = R @ p
        num = np.dot(u, np.cross(s, r))
        den = np.dot(s, r)
        q_list.append(math.atan2(num, den))
    return q_list

def angles_to_steps(q, steps_per_rev, microsteps, gear_ratio):
    steps_per_rad = (steps_per_rev * microsteps * gear_ratio) / (2 * math.pi)
    return [int(angle * steps_per_rad) for angle in q]

# FILTROS
class LowPass:
    def __init__(self,alpha=0.3):
        self.alpha=alpha
        self.state=None

    def update(self,v):
        v=np.array(v,dtype=float)
        if self.state is None:
            self.state=v
            return v
        self.state = self.alpha*v + (1-self.alpha)*self.state
        return self.state

# =======================
#   NODO PRINCIPAL
# =======================
class MadgwickIK(Node):
    def __init__(self):
        super().__init__("madgwick_inverse_kinematics")

        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(Int32MultiArray, '/spm/steps', 10)

        # Serial: mantenemos 115200 y timeout corto
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        time.sleep(1.5)
        self.get_logger().info("Serial conectado a Arduino")

        self.steps_per_rev = 200
        self.microsteps = 16
        self.gear_ratio = 1.0

        # Puntos geométricos
        self.p_list = [np.array([0.15*math.cos(a), 0.15*math.sin(a),0]) for a in (0,2*math.pi/3,4*math.pi/3)]
        self.s_list = [np.array([0.25*math.cos(a), 0.25*math.sin(a),0]) for a in (0,2*math.pi/3,4*math.pi/3)]
        self.u_list=[np.array([0,0,1]) for _ in range(3)]

        # Filtros
        self.joint_filtered = np.zeros(3)
        self.alpha_joints = 0.2
        self.GAIN = 0.2

        # Calibración
        self.calibrating = True
        self.calibration_samples = []
        self.calibration_duration = 2.0
        self.calibration_start_time = time.time()
        self.offset = np.zeros(3)

        # Estado handshake / protección
        self.waiting_for_ok = False
        self.last_sent_time = 0.0
        self.ok_timeout = 2.0   # tiempo máximo a esperar por OK (s)
        self._last_log = 0.0

    # ===========================
    #         CALLBACK IMU
    # ===========================
    def imu_callback(self,msg: Imu):
        # CONTROL DE FRECUENCIA (50 Hz)
        now = time.time()
        if not hasattr(self, "last_time"):
            self.last_time = now
        if now - self.last_time < 0.02:
            # Pero aún si estamos esperando OK, revisar si llegó
            if self.waiting_for_ok:
                self._check_for_ok()
            return
        self.last_time = now

        # Si estamos esperando OK, primero intentar procesar la respuesta y no enviar otro comando
        if self.waiting_for_ok:
            self._check_for_ok()
            # Si tras revisar aún estamos esperando → NO procesamos nueva orden (evita colas)
            if self.waiting_for_ok:
                return

        # -----------------------
        #     ORIENTACIÓN
        # -----------------------
        q = msg.orientation
        roll,pitch,yaw = quat_to_euler([q.x, q.y, q.z, q.w])
        q_arr = np.array([roll, pitch, yaw])

        # CINEMÁTICA INVERSA
        q_list = angles_from_euler(q_arr[0],q_arr[1],q_arr[2], self.p_list, self.u_list, self.s_list)

        # CALIBRACIÓN
        if self.calibrating:
            # ignorar muestras corruptas
            if any(math.isnan(x) for x in q_list):
                return
            self.calibration_samples.append(q_list)
            if time.time() - self.calibration_start_time >= self.calibration_duration:
                arr = np.array(self.calibration_samples, dtype=float)
                if np.isnan(arr).any():
                    arr = arr[~np.isnan(arr).any(axis=1)]
                if arr.size == 0:
                    self.get_logger().warning("Calibración fallida (muestras corruptas). Reintentando.")
                    self.calibration_samples = []
                    self.calibration_start_time = time.time()
                    return
                self.offset = np.mean(arr, axis=0)
                self.calibrating = False
                self.get_logger().info(f"Calibración completada. Offset = {self.offset}")
            else:
                return

        # Aplicar offset + unwrap + filtro
        q_arr = np.array(q_list) - self.offset
        diff = q_arr - self.joint_filtered
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        q_arr = self.joint_filtered + diff

        diff = q_arr - self.joint_filtered
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        q_arr = self.joint_filtered + diff

        q_smooth = (self.alpha_joints * q_arr + (1 - self.alpha_joints) * self.joint_filtered)
        self.joint_filtered = q_smooth

        # Ganancia y pasos
        q_gain = self.joint_filtered * self.GAIN
        abs_steps = angles_to_steps(q_gain, self.steps_per_rev, self.microsteps, self.gear_ratio)

        # Pasos absolutos (offset 700)
        send_msg = f"{700+abs_steps[0]},{700+abs_steps[1]},{700+abs_steps[2]}\n"

        # Enviar y activar espera por OK
        try:
            self.serial.write(send_msg.encode())
            self.waiting_for_ok = True
            self.last_sent_time = time.time()
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")
            # no bloquear, intentaremos reconnect en futuros writes
            try:
                if self.serial:
                    self.serial.close()
            except:
                pass
            self.serial = None

        # ROS2 publish (tu información interna)
        ros_msg = Int32MultiArray()
        ros_msg.data = abs_steps
        self.pub.publish(ros_msg)

        # Log throttled
        now = time.time()
        if now - self._last_log > 0.5:
            self.get_logger().debug(f"Enviado -> {700+abs_steps[0]},{700+abs_steps[1]},{700+abs_steps[2]}")
            self._last_log = now

    # Intentar leer OK sin bloquear (llamado desde el callback)
    def _check_for_ok(self):
        try:
            # leer solo si hay bytes (evita bloquear)
            if self.serial is None:
                return
            if self.serial.in_waiting > 0:
                resp = self.serial.readline().decode(errors='ignore').strip()
                if resp == "OK":
                    self.waiting_for_ok = False
                    return
                else:
                    # si llega otro texto, lo logueamos leve y seguimos esperando
                    self.get_logger().debug(f"Arduino respondió (no OK): '{resp}'")
            # Timeout: si hace demasiado que esperamos, soltamos la espera para no colgar
            if time.time() - self.last_sent_time > self.ok_timeout:
                self.get_logger().warning("Timeout esperando OK del Arduino — liberando y continuando.")
                self.waiting_for_ok = False
        except Exception as e:
            self.get_logger().warning(f"Error leyendo serial en _check_for_ok: {e}")
            self.waiting_for_ok = False
            try:
                if self.serial:
                    self.serial.close()
            except:
                pass
            self.serial = None

def main():
    rclpy.init()
    node=MadgwickIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
