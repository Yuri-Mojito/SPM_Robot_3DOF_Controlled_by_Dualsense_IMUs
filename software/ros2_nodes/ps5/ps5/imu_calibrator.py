import time
import math
import numpy as np
from pydualsense import pydualsense

class IMUProcessor:
    def __init__(self, controller):
        self.controller = pydualsense()
        self.controller.init() 
        
        # --- parámetros ---
        self.gyro_bias = np.zeros(3)    # [gx_bias, gy_bias, gz_bias] en rad/s
        self.accel_bias = np.zeros(3)   # opcional, en m/s^2
        self.got_bias = False
        self.bias_samples = 500
        self.alpha_acc = 0.03           # low-pass para accel (0..1)
        self.alpha_gyro_lp = 0.01       # LP para detectar offset lento si deseas HP
        self.alpha_comp = 0.98          # complementary filter peso para gyro (0.95-0.995)
        self.angle = np.zeros(3)        # estimate roll, pitch, yaw (rad)
        self.last_t = None

        # filtros simples
        self.acc_lpf_state = None
        self.gyro_lpf_state = None

        # iniciar estimación de bias al arrancar
        self.estimate_bias_on_startup()

    # ---------------- bias estimation ----------------
    def estimate_bias_on_startup(self):
        gx_s, gy_s, gz_s = [], [], []
        ax_s, ay_s, az_s = [], [], []
        print("Estimando bias de giroscopio: mantén el control quieto...")
        for i in range(self.bias_samples):
            stgyr = self.controller.state
            stacc = self.controller.state.accelerometer

            gx_s.append(stgyr.gyro.Yaw)    # ejemplo: revisa orden y nombres
            gy_s.append(stgyr.gyro.Roll)
            gz_s.append(stgyr.gyro.Pitch)

            # si tienes acelerómetro:
            ax_s.append(stacc.X)
            ay_s.append(stacc.Y)
            az_s.append(stacc.Z)

            time.sleep(0.005)  # 50 Hz aproximadamente

        # convertir a numpy y unidades correctas
        # Asumimos gyro en grados/s => pasar a rad/s
        mean_g = np.array([np.mean(gx_s), np.mean(gy_s), np.mean(gz_s)])
        self.gyro_bias = np.deg2rad(mean_g)   # bias en rad/s
        # accel bias (opcional) en m/s^2
        self.accel_bias = np.array([np.mean(ax_s), np.mean(ay_s), np.mean(az_s)])

        print(f"Gyro bias (deg/s): {mean_g}, (rad/s): {self.gyro_bias}")
        self.got_bias = True
        # inicializar filtros
        self.acc_lpf_state = self.accel_bias.copy()
        self.gyro_lpf_state = self.gyro_bias.copy()
        self.last_t = time.time()

    # ---------------- filtros util ----------------
    def lpf(self, x, state, alpha):
        """simple exponential low-pass; state is numpy array"""
        x = np.asarray(x, dtype=float)
        if state is None:
            return x
        return alpha * x + (1.0 - alpha) * state

    # ---------------- procesamiento en cada lectura ----------------
    def process_sample(self):
        # Llamar esto periódicamente cuando haya nueva lectura
        stgyr = self.controller.state
        stacc = self.controller.state.accelerometer

        # ---- leer sensores crudos ----
        # Ajusta nombres según tu objeto controller
        # ASUMIMOS: gyro valores en grados/s; accel en m/s^2
        raw_g = np.array([stgyr.gyro.Yaw, stgyr.gyro.Roll, stgyr.gyro.Pitch], dtype=float)
        raw_a = np.array([stacc.X, stacc.Y, stacc.Z], dtype=float)

        # convertir gyro a rad/s y restar bias
        g_rad = np.deg2rad(raw_g) - self.gyro_bias  # rad/s, bias-corrected

        # opcion: filtrar gyro para quitar offset lento (HP ≈ raw - LP(raw))
        self.gyro_lpf_state = self.lpf(g_rad, self.gyro_lpf_state, self.alpha_gyro_lp)
        gyro_hp = g_rad - self.gyro_lpf_state

        # filtrar acelerómetro (low-pass) para estimar gravedad
        self.acc_lpf_state = self.lpf(raw_a, self.acc_lpf_state, self.alpha_acc)
        acc_filtered = self.acc_lpf_state - self.accel_bias  # opcional corregir bias

        # ---- integrar giros y aplicar complementary ----
        now = time.time()
        if self.last_t is None:
            dt = 0.01
        else:
            dt = max(1e-6, now - self.last_t)
        self.last_t = now

        # integrador simple: usar el gyro (bias-corrected, en rad/s)
        # Nota: el orden de ejes depende de cómo interpretes Yaw/Roll/Pitch en tu controller
        # aquí usamos angle += omega * dt (vector por componente) — válido si ejes coinciden con los angulos
        self.angle += g_rad * dt   # g_rad ya en rad/s

        # estimación de roll/pitch por acelerómetro (asumiendo quieto o movimiento suave)
        ax, ay, az = acc_filtered
        # evitar división por cero
        acc_roll = math.atan2(ay, az) if (abs(az) > 1e-6 or abs(ay) > 1e-6) else 0.0
        acc_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # complementary: fusionar (gyro para dinámica, accel para corrección a baja freq)
        self.angle[0] = self.alpha_comp * self.angle[0] + (1.0 - self.alpha_comp) * acc_roll
        self.angle[1] = self.alpha_comp * self.angle[1] + (1.0 - self.alpha_comp) * acc_pitch
        # yaw: si tienes magnetómetro úsalo para corrección; si no, queda integrado (drift)
        # aquí dejamos yaw integrado por gyro
        # self.angle[2] = self.alpha_comp * self.angle[2] + (1.0 - self.alpha_comp) * mag_yaw_estimate

        # resultado:
        roll = self.angle[0]
        pitch = self.angle[1]
        yaw = self.angle[2]

        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'acc_filtered': acc_filtered,
            'gyro_corrected': g_rad,
            'gyro_hp': gyro_hp,
            'dt': dt
        }

    # ---------------- re-estimar bias si detectas quietud ----------------
    def try_reestimate_bias_if_still(self, window=200, gyro_thresh=0.5, acc_var_thresh=0.1):
        """
        Lógica opcional: si detectas que el control estuvo quieto, re-estimas bias.
        - window: número de muestras para promedio
        - gyro_thresh: rad/s umbral para considerar 'quieto'
        - acc_var_thresh: varianza del accel en m/s^2
        Implementar según tu bucle de muestreo.
        """
        pass  # implementar si necesitas (ej: acumular buffer y comprobar)

def main(args=None):
    imu_proc = IMUProcessor(None)
    while True:
        out = imu_proc.process_sample()
        roll_deg = math.degrees(out['roll'])
        pitch_deg = math.degrees(out['pitch'])
        yaw_deg = math.degrees(out['yaw'])
        print(f"Roll: {roll_deg:6.2f}°, Pitch: {pitch_deg:6.2f}°, Yaw: {yaw_deg:6.2f}°")
        time.sleep(0.05)  # ~20 Hz

if __name__ == '__main__':
    main()
    
    
# ---------------- ejemplo de uso ----------------
# imu_proc = IMUProcessor(controller)
# while True:
#     out = imu_proc.process_sample()
#     # usa out['roll'], out['pitch'], out['yaw'] para tu IK
#     time.sleep(0.01)
