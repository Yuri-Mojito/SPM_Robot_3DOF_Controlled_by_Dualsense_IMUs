# 🤖 Robot Esférico Manipulador --- 3 DOF

Este repositorio contiene el diseño, electrónica y software de un
**robot manipulador esférico de tres grados de libertad (3-DOF)**.\
El proyecto combina cinemática esférica, control basado en ROS2 y un
sistema de actuadores paso a paso para lograr movimientos suaves y
precisos.

## 📐 Características principales

-   **3 Grados de Libertad**
    1.  **Rotación base** (θ₁)
    2.  **Rotación del brazo medio** (θ₂)
    3.  **Rotación del brazo distal** (θ₃)
-   Arquitectura modular
-   Actuadores NEMA 17 + A4988
-   Uso de IMU para retroalimentación

## 🛠️ Estructura del proyecto

    /docs/               
    /hardware/           
    /software/           
    /tests/              
    /media/              

## 🧮 Modelo cinemático

**Cinemática directa:**

    T = RotZ(θ₁) · RotY(θ₂) · RotY(θ₃) · Translation(L)

**Cinemática inversa:** - θ₁ = atan2(y, x) - θ₂ y θ₃ derivadas por
geometría esférica

## 🔌 Electrónica

-   Arduino + ROS2
-   Drivers A4988
-   Alimentación 12V
-   Pines DIR, STEP y ENABLE

## 🖥️ Software

-   Firmware para motores y homing
-   Nodos ROS2: cinemática inversa, IMU, trayectorias

## 🎥 Media

Fotos y videos en `/media/`.

## 📄 Licencia

MIT o GPL, a elección.
