# TFM ROS 2 — Pioneer P3-DX

Repositorio asociado al Trabajo Fin de Máster (TFM) sobre el desarrollo,
integración y validación de una plataforma modular en ROS 2 para la adquisición
de información sensorial y la estimación del estado de un robot móvil
Pioneer P3-DX.

El trabajo incluye tanto la implementación sobre la plataforma física como un
entorno de simulación, permitiendo estudiar de forma independiente la
adquisición de datos, la integración de sensores y la estimación del estado.

---

## Español

### Contenido

- Integración del robot móvil Pioneer P3-DX real mediante `ros2aria`.
- Integración de una NVIDIA Jetson Orin Nano como computador embarcado.
- Adquisición de información inercial mediante una IMU MPU9250.
- Integración de un sensor LiDAR Hokuyo para percepción del entorno.
- Adquisición de información procedente de la odometría y los sensores del robot.
- Estimación del estado mediante fusión de odometría e información inercial
  utilizando `robot_localization` y un filtro de Kalman extendido (EKF).
- Registro experimental de datos mediante `rosbag2`.
- Simulación y descripción URDF del Pioneer P3-DX.
- Validación mediante pruebas estáticas y diferentes trayectorias de movimiento.

### Paquetes ROS 2

- `src/Pioneer-3DX-ROS2/ros2aria`
- `src/Pioneer-3DX-ROS2/p3dx_description`
- `src/pioneer_p3dx_description`
- `src/pioneer_imu_driver`
- `src/pioneer_real_driver`

### Datasets

El repositorio incluye registros experimentales obtenidos durante las pruebas
realizadas con la plataforma física.

- `datasets/preliminary`: pruebas iniciales.
- `datasets/static`: pruebas estáticas.
- `datasets/linear`: movimiento lineal.
- `datasets/rotation`: pruebas de giro.
- `datasets/fusion`: pruebas relacionadas con la estimación mediante EKF.

Los registros permiten analizar, entre otras señales, la odometría del robot,
la información inercial, los datos del LiDAR y la salida del estimador de estado.

### Configuración

La configuración del estimador basado en `robot_localization` se encuentra en:

- `config/ekf_pioneer.yaml`

### Plataforma hardware

- Pioneer P3-DX
- NVIDIA Jetson Orin Nano
- MPU9250 IMU
- LiDAR Hokuyo

### Objetivo del proyecto

El objetivo del proyecto es integrar y validar una plataforma robótica modular
basada en ROS 2 para la adquisición de información y la estimación del estado.
La arquitectura permite integrar diferentes fuentes sensoriales mediante
interfaces ROS 2 y proporciona una estructura reutilizable y extensible para
futuras aplicaciones de robótica móvil autónoma.

La estimación implementada combina principalmente la odometría del Pioneer
P3-DX y la información inercial mediante un filtro de Kalman extendido (EKF).
El LiDAR se integra como fuente de percepción y adquisición de información del
entorno, pero no forma parte de la fusión realizada por el EKF en la
configuración validada.

### Documentación y material complementario

Una descripción ampliada del proyecto, junto con la memoria del TFM,
la presentación y material multimedia adicional —incluyendo fotografías
y vídeos de la implementación, simulación y pruebas experimentales— está
disponible en:

https://fabacademy.org/2025/labs/unitec/students/oscar-gomez/Masters_Thesis

---

## English

### Overview

This repository accompanies the Master's Thesis focused on the development,
integration, and validation of a modular ROS 2 platform for sensor data
acquisition and state estimation using a Pioneer P3-DX mobile robot.

The project includes both the physical robot implementation and a simulation
environment, enabling sensor acquisition, system integration, and state
estimation to be evaluated in different operating conditions.

### Contents

- Real Pioneer P3-DX integration using `ros2aria`.
- NVIDIA Jetson Orin Nano integration as the onboard computer.
- Inertial data acquisition using an MPU9250 IMU.
- Hokuyo LiDAR integration for environmental perception.
- Acquisition of robot odometry and onboard sensor information.
- State estimation by combining odometry and inertial information using
  `robot_localization` and an Extended Kalman Filter (EKF).
- Experimental data recording using `rosbag2`.
- Pioneer P3-DX simulation and URDF description.
- Experimental validation through static and dynamic motion tests.

### ROS 2 Packages

- `src/Pioneer-3DX-ROS2/ros2aria`
- `src/Pioneer-3DX-ROS2/p3dx_description`
- `src/pioneer_p3dx_description`
- `src/pioneer_imu_driver`
- `src/pioneer_real_driver`

### Datasets

The repository contains experimental datasets recorded during tests performed
with the physical platform.

- `datasets/preliminary`: initial tests.
- `datasets/static`: static tests.
- `datasets/linear`: linear motion tests.
- `datasets/rotation`: rotation tests.
- `datasets/fusion`: EKF-related experiments.

The recorded datasets provide access to robot odometry, inertial measurements,
LiDAR data, and state-estimator outputs, among other signals.

### Configuration

The `robot_localization` state-estimation configuration is available at:

- `config/ekf_pioneer.yaml`

### Hardware Platform

- Pioneer P3-DX
- NVIDIA Jetson Orin Nano
- MPU9250 IMU
- Hokuyo LiDAR

### Project Objective

The project aims to integrate and validate a modular ROS 2 robotic platform
for sensor data acquisition and state estimation. The architecture provides
common ROS 2 interfaces for multiple information sources and establishes a
reusable and extensible foundation for future autonomous mobile robotics
applications.

The implemented state estimator primarily combines Pioneer P3-DX odometry and
inertial measurements using an Extended Kalman Filter (EKF). The LiDAR is
integrated as an environmental perception and data-acquisition source but is
not fused by the EKF in the validated configuration.

### Documentation and Additional Material

An extended project summary, the Master's Thesis document, presentation,
photographs, videos, and additional material from the implementation,
simulation, and experimental tests are available at:

https://fabacademy.org/2025/labs/unitec/students/oscar-gomez/Masters_Thesis
