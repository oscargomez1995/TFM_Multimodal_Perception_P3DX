# TFM ROS 2 — Pioneer P3-DX

Repositorio asociado al Trabajo Fin de Máster (TFM) sobre la integración y
validación de una plataforma modular en ROS 2 para la adquisición de información
sensorial y la estimación del estado de un robot móvil Pioneer P3-DX.

El repositorio reúne la implementación sobre la plataforma física, el entorno
de simulación, las configuraciones utilizadas, los registros experimentales,
los scripts de análisis y los resultados obtenidos durante la validación.

---

# Español

## Descripción

El proyecto integra diferentes fuentes de información del Pioneer P3-DX mediante
ROS 2 y proporciona una estructura modular para la adquisición de datos y la
estimación del estado.

La plataforma física utiliza una NVIDIA Jetson Orin Nano como computador
embarcado e integra la odometría del robot, una IMU MPU9250, un sensor LiDAR
Hokuyo y los sensores ultrasónicos del Pioneer P3-DX.

La estimación implementada combina la odometría y la información inercial
mediante `robot_localization` y un filtro de Kalman extendido (EKF). El LiDAR
se integra como fuente de percepción y adquisición de información del entorno,
pero no forma parte de la fusión realizada por el EKF en la configuración
validada.

El trabajo incluye además un entorno de simulación del Pioneer P3-DX mediante
ROS 2, URDF y Gazebo, utilizado para verificar de forma independiente la
descripción del robot y su comportamiento.

## Contenido principal

- Integración del Pioneer P3-DX real mediante `ros2aria`.
- NVIDIA Jetson Orin Nano como computador embarcado.
- Adquisición de información inercial mediante una IMU MPU9250.
- Integración de un LiDAR Hokuyo.
- Integración de los sensores ultrasónicos del Pioneer P3-DX.
- Adquisición de la odometría del robot.
- Estimación del estado mediante odometría + IMU utilizando un EKF.
- Registro de datos experimentales mediante `rosbag2`.
- Simulación y descripción URDF del Pioneer P3-DX.
- Validación mediante pruebas estáticas y diferentes trayectorias de movimiento.
- Procesamiento y análisis de los registros experimentales.

## Estructura del repositorio

```text
TFM_Multimodal_Perception_P3DX/
├── analysis_results/
│   ├── used_in_thesis/       Resultados utilizados en la memoria
│   └── additional_tests/     Resultados de pruebas adicionales
│
├── config/                   Configuración del estimador EKF
│
├── datasets/
│   ├── used_in_thesis/       Rosbags utilizados en la memoria
│   └── additional_tests/     Pruebas preliminares, adicionales y de respaldo
│
├── docs/                     Documentación del proyecto
├── scripts/                  Scripts de procesamiento y análisis experimental
├── simulation_results/       Registros y resultados de simulación
├── simulation_scripts/       Scripts utilizados para las pruebas de simulación
│
├── src/                      Paquetes ROS 2
│   ├── Pioneer-3DX-ROS2/
│   ├── pioneer_experiments/
│   ├── pioneer_imu_driver/
│   ├── pioneer_p3dx_description/
│   └── pioneer_real_driver/
│
└── tools/                    Herramientas auxiliares
```

## Paquetes ROS 2 principales

- `src/Pioneer-3DX-ROS2/ros2aria`: comunicación con el Pioneer P3-DX real.
- `src/Pioneer-3DX-ROS2/p3dx_description`: recursos asociados a la descripción del robot.
- `src/pioneer_imu_driver`: adquisición y publicación de información de la IMU.
- `src/pioneer_real_driver`: componentes desarrollados para la plataforma física.
- `src/pioneer_experiments`: herramientas relacionadas con las pruebas experimentales.
- `src/pioneer_p3dx_description`: descripción y recursos utilizados en simulación.

## Datos experimentales

Los registros obtenidos con la plataforma física se encuentran en `datasets/`
y están separados según su utilización en el TFM.

### Datos utilizados en la memoria

`datasets/used_in_thesis/` contiene los registros utilizados para los análisis
y resultados presentados en la memoria:

- `static_01`: prueba estática.
- `linear_02`: movimiento lineal.
- `rotation_left_90_01`: giro de 90° a la izquierda.
- `rotation_right_90_01`: giro de 90° a la derecha.
- `square_01`: trayectoria cuadrada.
- `manual_navigation_01`: navegación manual.

### Pruebas adicionales

`datasets/additional_tests/` contiene registros preliminares, pruebas de
desarrollo, repeticiones y registros de respaldo que no fueron utilizados
directamente en los resultados finales de la memoria.

Estos datos se conservan como material experimental complementario.

## Resultados del análisis

`analysis_results/` contiene los datos procesados, archivos CSV y gráficas
generadas a partir de los registros experimentales.

La estructura mantiene la misma separación que los datasets:

- `analysis_results/used_in_thesis/`: resultados utilizados en la memoria.
- `analysis_results/additional_tests/`: análisis de pruebas complementarias.

Los scripts empleados para procesar los registros se encuentran en `scripts/`.

## Simulación

La implementación de simulación se encuentra principalmente en
`src/pioneer_p3dx_description/`.

Los scripts utilizados para ejecutar y analizar las pruebas finales de
simulación se encuentran en:

- `simulation_scripts/`

Los registros y gráficas obtenidos durante estas pruebas se encuentran en:

- `simulation_results/`

## Configuración del estimador

La configuración utilizada por `robot_localization` para el estimador basado
en EKF se encuentra en:

- `config/ekf_pioneer.yaml`

## Plataforma hardware

- Pioneer P3-DX
- NVIDIA Jetson Orin Nano
- MPU9250 IMU
- LiDAR Hokuyo
- Sensores ultrasónicos del Pioneer P3-DX

## Material complementario

Una descripción ampliada del proyecto, junto con la memoria del TFM,
la presentación, fotografías, vídeos de las pruebas y de la simulación y
otros recursos complementarios está disponible en:

https://fabacademy.org/2025/labs/unitec/students/oscar-gomez/Masters_Thesis

---

# English

## Description

This repository accompanies a Master's Thesis focused on the integration and
validation of a modular ROS 2 platform for sensor data acquisition and state
estimation using a Pioneer P3-DX mobile robot.

The physical platform uses an NVIDIA Jetson Orin Nano as its onboard computer
and integrates robot odometry, an MPU9250 IMU, a Hokuyo LiDAR, and the
Pioneer P3-DX ultrasonic sensors.

The implemented state estimator combines wheel odometry and inertial
measurements using `robot_localization` and an Extended Kalman Filter (EKF).
The LiDAR is integrated as an environmental perception and data-acquisition
source but is not fused by the EKF in the validated configuration.

The project also includes a Pioneer P3-DX simulation environment based on
ROS 2, URDF, and Gazebo for independent verification of the robot description
and simulated behavior.

## Main Contents

- Real Pioneer P3-DX integration using `ros2aria`.
- NVIDIA Jetson Orin Nano as the onboard computer.
- Inertial data acquisition using an MPU9250 IMU.
- Hokuyo LiDAR integration.
- Pioneer P3-DX ultrasonic sensor integration.
- Robot odometry acquisition.
- Odometry + IMU state estimation using an EKF.
- Experimental data recording using `rosbag2`.
- Pioneer P3-DX simulation and URDF description.
- Validation through static and dynamic motion tests.
- Experimental data processing and analysis.

## Repository Structure

```text
TFM_Multimodal_Perception_P3DX/
├── analysis_results/
│   ├── used_in_thesis/       Results used in the thesis
│   └── additional_tests/     Additional experimental results
│
├── config/                   EKF configuration
│
├── datasets/
│   ├── used_in_thesis/       Rosbags used in the thesis
│   └── additional_tests/     Preliminary, additional, and backup tests
│
├── docs/                     Project documentation
├── scripts/                  Experimental processing and analysis scripts
├── simulation_results/       Simulation recordings and results
├── simulation_scripts/       Simulation and analysis scripts
│
├── src/                      ROS 2 packages
│   ├── Pioneer-3DX-ROS2/
│   ├── pioneer_experiments/
│   ├── pioneer_imu_driver/
│   ├── pioneer_p3dx_description/
│   └── pioneer_real_driver/
│
└── tools/                    Auxiliary tools
```

## Main ROS 2 Packages

- `src/Pioneer-3DX-ROS2/ros2aria`: communication with the physical Pioneer P3-DX.
- `src/Pioneer-3DX-ROS2/p3dx_description`: robot description resources.
- `src/pioneer_imu_driver`: IMU acquisition and ROS 2 publishing.
- `src/pioneer_real_driver`: components developed for the physical platform.
- `src/pioneer_experiments`: tools related to experimental testing.
- `src/pioneer_p3dx_description`: robot description and simulation resources.

## Experimental Datasets

The recordings obtained from the physical platform are stored in `datasets/`
and organized according to their use in the Master's Thesis.

### Datasets used in the thesis

`datasets/used_in_thesis/` contains the recordings used for the analyses and
results presented in the thesis:

- `static_01`: static test.
- `linear_02`: linear motion test.
- `rotation_left_90_01`: 90° left rotation.
- `rotation_right_90_01`: 90° right rotation.
- `square_01`: square trajectory.
- `manual_navigation_01`: manual navigation test.

### Additional tests

`datasets/additional_tests/` contains preliminary recordings, development
tests, repeated experiments, and backup datasets that were not directly used
in the final thesis results.

These datasets are preserved as complementary experimental material.

## Analysis Results

`analysis_results/` contains processed data, CSV files, and plots generated
from the experimental recordings.

- `analysis_results/used_in_thesis/`: results used in the thesis.
- `analysis_results/additional_tests/`: results from complementary tests.

The scripts used to process the experimental recordings are available in
`scripts/`.

## Simulation

The simulation implementation is mainly located in
`src/pioneer_p3dx_description/`.

Scripts used for the final simulation tests and analysis are available in:

- `simulation_scripts/`

Simulation recordings and generated plots are available in:

- `simulation_results/`

## State Estimator Configuration

The `robot_localization` EKF configuration is available at:

- `config/ekf_pioneer.yaml`

## Hardware Platform

- Pioneer P3-DX
- NVIDIA Jetson Orin Nano
- MPU9250 IMU
- Hokuyo LiDAR
- Pioneer P3-DX ultrasonic sensors

## Documentation and Additional Material

An extended project summary, the Master's Thesis document, presentation,
photographs, experimental and simulation videos, and other complementary
material are available at:

https://fabacademy.org/2025/labs/unitec/students/oscar-gomez/Masters_Thesis
