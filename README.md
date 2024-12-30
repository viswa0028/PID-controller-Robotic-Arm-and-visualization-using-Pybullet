# PID-controller-Robotic-Arm-and-visualization-using-Pybullet

This repository contains a Python implementation for predicting the torque of a robotic arm using PID (Proportional-Integral-Derivative) controllers. The project also includes visualization of the robotic arm's movements and torque predictions using PyBullet.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [PID Controllers](#pid-controllers)
- [PyBullet Visualization](#pybullet-visualization)
- [Contributing](#contributing)

## Overview
Robotic arms are essential in various applications, from industrial automation to medical surgery. Accurate torque prediction is crucial for efficient and precise control of robotic arms. This project implements PID controllers to predict and control the torque and uses PyBullet for real-time visualization.

## Features
- **Torque Prediction**: Predict the torque required for different movements of the robotic arm using PID controllers.
- **Real-Time Visualization**: Visualize the robotic arm's movements and torque predictions using PyBullet.
- **Configurable Parameters**: Easily adjust the PID controller parameters to fine-tune the torque predictions.

## Installation
To get started with this project, follow these steps:

1. **Clone the repository**:
    ```sh
    git clone https://github.com/yourusername/robotic-arm-torque-prediction.git
    cd robotic-arm-torque-prediction
    ```

2. **Install the required dependencies**:
    ```sh
    pip install pybullet
    pip install roboticarmtoolbox
    ```


### PID Controllers
PID controllers are widely used in control systems for their simplicity and effectiveness. The PID controller in this project is implemented with configurable parameters for proportional, integral, and derivative gains.

### PyBullet Visualization
PyBullet is an open-source physics engine for robotics, games, and machine learning. This project uses PyBullet to visualize the robotic arm's movements and torque predictions in real-time.

- **Visualizing with PyBullet**:
    The visualization will automatically start when you run the main script, showing the robotic arm's movements and torque.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes. Make sure to follow the contribution guidelines.
