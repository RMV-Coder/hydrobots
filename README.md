# HydroBots
GitHub for Robotics Project of Raymond M. Valdepeñas, Marie Glo C. Generalao, Selena B. Suario and Danna Mae Ricafort


## HydroBots: Mobile Autonomous Plant Watering Robot
### Overview
HydroBots is a smart, autonomous robot designed to address the challenges of manual plant care. By integrating advanced robotics, sensor technologies, and efficient watering mechanisms, HydroBots aims to optimize water usage and simplify plant maintenance, particularly for indoor environments. The system combines autonomous navigation, collision avoidance, and precise irrigation to deliver a sustainable, user-friendly solution for modern gardening.

### Introduction

Brief context about the increasing demand for sustainable and efficient plant care solutions.
Key problems addressed by HydroBots (e.g., water scarcity, manual labor, inconsistent watering).
Overview of HydroBots' innovative features (e.g., mobile platform, advanced sensors, targeted watering).

### Installation
1. Clone the Repository
    ```bash
    git clone https://github.com/<your-repo>/HydroBots_repository.git
    ```
2. Set Up Project Directory:
- Create a virtual environment.
    ```bash
    cd robotics
    ```
   
    ```bash
    python3 -m venv --system-site-packages robotics_env
    ```
    Activate virtual environment:
    ```bash
    source robotics_env/bin/activate
    ```
- Install necessary dependencies.
   ```bash
   pip install torch torchvision (not yet complete) get from myboardhub repo
   ```
4. Install Ultralytics
   ```bash
   pip install ultralytics
   ```
5. Set up the Arducam camera
> For this robot we are using `Arducam IMX708 Camera Module` for Raspberry Pi

6. Configuring Hardware:
Setting up Raspberry Pi, sensors, and watering mechanisms.

7. Run the Program:
    ```bash
    python main.py
    ```

### System Features

1. Autonomous Navigation: Pathfinding and collision avoidance in dynamic environments.
2. Watering Mechanism: Sensor-based, precise irrigation for indoor plants.
3. User Interface: Mobile app integration for monitoring and control.

### Usage

Steps to initialize and operate HydroBots.
Demonstration of setting up watering schedules and monitoring progress.
Code Structure

> core_scripts/: Main Python scripts for robot control.

> hardware/: Files related to sensor and hardware configuration.

> docs/: Documentation and user guides.

> Experimental Results

> Summarized performance metrics and results from pilot testing.
Contribution Guidelines

Steps to contribute to the project.
How to report issues or suggest enhancements.
License

Details about the licensing agreement for HydroBots.
