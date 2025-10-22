bdx Robot Project

(Overview)

This repository tracks the development of the "bdx" robot, a custom-built robotics platform. The project involves hardware assembly based on a Raspberry Pi, integration of servos and sensors, and a robust software stack using ROS (Robot Operating System).

Development is conducted in parallel, with a physical hardware prototype and a simulated "digital twin" in MuJoCo and Gazebo for testing and algorithm development.

🚀 Current Project Status

Status: Awaiting Final Parts for Assembly

The project is currently in a transitional phase. The core software, simulation environments, and initial hardware controller have been successfully built and validated. We are now waiting for the arrival of the final 3D-printed chassis and servo motors to begin full physical assembly.

✅ Completed Milestones

Hardware

OS Installation: Ubuntu OS has been successfully installed and configured on a 64GB SD card.

Core Controller: The Raspberry Pi has been booted successfully from the SD card and is operational.

Actuator Testing: Initial code for servo motor control has been written, deployed to the Pi, and successfully tested with development servos.

Sensor Testing: A camera module has been connected to the Raspberry Pi, and functional tests have confirmed that video streaming and image capture are working correctly.

Software & Simulation

ROS Integration: The team has successfully set up and begun development within the ROS framework for robot control, navigation, and perception.

MuJoCo Simulation: A virtual model of the bdx robot has been created and tested within the MuJoCo physics simulator.

Gazebo Simulation: A second virtual model has been built in Gazebo, primarily for validating ROS integration, sensor data, and control logic in a simulated environment.

⏳ Next Steps & Blockers

The project's advancement is currently pending the completion of the physical components.

[BLOCKER] 3D Printing: Awaiting the delivery of all custom 3D-printed parts for the robot's chassis and structure.

[BLOCKER] Servo Motors: Awaiting the arrival of the final set of servo motors required for the robot's full articulation.

[NEXT] Final Assembly: Once all parts are received, the next major milestone is the full assembly of the physical robot.

[NEXT] Software Deployment: Following assembly, the team will deploy the ROS software stack from the simulation onto the physical hardware and begin real-world testing and calibration.

🛠️ Technology Stack

Hardware: Raspberry Pi, 64GB SD Card, Servo Motors, Camera Module

Operating System: Ubuntu (for Raspberry Pi)

Framework: ROS (Robot Operating System)

Simulation: MuJoCo, Gazebo
