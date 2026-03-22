# Robotic Arm Simulation 

This project simulates a 4-DOF robotic arm in MATLAB using forward kinematics and inverse kinematics.
The robot is able to move from a starting position to a pick position and then to a place position with smooth motion.

## Features

* Forward kinematics implementation for a 4-DOF robotic arm
* Numerical inverse kinematics using optimization
* Smooth trajectory generation between configurations
* 3D visualization of the robot motion
* Pick and place simulation in workspace

## How It Works

The robot:

1. Starts from an initial joint configuration
2. Computes the required joint angles to reach a given Cartesian pick position using inverse kinematics
3. Moves smoothly to the pick position
4. Pauses to simulate grasping
5. Computes and moves to the place position

## Requirements

* MATLAB (R2018 or newer recommended)
* Optimization Toolbox

## How to Run

1. Clone the repository:
2. Open the project folder in MATLAB
3. Run the main script:
## Project Structure

* → Main script to run the simulation
## Results

* The robot smoothly moves to the pick position
* The end-effector reaches the target point in 3D space
* The arm transitions to the place position after grasping

## Future Improvements

* Add joint limits and constraints
* Include gripper animation and object manipulation
* Implement trajectory planning with obstacle avoidance
* Integrate with Simulink for real-time control

## License

This project is open-source and available for educational purposes.
<img width="959" height="617" alt="image" src="https://github.com/user-attachments/assets/10af9982-eadb-456c-a8a2-716f269a15fa" />


