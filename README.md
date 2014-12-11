Youbot-RVO
==========

This repository holds the code for enabling YouBots to navigate autonomously in a space populated by other intelligent independent agents (i.e. humans/other robots).

The HRVO library is from http://gamma.cs.unc.edu/HRVO/ and we are editing it for academic purposes.

The main contributors to the code are Alejandro Bordallo, Fabio Previtali and Federico Boniardi, with special support from Nantas Nardelli and Dr. Subramanian Ramamoorthy.

The main sections of this repository include:

 - PTracking library and its bridge (Fabio) publish real-time human and robot world position/velocity data acquired via Kinect sensors.
 
 - HRVO library and its bridge (Alex) subscribed to agents position and velocity estimates, publishing control signals for selected agents (In the form of preferred velocity).
 
 - V-REP scripts and ROS subscribers/publishers (Alex) regard the main simulator used for the project, containing the representation and physics simulation of all agents in the environment. This serves for the double purpose of representing the real world in an interactive and measurable virtual environment, and the ability to simulate new scenarios and agent configurations based on recorded real world data.
 
 - PTracking Wrapper (Alex) subscribes to the output messages of the PTracking bridge and pre-computes/organises/publishes the data in a proper format for the HRVO and V-REP modules.
 
 - YouBot Wrapper (Alex) subscribes to the output message of the HRVO bridge and transforms/publishes it into the appropriate control topics for the V-REP virtual YouBot and the actual YouBot robot.

Requirements
------------
On PC:
* `ros hydro`
* `hrvo`

On Youbot
* `ros hydro`
* `youbot_driver`
* `youbot_driver_ros_interface`
