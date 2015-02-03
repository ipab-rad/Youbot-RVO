# Youbot-RVO

This repository holds the code for enabling YouBots to navigate
autonomously in a space populated by other intelligent independent
agents (i.e. humans/other robots).

The [HRVO](http://gamma.cs.unc.edu/HRVO/) library is from
editing it for academic purposes.

## Documentation

The documentation lives in the repository
[wiki](https://github.com/ipab-rad/Youbot-RVO/wiki).

The main sections of this repository include:

 - PTracking library and its bridge publish real-time human
   and robot world position/velocity data acquired via Kinect sensors.
 - HRVO library and its bridge subscribed to agents position
   and velocity estimates, publishing control signals for selected
   agents (In the form of preferred velocity).

 - PTracking Wrapper subscribes to the output messages of the
   PTracking bridge and pre-computes/organises/publishes the data in a
   proper format for the HRVO and V-REP modules.

 - YouBot Wrapper subscribes to the output message of the HRVO
   bridge and transforms/publishes it into the appropriate control
   topics for the YouBot robot.

## Requirements

On PC:
* `ros hydro`
* `hrvo`

On Youbot
* `ros hydro`
* `youbot_driver`
* `youbot_driver_ros_interface`

## Contributors

Contributors are (in alphabetical order):
 * Alejandro Bordallo
 * Fabio Previtali
 * Federico Boniardi
 * Nantas Nardelli
 * Subramanian Ramamoorthy
