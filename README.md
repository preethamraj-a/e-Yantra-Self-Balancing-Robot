# e-Yantra Self-Balancing Robot

Simulation of a two-wheeled self-balancing robot using LQR control, goal-directed motion, and reactive obstacle avoidance in CoppeliaSim.

## Overview

In dynamic environments, mobile robots must navigate safely while avoiding both static and dynamic obstacles. Balancing robots offer a compact solution for mobile navigation, but their inherent instability adds an additional layer of complexity to real-time obstacle avoidance. This project addresses the challenge of implementing a two-wheeled self-balancing robot in the CoppeliaSim simulation environment. The robot utilizes onboard vision and proximity sensors to detect and avoid obstacles while maintaining balance and navigating toward a predefined goal.

The simulation integrates proximity sensing, vision processing, and moving cylindrical obstacles to explore navigation in dynamic environments. The robot can navigate toward a predefined goal, but interactions with moving obstacles can push it sideways and cause it to lose balance. This highlights the challenge of maintaining stability while navigating environments with dynamic obstacles.

## Inference:

The simulation demonstrates that a balancing robot can successfully navigate a structured environment, avoid collisions, and reach its goal position. Through this project, we have demonstrated that a two-wheeled robot with an inverted-pendulum configuration can balance itself, perceive obstacles, and navigate to a goal using a combination of optimal control and simple, reactive logic.

By modeling the robot’s physical parameters:

- Mass of the bot: 0.25 kg
- Wheel radius: 0.02 m
- Body height (from the ground): 0.08 m
- Chassis dimensions: 0.086 m × 0.077 m

We derived a linearized state-space representation with four key variables:

- Linear position
- Linear velocity
- Angular position (tilt)
- Angular velocity

Designing the LQR controller around these state variables yielded a 4 × 1 gain matrix K, which maps the real-time state error into a single control signal. This signal drives the wheel motors to generate the torques necessary for upright stabilization.

On top of the balancing controller, we superimposed navigation and obstacle-avoidance velocities. A front-mounted vision sensor and two side proximity sensors produce binary flags indicating obstacle presence. On top of the balancing controller, we superimposed navigation and obstacle-avoidance velocities. A front-mounted vision sensor and a proximity sensor produce binary flags indicating obstacle presence.

When an obstacle is detected, a rule-based logic module temporarily adjusts the forward and turning velocities—slowing down, veering, and then resuming course. When an obstacle is detected, a rule-based logic module temporarily adjusts the forward and turning velocities—slowing down, veering, and then resuming course—while continuously feeding these velocity offsets into the LQR loop. The robot maintained stability even during sudden turns, and the LQR control signal automatically modulated wheel torques to execute smooth, paced maneuvers without tipping.

Overall, the combined system demonstrates how an LQR-based balancing controller can be combined with simple, reactive obstacle-avoidance rules. The robot can maintain its upright posture, detect and bypass the moving obstacles and navigate toward predefined goal coordinates in CoppeliaSim. However, interactions with moving obstacles can sometimes push the robot sideways and cause it to lose balance.

## Tools and Technologies

- CoppeliaSim
- Python and Lua
- Linear Quadratic Regulator (LQR)
- State-space modeling
- Proximity sensing and vision processing

## Resources Referred

- CoppeliaSim manual for sensor configuration
- Research papers on balancing robots
- Tutorials and online forums (Stack Overflow and YouTube tutorials)
- Python scripting references for CoppeliaSim
- Resources by e-Yantra on control systems and LQR controllers, state-space equations, Jacobian matrices, and observability and controllability matrices
