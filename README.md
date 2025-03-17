CppAutoSim is a C++ simulation project that demonstrates various autonomous driving scenarios. Built using Visual Studio 2022 and the EasyX graphics library, this project features multiple driving behaviors—such as obstacle stops, parking, following vehicles, crosswalk management, and obstacle avoidance (including overtaking and oncoming traffic avoidance)—all simulated on a virtual road environment.

Features
Multiple Driving Scenarios:

Obstacle Stop: Simulates a vehicle stopping for obstacles (e.g., traffic cones).
Parking Stop: Demonstrates a parking maneuver at a designated station.
Vehicle Following: Implements safe following distances behind a leading vehicle.
Crosswalk Management: Simulates pedestrian detection and speed control at crosswalks.
Obstacle Avoidance:
Static Obstacle Avoidance: Maneuvering around stationary obstacles.
Overtaking: Simulates overtaking a slower vehicle.
Oncoming Traffic Avoidance: Demonstrates lane changes to avoid oncoming traffic.
Graphical Simulation:
Utilizes the EasyX graphics library to render roads, vehicles, obstacles, and pedestrians, providing a visual demonstration of the simulation.

Realistic Motion Dynamics:
Implements functions for straight-line movement, acceleration/deceleration, turning maneuvers, and lane changes, closely mimicking real driving behaviors.

Requirements
Development Environment:
Visual Studio 2022 or later.
Dependencies:
EasyX Graphics Library for rendering.
Operating System:
Windows.

Clone the Repository:
bash
Copy
git clone https://github.com/taizhenC/CppAutoSim.git
Open the Project:
Open the solution file in Visual Studio 2022.
Install EasyX:
Download and install the EasyX library from the EasyX official website.
Configure your project’s include and library directories to reference EasyX.
Build the Project:
Build the solution within Visual Studio.

Usage
Run the Simulation:
Execute the built application. The simulation will launch with a pre-configured scenario.
Select/Modify Scenarios:
The active scenario is set in main.cpp by passing a value from the PlanType enum (e.g., ObsPassMeetingType). Modify this parameter to test different scenarios such as:
StraightStopObsType
StraightStationType
StraightFollowType
StraightCrosswalkType
ObsPassStaticType
ObsPassOvertakeType
ObsPassMeetingType
Project Structure
main.cpp & main.h:
Entry point and definition of the simulation scenarios.
planning_base.h & planning_base.cpp:
Contains core functions for vehicle motion and planning.
traffic.h & traffic.cpp:
Defines obstacles (e.g., cones) and pedestrian behavior.
road.h & road.cpp:
Implements various road types and their display functions.
scene_base.h & scene_base.cpp:
Provides a base class for simulation scenes, including motion primitives such as lane changing and turning.
scene_straight.h & scene_straight.cpp:
Implements straight driving scenarios (obstacle stop, parking, following, and crosswalk).
scene_obs_pass.h & scene_obs_pass.cpp:
Implements obstacle avoidance scenarios (static obstacle, overtaking, and meeting oncoming traffic).
