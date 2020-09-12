[//]: # (Image References)
[image_sample]: ./Media/autonomous_flight.png "Autonomous Flight"

# FCND - Backyard Flyer (Drone) Project
The purpose of this project is to set up a state machine using event-driven programming to autonomously flying a drone. This is done using flying a quadcopter in Unity simulator, and involves sending commands to and receiving incoming data from the drone.

The code is similar to how a drone would be controlled from a ground station computer or an onboard flight computer. Since communication with the drone is done using MAVLink, the code can be used to control an PX4 quadcopter autopilot with very little modification!

## Sample Visual

![alt text][image_sample]

## Video Output
The task of this project was to command the drone to autonomously fly a 10 meter box at a 3 meter altitude (both can be changed in the code) using an event-driven state machine.

Video Output: [Video of the autonomous flight](./Media/autonomous_flight.mkv).

## Simulator
The latest version of the Unity simulator used can be downloaded [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).

## Python Environment
Python environment can be set up and all the relevant packages can be installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

## Drone API

To communicate with the simulator (and a real drone), I used the [UdaciDrone API](https://udacity.github.io/udacidrone/). This API handles all the communication between Python and the drone simulator.  A key element of the API is the `Drone` superclass that contains the commands to be passed to the simulator and allows you to register callbacks/listeners on changes to the drone's attributes. The goal of this project is to design a subclass from the Drone class implementing a state machine to autonomously fly a box. A subclass is implemented in `backyard_flyer.py`

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt" for logs created when running `python drone.py`. Each row contains a comma seperated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contains all the message properties. The types of messages relevant to this project are:

* `MsgID.STATE`: time (ms), armed (bool), guided (bool)
* `MsgID.GLOBAL_POSITION`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.GLOBAL_HOME`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.LOCAL_POSITION`: time (ms), north (meter), east (meter), down (meter)
* `MsgID.LOCAL_VELOCITY`: time (ms), north (meter), east (meter), down (meter) 

## Autonomous Control State Machine

The state machine is run continuously until either the mission is ended or the Mavlink connection is lost.

The six states predefined for the state machine:
* MANUAL: the drone is being controlled by the user
* ARMING: the drone is in guided mode and being armed
* TAKEOFF: the drone is taking off from the ground
* WAYPOINT: the drone is flying to a specified target position
* LANDING: the drone is landing on the ground
* DISARMING: the drone is disarming

While the drone is in each state, I check transition criteria with a registered callback. If the transition criteria are met, I set the next state and pass along any commands to the drone. For example:

```python
def state_callback(self):
	if self.state == States.DISARMING:
    	if !self.armed:
        	self.release_control()
        	self.in_mission = False
        	self.state = States.MANUAL
```

This is a callback on the state message. It only checks anything if it's in the DISARMING state. If it detects that the drone is successfully disarmed, it sets the mode back to manual and terminates the mission.       

### Running the State Machine

The mission can be run using the following command:

```sh
python backyard_flyer.py
```

The GPS data is automatically logged to the specified log file.


### Reference Frames

Two different reference frames are used. Global positions are defined [longitude, latitude, altitude (pos up)]. Local reference frames are defined [North, East, Down (pos down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame . The global reference frame is what is provided by the Drone's GPS, but degrees are difficult to work with on a small scale. Conversion to a local frame allows for easy calculation of m level distances. Two convenience function are provided to convert between the two frames. These functions are wrappers on `utm` library functions.

```python
# Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)
def local_to_global(local_position, global_home):

# Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position
def global_to_local(global_position, global_home):
```