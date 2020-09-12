import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

np.set_printoptions(suppress=True)

FLY_ALTITUDE = 3.0
SQUARE_LENGTH = 10.0


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.heading = [0, np.pi/2, np.pi, -np.pi/2]
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # Register all callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # err_thresh_north = 0.1 if self.target_position[0] == 0.0 else 0.02 * abs(self.target_position[0])
        # err_thresh_east = 0.1 if self.target_position[1] == 0.0 else 0.02 * abs(self.target_position[1])
        err_thresh_north = err_thresh_east = 0.1

        if self.flight_state == States.TAKEOFF:
            # coordinate conversion (2 coordinate systems, for one positive goes up for other positive goes down)
            altitude = -1.0 * self.local_position[2]
            # check whether altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                print("Starting global position: ", self.global_position)
                print("Starting local position: ", self.local_position)

                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:

            print("Current local position: ", self.local_position)

            if abs(self.local_position[0] - self.target_position[0]) < err_thresh_north and \
                            abs(self.local_position[1] - self.target_position[1]) < err_thresh_east:
                if len(self.all_waypoints):
                    self.target_position = np.array(self.all_waypoints.pop(0))
                    print("Next Target Position: ", self.target_position)
                    # self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
                    self.cmd_position(self.target_position[0], self.target_position[1],
                                      self.target_position[2], self.heading.pop(0))
                else:
                    self.landing_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:

            if (self.global_position[2] - self.global_home[2] < 0.1) and \
                            abs(self.local_position[2]) < 0.01:
                print("Landed at local position: ", self.local_position)
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                print("Global Position (long, lat, Alt): ", self.global_position)
                print("Global Home Position (long, lat, Alt): ", self.global_home)
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """
        Returns waypoints to fly a box
        """

        return [[SQUARE_LENGTH, 0.0, FLY_ALTITUDE],
                [SQUARE_LENGTH, SQUARE_LENGTH, FLY_ALTITUDE],
                [0.0, SQUARE_LENGTH, FLY_ALTITUDE],
                [0.0, 0.0, FLY_ALTITUDE]]

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current global location to home position
        self.set_home_position (self.global_position[0],
                                self.global_position[1],
                                self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to desired altitude
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[0] = 0.0
        self.target_position[1] = 0.0
        self.target_position[2] = FLY_ALTITUDE
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        self.all_waypoints = self.calculate_box()
        print("Waypoints: ", self.all_waypoints)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
