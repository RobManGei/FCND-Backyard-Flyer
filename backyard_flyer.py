import visdom
import argparse
import time
import math
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


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
        #self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        
        #callback for plots. Induces some delay.
        #self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        #self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

        #list of waypoints
        self.waypointList = []
        #desired altitude of box
        self.targetAlt = -3.0
        #desired length of one box leg
        self.selectedLegLength = 25.0
        #current target WP number
        self.currentWP = 0

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)", 
            xlabel='North', 
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])
        self.t = 0
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title="Altitude (meters)", 
            xlabel='Timestep', 
            ylabel='Down'
        ))

    #crate north east plot
    def update_ne_plot(self):
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    #create altitue plot
    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        # update timestep
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')


    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        #When in target altitute start waypoint flight
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()

        #when reaching the home position again, transition to land. The distance to home is set to a small value to achieve stable landing
        elif self.flight_state == States.WAYPOINT:
                #WP 3 is home
            if self.currentWP >= 3 and np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.2:
                self.landing_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        #When safely landed, disarm
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        #when not in mison return
        if not self.in_mission:
            return
        #transition to states
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.WAYPOINT:
            if self.armed:
                self.waypoint_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box

        calculates a box based on leg length and desired altitude
        """

        boxWaypoint = np.array([0.0, 0.0, 0.0])
        boxWaypoint = self.local_position
        boxWaypoint[2] = self.targetAlt

        waypointList = np.array([boxWaypoint, boxWaypoint, boxWaypoint, boxWaypoint])
        
        boxWaypoint[1] = boxWaypoint[1] + self.selectedLegLength
        waypointList [0] = boxWaypoint
        boxWaypoint[0] = boxWaypoint[0] + self.selectedLegLength
        waypointList [1] = boxWaypoint
        boxWaypoint[1] = boxWaypoint[1] - self.selectedLegLength
        waypointList [2] = boxWaypoint

        #print(waypointList)
        return(waypointList)



    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        
        #debug. sometimes home position is messed up in the sim
        #print(self.local_position)

        #calculate the wps for the box
        self.waypointList = self.calculate_box()
        #print(self.waypointList)

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = -1 * self.targetAlt
        self.takeoff(-1 * self.targetAlt)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.waypointList[self.currentWP]
 
        #debug
        #print("Current WP")
        #print(self.currentWP)
        #print(self.target_position)
        #print(self.local_position)
        
        #always command next wp. Set heading to next waypoint for camrea view
        self.cmd_position(self.waypointList[self.currentWP][0], self.waypointList[self.currentWP][1], self.waypointList[self.currentWP][2] * -1 , (((self.currentWP+1)/2) - self.currentWP) * math.pi )

        #when within a radius depending on the leg lentgh of the target wp, set next waypoint as active. Smaller values cause the drone to overshoot the wp and fly back
        if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < self.selectedLegLength * 0.1:
            #only 4 waypoints defined
            if self.currentWP < 3:
                self.currentWP = self.currentWP + 1
                self.target_position = self.waypointList[self.currentWP]
                #always command next wp. Set heading to next waypoint for camrea view
                self.cmd_position(self.waypointList[self.currentWP][0], self.waypointList[self.currentWP][1], self.waypointList[self.currentWP][2] * -1 , (((self.currentWP+1)/2) - self.currentWP) * math.pi)
        
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
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
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog_RG.txt")
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
