import time
import numpy as np
from pycrazyswarm import *
from class_hitch_traj import hitch_traj

class robot_control:

    def __init__(self, crazyflie, timeHelper, ellipsoid_radii, flight_id):


        
        self.cf = crazyflie 
        self.timeHelper = timeHelper
        self.ellipsoid_radii = ellipsoid_radii
        self.flight_id = flight_id  # Unique identifier for the robot
                
        #debug
        initial_position = self.cf.position()  # Ensure this method is called to get the position
        print(f"Robot initialized, ID: {self.flight_id}, Initial Position: {initial_position}")

    def enable_collision_avoidance(self, crazyflies):
        # Ensure 'crazyflies' is the list of Crazyflie objects
        others = [cf for cf in crazyflies if cf.id != self.cf.id]
        self.cf.enableCollisionAvoidance(others, self.ellipsoid_radii)
        print(f"Collision Avoidance Enabled for CF {self.flight_id} with ellipsoid radii {self.ellipsoid_radii}")

    def takeoff(self, height, duration):
        print(f"Takeoff for CF {self.flight_id}")
        self.cf.takeoff(targetHeight=height, duration=duration)

    def move_smoothly(self, start_pos, end_pos, duration):
        steps = int(duration * 100)  # 100 steps per second
        for step in range(steps):
            alpha = step / float(steps)
            current_pos = (1 - alpha) * start_pos + alpha * end_pos
            self.cf.cmdPosition(current_pos, yaw=0)
            self.timeHelper.sleep(0.01)  # Sleep for 10ms

    def land(self, height, duration):
        print(f"Landing CF {self.flight_id}")
        self.cf.land(targetHeight=height, duration=duration)
        self.timeHelper.sleep(duration)

    def emergency_land(self):
        print(f"Emergency landing CF {self.flight_id}")
        self.cf.land(targetHeight=0.01, duration=5)