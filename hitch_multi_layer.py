import time
import numpy as np
from pycrazyswarm import *
from class_hitch_traj import hitch_traj
from class_robot_control import robot_control

# Define multiple points and into a 2d array and each layer is in a subarray
points = np.array([
    [[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0], [-0.5, -0.5, 1.0], [0.5, -0.5, 1.0]],
    [[0.71, 0.0, 1.0], [0.0, 0.71, 1.0], [-0.71, 0.0, 1.0], [0.0, -0.71, 1.0]]
])


#robot and layer param, assume each points need 2
num_robot = len(points[0]) * 2
print("Number of required robots: ", num_robot)

desiredLayer = 3

# Parameters for the trajectory
object_pt = np.array([0.0, 0.0, 1.0])  #object xy and object height
object_radius = 0.15
ropeLength = 4.4
Rotation_num = 1
ellipsoid_radii = [0.15, 0.15,0.15] 

#process params
object_height = object_pt[2]


def main():


    #initialization
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    crazyflies = allcfs.crazyflies

    #mainparams
    currentLayer = 1
    currentRopeLength = ropeLength
    liftupHeight = 1


    #check robots
    if len(crazyflies) != num_robot:
        print(f"Error: Number of Crazyflies ({len(crazyflies    )}) does not match the required number ({num_robot}).")
        return  # Exit if the numbers don't match

    # Initialize robot controllers with unique identifiers
    robotControllers = [robot_control(cf, timeHelper, [0.15, 0.15, 0.15], cf.id) for cf in crazyflies]

    
    for robot in robotControllers:
        robot.enable_collision_avoidance(crazyflies)  # Pass the correct list

    # Take off each Crazyflie
    for robot in robotControllers:
        robot.takeoff(object_height, 5)
    timeHelper.sleep(5)

    #calculate contact
    
    hitchPoints1 = points[0]
    hitchPoints2 = points[1]
    contact_points1 = calculate_contact(hitchPoints1, object_radius, object_pt)
    contact_points2 = calculate_contact(hitchPoints2, object_radius, object_pt)

    # Print the contact points
    print("Contact Points Set 1:", contact_points1)
    print("Contact Points Set 2:", contact_points2)



    try:
        while currentLayer <= desiredLayer:
            #exceptions
            if swarm.input.checkIfAnyButtonIsPressed():
                raise Exception("Emergency button pressed")  # Use an exception to break out of the loop
            








            timeHelper.sleep(4)
            currentLayer += 1

    except Exception as e:
        print(f"Interrupt detected: {str(e)}")
        for robot in robotControllers:
            robot.emergency_land()
        timeHelper.sleep(5)
        print("Emergency landing completed for all robots.")



if __name__ == "__main__":
    main()