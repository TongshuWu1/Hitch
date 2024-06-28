import time
import numpy as np
from pycrazyswarm import *
from class_circle_traj_2points import Collinear_to_traj

# Define two points in 3D space
pt1 = np.array([0.6, 0.0, 1.])
pt2 = np.array([-0.6, 0.0, 1.])

pt3 = np.array([0.0, 0.6, 1.])
pt4 = np.array([0.0, -0.6, 1.])

desireLayer = 2

# Parameters for the trajectory
object_pt = np.array([0.0, 0.0, 1.])
object_r = 0.28

ropeLength = 4.1
Rotation_num = 1
ellipsoid_radii = [0.13, 0.13, 0.13]
duration_per_layer = 45  # Duration for each layer in seconds
initial_position_duration = 10  # Duration to move to initial positions

def move_smoothly_simultaneously(allcfs, start_positions, end_positions, duration, timeHelper):
    steps = int(duration * 100)  # 100 steps per second
    for step in range(steps):
        alpha = step / float(steps)
        for cf, start_pos, end_pos in zip(allcfs.crazyflies, start_positions, end_positions):
            current_pos = (1 - alpha) * start_pos + alpha * end_pos
            cf.cmdPosition(current_pos, yaw=0)
        timeHelper.sleep(0.01)  # Sleep for 10ms

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    currentLayer = 1
    currentRopeLength = ropeLength
    liftupHeight = 1

    for cf in allcfs.crazyflies:
        # List of other Crazyflies for the current Crazyflie to avoid
        others = [other_cf for other_cf in allcfs.crazyflies if other_cf.id != cf.id]
        cf.enableCollisionAvoidance(others, ellipsoid_radii)
        print(f"Collision Avoidance Enabled for CF {cf.id} with ellipsoid radii {ellipsoid_radii}")

    for cf in allcfs.crazyflies:
        print("Takeoff")
        cf.takeoff(targetHeight=object_pt[2]/2, duration=5)
    timeHelper.sleep(5)  # Ensure that takeoff completes

    start_time = timeHelper.time()
    
    while currentLayer <= desireLayer and swarm.input.checkIfAnyButtonIsPressed() == None:

        if swarm.input.checkIfAnyButtonIsPressed():
            print("emergency land")
            allcfs.land(targetHeight=0.01, duration=6)
            timeHelper.sleep(6)
            break
        # Create Collinear_to_traj object and calculate initial positions
        CT = Collinear_to_traj([pt1, pt2, pt3, pt4], object_pt, object_r, currentRopeLength, currentLayer)
        s1, s2, s3, s4 = CT.calculate_rope_distances(currentLayer, currentRopeLength)

        if currentLayer % 2 == 1:
            init1, init2, init3, init4 = CT.calculate_four_robot_position(pt1, pt2, s1, s2, s3, s4)
        else:
            init1, init2, init3, init4 = CT.calculate_four_robot_position(pt3, pt4, s1, s2, s3, s4)

        # Move Crazyflies to initial positions smoothly and simultaneously
        start_positions = [cf.position() for cf in allcfs.crazyflies]
        end_positions = [init1, init2, init3, init4]
        move_smoothly_simultaneously(allcfs, start_positions, end_positions, initial_position_duration, timeHelper)
        start_positions = end_positions  # Update start positions after reaching initial positions
        print("move to initial position")
        layer_start_time = timeHelper.time()


        # Perform the trajectory for the current layer
        while timeHelper.time() - layer_start_time < duration_per_layer and swarm.input.checkIfAnyButtonIsPressed() == None:
            current_time = timeHelper.time() - layer_start_time
            t = Rotation_num * ((current_time % duration_per_layer) / duration_per_layer)

            if currentLayer % 2 == 1:
                r1_pos, r2_pos = CT.trajz_to_t(t, object_pt, init1, init2, 1)
                r3_pos, r4_pos = CT.trajz_to_t(t, object_pt, init3, init4, -1)
            else:
                r1_pos, r3_pos = CT.trajz_to_t(t, object_pt, init1, init3)
                r2_pos, r4_pos = CT.trajz_to_t(t, object_pt, init2, init4)

            new_positions = [r1_pos, r2_pos, r3_pos, r4_pos]
            move_smoothly_simultaneously(allcfs, start_positions, new_positions, 1, timeHelper)
            start_positions = new_positions  # Update start positions for next move

            if swarm.input.checkIfAnyButtonIsPressed():
                print("emergency land")
                allcfs.land(targetHeight=0.01, duration=8)
                timeHelper.sleep(8)
                cf.emergency()
                break

        # Move to the next layer
        currentLayer += 1
        currentRopeLength -= np.pi * object_r + 0.1

    print("End of trajectory reached, preparing to land.")
    allcfs.land(targetHeight=0.01, duration=10)
    timeHelper.sleep(10)
    print("Landing complete.")

if __name__ == "__main__":
    main()
