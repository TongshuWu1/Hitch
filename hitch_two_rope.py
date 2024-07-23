import time
import numpy as np
from pycrazyswarm import *
from class_circle_traj_2points import Collinear_to_traj
import sys
import os

sys.path.append('/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/scripts/optitrack_natnet')

from optitrack_natnet.NatNetClient import NatNetClient
from optitrack_natnet.util import quaternion_to_euler

clientAddress = "192.168.0.39" #"192.168.0.59"
optitrackServerAddress = "192.168.0.4"

objects = {}
object_rotations = {}






global object_pt

desireLayer = 3

ropeLength = 4.5
Rotation_num = 1
ellipsoid_radii = [0.12, 0.12, 0.12]
duration_per_layer = 30  # Duration for each layer in seconds
initial_position_duration = 10  # Duration to move to initial positions

def move_smoothly_simultaneously(swarm, allcfs, start_positions, end_positions, duration, timeHelper, object_pt):
    steps = int(duration * 100)  # 100 steps per second
    for step in range(steps):
        alpha = step / float(steps)
        for i, cf in enumerate(allcfs.crazyflies):
            current_pos = (1 - alpha) * start_positions[i] + alpha * end_positions[i]
            yaw_angle = np.arctan2(object_pt[1] - current_pos[1], object_pt[0] - current_pos[0])
            cf.cmdPosition(current_pos, yaw=yaw_angle)  # Make sure the angle is in radians

            if swarm.input.checkIfAnyButtonIsPressed():
                print("emergency land")
                allcfs.land(targetHeight=0.01, duration=6)
                timeHelper.sleep(6)
                cf.emergency()
                break
        timeHelper.sleep(0.01)


def move_circularly(swarm, allcfs, start_positions, end_positions, duration, timeHelper, center_point):
    steps = int(duration * 100)
    start_radii = [np.linalg.norm(start_pos - center_point) for start_pos in start_positions]  # Radii from center to start positions
    end_radii = [np.linalg.norm(end_pos - center_point) for end_pos in end_positions]  # Radii from center to end positions if you want to change radius
    start_angles = [np.arctan2(start_pos[1] - center_point[1], start_pos[0] - center_point[0]) for start_pos in start_positions]
    end_angles = [np.arctan2(end_pos[1] - center_point[1], end_pos[0] - center_point[0]) for end_pos in end_positions]

    # Calculate angle differences ensuring shortest rotation direction
    angle_differences = [((end_angle - start_angle + np.pi) % (2 * np.pi) - np.pi) for start_angle, end_angle in zip(start_angles, end_angles)]

    for step in range(steps):
        alpha = step / float(steps)
        for i, cf in enumerate(allcfs.crazyflies):
            radius = start_radii[i] + alpha * (end_radii[i] - start_radii[i])  # Interpolating the radius
            angle = start_angles[i] + alpha * angle_differences[i]
            x = center_point[0] + radius * np.cos(angle)
            y = center_point[1] + radius * np.sin(angle)
            z = (1 - alpha) * start_positions[i][2] + alpha * end_positions[i][2]  # Linear interpolation for z
            current_pos = np.array([x, y, z])
            yaw_angle = np.arctan2(center_point[1] - y, center_point[0] - x)

            cf.cmdPosition(current_pos, yaw=yaw_angle)

            if swarm.input.checkIfAnyButtonIsPressed():
                print("Emergency landing initiated")
                allcfs.land(targetHeight=0.01, duration=6)
                timeHelper.sleep(6)
                cf.emergency()
                return
        timeHelper.sleep(0.01)  # Short delay for smoother simulation


def receive_rigid_body_frame(id, position, rotation_quaternion):
    global objects, object_rotations
    objects[id] = position
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    object_rotations[id] = (rotx, roty, rotz)

def setup_optitrack_client():
    client = NatNetClient()
    client.set_client_address(clientAddress)
    client.set_server_address(optitrackServerAddress)
    client.set_use_multicast(True)
    client.rigid_body_listener = receive_rigid_body_frame
    client.run()  # Start the client as part of the initial setup, assume it runs indefinitely

def get_object_position(id):
    # Returns the position and rotation if available
    position = objects.get(id)
    rotation = object_rotations.get(id)
    return position, rotation




def main():
    
    setup_optitrack_client()  # Start the client to begin receiving data

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    currentLayer = 1
    currentRopeLength = ropeLength
    target_object_id = 497  # ID of the object you want to track
    slack_shift = 0.06 # Value shifter for slack


    position, rotation = get_object_position(target_object_id)
    if position and rotation:
        print(f"Position of object {target_object_id}:", position, "Rotation:", rotation)
        object_pt = position
        object_plane_shift = np.array([object_pt[0],object_pt[1],slack_shift])
    else:
        print("no data from object")
        return
    object_r = 0.25

    print(f"target height = {object_pt[2]}")

        # Define two points in 3D space
    pt1 = np.array([0.35 , 0.0, object_pt[2]]) + object_plane_shift
    pt2 = np.array([-0.35, 0.0,object_pt[2]]) + object_plane_shift

    pt3 = np.array([0.0, 0.35,object_pt[2]]) + object_plane_shift
    pt4 = np.array([0.0, -0.35,object_pt[2]]) + object_plane_shift



    for cf in allcfs.crazyflies:
        # List of other Crazyflies for the current Crazyflie to avoid
        others = [other_cf for other_cf in allcfs.crazyflies if other_cf.id != cf.id]
        cf.enableCollisionAvoidance(others, ellipsoid_radii)
        print(f"Collision Avoidance Enabled for CF {cf.id} with ellipsoid radii {ellipsoid_radii}")

    for cf in allcfs.crazyflies:
        print("Takeoff")
        cf.takeoff(targetHeight=object_pt[2], duration=5)
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
        move_circularly(swarm, allcfs, start_positions, end_positions, initial_position_duration, timeHelper, object_pt)
        start_positions = end_positions  # Update start positions after reaching initial positions
        print("move to initial position")

        layer_start_time = timeHelper.time()


        # Perform the trajectory for the current layer
        init1_rep = init1
        init2_rep = init2
        init3_rep = init3
        init4_rep = init4
        while True:
            current_time = timeHelper.time() - layer_start_time
            if current_time >= duration_per_layer:
                t = Rotation_num  # Normalize to ensure it reaches exactly the intended rotation number
            else:
                t = (current_time / duration_per_layer) * Rotation_num
            shrink_distance = 0.0005  # Change this to the desired distance


            if currentLayer % 2 == 1:
                r1_pos, r2_pos = CT.trajz_to_t(t, pt1, init1_rep, init2_rep, 1)
                r3_pos, r4_pos = CT.trajz_to_t(t, pt2, init3_rep, init4_rep, -1)
                unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
                unit_vector_init2_to_pt1 = (pt1 - init2) / np.linalg.norm(pt1 - init2)
                unit_vector_init3_to_pt2 = (pt2 - init3) / np.linalg.norm(pt2 - init3)
                unit_vector_init4_to_pt2 = (pt2 - init4) / np.linalg.norm(pt2 - init4)

                init1_rep = init1_rep + unit_vector_init1_to_pt1 * shrink_distance
                init2_rep = init2_rep + unit_vector_init2_to_pt1 * shrink_distance
                init3_rep  = init3_rep + unit_vector_init3_to_pt2 * shrink_distance
                init4_rep  = init4_rep + unit_vector_init4_to_pt2 * shrink_distance
            else:
                r1_pos, r3_pos = CT.trajz_to_t(t, pt1, init1_rep, init3_rep, 1)
                r2_pos, r4_pos = CT.trajz_to_t(t, pt2, init2_rep, init4_rep, -1)
                unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
                unit_vector_init3_to_pt1 = (pt1 - init3) / np.linalg.norm(pt1 - init3)
                unit_vector_init2_to_pt2 = (pt2 - init2) / np.linalg.norm(pt2 - init2)
                unit_vector_init4_to_pt2 = (pt2 - init4) / np.linalg.norm(pt2 - init4)
                
                init1_rep  = init1_rep + unit_vector_init1_to_pt1 * shrink_distance
                init2_rep  = init2_rep + unit_vector_init2_to_pt2 * shrink_distance
                init3_rep  = init3_rep + unit_vector_init3_to_pt1 * shrink_distance
                init4_rep  = init4_rep + unit_vector_init4_to_pt2 * shrink_distance

            new_positions = [r1_pos, r2_pos, r3_pos, r4_pos]
            move_smoothly_simultaneously(swarm, allcfs, start_positions, new_positions, 1, timeHelper, object_pt)
            start_positions = new_positions  # Update start positions for next move

            if current_time >= duration_per_layer:
                print("Completed full rotation for the layer")
                break

            if swarm.input.checkIfAnyButtonIsPressed():
                print("emergency land")
                allcfs.land(targetHeight=0.01, duration=10)
                timeHelper.sleep(10)
                for cf in allcfs.crazyflies:
                    cf.emergency()
                break

            #decrease init


            unit_vector_init1_to_pt1 = (pt1 - init1) / np.linalg.norm(pt1 - init1)
            unit_vector_init2_to_pt1 = (pt1 - init2) / np.linalg.norm(pt1 - init2)

            
        ##lisftup
        if currentLayer == desireLayer:

            print("move away from object point")

            
            # Retrieve current positions of all Crazyflies
            current_positions = [np.array(cf.position()) for cf in allcfs.crazyflies]
            
            # Set the displacement distance away from the object point
            displacement_distance = 0.03  # Adjust this value as needed
            
            # Calculate the displacement vectors to move away from the object point
            displacement_vectors = [
                (pos - object_pt) / np.linalg.norm(pos - object_pt) * displacement_distance
                for pos in current_positions
            ]
            
            # Calculate positions after moving away from the object point
            away_positions = [
                current_positions[i] + displacement_vectors[i] 
                for i in range(len(current_positions))
            ]
            
            # Move Crazyflies smoothly to the positions away from the object point
            move_smoothly_simultaneously(swarm, allcfs, current_positions, away_positions, 10, timeHelper, object_pt)
            
            
            print("lift up")

            # Set the desired lift height
            liftupHeight = 0.8
            
            # Calculate new positions for lifting up
            end_positions = [
                pos + np.array([0.0, 0.0, liftupHeight]) 
                for pos in away_positions
            ]
            
            # Move Crazyflies smoothly to the new elevated positions
            move_smoothly_simultaneously(swarm, allcfs, away_positions, end_positions, 10, timeHelper, object_pt)
            
            allcfs.land(targetHeight=0.01, duration=10)
            timeHelper.sleep(10)

        # Move to the next layer    
        currentLayer += 1
        currentRopeLength -= np.pi * object_r

    print("End of trajectory reached, preparing to land.")
    allcfs.land(targetHeight=0.01, duration=10)
    timeHelper.sleep(10)
    print("Landing complete.")

if __name__ == "__main__":
    main()
