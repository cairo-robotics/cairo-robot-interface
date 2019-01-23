#!/usr/bin/env python

import rospy, tf, numpy as np, json, rospkg, yaml
from os import system
from scipy.optimize import minimize
from scipy.spatial.distance import euclidean
import pyquaternion as pq
import time


def record_calibration_points(world_frame_robot='/base',
                              world_frame_opt='/optitrack_world',
                              calib_frame_robot='/screen',
                              calib_frame_opt='/optitrack_screen',
                              continuous=True,
                              duration=12,
                              min_dist=0.01,
                              max_dist=.15,
                              max_dur=0.05):
    tfl = tf.TransformListener(True, rospy.Duration(2))  # tf will have 2 seconds of cache

    mat_robot = []  # Matrix of all calibration points of calib_frame_robot in frame world_frame_robot (position only)
    mat_opt = []  # Matrix of all calibration points of calib_frame_opt in frame world_frame_opt (position only)
    max_dur = rospy.Duration(max_dur)  # seconds
    duration = rospy.Duration(duration)

    start = rospy.Time.now()
    last_point = None
    entry = ""
    while continuous and rospy.Time.now() < start + duration or not continuous and entry == "":
        tfl.waitForTransform(world_frame_robot, calib_frame_opt, rospy.Time(0), rospy.Duration(4.0))
        ref_time = tfl.getLatestCommonTime(world_frame_robot, calib_frame_opt)
        now = rospy.Time.now()

        if ref_time > now - max_dur:
            try:
                tfl.waitForTransform(world_frame_robot, calib_frame_robot, rospy.Time(), rospy.Duration(4.0))
                pose_rg_robot = tfl.lookupTransform(world_frame_robot, calib_frame_robot, rospy.Time(0))
            except Exception, e:
                print("Robot <-> Optitrack transformation not available at the last known common time:", e.message)
            else:
                # only collect points if the distance in the robot frame is greater than the last obtained point.
                if last_point is None or euclidean(np.array(pose_rg_robot[0] + pose_rg_robot[1]), np.array(last_point[0] + last_point[1])) > min_dist and euclidean(np.array(pose_rg_robot[0] + pose_rg_robot[1]), np.array(last_point[0] + last_point[1])) < max_dist:
                    try:
                        tfl.waitForTransform(world_frame_opt, calib_frame_opt, rospy.Time(0), rospy.Duration(4.0))
                        pose_rg_opt = tfl.lookupTransform(world_frame_opt, calib_frame_opt, rospy.Time(0))
                    except:
                        print("Optitrack Marker not visible at the last know common time")
                    else:
                        mat_robot.append(np.array(pose_rg_robot))
                        mat_opt.append(np.array(pose_rg_opt))
                        last_point = pose_rg_robot
        else:
            print("TFs are", (now - ref_time).to_sec(), "sec late")

        if continuous:
            rospy.sleep(0.25)
        else:
            entry = raw_input("Press enter to record a new point or q-enter to quit ({} points)".format(len(mat_robot)))
    print("OPTITRACK MATRIX:")
    print(mat_opt)
    print("\n\nROBOT MATRIX")
    print(mat_robot)
    return mat_opt, mat_robot


def extract_transforms(flat_transforms):
    # a transform is 3 pos and 4 rot
    nb_transform = len(flat_transforms) / 7
    list_transforms = []
    for i in range(nb_transform):
        pose = []
        # extract the pose
        pose.append(flat_transforms[i * 7:i * 7 + 3])
        pose.append(flat_transforms[i * 7 + 3:i * 7 + 7])
        # append it to the list of transforms
        list_transforms.append(pose)
    return list_transforms


def result_to_calibration_matrix(result):
    trans_mat, rot_mat = get_pose_matrices(result[0], result[1])
    inv_trans_mat = tf.transformations.inverse_matrix(trans_mat)
    rot_mat = tf.transformations.inverse_matrix(rot_mat)
    pose = get_pose_from_matrices(inv_trans_mat, rot_mat)
    print(pose)
    return [map(float, pose[0]), map(float, pose[1].tolist())]


def get_pose_matrices(pos, quat):
    trans_mat = tf.transformations.translation_matrix(pos)
    rot_mat = tf.transformations.quaternion_matrix(quat)
    return trans_mat, rot_mat


def get_pose_from_matrices(trans_mat, rot_mat):
    trans = tf.transformations.translation_from_matrix(trans_mat)
    quat = tf.transformations.quaternion_from_matrix(rot_mat)
    return [trans, quat]


def multiply_A_B(A, B):
    A_trans_mat, A_rot_mat = get_pose_matrices(A[0], A[1])
    B_trans_mat, B_rot_mat = get_pose_matrices(B[0], B[1])
    prod_trans_mat = np.dot(A_trans_mat, B_trans_mat)
    prod_rot_mat = np.dot(A_rot_mat, B_rot_mat)
    pose = get_pose_from_matrices(prod_trans_mat, prod_rot_mat)
    return pose


def evaluate_calibration(calibrations, coords_robot, coords_opt):
    def quaternion_cost(norm_coeff):
        C = 0
        for transform in list_calibr:
            # norm of a quaternion is always 1
            C += norm_coeff * abs(np.linalg.norm(transform[1]) - 1)
        return C

    def distance_cost(pose1, pose2, rot_coeff=2):
        pos_cost = 0
        # calculate position distance
        pos_cost = np.linalg.norm(np.array(pose1[0]) - np.array(pose2[0]))
        # distance between two quaternions
        rot_cost = 1 - np.inner(pose1[1], pose2[1])**2
        return pos_cost + rot_coeff * rot_cost

    # first extract the transformations
    list_calibr = extract_transforms(calibrations)
    # set the base transform
    A = list_calibr[0]
    B = list_calibr[1]
    # loop trough all the transforms
    cost = quaternion_cost(1)
    nb_points = len(coords_robot)
    for i in range(nb_points):
        robot = coords_robot[i]
        opt = coords_opt[i]
        A_trans_mat, A_rot_mat = get_pose_matrices(A[0], A[1])
        B_trans_mat, B_rot_mat = get_pose_matrices(B[0], B[1])
        rob_trans_mat, rob_rot_mat = get_pose_matrices(robot[0], robot[1])
        # opt_trans_mat, opt_rot_mat = get_pose_matrices(opt[0], opt[1])

        prod_trans_mat = np.dot(rob_trans_mat, B_trans_mat)
        prod_rot_mat = np.dot(rob_rot_mat, B_rot_mat)

        # prod_trans_mat = np.dot(rob_trans_mat, B_trans_mat)
        # prod_rot_mat = np.dot(rob_rot_mat, B_rot_mat)
        prod_trans_mat = np.dot(A_trans_mat, prod_trans_mat)
        prod_rot_mat = np.dot(A_rot_mat, prod_rot_mat)
        product = get_pose_from_matrices(prod_trans_mat, prod_rot_mat)

        product[1] /= np.linalg.norm(product[1])
        cost += distance_cost(opt, product)
    return cost

if __name__ == "__main__":

    rospy.init_node('optitrack_calibrator')

    rospack = rospkg.RosPack()
    world_frame_robot = '/world'  # The world frame of the robot
    world_frame_opt = '/optitrack_world'  # The world frame of optitrack
    calib_frame_robot = '/right_electric_gripper_base'  # The frame used for calibration in the robot's frame
    calib_frame_opt = '/optitrack_gripper'  # The frame used for calibration in the optitrack's frame

    # set optimization bounds
    bounds = []
    pos_bounds = [-4, 4]
    rot_bounds = [-1, 1]
    for i in range(2):
        for j in range(3):
            bounds.append(pos_bounds)
        for j in range(4):
            bounds.append(rot_bounds)
    # Record during 60 sec... set continuous=False for an interactive mode
    mat_opt, mat_robot = record_calibration_points(world_frame_robot=world_frame_robot,
                                                   world_frame_opt=world_frame_opt,
                                                   calib_frame_robot=calib_frame_robot,
                                                   calib_frame_opt=calib_frame_opt,
                                                   continuous=True)
    print(len(mat_opt), "points recorded")
    print
    print
    # Optimization
    initial_guess = [0, 0, 0, 0, 0, 0, 1] * 2
    t0 = time.time()

    # Be patient, this can run for a long time....
    result = minimize(evaluate_calibration, initial_guess, args=(mat_robot, mat_opt, ),
                      method='L-BFGS-B', bounds=bounds,
                      options={'maxiter': 25000})
    print(time.time() - t0, "seconds of optimization")

    print(result)
    result_list = extract_transforms(result.x)

    # Dumping A i.e. the calibration matrix (the transformation between the optitrack frame and the robot base frame)
    calibration_matrix_a = result_to_calibration_matrix(result_list[0])
    rospy.set_param("/optitrack/calibration_matrix", calibration_matrix_a)
    with open(rospack.get_path("cairo_robot_interface") + "/config/calibration_matrix.yaml", 'w') as f:
        yaml.dump(calibration_matrix_a, f)

    # Dumping B (the transformation between the optitrack marker and the calibration frame)
    calibration_matrix_b = result_to_calibration_matrix(result_list[1])
    with open(rospack.get_path("cairo_robot_interface") + "/config/calibration_matrix_b.yaml", 'w') as f:
        yaml.dump(calibration_matrix_b, f)

    # Testing calibration quality
    # mat_opt_check, mat_robot_check = record_calibration_points(continuous=True)
    # print len(mat_opt), "points recorded"
    # calculate_cost(result_a, result_b, mat_robot_check, mat_opt_check)

    # avg_error = calculate_position_error(result_a, result_b, mat_robot_check, mat_opt_check) / len(mat_opt_chec)
    # print(avg_error)