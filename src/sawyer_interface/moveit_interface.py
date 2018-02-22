'''
MoveI wrappers for creating various servers to send commands to Sawyer and interface with MoveIt.
'''
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Float32MultiArray
from cairo_sawyer_interface.srv import ForwardKinematics
# from forward_kinematics.forward_kinematics_client import ForwardKinematicsClient
from kinematics_interface.servers import ForwardKinematicsServer, InverseKinematicsServer, RobotStateValidityServer


class SawyerMoveitInterface(object):
    '''creat various servers for reading in pose and joint data and moving
    sawyer accordingly'''

    def __init__(self, PLANNING_GROUP="right_arm",
                 sub_pose_topic="/commander/pose",
                 sub_pose_array_topic="/commander/pose_array",
                 sub_joint_state_topic="/commander/joint_state",
                 pub_topic="/test_topic"):
        '''creates subs pubs and moveit_commander groups'''

        self.sub_pose = rospy.Subscriber(sub_pose_topic, Pose,
                                         self._moveit_pose_callback,
                                         queue_size=1)
        self.sub_pose_array = rospy.Subscriber(sub_pose_array_topic, PoseArray,
                                               self._moveit_pose_array_callback,
                                               queue_size=1)
        self.sub_joint_state = rospy.Subscriber(sub_joint_state_topic,
                                                Float32MultiArray,
                                                self._moveit_joint_state_callback,
                                                queue_size=1)
        self.diagnostic = rospy.Publisher(pub_topic, String, queue_size=1)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        # self.fk_client = ForwardKinematicsClient()
        self.fk_server = ForwardKinematicsServer()
        self.ik_server = InverseKinematicsServer()
        self.rs_validity = RobotStateValidityServer()

    def _moveit_pose_callback(self, pose):
        '''pose callback function'''
        self.group.set_pose_target(pose)
        self.plan = self.group.plan()
        self.group.go()
        self.diagnostic.publish("future completion status message")

    def _moveit_pose_array_callback(self, pose_array):
        '''use an array of poses to creat waypoints for path plannner'''
        # TODO this still needs work
        waypoints = []
        for i in range(len(pose_array.poses)):
            waypoints.append(pose_array.poses[i])
            print i

        print "first"
        print waypoints[0]
        plan3 = self.group.plan(waypoints)
        self.group.execute(plan3)
        print "last:"
        print waypoints[9]

    def _moveit_joint_state_callback(self, joint_states):
        '''joint state callback function'''
        print("called joint state")
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(joint_states.data)
        self.group.go()
        self.diagnostic.publish("future completion status message")

    def set_joint_start(self, joint_positions):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        joint_state.position = joint_positions
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.group.set_start_state(moveit_robot_state)

    def set_pose_target(self, pose):
        '''set the pose target using the MoveGroupCommander'''
        self.group.set_pose_target(pose)

    def set_joint_target(self, joints):
        '''set the joint targets using the MoveGroupCommander'''
        self.group.set_joint_value_target(joints)

    def plan(self):
        '''set the pose plan using the MoveGroupCommander'''
        return self.group.plan()

    def execute(self, plan):
        '''execute the plan using MoveGroupCommander'''
        self.group.execute(plan)

    def move_to_pose_target(self, pose):
        '''use pose to set target, plan, and execute'''
        self.set_pose_target(pose)
        plan = self.plan()
        self.group.execute(plan)

    def move_to_pose_targets(self, poses):
        '''use poses to set targets, plan, and execute'''
        for pose in poses:
            self.set_pose_target(pose)
            plan = self.plan()
            self.group.execute(plan)

    def move_to_joint_target(self, joints):
        '''use joints to set target, plan, and execute'''
        self.set_joint_target(joints)
        plan = self.plan()
        self.group.execute(plan)

    def get_end_effector_pose(self, joint_positions):
        try:
            response = self.fk_server.call(joint_positions)
            return response.pose_stamped[0].pose
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_pose_IK_joints(self, pose_vector):
        pose = Pose()
        pose.position.x = pose_vector[0]
        pose.position.y = pose_vector[1]
        pose.position.z = pose_vector[2]
        pose.orientation.x = pose_vector[3]
        pose.orientation.y = pose_vector[4]
        pose.orientation.z = pose_vector[5]
        pose.orientation.w = pose_vector[6]
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.frame_id = "/base"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose
            response = self.ik_server.call(pose_stamped)
            # Caveat: No joints are returned if the Pose is within a collision object, which iteself is a from
            # of validation
            p = response.solution.joint_state.position
            if len(p) != 0:
                return [p[0], p[2], p[3], p[4], p[5], p[6], p[7]]
            else:
                return []
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def convert_plan_to_dict(self, plan):
        plan_dicts = []
        for point in plan.joint_trajectory.points:
            pose = self.get_end_effector_pose(point.positions)
            position = pose.position
            orientation = pose.orientation
            data = {
                "joints": point.positions,
                "position": [position.x, position.y, position.z],
                "orientation": [orientation.x, orientation.y, orientation.z, orientation.w]
            }
            plan_dicts.append(data)
        return plan_dicts

    def checkPointValidity(self, joint_positions, groups=[]):
        state = RobotState()
        state.joint_state.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        state.joint_state.position = joint_positions
        result = self.rs_validity.call(state, "right_arm")
        if not result.valid:
            rospy.logwarn("The following state is invalid:\n" + str(state) )
            return False
        else:
            return True