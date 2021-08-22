import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import tf, random

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True


class RandomMove(object):
    def __init__(self):
        super(RandomMove, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('random_move',
                        anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.listener = tf.TransformListener()
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def relative_pose(self, parent, child):
        self.listener.waitForTransform(parent, child, rospy.Time(), rospy.Duration(1.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(parent, child, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(parent, child, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


    def go_to_joint_state(self):
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def random_pose(self):
        pose = PoseStamped()

        x = random.uniform(-0.05, 0.05)
        y = random.uniform(-0.3, 0.3)
        z = random.uniform(0.1, 0.5)

        roll = random.uniform(-3.14, 3.14)
        pitch = random.uniform(-3.14, 3.14)
        yaw = random.uniform(-3.14, 3.14)



    def go_to_target_pose(self):
        group = self.group

        #Convert from frame Grand_Truth to frame J6
        pos, rot = self.relative_pose("/Grand_Truth", "/J6")
        pos = list(pos)
        rpy = list(tf.transformations.euler_from_quaternion(rot))
        group.set_pose_target(pos)



    def go_to_random_pose(self):
        group = self.group
        joint_goal = group.get_random_joint_values()
        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


if __name__ == '__main__':
    random_move = RandomMove()
    random_move.go_to_target_pose()
