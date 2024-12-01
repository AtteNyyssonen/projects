#!/usr/bin/env python3
import math
import rospy
import tf
import copy
import PyKDL 
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_commander import MoveGroupCommander
from std_srvs.srv import SetBool
from pick_place_application.srv import GenerateRandomPose
from motion_test_pkg.srv import BoxSpawner, BoxSpawnerRequest, BoxAttach, BoxAttachRequest
import actionlib
from pick_place_application.msg import MotionstartAction, MotionstartResult, MotionstartFeedback

def frame_to_pose(frame):
	pose_result = Pose()
	pose_result.position.x = frame.p[0] 
	pose_result.position.y = frame.p[1] 
	pose_result.position.z = frame.p[2] 
	ang = frame.M.GetQuaternion() 
	pose_result.orientation.x = ang[0] 
	pose_result.orientation.y = ang[1] 
	pose_result.orientation.z = ang[2] 
	pose_result.orientation.w = ang[3]
	return pose_result

def pose_stamped_message(pose):
    pose_stamped_wrist = PoseStamped()
    pose_stamped_wrist.header.frame_id = "world"
    pose_stamped_wrist.header.stamp = rospy.get_rostime()
    pose_stamped_wrist.pose = pose
    pose_stamped_offset_wrist = copy.deepcopy(pose_stamped_wrist)
    pose_stamped_offset_wrist.pose.position.z += 0.1

    return pose_stamped_wrist, pose_stamped_offset_wrist

class MotionControlAction:
    def __init__(self):
        rospy.init_node('motion_control', anonymous=True)

        self.arm1 = MoveGroupCommander("arm1")
        self.arm2 = MoveGroupCommander("arm2")
        self.gripper1 = MoveGroupCommander("gripper1")
        self.gripper2 = MoveGroupCommander("gripper2")

        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)

        # Service clients
        rospy.wait_for_service('/generate_random_pose')
        self.random_pose_srv = rospy.ServiceProxy('/generate_random_pose', GenerateRandomPose)

        rospy.wait_for_service('/scene_spawner/spawn_box')
        self.spawn_box_srv = rospy.ServiceProxy('/scene_spawner/spawn_box', BoxSpawner)

        rospy.wait_for_service('/scene_spawner/attach_release_box')
        self.attach_release_srv = rospy.ServiceProxy('/scene_spawner/attach_release_box', BoxAttach)

        # Action server
        self.action_server = actionlib.SimpleActionServer(
            'pick_place_action', MotionstartAction, self.execute_pick_place, False
        )

        self.initialize_grippers()
        self.action_server.start()
        rospy.loginfo("Pick-and-Place Action Server Started")

    def initialize_grippers(self):
        rospy.loginfo("Initializing grippers to open state...")
        self.control_gripper(self.gripper1, open=True)
        self.control_gripper(self.gripper2, open=True)
        rospy.loginfo("Grippers are set to open.")

    def execute_pick_place(self, goal):
        feedback = MotionstartFeedback()
        result = MotionstartResult()

        rospy.loginfo("Generating random poses...")
        try:
            start_pose_point = self.random_pose_srv().pose
            goal_pose_point = self.random_pose_srv().pose
            distance_x = goal_pose_point.x - start_pose_point.x
            distance_y = goal_pose_point.y - start_pose_point.y
            if abs(distance_x) <= 0.2 or abs(distance_y) <= 0.2: # Don't  spawn next to eachother
                start_pose_point = self.random_pose_srv().pose

            rospy.loginfo(f"Start pose: {start_pose_point}, Goal pose: {goal_pose_point}")

            self.spawn_box(start_pose_point, "box", base=False)
            self.spawn_box(goal_pose_point, "goal", base=True)

            feedback.status = "Boxes spawned"
            self.action_server.publish_feedback(feedback)

            object_frame = PyKDL.Frame()
            goal_frame = PyKDL.Frame()
            object_frame.p = PyKDL.Vector(start_pose_point.x, start_pose_point.y, start_pose_point.z)
            goal_frame.p = PyKDL.Vector(goal_pose_point.x, goal_pose_point.y, goal_pose_point.z)
            gripper_length = 0.22
            start_frame = copy.deepcopy(object_frame)
            goal_frame = copy.deepcopy(goal_frame)
            start_frame.p[2] += gripper_length
            goal_frame.p[2] += gripper_length
            goal_frame.p[2] += 0.05 # Offset of the goal platform
            start_frame.M.DoRotX(3.14)
            goal_frame.M.DoRotX(3.14)
            start_pose = frame_to_pose(start_frame)
            goal_pose = frame_to_pose(goal_frame)

            arm1_pose_stamped_target_wrist, arm1_pose_stamped_offset_target_wrist = pose_stamped_message(start_pose)
            arm1_pose_stamped_goal_wrist, arm1_pose_stamped_offset_goal_wrist = pose_stamped_message(goal_pose)
            arm2_pose_stamped_target_wrist, arm2_pose_stamped_offset_target_wrist = pose_stamped_message(start_pose)
            arm2_pose_stamped_goal_wrist, arm2_pose_stamped_offset_goal_wrist = pose_stamped_message(goal_pose)


            arm1_can_reach_start = self.check_reachability(self.arm1, arm1_pose_stamped_target_wrist)
            arm2_can_reach_start = self.check_reachability(self.arm2, arm2_pose_stamped_target_wrist)
            arm1_can_reach_goal = self.check_reachability(self.arm1, arm1_pose_stamped_goal_wrist)
            arm2_can_reach_goal = self.check_reachability(self.arm2, arm2_pose_stamped_goal_wrist)

            rospy.loginfo(f"Reachability - Arm1: Start={arm1_can_reach_start}, Goal={arm1_can_reach_goal}; "
                        f"Arm2: Start={arm2_can_reach_start}, Goal={arm2_can_reach_goal}")
            feedback.status = "Start moving arm towards box"
            self.action_server.publish_feedback(feedback)
    
            if arm1_can_reach_start and arm1_can_reach_goal:
                rospy.loginfo("Using a single arm (arm1) for pick-and-place")
                self.use_single_arm("robot1", arm1_pose_stamped_target_wrist, arm1_pose_stamped_offset_target_wrist,
                                    arm1_pose_stamped_goal_wrist, arm1_pose_stamped_offset_goal_wrist)
            elif arm2_can_reach_start and arm2_can_reach_goal:
                rospy.loginfo("Using a single arm (arm2) for pick-and-place")
                self.use_single_arm("robot2", arm2_pose_stamped_target_wrist, arm2_pose_stamped_offset_target_wrist,
                                    arm2_pose_stamped_goal_wrist, arm2_pose_stamped_offset_goal_wrist)
            elif arm1_can_reach_start and arm2_can_reach_goal:
                rospy.loginfo("Using both arms for pick-and-place")
                self.use_both_arms(arm1_pose_stamped_target_wrist, arm1_pose_stamped_offset_target_wrist, 
                                   arm2_pose_stamped_goal_wrist, arm2_pose_stamped_offset_goal_wrist, "robot1", "robot2")
            elif arm2_can_reach_start and arm1_can_reach_goal:
                rospy.loginfo("Using both arms for pick-and-place")
                self.use_both_arms(arm2_pose_stamped_target_wrist, arm2_pose_stamped_offset_target_wrist, 
                                   arm1_pose_stamped_goal_wrist, arm1_pose_stamped_offset_goal_wrist, "robot2", "robot1")
            else:
                rospy.logerr("Neither arm can reach both the start and goal poses. Aborting.")
                result.success = False
                self.action_server.set_aborted(result)
                return

            feedback.status = "Released box in the final location"
            self.action_server.publish_feedback(feedback)
            result.success = True
            self.action_server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error in pick-and-place: {e}")
            result.success = False
            self.action_server.set_aborted(result)
    
    def check_reachability(self, arm, pose):
        try:
            arm.set_pose_target(pose)
            plan = arm.plan()
            return plan[0]
        except Exception as e:
            rospy.loginfo(f"Hit a problem: {e}\n when creating a plan.")
            return False

    def spawn_box(self, pose, name, base):
        req = BoxSpawnerRequest()
        req.name = name
        req.x = pose.x
        req.y = pose.y
        req.base = base
        self.spawn_box_srv(req)

    def attach_release_box(self, robot_name, box_name, attach):
        req = BoxAttachRequest()
        req.robot_name = robot_name
        req.box_name = box_name
        req.attach = attach
        self.attach_release_srv(req)

    def control_gripper(self, gripper, open=True):
        if open:
            rospy.loginfo(f"Opening {gripper.get_name()}...")
            gripper.set_named_target("open")
        else:
            rospy.loginfo(f"Closing {gripper.get_name()}...")
            gripper.set_named_target("close")
        gripper.go(wait=True)

    def use_single_arm(self, arm_name, start_pose, start_pose_offset, goal_pose, goal_pose_offset):
        feedback = MotionstartFeedback()
        arm = self.arm1 if arm_name == "robot1" else self.arm2
        gripper = self.gripper1 if arm_name == "robot1" else self.gripper2

        arm.set_pose_target(start_pose_offset)
        arm.go(wait=True)
        rospy.sleep(0.5)
        (plan, fraction) = arm.compute_cartesian_path([start_pose_offset.pose, start_pose.pose], 0.01, False)
        arm.execute(plan, wait=True)

        self.attach_release_box(arm_name, "box", attach=True)
        rospy.sleep(0.5)
        gripper.set_named_target("grasp")
        gripper.go(wait=True)
        rospy.sleep(0.5)

        feedback.status = "Grasped box"
        self.action_server.publish_feedback(feedback)

        (plan, fraction) = arm.compute_cartesian_path([start_pose.pose, start_pose_offset.pose,], 0.01, False)
        arm.execute(plan, wait=True)

        feedback.status = "Moving box towards the goal"
        self.action_server.publish_feedback(feedback)

        arm.set_pose_target(goal_pose_offset)
        arm.go(wait=True)
        rospy.sleep(0.5)
        (plan, fraction) = arm.compute_cartesian_path([goal_pose_offset.pose, goal_pose.pose], 0.01, False)
        arm.execute(plan, wait=True)

        self.attach_release_box(arm_name, "box", attach=False)
        self.control_gripper(gripper, True)
        arm.set_named_target("home")
        arm.go(wait=True)

    def use_both_arms(self, start_pose, start_pose_offset, goal_pose, goal_pose_offset, first_arm_name, second_arm_name):
        feedback = MotionstartFeedback()
        first_arm = self.arm1 if first_arm_name == "robot1" else self.arm2
        first_gripper = self.gripper1 if first_arm_name == "robot1" else self.gripper2
        second_arm = self.arm2 if second_arm_name == "robot2" else self.arm1
        second_gripper = self.gripper2 if second_arm_name == "robot2" else self.gripper1

        start_position_data = start_pose.pose.position
        goal_position_data = goal_pose.pose.position
        mid_point_object = PyKDL.Frame()
        mid_point_object.p = PyKDL.Vector(0.000, 
                                         (start_position_data.y + goal_position_data.y) / 2, 
                                          0.699)
        gripper_length = 0.22
        mid_point_frame = copy.deepcopy(mid_point_object)
        if math.isclose(mid_point_frame.p[1],0, abs_tol=0.2) and mid_point_frame.p[1] < 0:
            mid_point_frame.p[1] -= 0.35
        elif math.isclose(mid_point_frame.p[1],0, abs_tol=0.2) and mid_point_frame.p[1] > 0:
            mid_point_frame.p[1] += 0.35
        mid_point_frame.p[2] += gripper_length
        mid_point_frame.M.DoRotX(3.14)
        mid_point_pose = frame_to_pose(mid_point_frame)
        arm1_mid_pose_stamped_target, arm1_mid_pose_stamped_offset = pose_stamped_message(mid_point_pose)
        arm2_mid_pose_stamped_target, arm2_mid_pose_stamped_offset = pose_stamped_message(mid_point_pose)
        rospy.loginfo(f"MID POINT pose: {mid_point_pose}")

        first_arm.set_pose_target(start_pose_offset)
        first_arm.go(wait=True)
        rospy.sleep(0.5)
        (plan, fraction) = first_arm.compute_cartesian_path([start_pose_offset.pose, start_pose.pose], 0.01, False)
        first_arm.execute(plan, wait=True)

        self.attach_release_box(first_arm_name, "box", attach=True)
        rospy.sleep(0.5)
        first_gripper.set_named_target("grasp")
        first_gripper.go(wait=True)
        rospy.sleep(0.5)
        feedback.status = "Grasped box with the first arm"
        self.action_server.publish_feedback(feedback)

        (plan, fraction) = first_arm.compute_cartesian_path([start_pose.pose, start_pose_offset.pose,], 0.01, False)
        first_arm.execute(plan, wait=True)

        feedback.status = "Move box towards midway point"
        self.action_server.publish_feedback(feedback)
        first_arm.set_pose_target(arm1_mid_pose_stamped_offset)
        first_arm.go(wait=True)
        rospy.sleep(0.5)
        first_arm.set_pose_target(arm1_mid_pose_stamped_target)
        first_arm.go(wait=True)
        # Issues with compute_cartesian_path() function in latest MoveIt version.
        #(plan, fraction) = first_arm.compute_cartesian_path([arm1_mid_pose_stamped_offset.pose, 
                                                             #arm1_mid_pose_stamped_target.pose], 0.05, False)
        #first_arm.execute(plan, wait=True)

        self.attach_release_box(first_arm_name, "box", attach=False)
        self.control_gripper(first_gripper, True)
        feedback.status = "Released box in the midway location"
        self.action_server.publish_feedback(feedback)
        first_arm.set_named_target("home")
        first_arm.go(wait=True)
        rospy.sleep(0.5)
        ### second arm pick-up from intermediate point
        feedback.status = "Move second arm towards the box"
        self.action_server.publish_feedback(feedback)
        second_arm.set_pose_target(arm2_mid_pose_stamped_offset)
        second_arm.go(wait=True)
        rospy.sleep(0.5)
        second_arm.set_pose_target(arm2_mid_pose_stamped_target)
        second_arm.go(wait=True)
        #(plan, fraction) = second_arm.compute_cartesian_path([arm2_mid_pose_stamped_offset.pose, 
                                                              #arm2_mid_pose_stamped_target.pose], 0.05, False)
        #second_arm.execute(plan, wait=True)

        self.attach_release_box(second_arm_name, "box", attach=True)
        rospy.sleep(0.5)
        second_gripper.set_named_target("grasp")
        second_gripper.go(wait=True)
        feedback.status = "Grasped box with the second arm"
        self.action_server.publish_feedback(feedback)
        #(plan, fraction) = second_arm.compute_cartesian_path([arm2_mid_pose_stamped_target.pose, 
                                                              #arm2_mid_pose_stamped_offset.pose], 0.05, False)
        #second_arm.execute(plan, wait=True)
        second_arm.set_pose_target(arm2_mid_pose_stamped_offset)
        second_arm.go(wait=True)
        feedback.status = "Move second arm towards the goal location"
        self.action_server.publish_feedback(feedback)
        second_arm.set_pose_target(goal_pose_offset)
        second_arm.go(wait=True)
        rospy.sleep(0.5)
        (plan, fraction) = second_arm.compute_cartesian_path([goal_pose_offset.pose, goal_pose.pose], 0.01, False)
        second_arm.execute(plan, wait=True)

        self.attach_release_box(second_arm_name, "box", attach=False)
        self.control_gripper(second_gripper, True)
        rospy.sleep(0.5)
        second_arm.set_named_target("home")
        second_arm.go(wait=True)

if __name__ == "__main__":
    MotionControlAction()
    rospy.spin()