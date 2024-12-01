#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Point
from pick_place_application.srv import GenerateRandomPose, GenerateRandomPoseResponse

class RandomPoseGeneratorService:
    def __init__(self):

        rospy.init_node('random_pose_service', anonymous=True)
        self.service = rospy.Service('/generate_random_pose', GenerateRandomPose, self.handle_generate_random_pose)


    def handle_generate_random_pose(self, req):
        # Generate a random pose
        random_pose = Point()
        while True:
            random_pose.x = random.uniform(-1.4, 1.4)
            random_pose.y = random.uniform(-0.7, 0.7)
            if self.is_pose_valid(random_pose):
                break
        
        random_pose.z = 0.699

        rospy.loginfo(f"Generated random pose: {random_pose}")

        return GenerateRandomPoseResponse(random_pose)

    def is_pose_valid(self, new_pose):
        # Check if the pose is outside of the robots bases
        if -1.3 <= new_pose.x <= 1.3 and -0.5 <= new_pose.y <= 0.5:
            return False
        else:
            return True

if __name__ == "__main__":
    RandomPoseGeneratorService()
    rospy.spin()
