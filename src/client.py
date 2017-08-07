#!/usr/bin/env python
import roslib
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# We are going to use OO by creating a Class called Joint with
# the goal as the Class method, therefore we just instantiate a Joint with the desired angle

class Joint:
        def __init__(self, motor_name):
            
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            # We MUST give the exact same name in the joint name as declared in the tilt.yaml
            goal.trajectory.joint_names = ['arm_joint']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1)                   
            goal.trajectory.points.append(point) 
            self.jta.send_goal_and_wait(goal)
            

def main():
            # Instantiate our class
            arm = Joint('arm')
            # If we wish more than 1 Joint, by the time we instantiate our Joint we would
            # give 2 or more points, each point is responsible to move each Joint, it is important
            # to say that we always must pass it as a vector/list
            point1 = float(input ("Type in the first point: "))
            point2 = float(input ("Type in the second point: "))
            while (1):
                arm.move_joint([point1])
                arm.move_joint([point2])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_test')
      main()