#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# criamos uma classe chamada junta, assim basta instanciar uma junta qualquer
# o metodo dessa classe sera mover a junta (bem obvio), bastando passar os pontos

class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            #char = self.name[0] ,either 'f' or 'b'
            # no tilt.yaml definimos os controladores e o nome da junta, agora vamos passar o tipo
            # usando "f" ou "b", nesse exemplo usaremos o "f" mesmo, no sentindo de mover o f_arm
            goal.trajectory.joint_names = ['joint_3f']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)                   
            goal.trajectory.points.append(point)
            #client.send_goal_and_wait...    
            self.jta.send_goal_and_wait(goal)
            

def main():
            f_arm = Joint('f_arm')
            # se fossem 3 juntas, entao passariamos uma lista de pontos [0.5,1.5,1.0]
            # tomar cuidado pois SEMPRE tem que ser um vetor, mesmo sendo 1 junta apenas
            mateuzao = float(input ("Digite a posicao da junta: "))
            f_arm.move_joint([mateuzao])
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()