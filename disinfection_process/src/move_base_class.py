#! /usr/bin/env python

import rospy
import actionlib
import math

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes


class ArmMove():
    def __init__(self):
        self.move_group = MoveGroupInterface("arm", "rbkairos_base_link")
        self.joints = ["rbkairos_ur10_elbow_joint", "rbkairos_ur10_shoulder_lift_joint", "rbkairos_ur10_shoulder_pan_joint",
                  "rbkairos_ur10_wrist_1_joint", "rbkairos_ur10_wrist_2_joint", "rbkairos_ur10_wrist_3_joint"]
    ''' # podria no hacer falta esto
    def set_joint_pose(self, pose = [-2.10, -0.45, 0.0, -1.0, 2.3, 1.3]):
        self.goal_pose = pose # = [-2.10, -0.45, 0.0, -1.0, 2.3, 1.3], default
    '''

    def move_joints(self, pose = [-2.10, -0.45, 0.0, -1.0, 2.3, 1.3]):

        while not rospy.is_shutdown():

            result = self.move_group.moveToJointPosition(self.joints, pose, 0.02)
            if result:

                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Trajectory successfully executed!")
                    break
                else:
                    rospy.logerr("Arm goal in state: %s",
                                    self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt failure! No result returned.")

        self.move_group.get_move_action().cancel_all_goals()




class CircularMove():
    def __init__(self, _coord, _radio): # TODO: ver el nombre correcto con el que llamar a estos parametros para que quede bien

        self.client = actionlib.SimpleActionClient('/rbkairos/move_base', MoveBaseAction)
        #self.pub = rospy.Publisher()
        self.rate = rospy.Rate(1)

        self.goal = MoveBaseGoal()
        self.pose = Pose()

        # no se si es el /robot/cmd_vel o /robot/move_base/cmd_vel
        # o cualquier otro topic de los que tiene en el nombre "cmd_vel"
        # podria ser el de '/robot/robotnik_base_control/cmd_vel' pero unicamente porque tiene publishers y el resto no
        self.pub = rospy.Publisher('/rbkairos/robotnik_base_control/cmd_vel', Twist, queue_size = 1)
        # puede ser que el /robot/move_base/cmd_vel funcoine mejor que el /robot/cmd_vel??
        # aun esta quedando un pequenio punto cuando miramos el odometry en el rviz
        # vamos a probar con el /robot/robotnik_base_control/cmd_vel

        
        
        self.set_object_point(_coord)
        self.aproximation(_radio)
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        #self.move()
        #self.girar(_radio)


    ## @brief Crea una instancia de la clase Point (geometry_msgs/Point) con las coordenadas indicadas.
    ##
    ## @param coord Coordenadas del objeto a desinfectar,
    ## indicadas como una lista de la forma [x, y, z]
    def set_object_point(self, coord):

        self.obj_point = Point(*coord)


    def aproximation(self, _radio): # TODO: en vez de aproximation seria aproach?? pues ni idea de como ponerlo

        # Se define la posicion y orientacion a la que tendra llegar el robot.
        self.pose.position = self.obj_point
        self.pose.position.x -= abs(_radio)
        self.pose.orientation.x = 0
        self.pose.orientation.y = 0
        self.pose.orientation.z = 0
        self.pose.orientation.w = 1

        self.goal.target_pose.pose=self.pose
        self.goal.target_pose.header.frame_id='rbkairos_odom' # TODO: probar con rbkairos/robotnik_base_control/odom

    def girar(self, _radio):
        rospy.loginfo("   girandooo")
        w = 2*2*math.pi/60 # vel angular (rad/s)
        v_l = w * _radio
        msg = Twist()
        msg.linear.y = v_l
        msg.angular.z = -w
        print("w:",w,"\nv_l:", v_l)
        while not self._ctrl_c:
            self.pub.publish(msg)
            self.rate.sleep()

    def move(self):
        label = "esto es una prueba"
        while not self._ctrl_c:
            
            self.client.wait_for_server()
            rospy.loginfo('Going to spot='+str(label))
            self.client.send_goal(self.goal, feedback_cb=self.callback)
            self.client.wait_for_result()
            result = self.client.get_state()

            #print result
            if result==3:
                print('successfuly reached point'+str(label))
                #self.shutdownhook()
                break


    def shutdownhook(self):

        rospy.loginfo("shutdown time!")
        self._ctrl_c = True
        msg = Twist()
        msg.linear.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)

    def callback(self, data):
        return


if __name__ == "__main__":
    rospy.init_node('send_coordinates_node', log_level=rospy.INFO)
    move_obj = CircularMove([11, 0, 0], 1)
    move_obj.move()

    arm_obj = ArmMove()
    # arm_obj.move_joints([-1.7, -1.19, 3.14, -0.3, 1.8, 0.0])
    arm_obj.move_joints([-0.88, -2.07, 3.14, -0.3, 1.8, 0.0])

    # move_ = ArmMove()
    # move_.move_joints()
    move_obj.girar(1)
    
    rospy.spin() # mantain the service open.