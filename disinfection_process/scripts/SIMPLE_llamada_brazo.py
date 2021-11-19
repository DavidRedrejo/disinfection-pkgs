#! /usr/bin/env python

""" ESTE ES EL QUE TENEMOS QUE EJECUTAR PARA PROBAR LA LLAMADA AL SERVICIO DEL BRAZO"""
import rospy
import numpy as np
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, PoseWithCovarianceStamped, Twist


import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal

from disinfection_process.srv import RBKairosARM, RBKairosARMRequest
import sys


#Literal:
PI = np.pi

            
class CircularMove(): # TODO: CAMBIAR NOMBRE O ALGO
    def __init__(self, _coord, _gap, _radius):
        rospy.logerr("   Entra en el constructor")

        # Publishers para visualizar en RVIZ la posicion objetivo de la base
        self.pub_base_goal = rospy.Publisher('/disinfection/base_goal_pose', PoseStamped, queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient('/robot/move_base', MoveBaseAction)
        
        # CONEXION CON EL SERVICIO DEL BRAZO
        rospy.wait_for_service('/disinfection_arm_service')
        self.move_arm_client = rospy.ServiceProxy('/disinfection_arm_service', RBKairosARM)
        
        self.move_arm_request = RBKairosARMRequest()
        self.move_arm_request.object_coord = Point(*_coord)
        self.move_arm_request.radius = _radius

        self.goal = MoveBaseGoal()
        self.pose = Pose()
        
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.approach_points(_coord, _gap)
        self.get_closer_approach_point()

        #self.start_process()

    def simple_go_to_point(self, coord=[0,5,0], ori=[0,0,PI/2]):
        self.pose.position = Point(*coord)
        quat_aux = tf.transformations.quaternion_from_euler(*ori)
        self.pose.orientation = Quaternion(*quat_aux)

        self.goal.target_pose.pose = self.pose
        self.goal.target_pose.header.frame_id = 'robot_map'
        rospy.logerr("  TEST.   justo antes de mandar a la posicion")
        self.navigate_to_pose(self.goal, "C")


    def pose_callback(self, pose_msg):
        self.robot_position = pose_msg.pose.pose # position and orientation
        rospy.logdebug("  in the pose_callback")
        # rospy.loginfo(self.robot_position)


    def get_robot_position(self):   # FIXME:  tal vez tengamos que cambiar "position" por "location" o algo asi
        rospy.logdebug("  in get_robot_position")
        sub_pose = rospy.Subscriber('/robot/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.sleep(1)


    def approach_points(self, obj_coord, gap):
        x_obj, y_obj = obj_coord[0:2] # se usan solo las coordenadas x e y

        self.approach_A = (x_obj + gap, y_obj)
        self.approach_B = (x_obj, y_obj + gap)
        self.approach_C = (x_obj - gap, y_obj)
        self.approach_D = (x_obj, y_obj - gap)
        

    ## @brief Calcula el punto 
    def get_closer_approach_point(self):
        self.get_robot_position()

        x_robot = self.robot_position.position.x
        y_robot = self.robot_position.position.y
        rospy.loginfo("X robot: %f, Y robot: %f", x_robot, y_robot)

        dist_A = np.sqrt( (self.approach_A[0] - x_robot)**2 + (self.approach_A[1] - y_robot)**2 )
        dist_B = np.sqrt( (self.approach_B[0] - x_robot)**2 + (self.approach_B[1] - y_robot)**2 )
        dist_C = np.sqrt( (self.approach_C[0] - x_robot)**2 + (self.approach_C[1] - y_robot)**2 )
        dist_D = np.sqrt( (self.approach_D[0] - x_robot)**2 + (self.approach_D[1] - y_robot)**2 )

        rospy.loginfo("A: %f, B: %f, C: %f, D: %f", dist_A, dist_B, dist_C, dist_D)

        if dist_A == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["A", "B", "C", "D"]
            #self.sequence = ["A", "C"]
            rospy.loginfo("Mejor aproximacion: punto A")

        elif dist_B == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["B", "C", "D", "A"]
            #self.sequence = ["B", "D"]
            rospy.loginfo("Mejor aproximacion: punto B")
            
        elif dist_C == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["C", "D", "A", "B"]
            #self.sequence = ["C", "A"]
            rospy.loginfo("Mejor aproximacion: punto C")

        elif dist_D == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["D", "A", "B", "C"]
            #self.sequence = ["D", "B"]
            rospy.loginfo("Mejor aproximacion: punto D")
    


    def start_process(self):
        
        offset = 0  # -PI/2

        ## Llamada al servicio del brazo para que se pliegue antes de comenzar la navegacion
        self.execute_move_arm("NAVIGATE")

        for app_point in self.sequence:
            if app_point == "A":
                self.pose.position.x = self.approach_A[0]
                self.pose.position.y = self.approach_A[1]
                self.pose.position.z = 0
                quat_aux = tf.transformations.quaternion_from_euler(0,0,PI + offset)
                self.pose.orientation = Quaternion(*quat_aux)

            elif app_point == "B":
                self.pose.position.x = self.approach_B[0]
                self.pose.position.y = self.approach_B[1]
                self.pose.position.z = 0
                quat_aux = tf.transformations.quaternion_from_euler(0,0,3*PI/2 + offset)
                self.pose.orientation = Quaternion(*quat_aux)

            elif app_point == "C":
                self.pose.position.x = self.approach_C[0]
                self.pose.position.y = self.approach_C[1]
                self.pose.position.z = 0                
                quat_aux = tf.transformations.quaternion_from_euler(0,0,0 + offset)
                self.pose.orientation = Quaternion(*quat_aux)

            elif app_point == "D":
                self.pose.position.x = self.approach_D[0]
                self.pose.position.y = self.approach_D[1]
                self.pose.position.z = 0               
                quat_aux = tf.transformations.quaternion_from_euler(0,0,PI/2 + offset)
                self.pose.orientation = Quaternion(*quat_aux)
                    
            self.goal.target_pose.pose = self.pose
            self.goal.target_pose.header.frame_id = 'robot_map'

            self.navigate_to_pose(self.goal, app_point)

    def execute_move_arm(self, motion_code):
        # motion_code = NAVIGATE o el punto que tenga que hacer
        rospy.logerr("  TEST.   dentro de EXECUTE_MOVE_ARM")
        self.move_arm_request.motion_code = motion_code
        #self.move_arm_request.motion_code = "C"

        rospy.logerr("  TEST.   antes de llamar al cliente")
        move_arm_result = self.move_arm_client(self.move_arm_request)
        rospy.logerr("  TEST.   despues de llamar al cliente")
        rospy.loginfo(move_arm_result.message)

        if not move_arm_result.move_successfull:
            rospy.logerr("  No se ha realizado el movimiento del brazo correctamente.")
            return

    def navigate_to_pose(self, goal_pose, app_point):

        self.pub_base_goal.publish(goal_pose.target_pose)

        ## Llamada al servicio del brazo para que se pliegue
        # self.move_arm_request.motion_code = "NAVIGATE"
        # move_arm_result = self.move_arm_client(self.move_arm_request)
        # rospy.loginfo(move_arm_result.message)

        # if not move_arm_result.move_successfull:
        #     rospy.logerr("  No se ha realizado el plegado del brazo correctamente. No se puede ir al siguiente punto.")
        #     return
        

        ## Navegacion hacia el punto indicado
        reached_goal = False
        while not(self._ctrl_c or reached_goal):
                
                self.move_base_client.wait_for_server()
                rospy.loginfo('Navegacion hacia el punto ' + app_point)
                self.move_base_client.send_goal(goal_pose, feedback_cb=self.callback)
                self.move_base_client.wait_for_result()
                result = self.move_base_client.get_state()

                rospy.logerr(result)


                if result==3:
                    rospy.loginfo("  Punto %c alcanzado con exito!", app_point)
                    reached_goal = True
                    #print('Punto', app_point, 'alcanzado con exito!')
                    #break
                rospy.sleep(2)
        
        ## Llamada al servicio del brazo para que haga el movimiento correspondiente al punto
        if not self._ctrl_c:
            rospy.logerr("  TEST.   justo antes de EXECUTE_MOVE_ARM")
            self.execute_move_arm(app_point)
            # self.execute_move_arm("C")
        # self.move_arm_request.motion_code = app_point
        # move_arm_result = self.move_arm_client(self.move_arm_request)
        # rospy.loginfo(move_arm_result.message)

        # if not move_arm_result.move_successfull:
        #     rospy.logerr("  No se ha realizado la trayectoria del brazo correctamente.")
        

    ## @brief Crea una instancia de la clase Point (geometry_msgs/Point) con las coordenadas indicadas.
    ##
    ## @param coord Coordenadas del objeto a desinfectar,
    ## indicadas como una lista de la forma [x, y, z]
    # def set_object_point(self, coord):

    #     self.obj_point = Point(*coord)


    def shutdownhook(self):

        rospy.loginfo("shutdown time!")
        self._ctrl_c = True

        self.move_base_client.cancel_all_goals()
        rospy.logerr("  parado!!")
        
    def callback(self, data):
        #print(data)
        return


## MAIN DEL MOVE_BASE_CLASS
if __name__ == "__main__":
    rospy.init_node('prueba_base', log_level=rospy.DEBUG)
    move_obj = CircularMove([0.8, 0, 1.0], 0.85, 0.25)
    
    # para probar la navegacion a un unico punto
    # move_obj.simple_go_to_point([1,0,1.0], [0,0,0])


    move_obj.execute_move_arm("mal")
    rospy.sleep(1)
    # para probar los 4 puntos
    move_obj.start_process()
    
    """ PRUEBA DE MOVIMIENTO DE BRAZO SOLO """
    #move_obj.execute_move_arm("D")

    """ PRUEBA DE NAVEGACION Y BRAZO x2 """
    # move_obj.simple_go_to_point([0.5,0,1.0], [0,0,0]) # hace navegacion y luego el brazo C
    #move_obj.execute_move_arm("C")

    # move_obj.simple_go_to_point([1.4,0.0,1.0], [0,0,0]) # navegacion y brazo C
    #move_obj.execute_move_arm("C")


