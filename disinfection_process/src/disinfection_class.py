#! /usr/bin/env python

""" CLASE DEL PROCESO DE DESINFECCION. CONTIENE LAS FUNCIONES
    PARA REALIZAR TODOS LOS PASOS DEL PROCESO """

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

            
class DisinfectionProcess():
    def __init__(self, _coord, _gap, _radius):

        # Publisher para visualizar en RVIZ la posicion objetivo de la base
        self.pub_base_goal = rospy.Publisher('/disinfection/base_goal_pose', PoseStamped, queue_size=1)

        # Cliente para mover la base con la navegacion autonoma
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
        # Funcion para probar el movimiento a un unico punto

        self.pose.position = Point(*coord)
        quat_aux = tf.transformations.quaternion_from_euler(*ori)
        self.pose.orientation = Quaternion(*quat_aux)

        self.goal.target_pose.pose = self.pose
        self.goal.target_pose.header.frame_id = 'robot_map'
        self.navigate_to_pose(self.goal, "C")


    def pose_callback(self, pose_msg):
        self.robot_position = pose_msg.pose.pose # posicion y orientacion
        rospy.logdebug("  in the pose_callback")
        # rospy.loginfo(self.robot_position)


    def get_robot_position(self):
        rospy.logdebug("  in get_robot_position")
        sub_pose = rospy.Subscriber('/robot/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.sleep(1)


    def approach_points(self, obj_coord, gap):
        # Calculo de los puntos de aproximacion

        x_obj, y_obj = obj_coord[0:2] # se usan solo las coordenadas x e y

        self.approach_A = (x_obj + gap, y_obj)
        self.approach_B = (x_obj, y_obj + gap)
        self.approach_C = (x_obj - gap, y_obj)
        self.approach_D = (x_obj, y_obj - gap)
        

    def get_closer_approach_point(self):
        # Calculo del punto de aproximacion mas cercano al robot

        self.get_robot_position()   # obtiene la posicion actual del robot

        x_robot = self.robot_position.position.x
        y_robot = self.robot_position.position.y
        rospy.loginfo("X robot: %f, Y robot: %f", x_robot, y_robot)

        # Calculo de la distancia euclidea
        dist_A = np.sqrt( (self.approach_A[0] - x_robot)**2 + (self.approach_A[1] - y_robot)**2 )
        dist_B = np.sqrt( (self.approach_B[0] - x_robot)**2 + (self.approach_B[1] - y_robot)**2 )
        dist_C = np.sqrt( (self.approach_C[0] - x_robot)**2 + (self.approach_C[1] - y_robot)**2 )
        dist_D = np.sqrt( (self.approach_D[0] - x_robot)**2 + (self.approach_D[1] - y_robot)**2 )

        rospy.loginfo("A: %f, B: %f, C: %f, D: %f", dist_A, dist_B, dist_C, dist_D)

        # Creacion de la secuencia de puntos segun el mas cercano
        if dist_A == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["A", "B", "C", "D"]
            rospy.loginfo("Mejor aproximacion: punto A")

        elif dist_B == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["B", "C", "D", "A"]
            rospy.loginfo("Mejor aproximacion: punto B")
            
        elif dist_C == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["C", "D", "A", "B"]
            rospy.loginfo("Mejor aproximacion: punto C")

        elif dist_D == min(dist_A, dist_B, dist_C, dist_D):
            self.sequence = ["D", "A", "B", "C"]
            rospy.loginfo("Mejor aproximacion: punto D")
    


    def start_process(self):
        # Comienza el proceso, siguiendo la secuencia calculada antes
        
        offset = 0  # offset para la orientacion de la posicion de la base

        ## Llamada al servicio del brazo para que se pliegue antes de comenzar la navegacion
        self.execute_move_arm("NAVIGATE")

        # Recorrido de la secuencia de puntos
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
        # Ejecuta el movimiento del brazo indicado en 'motion_code' haciendo una llamada al servicio del brazo

        self.move_arm_request.motion_code = motion_code

        move_arm_result = self.move_arm_client(self.move_arm_request)

        rospy.loginfo(move_arm_result.message)

        if not move_arm_result.move_successfull:
            rospy.logerr("  No se ha realizado el movimiento del brazo correctamente.")
            return

    def navigate_to_pose(self, goal_pose, app_point):
        # Ejecuta la navegacion autonoma a la posicion indicada en 'goal_pose'.
        # 'app_point' es la letra del punto de aproximacion

        self.pub_base_goal.publish(goal_pose.target_pose)   # se publica para visualizar en RVIZ el punto objetivo de la navegacion

        ## Navegacion hacia el punto indicado
        reached_goal = False
        while not(self._ctrl_c or reached_goal):
                
                self.move_base_client.wait_for_server()
                rospy.loginfo('Navegacion hacia el punto ' + app_point)
                self.move_base_client.send_goal(goal_pose, feedback_cb=self.callback)
                self.move_base_client.wait_for_result()
                result = self.move_base_client.get_state()

                if result==3:
                    rospy.loginfo("  Punto %c alcanzado con exito!", app_point)
                    reached_goal = True
                rospy.sleep(2)
        
        ## Llamada al servicio del brazo para que haga el movimiento correspondiente al punto
        if not self._ctrl_c:
            rospy.logerr("  TEST.   justo antes de EXECUTE_MOVE_ARM")
            self.execute_move_arm(app_point)


    def shutdownhook(self):
        rospy.loginfo("shutdown time!")
        self._ctrl_c = True
        self.move_base_client.cancel_all_goals()
        
    def callback(self, data):
        #print(data)
        return


## MAIN
if __name__ == "__main__":
    rospy.init_node('prueba_base', log_level=rospy.DEBUG)
    move_obj = DisinfectionProcess([0.8, 0, 1.0], 0.85, 0.25)
    
    # para probar la navegacion a un unico punto
    # move_obj.simple_go_to_point([1,0,1.0], [0,0,0])

    # para realizar los 4 puntos
    move_obj.start_process()
    