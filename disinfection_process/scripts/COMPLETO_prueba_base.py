#! /usr/bin/env python

""" ESTE ES EL QUE TENEMOS QUE EJECUTAR PARA PROBAR LA NAVEGACION HACIA LOS 4 PUNTOS JUNTO CON EL MOVIMIENTO DEL BRAZO"""
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


#Literal:
PI = np.pi

            
class CircularMove(): # TODO: CAMBIAR NOMBRE O ALGO
    # TODO: explicar esto bien:
    # hay dos "radios": el de giro del brazo que rdea al objeto y el de separacion de la base del robot.
    # Pues a la separacion de la base lo vamos a llamar "gap" de hueco

    # TODO:
    ## @brief Aqui va la definicion de la clase esta y de sus parametros.
    ##
    ## @param _coord Coordenadas del objeto a desinfectar,
    ## indicadas como una lista de la forma [x, y, z]
    ## @param _gap Distancia de separacion entre el objeto y la base del robot.
    ## @param _radius Radio de la circunferencia que describe el brazo alrededor del objeto.

    def __init__(self, _coord, _gap, _radius):
        
        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo
        rospy.logerr("Entra en el constructor")
        self.pub_base_goal = rospy.Publisher('/disinfection/base_goal_pose', PoseStamped, queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient('/robot/move_base', MoveBaseAction)
        rospy.logerr("Despues del move_base_client")
        
        # CONEXION CON EL SERVICIO DEL BRAZO
        rospy.wait_for_service('/disinfection_arm_service')
        self.move_arm_client = rospy.ServiceProxy('/disinfection_arm_service', RBKairosARM)
        
        self.move_arm_request = RBKairosARMRequest()
        self.move_arm_request.object_coord = Point(*_coord)
        self.move_arm_request.radius = _radius

        self.goal = MoveBaseGoal()
        self.pose = Pose()


        # FIXME: creo que no hace falta el publisher de la velocidad
        self.pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size = 1)   # CON ESTE FALLA EN EL REINICIO
        #self.pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size = 1)   # PROBAR CON ESTE
        #        # PROBAR A NO PONER NINGUNO y se quitarlo tambien del shutdown
        # FIXME: revisar, pero CREO QUE el robotnik_base_control/cmd_vel es el que hace como un multiplexor con todas las entrads de velocidad del robot, del mando etc
        
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
        # rospy.logerr("justo antes de mandar a la posicon")
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
        #self.approach_points()

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
        self.move_arm_request.motion_code = motion_code
        move_arm_result = self.move_arm_client(self.move_arm_request)
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
        self.execute_move_arm(app_point)
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

    """
    def girar(self, _gap):
        rospy.loginfo("   girandooo")
        w = 2*2*math.pi/60 # vel angular (rad/s)
        v_l = w * _gap
        msg = Twist()
        msg.linear.y = v_l
        msg.angular.z = -w
        print("w:",w,"\nv_l:", v_l)
        while not self._ctrl_c:
            self.pub.publish(msg)
            self.rate.sleep()
    """

    def shutdownhook(self):

        rospy.loginfo("shutdown time!")
        self._ctrl_c = True
        msg = Twist()
        msg.linear.y = 0
        msg.angular.z = 0

        while self.pub.get_num_connections()<1:
            rospy.logwarn("Esperando conexion con publisher de velocidad")
            rospy.sleep(0.5)

        self.pub.publish(msg)
        rospy.logerr("  parado!!")
        
        # TODO: probar con esto al final del programa
        self.move_base_client.cancel_all_goals()

    def callback(self, data):
        print(data)
        return






## MAIN DEL MOVE_BASE_CLASS
if __name__ == "__main__":
    rospy.init_node('prueba_base', log_level=rospy.DEBUG)
    move_obj = CircularMove([1, 0, 1.0], 0.45, 0.2)
    
    # para probar la navegacion a un unico punto
    move_obj.simple_go_to_point([0.5,0,1.0], [0,0,0])

    # para probar los 4 puntos
    # move_obj.start_process()

    #rospy.spin() # mantain the service open.

