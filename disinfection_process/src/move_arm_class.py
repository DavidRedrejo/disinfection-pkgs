#! /usr/bin/env python

""" CALCULA LOS PUNTOS DE LA TRAYECTORIA DEL BRAZO
    Y USA LAS FUNCIONES DE MOVEIT PARA MOVERLO SEGUN CORRESPONDA """

import rospy
import numpy as np
import tf

import moveit_commander
import sys
import copy

from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from disinfection_process.srv import RBKairosARM, RBKairosARMResponse

#Literal:
PI = np.pi

class MoveArm ():
    def __init__(self, group = "arm", frame = "robot_map", endeff = "robot_spray_tool_end_link"):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.group.set_end_effector_link("robot_spray_tool_end_link")
        self.group.set_pose_reference_frame(frame)

        # Se indican los valores maximos de velocidad y aceleracion para que el movimiento del brazo sea mas lento y seguro
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        
        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo y la trayectoria
        self.pub_tool_goal = rospy.Publisher('/disinfection/tool_goal_pose', PoseStamped, queue_size=1)
        self.pub_object_point = rospy.Publisher('/disinfection/object_point', PointStamped, queue_size=50)
        self.pub_display_trajectory = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

        # Cuando se termine el nodo se ejecutara la funcion shutdownhook, que por seguridad manda un mensaje de parada al brazo
        rospy.on_shutdown(self.shutdownhook)

    
    def shutdownhook(self):
        rospy.loginfo("shutdown!!")
        self.group.stop()


    def set_joint_pose_case(self, case):
    
        if case.upper() == "INIT":
            # posicion para posterior aproximacion al punto
            joint_values = [1.05, -1.57, -2.45, 0.75, -0.25, 0.0]

        elif case.upper() == "NAVIGATE":
            # posicion completamente plegada mientras navegacion de la base
            joint_values = [0.88, -1.57, -2.85, 1.32, 1.88, 0.0]
            
        else:
            joint_values = [-0.44, -1.26, -2.51, 0.57, 1.88, 0.0]  # es mas seguro esto que ponerlo todo a 0


        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)

        self.group.set_joint_value_target(joint_values)
        self.group.plan()
        rospy.loginfo("    Moviendo a la posicion articular indicada")
        self.group.go(wait=True)


    def get_theta_array(self, case_code):
        # Calcula el vector con los valores del angulo theta para 
        # todos los puntos del tramo correspondiente al 'case_code'

        poses_number = 50   # numero de puntos

        if case_code == "A":
            theta_array = np.linspace(-PI/2, 0, poses_number)

        elif case_code == "B":
            theta_array = np.linspace(0, PI/2, poses_number)

        elif case_code == "C":
            theta_array = np.linspace(PI/2, PI, poses_number)

        elif case_code == "D":
            theta_array = np.linspace(PI, 3*PI/2, poses_number)
        
        else:
            rospy.logerr("No existe el caso con el codigo: {}".format(case_code))
            theta_array = None

        return theta_array


    def move_case(self, case_code): 
        # Obtencion de puntos y orientaciones que rodean el objeto, segun el caso indicado por 'case_code'

        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius
        
        waypoints = []  # almacena todos los puntos que forman la trayectoria
        wpose = Pose()

        theta_array = self.get_theta_array(case_code)
        
        for theta in theta_array:
            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            wpose.position = Point(x,y,Zob)

            rot_z = theta - PI
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            wpose.orientation = Quaternion(*quat_aux)

            waypoints.append(copy.deepcopy(wpose))

        pub_msg = PoseStamped()
        pub_msg.header.frame_id="robot_map"
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.pose = waypoints[0]

        self.publish_(self.pub_tool_goal, pub_msg)  # publica para visualizar en RVIZ la primera posicion  

        # Se tiene que ejecutar la primera trayectoria que pone el pulverizador en el primer waypoint.
        self.group.set_pose_target(waypoints[0])  
        self.group.plan()
        self.group.go(wait=True)
        
        # Planificacion de la trayectoria completa con todos los puntos indicados
        ## eef_step = pi/2 * radio/ numero_puntos_trayectoria
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.006,        # eef_step
                                        0.0)         # jump_threshold

        # fraccion de la trayectoria seguida (1: completa, 0: no la sigue)
        rospy.loginfo("   fraccion de trayectoria seguida: {}".format(fraction))
        #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
        #rospy.logwarn(self.group.get_current_joint_values())

        # Ejecucion de la trayectoria planificada
        self.group.execute(plan, wait=True)

        rospy.loginfo("    [Punto {}] terminado!".format(case_code))
            

    def set_point(self, point):
        # Indica el punto del objeto como atributo de la clase y publica para visualizar en RVIZ.
        # El parametro 'point' es del tipo geometry_msgs/Point
        self.obj_point = point

        point_msg = PointStamped()
        point_msg.point = point
        point_msg.header.frame_id = "robot_map"

        self.publish_(self.pub_object_point, point_msg)


    def set_radius(self, radius):
        self.radius = radius

    
    def publish_(self, publisher, msg):
        # Esta funcion asegura que se publiquen los mensajes 
        # en el topico indicado. Hasta que no exista conexion no publica
        
        while publisher.get_num_connections() < 1:
            rospy.sleep(0.1)
            rospy.logwarn("   Esperando conexion para publicar")
        
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)

