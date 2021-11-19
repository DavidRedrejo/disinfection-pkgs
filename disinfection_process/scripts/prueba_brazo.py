#! /usr/bin/env python

import rospy
import numpy as np
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped

#Literal:
PI = np.pi

class Movimiento ():
    def __init__(self, group = "arm", frame = "rbkairos_odom", endeff = "rbkairos_ur10_tool0"): # endeff = robot_arm_tool0 # en la prueba de cambiar el punto usamos esto robot_egh_gripper_base_link
        
        self.my_pub = rospy.Publisher('/my_pose_topic', PoseStamped, queue_size=1)
        self.my_pub2 = rospy.Publisher('/my_point_topic', PointStamped, queue_size=1)
        
        
        self.move_group = MoveGroupInterface(group, frame) # grupo que se mueve, frame -> "sistema" respecto al que se realiza la planificacion
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = "rbkairos_odom" # era frame
        self.end_effector = endeff

        # si usamos el rbkairos_ur10_ee_link como end_effector nos ahorramos giros porque tiene el eje Z hacia arriba
        self.end_effector = "rbkairos_ur10_ee_link"
        rospy.logdebug("   Constructor done")
    
    def set_point(self, coord_punto):
        # lo de pasarlo como array o como x,y,z lo hacemos como queramos
        self.point_array = coord_punto
        self.point = Point(*coord_punto)
        #self.point.x -= 0.2
        self.pose_stamped.pose.position = self.point

        punto_msg = PointStamped()
        punto_msg.point = self.point
        punto_msg.header.frame_id = "rbkairos_odom"
        punto_msg.header.stamp = rospy.Time.now()
        self.my_pub2.publish(punto_msg)


    def aproximacion(self):
        #[x, y, z] = *coord_punto
        #point = Point(*coord_punto)
        #x = coord_punto[0]
        
        x = self.point.x #- 0.2
        rospy.logerr("Valor de x: %f", x)
        if x < 0:
            pitch = np.deg2rad(-90)
            roll = np.deg2rad(180)
        else:
            pitch = np.deg2rad(90)
            roll = np.deg2rad(0)

        quat_aux = tf.transformations.quaternion_from_euler(roll, pitch, 0)
        quat = Quaternion(*quat_aux)
        self._RPY_inicial = [roll, pitch, 0]

        # self.pose_stamped.pose.position = point
        self.pose_stamped.pose.orientation = quat
        # self.pose_stamped.header.stamp = rospy.Time.now() # creo que es mejor que el time se ponga en el de mover
        rospy.logdebug("   Aproximacion lista")
        self.mover()
        #a = tf.transformations.
        #a = tf_conversions.posemath.transformations.compose_matrix(,)
        #a = tf2_ros.
        #a = tf2_geometry_msgs.tf2_geometry_msgs.

    def rodeo(self):
        roll, pitch, _ = self._RPY_inicial

        num_ptos = 3
        incremento = 90/num_ptos
        # antes teniamos puesto de -120,121 y el incremento 240/num
        cont = 1
        for yaw in range(-45, -40, incremento): # se hace 121 porque si no, no hace el de 120
            quat_aux = tf.transformations.quaternion_from_euler(roll, pitch, np.deg2rad(yaw))
            #R=tf.transformations.rotation_matrix(yaw, [0,1,0], self.point_array)
            #quat_aux = tf.transformations.quaternion_from_matrix(R)
            quat = Quaternion(*quat_aux)

            self.pose_stamped.pose.orientation = quat
            self.mover()
            rospy.loginfo("Posicion numero: %d", cont)
            cont += 1 
        rospy.loginfo("Termina el rodeo en z")

            
        
        
        a = 3 # de momento aqui nada, pero tendremos que poner lo que es el movimiento de rodear el objeto
        # al final de esto, tambien tiene que quedarse listo el mensaje del pose stamped
        # self.pose_stamped.pose.position = point
        self.pose_stamped.pose.orientation = quat
        # self.pose_stamped.header.stamp = rospy.Time.now()

    def mover(self):
        rospy.logdebug("   Comienza el de mover")
        while not rospy.is_shutdown():
            self.pose_stamped.header.stamp = rospy.Time.now()

            result = self.move_group.moveToPose(self.pose_stamped, self.end_effector)

            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Trayectoria ejecutada con exito!")
                    break
                else:
                    rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("Fallo de MoveIt! No se ha devuelto resultado.")
        
        self.move_group.get_move_action().cancel_all_goals() # se cancelan todos los objetivos para evitar posibles movimientos no deseado
        rospy.logdebug("   Termina el de mover. Ahora esta quieto")


    def caso_A(self, radio): # obtencion de puntos y orientaciones para el caso A (2 y 3 cuadrantes)
        Xob = self.point_array[0]   # x del objeto
        Yob = self.point_array[1]   # y del objeto
        Zob = self.point_array[2]
        r = radio # lo suyo seria tenerlo como self. 
        
        #self.pose_stp_array = []
        aux_pose_stp = PoseStamped()
        aux_pose_stp.header.frame_id = "rbkairos_odom"

        # valores de x e y sobre la circunferencia
        # x,y = []
        theta_array = np.linspace(PI/2, 3*PI/2, 10) # inicio,final,numero de ptos (el final esta incluido, para no incluirlo se indica endpoint=False)
        for theta in theta_array:
            # x.append(Xob + r * np.cos(theta))
            # y.append(Yob + r * np.sin(theta))

            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            rot_z = theta - PI
            punto = Point(x,y,Zob)
            aux_pose_stp.pose.position = punto
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            quat = Quaternion(*quat_aux)

            aux_pose_stp.pose.orientation = quat

            self.mover_pos(aux_pose_stp)
            rospy.loginfo("    Punto terminado")


    def caso_B(self, radio): # obtencion de puntos y orientaciones para el caso B (1 y 4 cuadrantes)
        Xob = self.point_array[0]   # x del objeto
        Yob = self.point_array[1]   # y del objeto
        Zob = self.point_array[2]   # z del objeto
        r = radio                   # FIXME: lo suyo seria tenerlo como self. 
        
        
        aux_pose_stp = PoseStamped()
        aux_pose_stp.header.frame_id = "rbkairos_odom"

        theta_array_1 = np.linspace(0, PI/2, 5)
        theta_array_2 = np.linspace(3*PI/2, 2*PI, 5)
        theta_array = np.append(theta_array_1, theta_array_2)
        
        for theta in theta_array:
            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            rot_z = theta - PI
            punto = Point(x,y,Zob)
            aux_pose_stp.pose.position = punto
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            quat = Quaternion(*quat_aux)

            aux_pose_stp.pose.orientation = quat

            self.mover_pos(aux_pose_stp)
            rospy.loginfo("    Punto terminado")
            

    def caso_C(self, radio): # obtencion de puntos y orientaciones para el caso C (1 y 2 cuadrantes)
        Xob = self.point_array[0]   # x del objeto
        Yob = self.point_array[1]   # y del objeto
        Zob = self.point_array[2]   # z del objeto
        r = radio                   # FIXME: lo suyo seria tenerlo como self. 
        
        aux_pose_stp = PoseStamped()
        aux_pose_stp.header.frame_id = "rbkairos_odom"

        theta_array = np.linspace(0, PI, 10)

        for theta in theta_array:
            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            rot_z = theta - PI
            punto = Point(x,y,Zob)
            aux_pose_stp.pose.position = punto
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            quat = Quaternion(*quat_aux)

            aux_pose_stp.pose.orientation = quat

            self.mover_pos(aux_pose_stp)
            rospy.loginfo("    Punto terminado")

    def caso_D(self, radio): # obtencion de puntos y orientaciones para el caso D (3 y 4 cuadrantes)
        Xob = self.point_array[0]   # x del objeto
        Yob = self.point_array[1]   # y del objeto
        Zob = self.point_array[2]   # z del objeto
        r = radio                   # FIXME: lo suyo seria tenerlo como self. 
        
        aux_pose_stp = PoseStamped()
        aux_pose_stp.header.frame_id = "rbkairos_odom"

        theta_array = np.linspace(PI, 2*PI, 10)
        
        for theta in theta_array:
            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            rot_z = theta - PI
            punto = Point(x,y,Zob)
            aux_pose_stp.pose.position = punto
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            quat = Quaternion(*quat_aux)

            aux_pose_stp.pose.orientation = quat

            self.mover_pos(aux_pose_stp)
            rospy.loginfo("    Punto terminado")




    def mover_pos(self, pose):
        rospy.logdebug("   Comienza el de mover a 1 posicion")
        while not rospy.is_shutdown():
            pose.header.stamp = rospy.Time.now()

            self.my_pub.publish(pose)

            result = self.move_group.moveToPose(pose, self.end_effector)

            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Trayectoria ejecutada con exito!")
                    break
                else:
                    rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("Fallo de MoveIt! No se ha devuelto resultado.")
        
        self.move_group.get_move_action().cancel_all_goals() # se cancelan todos los objetivos para evitar posibles movimientos no deseado
        rospy.logdebug("   Termina el de mover. Ahora esta quieto")


            














# se inicia el nodo 
rospy.init_node('prueba_brazo', anonymous=True, log_level=rospy.DEBUG)

mov = Movimiento()
mov.set_point([5.0, -0.5, 0.9])
#mov.aproximacion()
rospy.sleep(0.5)
#mov.rodeo()
mov.caso_C(0.2)

rospy.loginfo("Acaba el programa!")
rospy.signal_shutdown("terminado")