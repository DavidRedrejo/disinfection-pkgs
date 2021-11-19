#! /usr/bin/env python

import rospy
import numpy as np
import tf

from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from disinfection_process.srv import RBKairosARM, RBKairosARMResponse


#Literal:
PI = np.pi

class Movimiento ():
    def __init__(self, group = "arm", frame = "rbkairos_odom", endeff = "rbkairos_ur10_tool0"):

        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo
        self.pub_tool_goal = rospy.Publisher('/disinfection/tool_goal_pose', PoseStamped, queue_size=1)
        self.pub_object_point = rospy.Publisher('/disinfection/object_point', PointStamped, queue_size=1)
        
        self.move_group = MoveGroupInterface(group, frame) # grupo que se mueve, frame -> "sistema" respecto al que se realiza la planificacion

        self.move_group.setPlannerId("RRTstar")

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = frame # "rbkairos_odom"
        
        self.end_effector = "rbkairos_spray_tool_end_link" # usabamos el rbkairos_ur10_ee_link
    
    def set_point(self, point):
        # El parametro 'point' es del tipo geometry_msgs/Point
        self.obj_point = point

        punto_msg = PointStamped()
        punto_msg.point = point
        punto_msg.header.frame_id = "rbkairos_odom"
        punto_msg.header.stamp = rospy.Time.now()
        self.pub_object_point.publish(punto_msg)

    def set_radius(self, radius):
        self.radius = radius

    """ def aproximacion(self):
        #[x, y, z] = *coord_punto
        #point = Point(*coord_punto)
        #x = coord_punto[0]
        
        x = 0.0 #- 0.2
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

        # self.goal_pose.pose.position = point
        self.goal_pose.pose.orientation = quat
        # self.goal_pose.header.stamp = rospy.Time.now() # creo que es mejor que el time se ponga en el de mover
        rospy.logdebug("   Aproximacion lista")
        self.mover() """


    """ def rodeo(self):
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

            self.goal_pose.pose.orientation = quat
            self.mover()
            rospy.loginfo("Posicion numero: %d", cont)
            cont += 1 
        rospy.loginfo("Termina el rodeo en z")

        a = 3 # de momento aqui nada, pero tendremos que poner lo que es el movimiento de rodear el objeto
        # al final de esto, tambien tiene que quedarse listo el mensaje del pose stamped
        # self.goal_pose.pose.position = point
        self.goal_pose.pose.orientation = quat
        # self.goal_pose.header.stamp = rospy.Time.now() """

    """ def mover(self):
        rospy.logdebug("   Comienza el de mover")
        while not rospy.is_shutdown():
            self.goal_pose.header.stamp = rospy.Time.now()

            result = self.move_group.moveToPose(self.goal_pose, self.end_effector)

            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Trayectoria ejecutada con exito!")
                    break
                else:
                    rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("Fallo de MoveIt! No se ha devuelto resultado.")
        
        self.move_group.get_move_action().cancel_all_goals() # se cancelan todos los objetivos para evitar posibles movimientos no deseado
        rospy.logdebug("   Termina el de mover. Ahora esta quieto") """


    def get_theta_array(self, case_code):

        poses_number = 5

        if case_code == "A":
            """ theta_array_1 = np.linspace(0, PI/2, poses_number//2.0)
            theta_array_2 = np.linspace(3*PI/2, 2*PI, poses_number//2.0)
            theta_array = np.append(theta_array_1, theta_array_2) """
            theta_array = np.linspace(-PI/2, PI/2, poses_number)

        elif case_code == "B":
            theta_array = np.linspace(0, PI, poses_number)

        elif case_code == "C":
            theta_array = np.linspace(PI/2, 3*PI/2, poses_number)

        elif case_code == "D":
            theta_array = np.linspace(PI, 2*PI, poses_number)
        
        else:
            rospy.logerr("No existe el caso con el codigo: {}".format(case_code))
            theta_array = 0

        return theta_array


    def move_case(self, case_code): # obtencion de puntos y orientaciones que rodean el objeto, segun el caso indicado por 'case_code'
        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius
        
        theta_array = self.get_theta_array(case_code)
        
        for theta in theta_array:
            x = Xob + r * np.cos(theta)
            y = Yob + r * np.sin(theta)
            self.goal_pose.pose.position = Point(x,y,Zob)
            
            rot_z = theta - PI
            quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
            self.goal_pose.pose.orientation = Quaternion(*quat_aux)

            self.move_to_pose(self.goal_pose)
            
        rospy.loginfo("    [Punto {}] terminado!".format(case_code))
            



    #def move_case_A(self, radio): # obtencion de puntos y orientaciones para el caso A (1 y 4 cuadrantes)
    """ def move_case_A(self): # obtencion de puntos y orientaciones para el caso A (1 y 4 cuadrantes)
        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius
        
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

            self.move_to_pose(aux_pose_stp)
            rospy.loginfo("    Punto terminado")
            

    #def move_case_B(self, radio): # obtencion de puntos y orientaciones para el caso C (1 y 2 cuadrantes)
    def move_case_B(self): # obtencion de puntos y orientaciones para el caso C (1 y 2 cuadrantes)
        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius
        
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

            self.move_to_pose(aux_pose_stp)
            rospy.loginfo("    Punto terminado")


    #def move_case_C(self, radio): # obtencion de puntos y orientaciones para el caso A (2 y 3 cuadrantes)
    def move_case_C(self): # obtencion de puntos y orientaciones para el caso A (2 y 3 cuadrantes)
        Xob = self.obj_point.x  # x del objeto
        Yob = self.obj_point.y  # y del objeto
        Zob = self.obj_point.z  # z del objeto
        r = self.radius  
        
        aux_pose_stp = PoseStamped()
        aux_pose_stp.header.frame_id = "rbkairos_odom"

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

            self.move_to_pose(aux_pose_stp)
            rospy.loginfo("    Punto terminado")


    #def move_case_D(self, radio): # obtencion de puntos y orientaciones para el caso D (3 y 4 cuadrantes)
    def move_case_D(self): # obtencion de puntos y orientaciones para el caso D (3 y 4 cuadrantes)
        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius 
        
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

            self.move_to_pose(aux_pose_stp)
            rospy.loginfo("    Punto terminado") """


    def fold_joints(self):
        joints = ["rbkairos_ur10_elbow_joint", "rbkairos_ur10_shoulder_lift_joint", "rbkairos_ur10_shoulder_pan_joint",
                  "rbkairos_ur10_wrist_1_joint", "rbkairos_ur10_wrist_2_joint", "rbkairos_ur10_wrist_3_joint"]
        fold_pose = [-2.50, -1.05, 3.14, 0.5, 1.8, 0.0]

        while not rospy.is_shutdown():
            # se recoge el resultado de mover las articulaciones a los valores indicados,
            # con una tolerancia de 0.02
            result = self.move_group.moveToJointPosition(joints, fold_pose, 0.02)
            
            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Trayectoria ejecutada con exito!")
                    break
                else:
                    rospy.logerr("Resultado no exitoso. Estado del brazo: %s",
                                    self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("Fallo en MoveIt! No se ha devuelto ningun resultado.")

        # se cancelan todos los objetivos al finalizar para que se detenga por completo
        self.move_group.get_move_action().cancel_all_goals()





    def move_to_pose(self, pose):
        rospy.logdebug("   Comienza el de mover a 1 posicion")
        while not rospy.is_shutdown():
            pose.header.stamp = rospy.Time.now()

            self.pub_tool_goal.publish(pose)

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


""" def service_callback(request):
    rospy.loginfo("  Callback service")
    move_arm = Movimiento()
    move_arm.set_point(request.object_coord)
    #r = request.radius
    move_arm.set_radius(request.radius)

    response_msg = RBKairosARMResponse()

    if request.motion_code == "A":
        #move_arm.move_case_A(r)
        move_arm.move_case_A()
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento 'A' correctamente."

    elif request.motion_code == "B":
        #move_arm.move_case_B(r)
        move_arm.move_case_B()
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento 'B' correctamente."

    elif request.motion_code == "C":
        #move_arm.move_case_C(r)
        move_arm.move_case_C()
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento 'C' correctamente."

    elif request.motion_code == "D":
        #move_arm.move_case_D(r)
        move_arm.move_case_D()
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento 'D' correctamente."

    elif request.motion_code == "fold":
        move_arm.fold_joints()
        response_msg.move_successfull = True
        response_msg.message = "El brazo se ha plegado correctamente."

    else:
        rospy.logerr("No se ha introducido un codigo valido [motion_code].")
        rospy.logwarn("[motion_code] valores posibles: 'A', 'B', 'C', 'D', 'fold'.")
        response_msg.move_successfull = False
        response_msg.message = "No se ha introducido un codigo valido [motion_code]."

    return response_msg

 """


def service_callback(request):
    rospy.loginfo("  Callback service")
    move_arm = Movimiento()
    move_arm.set_point(request.object_coord)
    move_arm.set_radius(request.radius)

    response_msg = RBKairosARMResponse()

    if request.motion_code == "fold":
        move_arm.fold_joints()
        response_msg.move_successfull = True
        response_msg.message = "El brazo se ha plegado correctamente."
    
    elif request.motion_code in ["A", "B", "C", "D"]:
        move_arm.move_case(request.motion_code)
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento" + request.motion_code + "correctamente."

    else:
        rospy.logerr("No se ha introducido un codigo valido [motion_code].")
        rospy.logwarn("[motion_code] valores posibles: 'A', 'B', 'C', 'D', 'fold'.")
        response_msg.move_successfull = False
        response_msg.message = "No se ha introducido un codigo valido [motion_code]."

    return response_msg





rospy.init_node('arm_service_server', log_level=rospy.DEBUG)
arm_service = rospy.Service('/disinfection_arm_service', RBKairosARM, service_callback)
rospy.spin() # mantiene el servicio en ejecucion








""" 

# se inicia el nodo 
rospy.init_node('prueba_brazo', anonymous=True, log_level=rospy.DEBUG)

mov = Movimiento()
mov.set_point([5.0, -0.5, 0.9])
#mov.aproximacion()
rospy.sleep(0.5)
#mov.rodeo()
mov.move_case_C(0.2)

rospy.loginfo("Acaba el programa!")
rospy.signal_shutdown("terminado") """