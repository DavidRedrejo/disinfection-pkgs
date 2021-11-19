#! /usr/bin/env python

""" ESTE ES EL QUE TENEMOS QUE EJECUTAR PARA PROBAR EL MOVIMIENTO DEL BRAZO """
""" solo se hacen movimientos referidos a la base del robot para ver si funciona el servicio """
import rospy
import numpy as np
import tf

#from moveit_python import MoveGroupInterface
import moveit_commander
import sys
import copy

from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from disinfection_process.srv import RBKairosARM, RBKairosARMResponse

# Esto es lo mismo que el codgo que teniamos en el prueba_bazo_service pero haciendo el movimiento con moveit_commander en vez de moveit_python
# Esto se ha hecho para poder hacer la planificacion con todos los waypoints pra ver si asi no hace saltos.


#Literal:
PI = np.pi

class Movimiento ():
    def __init__(self, group = "manipulator", frame = "robot_base_footprint", endeff = "rbkairos_ur10_tool0"):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.group.set_end_effector_link("robot_ur10_ee_link")    # creo que no haria falta porque en moveit ya lo hemos puesto como el ultimo link
        self.group.set_pose_reference_frame("robot_base_footprint") # TODO: luego cuando hagamos el final esto tiene que ser robot_map

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        
        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo y la trayectoria
        # TODO: probar a cambiar los valores de queue_size para ver si afecta a la velocidad y lo que tarda en conectarse al publisher (lo del num_conex o algo asi)
        self.pub_tool_goal = rospy.Publisher('/disinfection/tool_goal_pose', PoseStamped, queue_size=1)
        self.pub_object_point = rospy.Publisher('/disinfection/object_point', PointStamped, queue_size=50)
        self.pub_display_trajectory = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

        #self.set_waypoints(pub_display_trajectory)

        rospy.logerr("  TEST.   CONSTRUCTOR FINALIZADO") #FIXME: test        

        
        # self.set_joint_pose_case()
        # self.move_case("C")

        """ self.move_group = MoveGroupInterface(group, frame) # grupo que se mueve, frame -> "sistema" respecto al que se realiza la planificacion """

        """ self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = frame # "rbkairos_odom"
        
        self.end_effector = "rbkairos_spray_tool_end_link" # usabamos el rbkairos_ur10_ee_link """
    
    def finalizar(self):
        moveit_commander.roscpp_shutdown()

    """ def set_waypoints(self, pub_display_trajectory):
        waypoints = []
        scale = 5
        wpose = self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.pub_display_trajectory.publish(display_trajectory)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        #return plan, fraction
        self.group.execute(plan) """

    def set_joint_pose_case(self, case):
        # joint_values = [0.0, -2.13, 2.13, 0, 1.57, 3.14]
        # joint_values = [-3.14, -1.09, -2.58, -2.83, -2.26, -3.14]
    
        if case.upper() == "INIT":
            # posicion ligeramente plegada para posterior aproximacion al punto
            # joint_values = [-2.54, -1.07, -2.45, 0.50, -0.30, 0.0]
            joint_values = [0.70, -1.57, -2.45, 0.75, -0.25, 0.0]
        elif case.upper() == "NAVIGATE":
            # posicion completamente plegada mientras navegacion de la base
            #joint_values = [-3.14, -1.26, -2.83, 0.88, 2.15, 0.0]   # PLEGADO NORMAL
            joint_values = [0.0, -1.57, -2.85, 1.32, 1.57, 0.0]   # PLEGADO VERTICAL
            #joint_values = [0.0, -1.1, -2.75, 0.725, 2.0, 0.0]   # PLEGADO VERTICAL_2, sept
            #joint_values = [-3.14, 0.0, -2.86, 0.18, 2.15, 0.0]     # PLEGADO HORIZONTAL
            
        else:
            joint_values = [0.0, -1.57, -2.85, 1.32, 1.57, 0.0]

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)

        self.group.set_joint_value_target(joint_values)
        self.group.plan()
        rospy.logerr("    Yendo a la posicion inicial/navegacion")
        self.group.go(wait=True)


    def get_theta_array(self, case_code):

        poses_number = 50

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
            theta_array = 0

        return theta_array


    def move_case(self, case_code): # obtencion de puntos y orientaciones que rodean el objeto, segun el caso indicado por 'case_code'
        
        """ if case_code == "NAVIGATE":
            self.set_joint_pose_case(case_code)
            rospy.logerr("    POSICION DE NAVEGACION TERMINADA")
            return """
        

        ## Para cualquiera de las posiciones A, B, C, D
        # self.set_joint_pose_case("INIT")     # se llama a la funcion que coloca el brazo en la posicion articular de inicio
        
        """ Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius """
        
        # LO QUE HA DICHO ANDRES: aproximar primera posicion con posiciones articulares conocidas 
        Xob = 0.8   # x del objeto
        Yob = 0.0   # y del objeto
        Zob = 1.0   # z del objeto
        r = 0.15
        
        waypoints = []
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
        pub_msg.header.frame_id="robot_base_footprint" # pondria el rbkairos_odom, pero con base_footprint esta funcionando bien
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.pose = waypoints[0]

        """ while self.pub_tool_goal.get_num_connections() < 1:
            rospy.sleep(0.5)
            rospy.logerr("Esperando publisher del tool_goal")

        self.pub_tool_goal.publish(pub_msg) """  

        # Se tiene que ejecutar la primera trayectoria que pone el EE en el primer waypoint.
        # TODO: se ha comprobado que al poner un planningn time con lo de la siguiente linea lo que pasa es que tarda T0DO ese tiempo, no que lo tenga de maximo
        # que era lo que yo pensaba que pasaba. Pero no... Asi que mejor quitarlo y ya
        #self.group.set_planning_time(20)    # se aumenta el tiempo de planificacion a 20 segundos
        self.group.set_pose_target(waypoints[0])  
        self.group.plan()
        self.group.go(wait=True)
        
        # FIXME: Este sleep lo teniamos para 'asegurarnos' de que al comenzar el arco de 90 el robot estuviese ya si o si en la primera posicion.
        # Pero hemos visto que parece no hacer falta asi que se quita y ya no espera entre la primera pos y el resto de puntos.
        #rospy.sleep(10)
        
        # echar un ojo al set_start_state """"

        ## eef_step = pi/2 * radio/ numero_puntos_trayectoria
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.006,        # eef_step
                                        0.0)         # jump_threshold

        # FIXME: esto era por tener informacion sobre las posiciones articulares cuando estabamos mirando que porcentaje de 
        # trayectoria de 90 hacia. porque al principio no la hacia casi nunca por completo.
        #rospy.logerr("fraction: {}".format(fraction))
        #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
        #rospy.logwarn(self.group.get_current_joint_values())

        """ display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.pub_display_trajectory.publish(display_trajectory) """

        # Ejecucion de la trayectoria planificada
        self.group.execute(plan, wait=True)

        # FIXME: same que antes. por tener info sobre las articulaciones cuando si hacia bien la trayectoria y queriamos verlo
        """ rospy.logwarn("Posicon actual (FINAL) de las articualciones:")
        rospy.logwarn(self.group.get_current_joint_values()) """


        rospy.loginfo("    [Punto {}] terminado!".format(case_code))
            

    def set_point(self, point):
        # El parametro 'point' es del tipo geometry_msgs/Point
        self.obj_point = point

        # FIXME: como esto es solo para publicar el punto en el RVIZ pues de momento lo comentamos para que no ralentice todo el proceso y ya
        # Si despues queremos mostrar las cosas bien el rn RVIZ pues habra que descomentarlo
        """ point_msg = PointStamped()
        point_msg.point = point
        point_msg.header.frame_id = "rbkairos_odom"

        while self.pub_object_point.get_num_connections() < 1:
            rospy.sleep(0.5)
            rospy.logwarn("Esperando conexion para publicar el punto")
        
        point_msg.header.stamp = rospy.Time.now()
        self.pub_object_point.publish(point_msg) """

    def set_radius(self, radius):
        self.radius = radius

#     """ def aproximacion(self):
#         #[x, y, z] = *coord_punto
#         #point = Point(*coord_punto)
#         #x = coord_punto[0]
        
#         x = 0.0 #- 0.2
#         rospy.logerr("Valor de x: %f", x)
#         if x < 0:
#             pitch = np.deg2rad(-90)
#             roll = np.deg2rad(180)
#         else:
#             pitch = np.deg2rad(90)
#             roll = np.deg2rad(0)

#         quat_aux = tf.transformations.quaternion_from_euler(roll, pitch, 0)
#         quat = Quaternion(*quat_aux)
#         self._RPY_inicial = [roll, pitch, 0]

#         # self.goal_pose.pose.position = point
#         self.goal_pose.pose.orientation = quat
#         # self.goal_pose.header.stamp = rospy.Time.now() # creo que es mejor que el time se ponga en el de mover
#         rospy.logdebug("   Aproximacion lista")
#         self.mover() """


#     """ def rodeo(self):
#         roll, pitch, _ = self._RPY_inicial

#         num_ptos = 3
#         incremento = 90/num_ptos
#         # antes teniamos puesto de -120,121 y el incremento 240/num
#         cont = 1
#         for yaw in range(-45, -40, incremento): # se hace 121 porque si no, no hace el de 120
#             quat_aux = tf.transformations.quaternion_from_euler(roll, pitch, np.deg2rad(yaw))
#             #R=tf.transformations.rotation_matrix(yaw, [0,1,0], self.point_array)
#             #quat_aux = tf.transformations.quaternion_from_matrix(R)
#             quat = Quaternion(*quat_aux)

#             self.goal_pose.pose.orientation = quat
#             self.mover()
#             rospy.loginfo("Posicion numero: %d", cont)
#             cont += 1 
#         rospy.loginfo("Termina el rodeo en z")

#         a = 3 # de momento aqui nada, pero tendremos que poner lo que es el movimiento de rodear el objeto
#         # al final de esto, tambien tiene que quedarse listo el mensaje del pose stamped
#         # self.goal_pose.pose.position = point
#         self.goal_pose.pose.orientation = quat
#         # self.goal_pose.header.stamp = rospy.Time.now() """

#     """ def mover(self):
#         rospy.logdebug("   Comienza el de mover")
#         while not rospy.is_shutdown():
#             self.goal_pose.header.stamp = rospy.Time.now()

#             result = self.move_group.moveToPose(self.goal_pose, self.end_effector)

#             if result:
#                 if result.error_code.val == MoveItErrorCodes.SUCCESS:
#                     rospy.loginfo("Trayectoria ejecutada con exito!")
#                     break
#                 else:
#                     rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
#             else:
#                 rospy.logerr("Fallo de MoveIt! No se ha devuelto resultado.")
        
#         self.move_group.get_move_action().cancel_all_goals() # se cancelan todos los objetivos para evitar posibles movimientos no deseado
#         rospy.logdebug("   Termina el de mover. Ahora esta quieto") """






#     #def move_case_A(self, radio): # obtencion de puntos y orientaciones para el caso A (1 y 4 cuadrantes)
#     """ def move_case_A(self): # obtencion de puntos y orientaciones para el caso A (1 y 4 cuadrantes)
#         Xob = self.obj_point.x   # x del objeto
#         Yob = self.obj_point.y   # y del objeto
#         Zob = self.obj_point.z   # z del objeto
#         r = self.radius
        
#         aux_pose_stp = PoseStamped()
#         aux_pose_stp.header.frame_id = "rbkairos_odom"

#         theta_array_1 = np.linspace(0, PI/2, 5)
#         theta_array_2 = np.linspace(3*PI/2, 2*PI, 5)
#         theta_array = np.append(theta_array_1, theta_array_2)
        
#         for theta in theta_array:
#             x = Xob + r * np.cos(theta)
#             y = Yob + r * np.sin(theta)
#             rot_z = theta - PI
#             punto = Point(x,y,Zob)
#             aux_pose_stp.pose.position = punto
#             quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
#             quat = Quaternion(*quat_aux)

#             aux_pose_stp.pose.orientation = quat

#             self.move_to_pose(aux_pose_stp)
#             rospy.loginfo("    Punto terminado")
            

#     #def move_case_B(self, radio): # obtencion de puntos y orientaciones para el caso C (1 y 2 cuadrantes)
#     def move_case_B(self): # obtencion de puntos y orientaciones para el caso C (1 y 2 cuadrantes)
#         Xob = self.obj_point.x   # x del objeto
#         Yob = self.obj_point.y   # y del objeto
#         Zob = self.obj_point.z   # z del objeto
#         r = self.radius
        
#         aux_pose_stp = PoseStamped()
#         aux_pose_stp.header.frame_id = "rbkairos_odom"

#         theta_array = np.linspace(0, PI, 10)

#         for theta in theta_array:
#             x = Xob + r * np.cos(theta)
#             y = Yob + r * np.sin(theta)
#             rot_z = theta - PI
#             punto = Point(x,y,Zob)
#             aux_pose_stp.pose.position = punto
#             quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
#             quat = Quaternion(*quat_aux)

#             aux_pose_stp.pose.orientation = quat

#             self.move_to_pose(aux_pose_stp)
#             rospy.loginfo("    Punto terminado")


#     #def move_case_C(self, radio): # obtencion de puntos y orientaciones para el caso A (2 y 3 cuadrantes)
#     def move_case_C(self): # obtencion de puntos y orientaciones para el caso A (2 y 3 cuadrantes)
#         Xob = self.obj_point.x  # x del objeto
#         Yob = self.obj_point.y  # y del objeto
#         Zob = self.obj_point.z  # z del objeto
#         r = self.radius  
        
#         aux_pose_stp = PoseStamped()
#         aux_pose_stp.header.frame_id = "rbkairos_odom"

#         theta_array = np.linspace(PI/2, 3*PI/2, 10) # inicio,final,numero de ptos (el final esta incluido, para no incluirlo se indica endpoint=False)

#         for theta in theta_array:
#             # x.append(Xob + r * np.cos(theta))
#             # y.append(Yob + r * np.sin(theta))

#             x = Xob + r * np.cos(theta)
#             y = Yob + r * np.sin(theta)
#             rot_z = theta - PI
#             punto = Point(x,y,Zob)
#             aux_pose_stp.pose.position = punto
#             quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
#             quat = Quaternion(*quat_aux)

#             aux_pose_stp.pose.orientation = quat

#             self.move_to_pose(aux_pose_stp)
#             rospy.loginfo("    Punto terminado")


#     #def move_case_D(self, radio): # obtencion de puntos y orientaciones para el caso D (3 y 4 cuadrantes)
#     def move_case_D(self): # obtencion de puntos y orientaciones para el caso D (3 y 4 cuadrantes)
#         Xob = self.obj_point.x   # x del objeto
#         Yob = self.obj_point.y   # y del objeto
#         Zob = self.obj_point.z   # z del objeto
#         r = self.radius 
        
#         aux_pose_stp = PoseStamped()
#         aux_pose_stp.header.frame_id = "rbkairos_odom"

#         theta_array = np.linspace(PI, 2*PI, 10)
        
#         for theta in theta_array:
#             x = Xob + r * np.cos(theta)
#             y = Yob + r * np.sin(theta)
#             rot_z = theta - PI
#             punto = Point(x,y,Zob)
#             aux_pose_stp.pose.position = punto
#             quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
#             quat = Quaternion(*quat_aux)

#             aux_pose_stp.pose.orientation = quat

#             self.move_to_pose(aux_pose_stp)
#             rospy.loginfo("    Punto terminado") """


#     def fold_joints(self):
#         joints = ["rbkairos_ur10_elbow_joint", "rbkairos_ur10_shoulder_lift_joint", "rbkairos_ur10_shoulder_pan_joint",
#                   "rbkairos_ur10_wrist_1_joint", "rbkairos_ur10_wrist_2_joint", "rbkairos_ur10_wrist_3_joint"]
#         fold_pose = [2.14, -2.13, 0.0, 0.0, 1.57, 0.0]

#         while not rospy.is_shutdown():
#             # se recoge el resultado de mover las articulaciones a los valores indicados,
#             # con una tolerancia de 0.02
#             result = self.move_group.moveToJointPosition(joints, fold_pose, 0.02)
            
#             if result:
#                 if result.error_code.val == MoveItErrorCodes.SUCCESS:
#                     rospy.loginfo("Trayectoria ejecutada con exito!")
#                     break
#                 else:
#                     rospy.logerr("Resultado no exitoso. Estado del brazo: %s",
#                                     self.move_group.get_move_action().get_state())
#             else:
#                 rospy.logerr("Fallo en MoveIt! No se ha devuelto ningun resultado.")

#         # se cancelan todos los objetivos al finalizar para que se detenga por completo
#         self.move_group.get_move_action().cancel_all_goals()





#     def move_to_pose(self, pose):
#         rospy.logdebug("   Comienza el de mover a 1 posicion")
#         while not rospy.is_shutdown():
#             pose.header.stamp = rospy.Time.now()

#             self.pub_tool_goal.publish(pose)

#             result = self.move_group.moveToPose(pose, self.end_effector)

#             if result:
#                 if result.error_code.val == MoveItErrorCodes.SUCCESS:
#                     rospy.loginfo("Trayectoria ejecutada con exito!")
#                     break
#                 else:
#                     rospy.logerr("Arm goal in state: %s", self.move_group.get_move_action().get_state())
#             else:
#                 rospy.logerr("Fallo de MoveIt! No se ha devuelto resultado.")
        
#         self.move_group.get_move_action().cancel_all_goals() # se cancelan todos los objetivos para evitar posibles movimientos no deseado
#         rospy.logdebug("   Termina el de mover. Ahora esta quieto")


# """ def service_callback(request):
#     rospy.loginfo("  Callback service")
#     move_arm = Movimiento()
#     move_arm.set_point(request.object_coord)
#     #r = request.radius
#     move_arm.set_radius(request.radius)

#     response_msg = RBKairosARMResponse()

#     if request.motion_code == "A":
#         #move_arm.move_case_A(r)
#         move_arm.move_case_A()
#         response_msg.move_successfull = True
#         response_msg.message = "Se ha realizado el movimiento 'A' correctamente."

#     elif request.motion_code == "B":
#         #move_arm.move_case_B(r)
#         move_arm.move_case_B()
#         response_msg.move_successfull = True
#         response_msg.message = "Se ha realizado el movimiento 'B' correctamente."

#     elif request.motion_code == "C":
#         #move_arm.move_case_C(r)
#         move_arm.move_case_C()
#         response_msg.move_successfull = True
#         response_msg.message = "Se ha realizado el movimiento 'C' correctamente."

#     elif request.motion_code == "D":
#         #move_arm.move_case_D(r)
#         move_arm.move_case_D()
#         response_msg.move_successfull = True
#         response_msg.message = "Se ha realizado el movimiento 'D' correctamente."

#     elif request.motion_code == "fold":
#         move_arm.fold_joints()
#         response_msg.move_successfull = True
#         response_msg.message = "El brazo se ha plegado correctamente."

#     else:
#         rospy.logerr("No se ha introducido un codigo valido [motion_code].")
#         rospy.logwarn("[motion_code] valores posibles: 'A', 'B', 'C', 'D', 'fold'.")
#         response_msg.move_successfull = False
#         response_msg.message = "No se ha introducido un codigo valido [motion_code]."

#     return response_msg

#  """


def service_callback(request):
    rospy.logdebug("  service callback")
    rospy.logerr("  TEST.   service callback") #FIXME: test
    move_arm = Movimiento()
    move_arm.set_point(request.object_coord)
    move_arm.set_radius(request.radius)
    rospy.logerr("  TEST.   objeto del movimiento creado") #FIXME: test

    response_msg = RBKairosARMResponse()

    if request.motion_code == "NAVIGATE":
        move_arm.set_joint_pose_case("NAVIGATE")
        response_msg.move_successfull = True
        response_msg.message = "El brazo se ha plegado correctamente."
    
    elif request.motion_code in ["A", "B", "C", "D"]:
        move_arm.set_joint_pose_case("INIT")
        move_arm.move_case(request.motion_code)
        move_arm.set_joint_pose_case("NAVIGATE")
        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento " + request.motion_code + " correctamente."

    else:
        rospy.logerr("No se ha introducido un codigo valido [motion_code].")
        rospy.logwarn("[motion_code] valores posibles: 'A', 'B', 'C', 'D', 'fold'.")
        response_msg.move_successfull = False
        response_msg.message = "No se ha introducido un codigo valido [motion_code]."
    
    #move_arm.finalizar()
    return response_msg





rospy.init_node('PRUEBA_arm_service_server_COMMANDER', log_level=rospy.ERROR)
arm_service = rospy.Service('/disinfection_arm_service', RBKairosARM, service_callback)

""" rospy.logerr("   Se crea el objeto del brazo")
move_arm = Movimiento()
move_arm.set_joint_pose_case("INIT") # INIT, NAVIGATE o CUALQUIER OTRA

move_arm.move_case("C")

move_arm.set_joint_pose_case("NAVIGATE")
rospy.logerr("  Termina todo") """
rospy.spin() # mantiene el servicio en ejecucion



# rospy.init_node('prueba_brazo', anonymous=True, log_level=rospy.DEBUG)

# move = Movimiento()


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