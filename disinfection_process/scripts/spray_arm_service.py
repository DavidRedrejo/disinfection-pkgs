#! /usr/bin/env python

""" ESTE ES EL QUE TENEMOS QUE EJECUTAR PARA ACTIVAR EL SERVIDOR DEL MOVIMIENTO DEL BRAZO
    CON EL PAQUETE DE MOVEIT QUE TIENE EL SPRAY """
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

class Movimiento ():
    def __init__(self, group = "arm", frame = "robot_map", endeff = "robot_spray_tool_end_link"):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.group.set_end_effector_link("robot_spray_tool_end_link")  # FIXME: cuando se ponga el spray, el end_effector sera: robot_spray_tool_end_link 
                                                                # o el nombre que le pongamos, pero que hay que cambiarlo
        self.group.set_pose_reference_frame(frame)

        # Se indican los valores maximos de velocidad y aceleracion para que el movimiento del brazo sea mas lento y seguro
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        
        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo y la trayectoria
        # TODO: probar a cambiar los valores de queue_size para ver si afecta a la velocidad y lo que tarda en conectarse al publisher (lo del num_conex o algo asi)
        self.pub_tool_goal = rospy.Publisher('/disinfection/tool_goal_pose', PoseStamped, queue_size=1)
        self.pub_object_point = rospy.Publisher('/disinfection/object_point', PointStamped, queue_size=50)
        self.pub_display_trajectory = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)

        # Cuando se termine el nodo se ejecutara la funcion shutdownhook, que por seguridad manda un mensaje de parada al brazo
        rospy.on_shutdown(self.shutdownhook)
        
        rospy.logerr("  TEST.   CONSTRUCTOR FINALIZADO") #FIXME: test


        """ --------------------------------------------------------------- """
        """ self.move_group = MoveGroupInterface(group, frame) # grupo que se mueve, frame -> "sistema" respecto al que se realiza la planificacion """

        """ self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = frame # "rbkairos_odom"
        
        self.end_effector = "rbkairos_spray_tool_end_link" # usabamos el rbkairos_ur10_ee_link """
    
    def shutdownhook(self):
        rospy.loginfo("shutdown!!")
        self.group.stop()

    def finalizar(self):
        moveit_commander.roscpp_shutdown()

    def set_joint_pose_case(self, case):
        # joint_values = [0.0, -2.13, 2.13, 0, 1.57, 3.14]
        # joint_values = [-3.14, -1.09, -2.58, -2.83, -2.26, -3.14]
    
        if case.upper() == "INIT":
            # posicion ligeramente plegada para posterior aproximacion al punto
            # joint_values = [-2.54, -1.07, -2.45, 0.50, -0.30, 0.0]
            joint_values = [1.05, -1.57, -2.45, 0.75, -0.25, 0.0]
        elif case.upper() == "NAVIGATE":
            # posicion completamente plegada mientras navegacion de la base
            #joint_values = [-3.14, -1.26, -2.83, 0.88, 2.15, 0.0]   # PLEGADO NORMAL
            joint_values = [0.0, -1.57, -2.85, 1.32, 1.57, 0.0]   # PLEGADO VERTICAL
            joint_values = [0.88, -1.57, -2.85, 1.32, 1.88, 0.0]   # PLEGADO VERTICAL bueno sin sobresalir spray
            #joint_values = [0.0, -1.1, -2.75, 0.725, 2.0, 0.0]   # PLEGADO VERTICAL_2, sept
            #joint_values = [-3.14, 0.0, -2.86, 0.18, 2.15, 0.0]     # PLEGADO HORIZONTAL
            
        else:
            joint_values = [-0.44, -1.26, -2.51, 0.57, 1.88, 0.0]     # es mas seguro esto que ponerlo todo a 0

        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)

        self.group.set_joint_value_target(joint_values)
        self.group.plan()
        rospy.logerr("    Yendo a la posicion articular indicada")
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
        
        ## Se coloca primero en la posicion de aproximacion
        # self.set_joint_pose_case("INIT")     # se llama a la funcion que coloca el brazo en la posicion articular de inicio
        # 1-10 el INIT de arriba lo hemos comentado porque ya se hace el INIT en el callback del service

        Xob = self.obj_point.x   # x del objeto
        Yob = self.obj_point.y   # y del objeto
        Zob = self.obj_point.z   # z del objeto
        r = self.radius
        
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
        pub_msg.header.frame_id="robot_map"
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.pose = waypoints[0]

        self.publish_(self.pub_tool_goal, pub_msg)

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
        
        # echar un ojo al set_start_state """" de momento no ha hecho falta para nada...

        ## eef_step = pi/2 * radio/ numero_puntos_trayectoria
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.006,        # eef_step
                                        0.0)         # jump_threshold

        # FIXME: esto era por tener informacion sobre las posiciones articulares cuando estabamos mirando que porcentaje de 
        # trayectoria de 90 hacia. porque al principio no la hacia casi nunca por completo.
        rospy.logwarn("   fraccion de trayectoria seguida: {}".format(fraction))
        #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
        #rospy.logwarn(self.group.get_current_joint_values())

        
        # TODO: este podria servir para mostrar solo la planificacion del robot, poniendo un plugin de trayectoria
        # en vez del que tenemos ahora, que seria el plugin para controlarlo todo por asi decirlo, aunque
        # no podemos controlarlo porque no le cargan las librerias, asi que para eso mejor quitarlo y probar esto
        """ display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.pub_display_trajectory.publish(display_trajectory) """

        # Ejecucion de la trayectoria planificada
        self.group.execute(plan, wait=True)

        rospy.loginfo("    [Punto {}] terminado!".format(case_code))
            

    def set_point(self, point):
        # El parametro 'point' es del tipo geometry_msgs/Point
        self.obj_point = point

        # FIXME: como esto es solo para publicar el punto en el RVIZ pues de momento lo comentamos para que no ralentice todo el proceso y ya
        # Si despues queremos mostrar las cosas bien el rn RVIZ pues habra que descomentarlo
        point_msg = PointStamped()
        point_msg.point = point
        point_msg.header.frame_id = "robot_map"

        self.publish_(self.pub_object_point, point_msg)

        """ point_msg = PointStamped()
        point_msg.point = point
        point_msg.header.frame_id = "robot_odom" # antes: rbkairos_odom """

        """ while self.pub_object_point.get_num_connections() < 1:
            rospy.sleep(0.5)
            rospy.logwarn("Esperando conexion para publicar el punto")
        
        point_msg.header.stamp = rospy.Time.now()
        self.pub_object_point.publish(point_msg) """

    def set_radius(self, radius):
        self.radius = radius

    
    def publish_(self, publisher, msg):
        
        while publisher.get_num_connections() < 1:
            rospy.sleep(0.1)
            rospy.logwarn("   Esperando conexion para publicar")
        
        msg.header.stamp = rospy.Time.now()
        rospy.logerr(" antes de publicar")
        publisher.publish(msg)



# FIXME: esto comentado lo dejamos de momento porque TENEMOS QUE VER COMO CONTROLAR QUE SE REALICE BIEN EL MOVIMIENTO 
# Aqui en esta funcion se usaba la comparacion del resultado con el MoveItErrorCodes.SUCCESS, 
# asi que igual luego pdriamos ver algo parecido pero que sea con el moveit_commander no el moveit_python como era aqui


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


def service_callback(request):
    rospy.logdebug("  service callback")
    rospy.logerr("  TEST.   service callback") #FIXME: test
    #move_arm = Movimiento()
    #global move_arm # esto en caso de que no funcione sin el
    move_arm.set_point(request.object_coord)
    move_arm.set_radius(request.radius)
    rospy.logerr("  TEST.   objeto del movimiento creado") #FIXME: test

    response_msg = RBKairosARMResponse()

    if request.motion_code == "NAVIGATE":
        move_arm.set_joint_pose_case("NAVIGATE")
        response_msg.move_successfull = True
        response_msg.message = "El brazo se ha plegado correctamente."

    elif request.motion_code == "INIT":
        move_arm.set_joint_pose_case("INIT")
        response_msg.move_successfull = True
        response_msg.message = "El brazo se ha plegado correctamente."
    
    elif request.motion_code == "mal":
        move_arm.set_joint_pose_case("mal")
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


# --- MAIN ---

rospy.init_node('disinfection_arm_service_server', log_level=rospy.DEBUG)

""" PRUEBA DE PONER EL CONSTRUCTOR FUERA DEL SERVICE_CALLBACK """
#muy posiblemente lo que pase es que no se pueda hacer esto y el move_arm de dentro del callback no se enlace con este
# si al hacerlo solo asi no funciona, probamos a poner lo de 'global move_arm' dentro del callback
move_arm = Movimiento()
rospy.logerr("   Se crea el objeto del brazo")

arm_service = rospy.Service('/disinfection_arm_service', RBKairosARM, service_callback)

#move_arm.set_joint_pose_case("INIT")
rospy.spin() # mantiene el servicio en ejecucion
