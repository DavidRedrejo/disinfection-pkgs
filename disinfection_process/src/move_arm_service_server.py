#! /usr/bin/env python

""" ESTE ES EL QUE TENEMOS QUE EJECUTAR PARA ACTIVAR EL SERVIDOR DEL MOVIMIENTO DEL BRAZO
    CON EL PAQUETE DE MOVEIT QUE TIENE EL SPRAY """
import rospy

from disinfection_process.srv import RBKairosARM, RBKairosARMResponse
from move_arm_class import MoveArm


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

move_arm = MoveArm()

arm_service = rospy.Service('/disinfection_arm_service', RBKairosARM, service_callback)

rospy.spin() # mantiene el servicio en ejecucion
