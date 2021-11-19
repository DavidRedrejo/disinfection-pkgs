#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

# se inicia el nodo 
rospy.init_node('initial_ur10_pose', anonymous=True)

move_group = MoveGroupInterface("arm", "rbkairos_base_link")

# se indica el nombre de las articulaciones y el valor que se le quiere dar
joints = ["rbkairos_ur10_elbow_joint", "rbkairos_ur10_shoulder_lift_joint", "rbkairos_ur10_shoulder_pan_joint",
                  "rbkairos_ur10_wrist_1_joint", "rbkairos_ur10_wrist_2_joint", "rbkairos_ur10_wrist_3_joint"]
pose = [-2.50, -1.05, 3.14, 0.5, 1.8, 0.0]

rospy.loginfo("Comenzando posicinamiento inicial del brazo")
rospy.sleep(1)

while not rospy.is_shutdown():

    # se recoge el resultado de mover las articulaciones a la posicion indicada,
    # con una tolerancia de 0.02
    result = move_group.moveToJointPosition(joints, pose, 0.02)
    
    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Trayectoria ejecutada con exito!")
            break
        else:
            rospy.logerr("Resultado no exitoso. Estado del brazo: %s",
                            move_group.get_move_action().get_state())
    else:
        rospy.logerr("Fallo en MoveIt! No se ha devuelto ningun resultado.")

# se cancelan todos los objetivos al finalizar para que se detenga por completo
move_group.get_move_action().cancel_all_goals()

rospy.loginfo("Acaba el programa!")
rospy.signal_shutdown("terminado")