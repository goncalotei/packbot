#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import struct
import sys
import rospy
import subprocess
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import actionlib
import random

class GripperClient(object):
    def __init__(self, gripper):
        ns = 'robot/end_effector/' + gripper + '_gripper/'
        self._client = actionlib.SimpleActionClient(
            ns + "gripper_action",
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()

        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Servidor de ação do grip {} não encontrado. Encerrando...".format(gripper))
            rospy.signal_shutdown("Servidor de ação não encontrado")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal)
        self._client.wait_for_result()

    def clear(self):
        self._goal = GripperCommandGoal()

def read_objects_from_file(filename):
    try:
        with open(filename, 'r') as file:
            for line in file:
                yield line.strip()  
    except FileNotFoundError:
        rospy.logerr("File not found: %s", filename)

def start_gripper_action_server():
    try:
        subprocess.Popen(["rosrun", "baxter_interface", "gripper_action_server.py"])
        rospy.sleep(5)
    except Exception as e:
        rospy.logerr("Falha ao iniciar o servidor de ações do gripper: {}".format(e))
        sys.exit(1)

def move_to_position(limb, joint_angles):
    limb_interface = baxter_interface.Limb(limb)
    print("Moving arm to position: ", joint_angles)
    limb_interface.set_joint_position_speed(0.5)
    limb_interface.move_to_joint_positions(joint_angles)

def get_object_position(object_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = model_coordinates(object_name, 'world') 
        x, y, z = resp.pose.position.x, resp.pose.position.y, resp.pose.position.z
        print("{} position: x={}, y={}, z={}".format(object_name, x, y, z))
        return x, y, z
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None, None, None

def execute_sequence(limb, iksvc, ikreq):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = [
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=0.43751842625508053, y=0.21810565906522353, z=0.0),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=0.43751842625508053, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=0.8, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
    ]

    for pose in poses:
        ikreq.pose_stamp = [pose]
        try:
            rospy.wait_for_service(iksvc.resolved_name, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % e)
            return 1

        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            move_to_position(limb, limb_joints)
        else:
            print("INVALID POSE - No Valid Joint Solution Found. (sequence)")
            return 1

    return 0

def normalization (x_normalized, y_normalized):
    x_min, x_max = 0.73, 1.05
    y_min, y_max = -0.09, 0.12
    
    x = x_normalized * (-x_max + x_min) + x_max
    y = y_normalized * (-y_max + y_min) + y_max
    
    return x, y

def handle_object(limb, iksvc, ikreq, gc, object_name, attach_srv, detach_srv):
    gc.command(position=100.0, effort=50.0)
    x, y, z = get_object_position(object_name)
    if x is None or y is None or z is None:
        rospy.logerr("Posição inválida para o objeto: {}".format(object_name))
        return 1

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pose_object = [
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=x, y=y, z=-0.15),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
    ]

#        PoseStamped(
#            header=hdr,
#            pose=Pose(
#                position=Point(x=x, y=y, z=z - 0.723775 - 0.205),
#                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
#            ),
#        ),
#    ]

#    if z < 0.723775:
#        pose_object.append(
#            PoseStamped(
#                header=hdr,
#                pose=Pose(
#                    position=Point(x=x, y=y, z=-0.205),
#                    orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
#                ),
#            )
#        )

    for pose in pose_object:
        if not move_with_ik(limb, iksvc, ikreq, pose):
            rospy.logerr("Falha ao mover para a posição do objeto {}. Pose inválida.".format(object_name))
            return 1

    rospy.loginfo("Realizando attach do objeto '{}' ao link do robô".format(object_name))
    req = AttachRequest()
    req.model_name_1 = "baxter"
    req.link_name_1 = limb + "_wrist"
    req.model_name_2 = object_name
    req.link_name_2 = "link"

    try:
        attach_srv.call(req)
        rospy.loginfo("Attach realizado com sucesso")
    except rospy.ServiceException as e:
        rospy.logerr("Falha ao realizar attach: {}".format(e))
        return 1
    
    x_normalized, y_normalized = 1.0, 1.0
    x_real, y_real = normalization(x_normalized, y_normalized)

    pose_object = [
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=x, y=y, z=0.1),  
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x_real,  
                    y=y_real,
                    z=0.1,
                ),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x_real,  
                    y=y_real,
                    z=-0.15,
                ),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
    ]

    for pose in pose_object:
        if not move_with_ik(limb, iksvc, ikreq, pose):
            rospy.logerr("Falha ao mover para a posição do objeto {}. Pose inválida.".format(object_name))
            return 1

    gc.command(position=0.0, effort=50.0)
    rospy.loginfo("Realizando detach do objeto '{}' do link do robô".format(object_name))
    detach_req = AttachRequest()
    detach_req.model_name_1 = "baxter"
    detach_req.link_name_1 = limb + "_wrist"
    detach_req.model_name_2 = object_name
    detach_req.link_name_2 = "link" 

    try:
        detach_srv.call(detach_req)  
        rospy.loginfo("Detach realizado com sucesso")
    except rospy.ServiceException as e:
        rospy.logerr("Falha ao realizar detach: {}".format(e))
        return 1

    rospy.sleep(1) 

    pose_object = [
        PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.8,  
                    y=0.0,
                    z=-0.0,
                ),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
            ),
        ),
    ]

    for pose in pose_object:
        if not move_with_ik(limb, iksvc, ikreq, pose):
            rospy.logerr("Falha ao mover para a posição do objeto {}. Pose inválida.".format(object_name))
            return 1

    return 0


def move_with_ik(limb, iksvc, ikreq, pose):
    """
    Auxiliar para calcular e mover o braço para uma pose usando IK.
    """
    ikreq.pose_stamp = [pose]
    try:
        rospy.wait_for_service(iksvc.resolved_name, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Falha na chamada ao serviço IK: {}".format(str(e)))
        return False

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if resp_seeds[0] != resp.RESULT_INVALID:
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        move_to_position(limb, limb_joints)
        return True
    else:
        rospy.logwarn("POSE INVÁLIDA - Sem solução válida encontrada para a posição: x={}, y={}, z={}".format(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ))
        return False
    
def initialize_attach_service():
    rospy.loginfo("Criando ServiceProxy para /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("ServiceProxy para /link_attacher_node/attach criado")
    return attach_srv

def initialize_detach_service():
    rospy.loginfo("Criando ServiceProxy para /link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("ServiceProxy para /link_attacher_node/detach criado")
    return detach_srv

def main():
    parser = argparse.ArgumentParser(description="Baxter RSDK Inverse Kinematics Example")
    parser.add_argument('-l', '--limb', choices=['left', 'right'], required=True, help="the limb to test")
    start_gripper_action_server()
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + args.limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    gc = GripperClient(args.limb)

    attach_srv = initialize_attach_service()
    detach_srv = initialize_detach_service()
    gc.command(position=100.0, effort=50.0)
    rospy.sleep(1)
    execute_sequence(args.limb, iksvc, ikreq)

    for object_name in read_objects_from_file('/home/packbot/Desktop/objects.txt'):
        handle_object(args.limb, iksvc, ikreq, gc, object_name, attach_srv, detach_srv)

    return 0

if __name__ == '__main__':
    sys.exit(main())
