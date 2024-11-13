'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
import numpy as np
from math import cos, sin
from numpy.matlib import matrix, identity
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from recognize_posture import PostureRecognitionAgent




class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'LAnkleRoll'],
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''


        # Define joint-specific parameters (example values, replace with actual robot-specific parameters)
        joint_params = {
            'HeadYaw': {'axis': 'z', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0.12650 },
            'HeadPitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'LShoulderPitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0.098, 'offset_z': 0.100 },
            'LShoulderRoll': {'axis': 'z', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'LElbowYaw': {'axis': 'x', 'offset_x': 0.105, 'offset_y': 0.015, 'offset_z': 0 },
            'LElbowRoll': {'axis': 'z', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'LHipYawPitch': {'axis': 'z', 'offset_x': 0, 'offset_y': 0.050, 'offset_z': -0.085 },
            'LHipRoll': {'axis': 'x', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'LHipPitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'LKneePitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': -0.100 },
            'LAnklePitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': -0.10290 },
            'RAnkleRoll': {'axis': 'x', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'RShoulderPitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0.098, 'offset_z': 0.100 },
            'RShoulderRoll': {'axis': 'z', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'RElbowYaw': {'axis': 'x', 'offset_x': 0.105, 'offset_y': 0.015, 'offset_z': 0 },
            'RElbowRoll': {'axis': 'z', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'RHipYawPitch': {'axis': 'z', 'offset_x': 0, 'offset_y': 0.050, 'offset_z': -0.085 },
            'RHipRoll': {'axis': 'x', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'RHipPitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },
            'RKneePitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': -0.100 },
            'RAnklePitch': {'axis': 'y', 'offset_x': 0, 'offset_y': 0, 'offset_z': -0.10290},
            'LAnkleRoll': {'axis': 'x', 'offset_x': 0, 'offset_y': 0, 'offset_z': 0 },

        }

        params = joint_params.get(joint_name)
        if params is None:
            raise ValueError(f"Joint '{joint_name}' parameters are not defined.")

        axis = params['axis']
        x = params['offset_x']
        y = params['offset_y']
        z = params['offset_z']


        # Create rotation and translation matrices
        if axis == 'z':
            R = np.array([
                [cos(joint_angle), -sin(joint_angle), 0, 0],
                [sin(joint_angle), cos(joint_angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        elif axis == 'y':
            R = np.array([
                [cos(joint_angle), 0, sin(joint_angle), 0],
                [0, 1, 0, 0],
                [-sin(joint_angle), 0, cos(joint_angle), 0],
                [0, 0, 0, 1]
            ])
        elif axis == 'x':
            R = np.array([
                [1, 0, 0, 0],
                [0, cos(joint_angle), -sin(joint_angle), 0],
                [0, sin(joint_angle), cos(joint_angle), 0],
                [0, 0, 0, 1]
            ])

        # Translation matrix
        T_translation = np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
        ])

        # Combine rotation and translation
        T = T_translation @ R


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T = T @ Tl  # Update cumulative transformation matrix

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
