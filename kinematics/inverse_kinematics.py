'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''
import numpy as np

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE


        if effector_name not in ['LLeg', 'RLeg']:
            raise ValueError("Effector name must be 'LLeg' or 'RLeg'")

        # Extract position from the transform matrix
        px, py, pz = transform[0, 3], transform[1, 3], transform[2, 3]

        # NAO leg parameters in millimeters, convert to meters
        l1 = 100.00 / 1000  # Distance from hip to knee in meters
        l2 = 102.90 / 1000  # Distance from knee to ankle in meters

        # Solving for joint angles step-by-step
        # Example: Solve for knee pitch using the law of cosines
        d = np.sqrt(px**2 + py**2 + (pz + l1)**2)  # Effective length
        if d > (l1 + l2):
            raise ValueError("Target position is out of reach")

        # Knee pitch (using the law of cosines)
        knee_pitch = np.arccos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))

        # Hip pitch (using geometric decomposition)
        alpha = np.arctan2(pz + l1, np.sqrt(px**2 + py**2))
        beta = np.arctan2(l2 * np.sin(knee_pitch), l1 + l2 * np.cos(knee_pitch))
        hip_pitch = alpha - beta

        # Hip yaw (based on x, y projection)
        hip_yaw = np.arctan2(py, px)
        hip_roll = 0  # For simplicity, assume no lateral deviation

        # Ankle pitch and roll (set to match foot orientation)
        ankle_pitch = -hip_pitch + knee_pitch
        ankle_roll = 0  # Assume aligned foot for simplicity

        # Create the dictionary of joint angles
        if effector_name == 'LLeg':
            joint_angles = {
                'LHipYawPitch': hip_yaw,
                'LHipRoll': hip_roll,
                'LHipPitch': hip_pitch,
                'LKneePitch': knee_pitch,
                'LAnklePitch': ankle_pitch,
                'RAnkleRoll': ankle_roll  # Keep the naming convention consistent with NAO's model
            }
        elif effector_name == 'RLeg':
            joint_angles = {
                'RHipYawPitch': hip_yaw,
                'RHipRoll': hip_roll,
                'RHipPitch': hip_pitch,
                'RKneePitch': knee_pitch,
                'RAnklePitch': ankle_pitch,
                'LAnkleRoll': ankle_roll  # Keep the naming convention consistent with NAO's model
            }
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        names=[]
        times=[]
        keys=[]
        for name, key in joint_angles.items():
            names.append(name)
            keys.append(key)
            times.append([0])

        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
