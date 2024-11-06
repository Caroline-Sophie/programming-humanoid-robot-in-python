'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from joint_control.keyframes import leftBellyToStand, rightBackToStand, leftBackToStand
from keyframes import hello
import pickle
import numpy as np

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = self.load_classifier('robot_pose.pkl')

    def load_classifier(self, filename):
        with open(filename, 'rb') as file:
            classifier = pickle.load(file)
        return classifier

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # Extract joint angles from perception
        joint_angles = [
            perception.joint['LHipYawPitch'],
            perception.joint['LHipPitch'],
            perception.joint['LHipRoll'],
            perception.joint['LKneePitch'],
            perception.joint['LShoulderPitch'],
            perception.joint['RHipYawPitch'],
            perception.joint['RHipPitch'],
            perception.joint['RHipRoll'],
            perception.imu[0],
            perception.imu[1]
        ]

        # Convert to a format suitable for the classifier (e.g., a numpy array)
        input_data = np.array(joint_angles).reshape(1, -1)

        # Make prediction
        predicted_class = self.posture_classifier.predict(input_data)[0]

        classes = ['HeadBack', 'Stand', 'Left', 'Sit', 'Back', 'StandInit', 'Right', 'Crouch', 'Belly', 'Frog', 'Knee']

        posture = classes[predicted_class]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
