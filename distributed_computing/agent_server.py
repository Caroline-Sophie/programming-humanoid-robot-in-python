'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import time
from threading import Thread

import numpy as np
from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

import jsonrpclib
import simplejson


class ServerAgent(InverseKinematicsAgent):

    '''ServerAgent provides RPC service'''

    def start_server(self, host='localhost', port=8000):
        self.server = SimpleJSONRPCServer((host, port))
        self.register_functions()

    def run_agent(self):
        agent_thread = Thread(target=self.run)
        agent_thread.daemon = True
        agent_thread.start()

        return

    def register_functions(self):
        '''Register functions to be called remotely'''
        self.server.register_function(self.get_angle, 'get_angle')
        self.server.register_function(self.set_angle, 'set_angle')
        self.server.register_function(self.get_posture, 'get_posture')
        self.server.register_function(self.execute_keyframes, 'execute_keyframes')
        self.server.register_function(self.get_transform, 'get_transform')
        self.server.register_function(self.set_transform, 'set_transform')


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.target_joints[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        return

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        # Unpack keyframes data
        names, times, keys = keyframes

        # Calculate total duration of the keyframes to normalize perception time
        try: total_duration = max(max(t) for t in times)
        except TypeError: total_duration = max(times) # if it's a flat list

        time.sleep(total_duration + 1)

        #self.keyframes = ([], [], []) only use when key frame is supposed to be executed once

        return


    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        result = self.transforms.get(name)
        return result.tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        transform = np.array(transform)
        self.set_transforms(effector_name, transform)
        return

if __name__ == '__main__':
    agent = ServerAgent()
    agent.start_server()
    agent.run()

