'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''
import os
import sys
import weakref
import threading
from ast import literal_eval

import jsonrpclib
import time

from joint_control.keyframes import leftBackToStand, leftBellyToStand, wipe_forehead, rightBellyToStand, \
    rightBackToStand
from numpy.matlib import identity
import agent_server
from threading import Thread


class PostHandler(object):
    '''The post handler wraps functions to be executed in parallel.'''

    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''Non-blocking call to ClientAgent.execute_keyframes'''

        def task():
            self.proxy.execute_keyframes(keyframes)

        # Create and start a new thread for non-blocking execution
        thread = threading.Thread(target=task)
        thread.daemon=True
        thread.start()
        return

    def set_transform(self, effector_name, transform):
        '''Non-blocking call to ClientAgent.set_transform'''

        def task():
            self.proxy.set_transform(effector_name, transform)

        # Create and start a new thread for non-blocking execution
        thread = threading.Thread(target=task)
        thread.daemon=True
        thread.start()
        return



class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    def __init__(self):
        try:
            self.post = PostHandler(self) # create a post handler object
            self.conn = jsonrpclib.Server('http://localhost:8000') # create a connection to the server

            print("Client connected to server.")

        except Exception as e:
            print(f"Failed to connect to server: {e}")
            raise


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.conn.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.conn.set_angle(joint_name, angle)
        return

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.conn.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.conn.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name'''
        # YOUR CODE HERE
        result = self.conn.get_transform(name)
        return result

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        # Convert the numpy array to a nested list for JSON serialization
        # Check if T is already a list; if not, convert it
        if not isinstance(transform, list):
            transform = transform.tolist()
        return self.conn.set_transform(effector_name, transform)



if __name__ == '__main__':
    # Just for testing, so I don't have to use two separate terminals each time
    def start_server():
        def run_server():
            server.start_server(host='localhost', port=8000)
            server.server.serve_forever()

        server = agent_server.ServerAgent()
        server_thread = Thread(target=run_server)
        server_thread.daemon = True
        server_thread.start()
        server.run_agent()

        return server


    def start_interactive_command_loop(agent):
        '''keeps client running and excepts the commands
        example calls from terminal:
        get_angle HeadYaw
        set_angle HeadYaw 45
        execute_keyframes leftBackToStand()
        get_posture
        get_transform HeadYaw
        set_transform LLeg [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0.05, -0.26, 1]]
        '''

        while True:
            command = input("Enter a command (or type 'quit' to exit): ")
            if command == 'quit':
                break
            try:
                parts = command.split(maxsplit=2)
                if parts[0] == 'get_angle' and len(parts) == 2:
                    print(agent.get_angle(parts[1]))

                elif parts[0] == 'set_angle' and len(parts) == 3:
                    joint_name = parts[1]
                    angle = float(parts[2])
                    agent.set_angle(joint_name, angle)
                    print(f"Set angle for {joint_name} to {angle}")

                elif parts[0] == 'get_posture':
                    print(agent.get_posture())

                elif parts[0] == 'get_transform' and len(parts) == 2:
                    print(agent.get_transform(parts[1]))

                elif parts[0] == 'set_transform' and len(parts) == 3:
                    matrix_input = parts[2]
                    try:
                        # Safely evaluate the matrix string input using literal_eval
                        T = literal_eval(matrix_input)
                        agent.post.set_transform(parts[1], T)
                    except (ValueError, SyntaxError) as e:
                        print(f"Invalid matrix input: {e}")

                elif parts[0] == 'execute_keyframes' and len(parts) == 2:
                    print(parts[1])
                    keyframes = eval(parts[1])
                    agent.post.execute_keyframes(keyframes)

                else:
                    print("Unknown command or invalid number of arguments.")
            except Exception as e:
                print(f"Error: {e}")

    # start the server and keep it running
    start_server()
    # Create a client
    agent = ClientAgent()
    # keeps the client running and waits for user inputs
    start_interactive_command_loop(agent)



