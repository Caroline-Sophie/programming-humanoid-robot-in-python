'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''
from joint_control.keyframes import leftBackToStand, leftBellyToStand, wipe_forehead, rightBellyToStand, \
    rightBackToStand
from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])


    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        # Check if 'LHipYawPitch' exists in target_joints before copying to 'RHipYawPitch'
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        # Unpack keyframes data
        names, times, keys = keyframes

        # If there are no keyframes, return empty target joints
        if not times or not keys:
            return target_joints

        # Calculate total duration of the keyframes to normalize perception time

        try:
            total_duration = max(max(t) for t in times)
        except TypeError:
            # If t is a flat list or contains non-iterable elements
            total_duration = max(times)


        # Scale perception time to fit within the range of the keyframes
        if total_duration!=0:
            scaled_time = (perception.time % total_duration)
        else : scaled_time = 0

        # Iterate over each joint
        for i, name in enumerate(names):
            joint_times = times[i]
            joint_keys = keys[i]

            # Loop through time intervals for the joint's motion
            if len(joint_times) == 1:
                target_joints[name] = joint_keys
            else:

                for j in range(len(joint_times) - 1):
                    t0, t1 = joint_times[j], joint_times[j + 1]

                    # Only interpolate if the scaled time is within the keyframe interval
                    if t0 <= scaled_time <= t1:
                        # Define key angles and dynamic Bezier control points for each interval
                        p0 = joint_keys[j][0]
                        p3 = joint_keys[j + 1][0]

                        # Dynamically adjust control points for fuller motion
                        if isinstance(joint_keys[j], list) and len(joint_keys[j]) > 1:
                            _, h1_time, h1_angle = joint_keys[j][1]
                            _, h2_time, h2_angle = joint_keys[j + 1][2]

                            # Set control points to emphasize the movement
                            p1 = p0 + 1.5 * h1_angle
                            p2 = p3 + 1.5 * h2_angle
                        else:
                            # Default control points for smoother but still full motion
                            p1 = p0 + 0.75 * (p3 - p0)
                            p2 = p3 - 0.75 * (p3 - p0)

                        # Calculate normalized time within this interval
                        elapsed_time = scaled_time - t0
                        total_time = t1 - t0
                        normalized_t = elapsed_time / total_time

                        # Apply cubic Bezier interpolation
                        interpolated_angle = (1 - normalized_t) ** 3 * p0 + \
                                             3 * (1 - normalized_t) ** 2 * normalized_t * p1 + \
                                             3 * (1 - normalized_t) * normalized_t ** 2 * p2 + \
                                             normalized_t ** 3 * p3

                        # Set interpolated angle for joint
                        target_joints[name] = interpolated_angle
                        break  # Break since we've found the right interval



        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = wipe_forehead()
    agent.run()
