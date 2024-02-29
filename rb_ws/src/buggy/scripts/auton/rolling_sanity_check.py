from autonsystem import AutonSystem
import numpy as np

def sanity_check():
    # checks that messages are being receieved
        # (from both buggies if relevant),
        # RTK status is fixed
        # covariance is less than 1 meter
        if (state.self_odom_msg == None) or (state.has_other_buggy and state.other_odom_msg == None) or (state.self_odom_msg.pose.covariance[0] ** 2 + state.self_odom_msg.pose.covariance[7] ** 2 > 1**2):
            return False

        # waits until covariance is acceptable to check heading

        with self.lock:
            self_pose, _ = state.get_world_pose_and_speed(state.self_odom_msg)
            current_heading = self_pose.theta
            closest_heading = state.cur_traj.get_heading_by_index(trajectory.get_closest_index_on_path(self_pose.x, self_pose.y))

        # TENTATIVE:
        # headings are originally between -pi and pi
        # if they are negative, convert them to be between 0 and pi
        if current_heading < 0:
            current_heading = 2*np.pi + current_heading

        if closest_heading < 0:
            closest_heading = 2*np.pi + closest_heading

        if (abs(current_heading - closest_heading) >= np.pi/2):
            print("WARNING: INCORRECT HEADING! restart stack")
            return False

        return True
