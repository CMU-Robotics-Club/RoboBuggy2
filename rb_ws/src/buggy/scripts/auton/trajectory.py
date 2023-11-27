import numpy as np
import json
from scipy.interpolate import Akima1DInterpolator, CubicSpline

from world import World


class Trajectory:
    """A wrapper around a trajectory JSON file that does some under-the-hood math. Will
    interpolate the data points and calculate other information such as distance along the trajectory
    and instantaneous curvature.

    Currently, the trajectory is assumed to be a straight line between each waypoint. This is
    not a good assumption, but it's good enough for now. This means that the curvature is zero
    anywhere between waypoints.

    This class also has a method that will find the closest point on the trajectory to a given
    point in the world. This is useful for finding where the buggy is on the trajectory.

    Use https://rdudhagra.github.io/eracer-portal/ to make trajectories and save the JSON file
    """

    def __init__(self, json_filepath=None, positions=[], interpolator="CubicSpline") -> None:
        """
        Args:
            json_filepath (String): file path to the path json file (begins at /rb_ws)
            positions [[float, float]]: reference trajectory
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        self.distances = np.zeros((0, 1))  # (N/dt x 1) [d, d, ...]
        self.positions = np.zeros((0, 2))  # (N x 2) [(x,y), (x,y), ...]
        self.indices = None  # (N x 1) [0, 1, 2, ...]
        self.interpolation = None  # scipy.interpolate.PPoly

        if not json_filepath is None:
            pos = []
            # Load the json file
            with open(json_filepath, "r") as f:
                data = json.load(f)

            # Iterate through the waypoints and extract the positions
            num_waypoints = len(data)
            for i in range(0, num_waypoints):

                waypoint = data[i]

                lat = waypoint["lat"]
                lon = waypoint["lon"]

                # Convert to world coordinates
                x, y = World.gps_to_world(lat, lon)
                pos.append([x, y])
        
            num_indices = len(pos)
        else:
            pos = positions

        
        if interpolator == "Akima":
            self.positions = np.array(pos)
            self.indices = np.arange(num_indices)
            self.interpolation = Akima1DInterpolator(self.indices, self.positions)
        else:
            temp_traj = Trajectory(json_filepath, interpolator="Akima")
            tot_len = temp_traj.distances[-1]
            interp_dists = np.linspace(0, tot_len, num_indices)

            self.indices = np.arange(num_indices)
            self.positions = [
                temp_traj.get_position_by_distance(interp_dist)
                for interp_dist in interp_dists
            ]
            self.positions = np.array(self.positions)

            self.interpolation = CubicSpline(self.indices, self.positions)
        self.interpolation.extrapolate = True

        # Calculate the distances along the trajectory
        dt = 0.01
        ts = np.arange(len(self.positions) - 1, step=dt)
        drdt = self.interpolation(
            ts, nu=1
        )  # Calculate derivatives of polynomial wrt indices
        ds = np.sqrt(drdt[:, 0] ** 2 + drdt[:, 1] ** 2) * dt
        s = np.cumsum(np.hstack([[0], ds[:-1]]))
        self.distances = s
        self.dt = dt

    def get_num_points(self):
        """Gets the number of points along the trajectory

        Returns:
            int: number of points
        """
        return len(self.positions)

    def get_heading_by_index(self, index):
        """Gets the heading at given index along trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: (theta) in rads
        """
        # theta = np.interp(index, self.indices, self.positions[:, 2])
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        theta = np.arctan2(dydt, dxdt)

        return theta

    def get_heading_by_distance(self, distance):
        """Gets the heading at given distance along trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            float: (theta) in rads
        """
        index = self.get_index_from_distance(distance)
        return self.get_heading_by_index(index)

    def get_position_by_index(self, index):
        """Gets the position at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position

        return self.interpolation(index)

    def get_position_by_distance(self, distance):
        """Gets the position at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position
        index = self.get_index_from_distance(distance)
        return self.get_position_by_index(index)

    def get_steering_angle_by_index(self, index, wheelbase):
        """Gets the bicycle-model steering angle at a given distance along the trajectory,
        interpolating if necessary. Assumes that the origin point of the buggy is at the
        center of the rear axle.

        Args:
            index (float): index along the trajectory
            wheelbase (float): wheelbase of the buggy in meters

        Returns:
            float: steering angle in rads
        """
        curvature = self.get_curvature_by_index(index)
        return np.arctan(wheelbase * curvature)

    def get_steering_angle_by_distance(self, distance, wheelbase):
        """Gets the bicycle-model steering angle at a given distance along the trajectory,
        interpolating if necessary. Assumes that the origin point of the buggy is at the
        center of the rear axle.

        Args:
            distance (float): distance along the trajectory in meters
            wheelbase (float): wheelbase of the buggy in meters

        Returns:
            float: steering angle in rads
        """
        index = self.get_index_from_distance(distance)
        return self.get_steering_angle_by_index(index, wheelbase)

    def get_index_from_distance(self, distance):
        """Gets the index at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            int: index along the trajectory
        """
        # Interpolate the index
        index = np.interp(
            distance,
            self.distances,
            np.linspace(0, len(self.distances), len(self.distances)),
        )

        return index * self.dt

    def get_distance_from_index(self, index):
        """Gets the distance at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: distance along the trajectory in meters
        """
        # Interpolate the distance
        distance = np.interp(
            index / self.dt, np.arange(len(self.distances)), self.distances
        )

        return distance

    def get_curvature_by_index(self, index):
        """Gets the curvature at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (float): index along the trajectory

        Returns:
            float: curvature
        """
        # Interpolate the curvature
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        ddxdtt, ddydtt = self.interpolation(index, nu=2).reshape((-1, 2)).T

        curvature = (dxdt * ddydtt - dydt * ddxdtt) / (
            (dxdt**2 + dydt**2) ** (3 / 2)
        )

        return curvature

    def get_curvature_by_distance(self, distance):
        """Gets the curvature at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            float: curvature
        """
        # Interpolate the curvature
        index = self.get_index_from_distance(distance)

        return self.get_curvature_by_index(index)

    def get_dynamics_by_index(self, index, wheelbase):
        """Gets the dynamics at a given index along the trajectory,
        interpolating if necessary. Convenience function that returns
        all of the dynamics at once rather than requiring multiple
        calls to other helper functions. In this way, we can reuse calls
        to the interpolator, improving performance.

        Args:
            index (float): index along the trajectory

        Returns:
            tuple: (x, y, theta, steering_angle)
        """
        # Interpolate the dynamics
        x, y = self.interpolation(index).reshape((-1, 2)).T
        dxdt, dydt = self.interpolation(index, nu=1).reshape((-1, 2)).T
        ddxdtt, ddydtt = self.interpolation(index, nu=2).reshape((-1, 2)).T

        curvature = (dxdt * ddydtt - dydt * ddxdtt) / (
            (dxdt**2 + dydt**2) ** (3 / 2)
        )
        theta = np.arctan2(dydt, dxdt)

        return np.stack((x, y, theta, np.arctan(wheelbase * curvature)), axis=-1)

    def get_closest_index_on_path(
        self, x, y, start_index=0, end_index=None, subsample_resolution=10000
    ):
        """Gets the index of the closest point on the trajectory to the given point

        Args:
            x (float): x coordinate
            y (float): y coordinate
            start_index (int, optional): index to start searching from. Defaults to 0.
            end_index (int, optional): index to end searching at. Defaults to None (disable).

        Returns:
            int: index along the trajectory
        """
        # If end_index is not specified, use the length of the trajectory
        if end_index is None:
            end_index = len(self.positions)

        # Floor/ceil the start/end indices
        start_index = int(np.floor(start_index))
        end_index = int(np.ceil(end_index))

        # Calculate the distance from the point to each point on the trajectory
        distances = (self.positions[start_index : end_index + 1, 0] - x) ** 2 + (
            self.positions[start_index : end_index + 1, 1] - y
        ) ** 2

        min_ind = np.argmin(distances) + start_index

        start_index = max(0, min_ind - 1)
        end_index = min(len(self.positions), min_ind + 1)

        # Now interpolate at a higher resolution to get a more accurate result
        r_interp = self.interpolation(
            np.linspace(start_index, end_index, subsample_resolution + 1)
        )
        x_interp, y_interp = r_interp[:, 0], r_interp[:, 1]

        distances = (x_interp - x) ** 2 + (y_interp - y) ** 2

        # Return the index of the closest point
        return (
            np.argmin(distances) / subsample_resolution * (end_index - start_index)
            + start_index
        )


if __name__ == "__main__":
    # Example usage
    trajectory = Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json")

    import json
    import uuid

    interp_dat = []
    for k in np.linspace(0, trajectory.indices[-1], 500):
        x, y = trajectory.get_position_by_index(k)
        lat, lon = World.world_to_gps(x, y)

        interp_dat.append(
            {"lat": lat, "lon": lon, "key": str(uuid.uuid4()), "active": False}
        )

    import matplotlib.pyplot as plt

    ts = np.linspace(0, trajectory.indices[-1], 500)
    kurv = [trajectory.get_curvature_by_index(t) for t in ts]
    plt.plot(ts, kurv)
    plt.show()

    with open("/rb_ws/src/buggy/paths/traj_spline_interp.json", "w") as f:
        json.dump(interp_dat, f, indent=4)

    # knot_point_distances = np.arange(0, 20, 1)
    # reference_trajectory = np.hstack(
    #     [
    #         (
    #             *trajectory.get_position_by_distance(d),
    #             trajectory.get_heading_by_distance(d),
    #             trajectory.get_steering_angle_by_distance(d, 1.3),
    #         )
    #         for d in knot_point_distances
    #     ]
    # )
    # print(reference_trajectory)
