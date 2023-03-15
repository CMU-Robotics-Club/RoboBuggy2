import numpy as np
import json

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

    distances = np.zeros((0, 1))  # (N x 1) [d, d, ...]
    positions = np.zeros((0, 3))  # (N x 2) [(x,y,th), (x,y,th), ...]
    curvatures = np.zeros((0, 1))  # (N x 1) [k, k, ...]

    def __init__(self, json_filepath) -> None:
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

            # find appropriate heading for use with Stanley Controller
            heading = 0
            if (i > 0):
                # at initial index, heading just remains 0
                prev_waypoint = data[i-1]
                prev_lat = prev_waypoint["lat"]
                prev_lon = prev_waypoint["lon"]
                prev_x, prev_y = World.gps_to_world(prev_lat, prev_lon)
                dx = x - prev_x
                dy = y - prev_y
                heading = np.arctan2(dy, dx)


            # Append to the arrays
            self.positions = np.append(self.positions, [[x, y, heading]], axis=0)

        # Calculate the distances along the trajectory
        self.distances = np.hstack(
            (
                0,
                np.cumsum(
                    np.sqrt(
                        np.sum(
                            np.diff(self.positions, axis=0) ** 2, axis=1, keepdims=True
                        )
                    )
                ),
            )
        )

        # Calculate the curvatures
        dx_dt = np.gradient(self.positions[:, 0])
        dy_dt = np.gradient(self.positions[:, 1])
        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)

        self.curvatures = (d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (
            dx_dt * dx_dt + dy_dt * dy_dt
        ) ** 1.5

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
            index (int): index along the trajectory

        Returns:
            tuple: (theta) in rads
        """
        theta = np.interp(index, np.arange(len(self.positions)), self.positions[:, 2])
        return theta

    def get_position_by_index(self, index):
        """Gets the position at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (int): index along the trajectory

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position
        x = np.interp(index, np.arange(len(self.positions)), self.positions[:, 0])
        y = np.interp(index, np.arange(len(self.positions)), self.positions[:, 1])

        return x, y

    def get_position_by_distance(self, distance):
        """Gets the position at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            tuple: (x, y)
        """
        # Interpolate the position
        x = np.interp(distance, self.distances, self.positions[:, 0])
        y = np.interp(distance, self.distances, self.positions[:, 1])

        return x, y

    def get_index_from_distance(self, distance):
        """Gets the index at a given distance along the trajectory,
        interpolating if necessary

        Args:
            distance (float): distance along the trajectory in meters

        Returns:
            int: index along the trajectory
        """
        # Interpolate the index
        index = np.interp(distance, self.distances, np.arange(len(self.distances)))

        return index

    def get_distance_from_index(self, index):
        """Gets the distance at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (int): index along the trajectory

        Returns:
            float: distance along the trajectory in meters
        """
        # Interpolate the distance
        distance = np.interp(index, np.arange(len(self.distances)), self.distances)

        return distance

    def get_curvature_by_index(self, index):
        """Gets the curvature at a given index along the trajectory,
        interpolating if necessary

        Args:
            index (int): index along the trajectory

        Returns:
            float: curvature
        """
        # Interpolate the curvature
        curvature = np.interp(index, np.arange(len(self.curvatures)), self.curvatures)

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
        curvature = np.interp(distance, self.distances, self.curvatures)

        return curvature

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
        x_interp = np.interp(
            np.linspace(start_index, end_index, subsample_resolution + 1),
            np.arange(len(self.positions)),
            self.positions[:, 0],
        )
        y_interp = np.interp(
            np.linspace(start_index, end_index, subsample_resolution + 1),
            np.arange(len(self.positions)),
            self.positions[:, 1],
        )

        distances = (x_interp - x) ** 2 + (y_interp - y) ** 2

        # Return the index of the closest point
        return (
            np.argmin(distances) / subsample_resolution * (end_index - start_index)
            + start_index
        )


if __name__ == "__main__":
    # Example usage
    trajectory = Trajectory("/rb_ws/src/buggy/paths/buggycourse.json")

    # Get the position at a given index
    x, y = trajectory.get_position_by_index(10)
    print(f"Position at index 10: ({x}, {y})")

    # Get the position at a given distance
    x, y = trajectory.get_position_by_distance(10)
    print(f"Position at distance 10: ({x}, {y})")

    # Get the index closest to a given position
    index = trajectory.get_closest_index_on_path(0, 0)
    print(f"Closest index to (0, 0): {index}")

    # Apply filter
    index = trajectory.get_closest_index_on_path(0, 0, start_index=10, end_index=20)
    print(f"Closest index to (0, 0) between 10 and 20: {index}")
