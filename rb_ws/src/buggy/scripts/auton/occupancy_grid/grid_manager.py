#!/usr/bin/env python3

import sys
import json

import numpy as np
# import scipy
import cv2
# from matplotlib import pyplot as plt
import utm

sys.path.append('rb_ws/src/buggy/scripts/auton')
# from pose import Pose

"""
####################################
=============== USAGE ==============
####################################
Initialize:
- Create occupancy grid object

Normal Op:
- Put NAND in the cost map via set_cost(utm_coords, cost)
- Get the normalized cost via get_utm_cost(utm_coords) by passing in the coordinates of the path
- Call reset_grid() to remove NAND from the grid
- LOOP!
"""

# Do we want to initialize the grid as zero cost? Or do we want to load the pre-fabricated cost map in?
INIT_WITH_EMPTY_GRID = True
class OccupancyGrid:
    def __init__(self):

        # Grid 0, 0 is NW
        # 1 pixel = 0.5m
        # X = East
        # Y = North

        # the original grid, should never change
        self.sat_img = np.array(cv2.cvtColor(cv2.imread("/rb_ws/src/buggy/assets/sat_img_resized.png"), cv2.COLOR_BGR2GRAY), np.uint8)

        # For original sat_img.png
        # 145.77 pixels = 88.62m
        # 0.5m resolution desired
        # Need to scale image by (2/(145.77/88.62)) = 1.215888

        if INIT_WITH_EMPTY_GRID:
            self.grid_og = np.zeros(np.shape(self.sat_img))
            self.grid = np.zeros(np.shape(self.sat_img))
        else:
            self.grid_og = np.array(cv2.cvtColor(cv2.imread("/rb_ws/src/buggy/assets/sat_img_resized.png"), cv2.COLOR_BGR2GRAY), np.uint8)
            # this grid will vary from call to call since NAND will be plotted in here and removed by reverting it back to grid_og
            self.grid = np.array(cv2.cvtColor(cv2.imread("/rb_ws/src/buggy/assets/cost_grid.png"), cv2.COLOR_BGR2GRAY), np.uint8)



        correspondence_f = open("/rb_ws/src/buggy/assets/landmarks.json")
        self.correspondence = json.load(correspondence_f)
        correspondence_f.close()

        pts_src = []
        pts_dst = []

        for i in range(0, len(self.correspondence)):
            ref_lat = self.correspondence[i]["lat"]
            ref_lon = self.correspondence[i]["lon"]
            ref_utm = utm.from_latlon(ref_lat, ref_lon)
            ref_utm = [ref_utm[0], ref_utm[1], 1]
            pts_src.append(ref_utm)

            sat_img_pixel = self.correspondence[i]["pixel"]
            sat_img_pixel.append(1) # homogenous coordinates
            pts_dst.append(sat_img_pixel)

        self.pts_src = np.array(pts_src)
        self.pts_dst = np.array(pts_dst)


        # REF_LOC_UTM_1 = utm.from_latlon(40.438834, -79.946334)
        # REF_LOC_UTM_1 = [REF_LOC_UTM_1[0], REF_LOC_UTM_1[1], 1]
        # REF_LOC_UTM_2 = utm.from_latlon(40.440482, -79.942299)
        # REF_LOC_UTM_2 = [REF_LOC_UTM_2[0], REF_LOC_UTM_2[1], 1]
        # REF_LOC_UTM_3 = utm.from_latlon(40.443738, -79.941774)
        # REF_LOC_UTM_3 = [REF_LOC_UTM_3[0], REF_LOC_UTM_3[1], 1]
        # REF_LOC_UTM_4 = utm.from_latlon(40.443451, -79.945972)
        # REF_LOC_UTM_4 = [REF_LOC_UTM_4[0], REF_LOC_UTM_4[1], 1]

        # SAT_IMG_LOC_1 = [354, 1397, 1]
        # SAT_IMG_LOC_2 = [1043, 1048, 1]
        # SAT_IMG_LOC_3 = [1128, 290, 1]
        # SAT_IMG_LOC_4 = [406, 366, 1]

        # pts_src = np.array([REF_LOC_UTM_1, REF_LOC_UTM_2, REF_LOC_UTM_3, REF_LOC_UTM_4])
        # pts_dst = np.array([SAT_IMG_LOC_1, SAT_IMG_LOC_2, SAT_IMG_LOC_3, SAT_IMG_LOC_4])

        self.homography, status = cv2.findHomography(self.pts_src, self.pts_dst, 0)

        if not np.allclose(status, 1, atol=1e-5):
            raise Exception("Cannot compute homography")

    def get_pixel_from_utm(self, utm_coord: np.array):
        """Calculate the pixel coordinates from the utm coordinates

        Args:
            utm_coord (np.array): utm coordinates
                                    [[0,0],
                                     [100, 100],
                                     [125, 400]] as an example

        Returns:
            pixel_coordinates: coordinates in pixels (x, y)
        """
        ones = np.ones(utm_coord.shape[0])
        utm_coord_homogenous = np.array([utm_coord[:, 0], utm_coord[:, 1], ones])
        loc =  self.homography @ utm_coord_homogenous
        return np.array([loc[0]/loc[2], loc[1]/loc[2]]).T

    def get_pixel_from_latlon(self, latlon_coord: np.array):
        """Calculate pixel coordintes from latlong coordinates

        Args:
            latlon_coord (np.array): latlong coordinates
                                    [[0,0],
                                     [100, 100],
                                     [125, 400]] as an example

        Returns:
            _type_: _description_
        """
        utm_coord = utm.from_latlon(latlon_coord[:, 0], latlon_coord[:, 1])
        ones = np.ones(latlon_coord.shape[0])
        utm_coord_homogenous = np.array([utm_coord[0], utm_coord[1], ones])
        loc =  self.homography @ utm_coord_homogenous
        return np.array([loc[0]/loc[2], loc[1]/loc[2]]).T

    def plot_points(self, utm_coords: list):
        """Mark the points on the grid completely BLACK

        Args:
            utm_coords (list): list of UTM coordinates to mark
                                [[0,0],
                                 [100, 100],
                                 [125, 400]] as an example
        """
        utm_coords = np.array(utm_coords)
        utm_coords = utm_coords[:, 0:2]
        pixel_coords = self.get_pixel_from_utm(utm_coords).astype(int)
        self.grid[pixel_coords[:, 0], pixel_coords[:, 1]] = 255

    def set_cost(self, utm_coords: list, cost: list):
        """Set the grid's cost

        Args:
            utm_coords (list): list of utm coordinates to mark (x, y)
                                [[0,0],
                                 [100, 100],
                                 [125, 400]] as an example
            cost (list): list of cost corresponding to coords
        """
        utm_coords = np.array(utm_coords)
        utm_coords = utm_coords[:, 0:2]
        pxl_coords = self.get_pixel_from_utm(utm_coords).astype(int)

        self.grid[pxl_coords[:, 0], pxl_coords[:, 1]] = cost

    def set_cost_persistent(self, utm_coords: list, cost: list):
        """Set the grid's cost but further calls to reset_grid() will not remove these points

        Args:
            utm_coords (list): list of utm coordinates to mark (x, y)
                                [[0,0],
                                 [100, 100],
                                 [125, 400]] as an example
            cost (list): list of cost corresponding to coords
        """
        utm_coords = np.array(utm_coords)
        utm_coords = utm_coords[:, 0:2]
        pxl_coords = self.get_pixel_from_utm(utm_coords).astype(int)
        self.grid[pxl_coords[:, 0], pxl_coords[:, 1]] = cost
        self.grid_og[pxl_coords[:, 0], pxl_coords[:, 1]] = cost

    def reset_grid(self):
        """Reset the grid back to the original grid (i.e. we remove NAND from the grid)
        """
        self.grid = np.copy(self.grid_og)

    def get_utm_cost(self, coords):
        """Get the cost of the trajectory passed in as utm_coordinates

        Args:
            coords (np array): coordinates in utm
                                 [[0,0],
                                 [100, 100],
                                 [125, 400]] as an example

        Returns:
            normalized cost: sum of all the pixels / number of pixels crossed
        """
        utm_coords = coords[:, 0:2]
        pxl_coords = self.get_pixel_from_utm(utm_coords).astype(int)
        filtered_pxl_coords = np.unique(pxl_coords[:, 0:2], axis=0)
        num_pixels_passed_thru = len(filtered_pxl_coords)
        total = np.sum(self.grid[filtered_pxl_coords[:, 0], filtered_pxl_coords[:, 1]])
        return total/num_pixels_passed_thru

    def get_pxl_cost(self, pxl_coords: list):
        """Get the cost of the trajectory passed in as pxl_coordinates

        Args:
            pxl_coords (list): coordinates in utm
                                [[0,0],
                                 [100, 100],
                                 [125, 400]] as an example

        Returns:
            normalized cost: sum of all the pixels / number of pixels crossed
        """
        pxl_coords = np.array(pxl_coords)
        filtered_pxl_coords = np.unique(pxl_coords[:, 0:2], axis=0)
        num_pixels_passed_thru = len(filtered_pxl_coords)
        total = np.sum(self.grid[filtered_pxl_coords[:, 0], filtered_pxl_coords[:, 1]])
        return total/num_pixels_passed_thru


if __name__ == "__main__":
    grid = OccupancyGrid()
    calculated_pxl = grid.get_pixel_from_utm(grid.pts_src)
    error = calculated_pxl - grid.pts_dst[:, 0:2]
    # print(f"ERROR: \n {error}")

    # Ensure the duplicate coordinates are removed
    test_points_utm = list(grid.pts_src[0:6, 0:2])
    test_points_utm.append([589591.7, 4477381.5]) # append a duplicate point
    test_points_utm = np.array(test_points_utm)
    print(test_points_utm)
    grid.set_cost(test_points_utm, [100, 100, 100, 100, 100, 100, 100])
    cost = grid.get_utm_cost(test_points_utm)
    print(cost)
    # cv2.imshow("IMG", grid.grid)
    # cv2.waitKey(0)

    # print(grid.grid)
    # plt.imshow(grid.grid)
    # plt.show()