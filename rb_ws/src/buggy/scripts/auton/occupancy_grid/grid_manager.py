#!/usr/bin/env python3

import numpy as np
import cv2
from matplotlib import pyplot as plt
import utm

class OccupancyGrid:
    def __init__(self):
        
        # Grid 0, 0 is NW
        # 1 pixel = 0.5m
        # X = East
        # Y = North
        self.sat_img = np.array(cv2.cvtColor(cv2.imread("rb_ws/src/buggy/assets/sat_img_resized.png"), cv2.COLOR_BGR2GRAY), np.uint8)

        # For original sat_img.png
        # 145.77 pixels = 88.62m
        # 0.5m resolution desired
        # Need to scale image by (2/(145.77/88.62)) = 1.215888

        self.grid = np.array(cv2.cvtColor(cv2.imread("rb_ws/src/buggy/assets/sat_img_resized.png"), cv2.COLOR_BGR2GRAY), np.uint8)
        # self.grid = np.array(self.grid == 0, np.uint8) * 255

        REF_LOC_UTM_1 = utm.from_latlon(40.438834, -79.946334)
        REF_LOC_UTM_1 = [REF_LOC_UTM_1[0], REF_LOC_UTM_1[1], 1]
        REF_LOC_UTM_2 = utm.from_latlon(40.440482, -79.942299)
        REF_LOC_UTM_2 = [REF_LOC_UTM_2[0], REF_LOC_UTM_2[1], 1]
        REF_LOC_UTM_3 = utm.from_latlon(40.443738, -79.941774)
        REF_LOC_UTM_3 = [REF_LOC_UTM_3[0], REF_LOC_UTM_3[1], 1]
        REF_LOC_UTM_4 = utm.from_latlon(40.443451, -79.945972)
        REF_LOC_UTM_4 = [REF_LOC_UTM_4[0], REF_LOC_UTM_4[1], 1]

        SAT_IMG_LOC_1 = [354, 1397, 1]
        SAT_IMG_LOC_2 = [1043, 1048, 1]
        SAT_IMG_LOC_3 = [1128, 290, 1]
        SAT_IMG_LOC_4 = [406, 366, 1]
    
        pts_src = np.array([REF_LOC_UTM_1, REF_LOC_UTM_2, REF_LOC_UTM_3, REF_LOC_UTM_4])
        pts_dst = np.array([SAT_IMG_LOC_1, SAT_IMG_LOC_2, SAT_IMG_LOC_3, SAT_IMG_LOC_4])
        self.homography, status = cv2.findHomography(pts_src, pts_dst, 0)

        if not np.allclose(status, 1, atol=1e-5):
            raise Exception("Cannot compute homography")
    
    def get_pixel_from_utm(self, utm_coord: np.array):
        utm_coord_homogenous = np.array([[utm_coord[0]], [utm_coord[1]], [1]])
        loc =  self.homography @ utm_coord_homogenous
        return loc[0]/loc[2], loc[1]/loc[2]

    def get_pixel_from_latlon(self, latlon_coord: np.array):
        utm_coord = utm.from_latlon(latlon_coord[:, 0], latlon_coord[:, 1])
        ones = np.ones(latlon_coord.shape[0])
        utm_coord_homogenous = np.array([utm_coord[0], utm_coord[1], ones])
        loc =  self.homography @ utm_coord_homogenous
        print(loc)
        return loc[0]/loc[2], loc[1]/loc[2]
        

    # def get_cost(self, coords: np.array):
    #     # coords is 2d array, col0 = x, col1 = y UTM coordinates
        


if __name__ == "__main__":
    grid = OccupancyGrid()
    latlon = np.array([[40.438834, -79.946334], [40.440482, -79.942299], [40.443738, -79.941774]])
    print(grid.get_pixel_from_latlon(latlon))
    cv2.imshow("IMG", grid.sat_img)
    cv2.waitKey(0)






# cv.imshow("IMG", sat_img)
# cv.waitKey(0)