#!/usr/bin/env python3

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import utm

class OccupancyGrid:
    def __init__(self):
        
        # Grid 0, 0 is NW
        # X = East
        # Y = North
        self.sat_img = np.array(cv.cvtColor(cv.imread("rb_ws/src/buggy/assets/satellite_unflip_crop_raw.png"), cv.COLOR_BGR2GRAY), np.uint8)
        # self.sat_img = self.sat_img[350:850, 1500:2000]

        self.grid = np.array(cv.cvtColor(cv.imread("rb_ws/src/buggy/assets/satellite_unflip_crop_blacked.png"), cv.COLOR_BGR2GRAY), np.uint8)
        # self.grid = self.grid[350:850, 1500:2000]
        self.grid = np.array(self.grid == 0, np.uint8) * 255
    
    @staticmethod
    def get_pixel_from_coord(utm_coord: np.array):
        REF_LOC_UTM_1 = (589184.50, 4477217.38)
        REF_LOC_UTM_2 = (589951.73, 4477498.55) # https://www.google.com/maps/place/40%C2%B026'35.8%22N+79%C2%B056'21.6%22W/@40.443277,-79.9399667,189m/data=!3m2!1e3!4b1!4m4!3m3!8m2!3d40.443276!4d-79.939323?entry=ttu

        SAT_IMG_LOC_1 = (26, 1211)
        SAT_IMG_LOC_2 = (1946, 584)

        dy = (SAT_IMG_LOC_2[1] - SAT_IMG_LOC_1[1]) / (REF_LOC_UTM_2[1] - REF_LOC_UTM_1[1])
        dx = (SAT_IMG_LOC_2[0] - SAT_IMG_LOC_1[0]) / (REF_LOC_UTM_2[0] - REF_LOC_UTM_1[0])

        y = SAT_IMG_LOC_1[1] + dy*(utm_coord[1] - REF_LOC_UTM_1[1])
        x = SAT_IMG_LOC_1[0] + dx*(utm_coord[0] - REF_LOC_UTM_1[0])

        return x, y

    # def get_cost(self, coords: np.array):
    #     # coords is 2d array, col0 = x, col1 = y UTM coordinates
        


if __name__ == "__main__":
    grid = OccupancyGrid()
    print(grid.get_pixel_from_coord((589703.06, 4477185.38)))
    cv.imshow("IMG", grid.grid)
    cv.waitKey(0)






# cv.imshow("IMG", sat_img)
# cv.waitKey(0)