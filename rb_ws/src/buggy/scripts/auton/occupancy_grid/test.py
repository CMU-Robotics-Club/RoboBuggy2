import cv2
import numpy as np
import sys
sys.path.append('rb_ws/src/buggy/scripts/auton')
from grid_manager import OccupancyGrid
from path_projection import Projector
from pose import Pose
import utm

if __name__ == '__main__':
    # src = np.array([[-47.59, -57.43, 139.59, 149.43, 46.],
    #                 [141.43, 31.03, -1.43, 108.97, 70.],
    #                 [579.23, 412.75, 422.55, 589.03, 500.89]])

    # dst = np.array([[-100, -100, 100, 100, 0],
    #                 [-100, 100, 100, -100, 0],
    #                 [1, 1, 1, 1, 1]])  # converting to homogeneous coordinates

    # print(dst.T)

    # H, _ = cv2.findHomography(src.T, dst.T)
    # dst_est = H @ src
    # print(np.allclose(dst_est / (dst_est[2, :]), dst / dst[2, :], atol=1e-2))  # dividing by last component to fix the scaling and adjusting the tolerance

    # test_point = np.array([[-47.59], [141.43], [579.23]])
    # res_test = H @ test_point
    # res_test = res_test/res_test[2]
    # print(res_test)
    WHEELBASE = 1.3
    p = Projector(WHEELBASE)
    latlon_start = [40.441798, -79.943976] # on the mall
    utm_start = utm.from_latlon(latlon_start[0], latlon_start[1])
    start_pose = Pose(utm_start[0], utm_start[1], 0)
    path = p.project(start_pose, 1, 5, 15, 10)
    grid = OccupancyGrid()
    grid.plot_points(path)
    grid.get_cost(path)
    
    cv2.imshow("IMG", grid.sat_img)
    cv2.waitKey(0)