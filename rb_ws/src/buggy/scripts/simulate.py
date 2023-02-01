#! /usr/bin/env python3
import rospy
import trimesh
import resource_retriever
import threading
import numpy as np
from std_msgs.msg import Float32, Bool, Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler


class MeshQuery:
  THROW_HEIGHT = 10000.0
  def __init__(self, mesh_uri) -> None:
    # load the mesh
    # use resource_retriever to get filepath to uri

    filename = resource_retriever.get_filename(mesh_uri)
    self.mesh = trimesh.load_mesh(filename[7:])
    self.intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(self.mesh)

  def query_mesh_slope(self, x, y):
    """ Throw a ray from the point (x, y, 10000) in the -z direction
    find the normal vector at the intersection point  

    Args:
        x (float): position
        y (float): position

    Returns:
        ([float, 3], [float, 3]): location and normal, respectively.
                                  normal is represented such that the z coordinate is positive
    """
    # Throw a ray from the point (x, y, 10000) in the -z direction
    # find the normal vector at the intersection point  
    locations, _, index_tri = self.mesh.ray.intersects_location(
      ray_origins=np.array([[x, y, self.THROW_HEIGHT]]),
      ray_directions=np.array([[0.0, 0.0, -1.0]]),
    )

    # Take the first triangle that was hit
    try:
      location, index_tri = locations[0], index_tri[0]
    except IndexError:
      # Off map; Place at z = 0, normal = (0, 0, 1)
      return np.array([x, y, 0.0]), np.array([0.0, 0.0, 1.0])

    # Get the normal vector of the triangle
    normal = self.mesh.face_normals[index_tri]
    if normal[2] < 0:
      normal = -normal

    return location, normal



class Simulator:
  # Simulaton
  RATE = 50.0                   # Hz
  GRAVITY = 9.81
  START_POSITION = [0.0, 0.0]   # change later to hill 1, 2, 3, 4, 5
  START_DIRECTION = [1.0, 0.0]  # change later to hill 1, 2, 3, 4, 5

  # Buggy Intrinsics
  CROSS_SECTION_AREA = 0.3      # m^2
  DRAG_COEFF = 0.3              # ~passenger car
  WEIGHT = 31.0                 # kg
  WHEELBASE = 1.3               # m
  STEERING_MAX = 30             # degrees to left and degrees to right
  def __init__(self, map_uri) -> None:
    self.topo = MeshQuery(map_uri)
    self.lock = threading.Lock()
    
    self.position, self.direction = np.array(self.START_POSITION), np.array(self.START_DIRECTION)
    self.elevation = 0.0
    self.speed = 0.0

    self.brake = False # On/Off
    self.steering = 0.0 # angle from neutral. Positive is right, negative is left
    self.push_force = 0.0 # Newtons

    # Setup Subscriber/Publisher Hooks
    rospy.Subscriber("simulator/input/steering", Float32, self.set_steering)
    rospy.Subscriber("simulator/input/brake", Bool, self.set_brake)
    rospy.Subscriber("simulator/input/push", Float32, self.set_push) # Forces, in newtons

    self.pose_pub = rospy.Publisher("simulator/output/pose", PoseStamped, queue_size=10)
    self.speed_pub = rospy.Publisher("simulator/output/speed", Float32, queue_size=10)
    # self.mesh_pub = rospy.Publisher("simulator/output/mesh", PoseStamped, queue_size=10)

  def compute_drag_force(self):
    """ Calculate the drag force on the buggy

    Returns:
        float: drag force
    """
    # magnitude = 0.5 pAv^2
    magnitude = 0.5 * self.DRAG_COEFF * self.CROSS_SECTION_AREA * (self.speed ** 2)

    # Apply in opposite direction of self.direction
    return -magnitude * self.direction

  def compute_gravity_force(self):
    # compute the horizontal forces due to gravity
    _, surface_normal = self.topo.query_mesh_slope(self.position[0], self.position[1])
    # compute cross product of surface_normal with (0, 0, 1.0)
    # to get the amount of gravity in the x and y directions
    cross = np.cross(surface_normal, np.array([0.0, 0.0, 1.0]))

    # compute the magnitude of the force
    magnitude = self.GRAVITY * self.WEIGHT * np.linalg.norm(cross)

    # compute the direction of the force
    denom = np.linalg.norm(surface_normal[:2])
    if denom == 0.0:
      direction = np.array([0.0, 0.0])
    else:
      direction = surface_normal[:2] / denom

    return magnitude * direction

  def compute_push_force(self):
    # Place force along direction
    return self.push_force * self.direction


  def get_steering_arc(self):
    # calculate the radius of the steering arc
    #   1) If steering angle is 0, return infinity
    #   2) Otherwise, return radius of arc

    if self.steering > self.STEERING_MAX:
      self.steering = self.STEERING_MAX
    elif self.steering < -self.STEERING_MAX:
      self.steering = -self.STEERING_MAX


    if self.steering == 0.0:
      return np.inf

    return self.WHEELBASE / np.tan(self.steering)



  def step(self):
    # calculate force on buggy
    #   1) Gravity
    #   2) Drag 
    #   3) Friction (ignored)
    #   4) Push
    # 
    # calculate new velocity
    # calculate new position

    force = self.compute_drag_force() + self.compute_gravity_force() + self.compute_push_force()

    force_along_direction = np.dot(force, self.direction)

    # Calculate new velocity
    self.speed += force_along_direction / self.WEIGHT
    if self.brake:
      self.speed = 0.0

    if not -2.0 < self.speed < 2.0:
      self.speed = np.sign(self.speed) * 2.0

    # Calculate new position
    distance = np.linalg.norm(self.speed) / self.RATE
    if self.steering == 0.0:
      # Straight
      self.position += distance * self.direction
    else:
      # Arc
      radius = self.get_steering_arc()
      # Calculate the angle of the arc traveled
      angle_rad = distance / radius

      x_disp = abs(radius) * np.sin(angle_rad)
      y_disp = -radius * (1 - np.cos(angle_rad))
      disp = np.array([[x_disp, y_disp]])
      
      # rotate the displacement vector by the direction of the buggy
      rotation_matrix = np.array(
        [[self.direction[0], -self.direction[1]],
         [self.direction[1], self.direction[0]]]
      )
      
      # (2x2 * (1x2).T).T.squeeze
      rotated_disp = np.dot(rotation_matrix, disp.T).T.squeeze()

      self.position = self.position + rotated_disp

    # Calculate z position
    location, _ = self.topo.query_mesh_slope(self.position[0], self.position[1])
    self.elevation = location[2]

  def set_brake(self, msg: Bool):
    with self.lock:
      self.brake = msg.data

  def set_steering(self, msg: Float32):
    with self.lock:
      self.steering = msg.data

  def set_push(self, msg: Float32):
    with self.lock:
      self.push_force = msg.data

  def publish(self):
    location = Pose()
    location.position.x = self.position[0]
    location.position.y = self.position[1]
    location.position.z = self.elevation
    # convert direction to angle around z axis
    angle = np.arctan2(self.direction[1], self.direction[0]) # passed in as y, x, returned in radians
    location.orientation = Quaternion(*quaternion_from_euler(0, 0, angle))
    stamped_pose = PoseStamped()
    stamped_pose.header.stamp = rospy.Time.now()
    stamped_pose.header.frame_id = "base"
    stamped_pose.pose = location
    self.pose_pub.publish(stamped_pose)
    speed = Float32()
    speed.data = self.speed
    self.speed_pub.publish(speed)


  def run(self):
    rate = rospy.Rate(self.RATE)
    while not rospy.is_shutdown():
      with self.lock:
        print("Step")
        self.step()
        self.publish()
      rate.sleep()


if __name__ == "__main__":
  rospy.init_node("simulator")
  # Centered at 40.441687, -79.944276
  sim = Simulator("package://buggy/meshes/cmutopo.stl")
  sim.run()  