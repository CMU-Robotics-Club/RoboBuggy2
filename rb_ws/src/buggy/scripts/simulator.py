#! /usr/bin/env python3
# ROS imports
import rospy
import resource_retriever

# ROS message imports
from std_msgs.msg import Float32, Bool, Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

# Utility imports
import numpy as np
import threading
import trimesh




class MeshQuery:
  THROW_HEIGHT = 10000.0 # meters
  def __init__(self, mesh_uri, center_lat, center_long):
    """ Load the mesh from a given URI. The URI is resolved using ROS's 
    resource_retriever. The center of the mesh should be at the given
    latitude and longitude.

    Args:
        mesh_uri (string): URI to the mesh
        center_lat (float): latitude of the center of the mesh
        center_long (float): longitude of the center of the mesh
    """
    filename = resource_retriever.get_filename(mesh_uri)
    self.mesh = trimesh.load_mesh(filename[7:])
    self.intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(self.mesh)
    self.center_lat, self.center_long = center_lat, center_long

  def query_mesh(self, x, y):
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

  def get_gps(self, x, y):
    """ Get the GPS coordinates of a point on the map

    Args:
        x (float): position, meters, relative to the center of the map
        y (float): position, meters, relative to the center of the map

    Returns:
        [float, 2]: GPS coordinates, latitude and longitude, respectively
    """
    # Get the GPS coordinates of a point on the map
    # Use the center of the map as the origin
    # 111,111 m per degree of latitude
    # 111,111 * cos(latitude) m per degree of longitude
    lat = self.center_lat + y / 111111.0
    long = self.center_long + x / (111111.0 * np.cos(self.center_lat / 180.0 * np.pi))
    return (lat, long)




class Simulator:
  # Coordinate system:
  #   x: +east/-west     == longitude
  #   y: +north/-south   == latitude
  #   z: +up/-down       == elevation
  #   angle: +cw/-ccw    == heading(compass)

  # Rotation matrix in these coordinates is the transpose 
  # of the rotation matrix in the standard coordinate system
  
  # ===== CONSTANTS =====

  # Simulaton
  RATE = 50.0          # Hz
  GRAVITY = 9.81       # m/s^2
  START_POSITION = [-50.0, -257.0]    # change later to hill 1, 2, 3, 4, 5
  START_DIRECTION = [0.0, -1.0]       # change later to hill 1, 2, 3, 4, 5

  # Buggy Intrinsics
  CROSS_SECTION_AREA = 0.3      # m^2
  DRAG_COEFF = 0.3              # ~passenger car
  MASS = 31.0                   # kg
  WHEELBASE = 1.3               # m
  STEERING_MAX = 30             # degrees to left and degrees to right
  ROLLING_RESISTANCE = 0.03     # ~passenger car
  HEIGHT = 0.1                  # m, (distance btw ground and center of wheels)

  def __init__(self, map_uri, map_center_lat, map_center_long):
    self.topo = MeshQuery(map_uri, map_center_lat, map_center_long)
    self.lock = threading.Lock()
    
    self.position, self.direction = np.array(self.START_POSITION), np.array(self.START_DIRECTION)
    self.elevation = 0.0
    self.speed = 0.0

    self.brake = False # On/Off
    self.steering = 1.0 # angle from neutral. Positive is right, negative is left
    self.push_force = 0.0 # Newtons

    # Setup Subscriber/Publisher Hooks
    rospy.Subscriber("simulator/input/steering", Float32, self.set_steering)
    rospy.Subscriber("simulator/input/brake", Bool, self.set_brake)
    rospy.Subscriber("simulator/input/push", Float32, self.set_push) # Forces, in newtons

    self.pose_pub = rospy.Publisher("simulator/output/pose", PoseStamped, queue_size=10)
    self.speed_pub = rospy.Publisher("simulator/output/speed", Float32, queue_size=10)
    self.mesh_pub = rospy.Publisher("simulator/output/mesh", Marker, queue_size=10)

  @staticmethod
  def rotate(vec, theta):
    """ Rotate a vector by theta radians"""
    matrix = np.array(
      [[np.cos(theta), np.sin(theta)],
      [-np.sin(theta), np.cos(theta)]]
    )

    return np.dot(matrix, vec.reshape(1, 2).T).T.squeeze()

  def compute_drag_force(self):
    """ Calculate the drag forces on the buggy

    Returns:
        float: drag force
    """
    # air resistance: magnitude = 0.5 pAv^2
    magnitude = 0.5 * self.DRAG_COEFF * self.CROSS_SECTION_AREA * (self.speed ** 2)

    # rolling resistance: magnitude += nu * g * m
    magnitude += self.ROLLING_RESISTANCE * self.GRAVITY * self.MASS

    # Apply in opposite direction of self.speed, along self.direction
    return (-1.0 if self.speed > 0 else 1.0) * magnitude * self.direction

  def compute_gravity_force(self):
    # compute horizontal forces on the buggy due to gravity
    _, surface_normal = self.topo.query_mesh(self.position[0], self.position[1])
    # compute cross product of surface_normal with (0, 0, 1.0)
    # to get the amount of gravity in the x and y directions
    cross = np.cross(surface_normal, np.array([0.0, 0.0, 1.0]))

    # compute the magnitude of the force
    magnitude = self.GRAVITY * self.MASS * np.linalg.norm(cross)

    # compute the direction of the force projected onto x-y plane
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
    # If turning right, steering radius is positive
    # If turning left, steering radius is negative
    if self.steering == 0.0:
      return np.inf

    return self.WHEELBASE / np.tan(np.deg2rad(self.steering))



  def step(self):
    # calculate force on buggy
    #   1) Gravity
    #   2) Drag 
    #   3) Friction (ignored)
    #   4) Push
    # 
    # calculate new velocity
    # calculate new position
    # calculate new heading

    force = self.compute_drag_force() + self.compute_gravity_force() + self.compute_push_force()

    force_along_direction = np.dot(force, self.direction)

    # Calculate new velocity
    self.speed += force_along_direction / self.MASS / self.RATE
    if self.brake:
      self.speed = 0.0

    # NOTE: Remove this later
    # if not -5.0 < self.speed < 5.0:
      # self.speed = np.sign(self.speed) * 5.0
    distance = self.speed / self.RATE



    # Calculate new position
    if self.steering == 0.0:
      # Straight
      self.position += distance * self.direction
    else:
      # steering radius
      radius = self.get_steering_arc()

      # Calculate new heading
      # Forward, Right: +delta_h  +r, +d
      # Forward, Left : -delta_h  -r, +d
      # Backwrd, Right: -delta_h  +r, -d
      # Backwrd, Left : +delta_h  -r, -d
      delta_heading = distance / radius

      arc_traveled = delta_heading # These are the same

      # Calculate the displacement vector due to travel along arc


      delta_y = np.sin(arc_traveled) * radius
      delta_x = radius * (1 - np.cos(arc_traveled))
      displacement = np.array([delta_x, delta_y])

      # rotate the displacement vector by the direction of the buggy
      # get the rotation angle by taking the arctan2 of the direction vector
      # arctan2 acts with x-axis as 0, and rotates ccw
      # we operate with y-axis as 1, and rotate cw
      # Can just transpose the direction vector to get the correct angle
      # arctan2 asks for y-axis as first argument, x-axis as second
      # so we provide the direction vector as [x, y] (transposed)
      rotated_disp = self.rotate(displacement, np.arctan2(self.direction[0], self.direction[1]))
      
      self.position = self.position + rotated_disp
    
      # Calculate the angle change of the heading 
      self.direction = self.rotate(self.direction, delta_heading)

    # Refresh z position
    location, _ = self.topo.query_mesh(self.position[0], self.position[1])
    self.elevation = location[2] + self.HEIGHT

  def set_brake(self, msg: Bool):
    with self.lock:
      self.brake = msg.data

  def set_steering(self, msg: Float32):
    with self.lock:
      self.steering = msg.data
      # Steering limits
      if self.steering > self.STEERING_MAX:
        self.steering = self.STEERING_MAX
      elif self.steering < -self.STEERING_MAX:
        self.steering = -self.STEERING_MAX

  def set_push(self, msg: Float32):
    with self.lock:
      self.push_force = msg.data

  def publish(self, iter_ct):
    # Publish 3d Pose
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
    
    # Publish speed
    speed = Float32()
    speed.data = self.speed
    self.speed_pub.publish(speed)
    if iter_ct % (2 * self.RATE) == 0:
      # Publish Mesh infrequently
      marker = Marker()

      marker.header.frame_id = "base"
      marker.header.stamp = rospy.Time.now()
      marker.ns = ""

      # Shape (resource type = 10 (mesh))
      marker.type = 10
      marker.id = 0
      marker.action = 0

      # Note: Must set mesh_resource to a valid URL for a model to appear
      marker.mesh_resource = "http://127.0.0.1:8760/cmutopo.dae"
      marker.mesh_use_embedded_materials = True

      # Scale
      marker.scale.x = 1.0
      marker.scale.y = 1.0
      marker.scale.z = 1.0

      # Color
      marker.color.r = 0.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 1.0

      # Pose
      marker.pose.position.x = 0
      marker.pose.position.y = 0
      marker.pose.position.z = 0
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 0.0

      self.mesh_pub.publish(marker)



  def run(self):
    rate = rospy.Rate(self.RATE)
    count = 0
    while not rospy.is_shutdown():
      with self.lock:
        self.step()
        self.publish(count)
      rate.sleep()
      count += 1


if __name__ == "__main__":
  rospy.init_node("simulator")
  sim = Simulator("package://buggy/assets/cmutopo.stl", 40.441687, -79.944276)
  sim.run()  
