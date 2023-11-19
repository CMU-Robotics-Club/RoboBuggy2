import rospy
from rospy.std_msgs import Int64

class Heartbeat:
    RATE_HZ: float = 10
    heartbeat_num: int
    heartbeat_publisher: rospy.Publisher

    def __init__(self, buggy_name: str):
        heartbeat_num = 0
        heartbeat_publisher = rospy.Publisher(
            "/heartbeat",
            Int64
        )
    
    def update_heartbeat(self):
        self.heartbeat_publisher.publish(Int64(self.heartbeat_num))
        self.heartbeat_num += 1


if __name__ == "__main__":
    pass
