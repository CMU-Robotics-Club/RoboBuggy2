import rospy
from rospy.std_msgs import Int64

class Heartbeat:
    rate: rospy.Rate
    heartbeat_num: int
    heartbeat_publisher: rospy.Publisher

    def __init__(self):
        rate = rospy.Rate(10)  # rate in hz
        heartbeat_num = 0
        heartbeat_publisher = rospy.Publisher(
            "/heartbeat",
            Int64
        )
    
    def update_heartbeat(self):
        self.heartbeat_publisher.publish(Int64(self.heartbeat_num))
        self.heartbeat_num += 1

    def start(self):
        while not rospy.is_shut_down():
            self.update_heartbeat()
            self.rate.sleep()


if __name__ == "__main__":
    hb_publisher = Heartbeat()
    hb_publisher.start()
