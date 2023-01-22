#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64 # Callback function to print the subscribed data on the terminal

def callback_str(subscribedData): # print out the message that we received
    rospy.loginfo(subscribedData) # Subscriber node function which will subscribe the messages from the Topic
def messageSubscriber():
    # initialize the subscriber node called 'messageSubNode'
    rospy.init_node("buggyTestSubscriber", anonymous=False)    # This is to subscribe to the messages from the topic named 'messageTopic'
    rospy.Subscriber("testMessage", Float64, callback_str)    # rospy.spin() stops the node from exiting until the node has been shut down
    rospy.spin()

if __name__ == '__main__':
    try:
        messageSubscriber()
    except rospy.ROSInterruptException:
        pass