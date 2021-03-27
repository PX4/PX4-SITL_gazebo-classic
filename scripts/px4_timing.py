# Author: Garrett Gibo, Thanh Tran, Allen Maung

#!/usr/bin/env python3
import rospy
import time
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class PX4Listener:
    def __init__(self) -> None:
        rospy.init_node("px4_listener", anonymous=True)
        self.mavros_state_topic = "/mavros/state"
        self.mavros_local_position_topic = "/mavros/local_position/pose"
        self.init = None
        self.final = None
        self.start_time = None
        self.end_time = None
        self.flight_dur = None
        self.start_pos = None
        self.final_pos = None
        self.armed = False
        self.takeoff = False
        self.landed = False
        print("PX4Listener initialized")

        # Controller commands subscriber
        rospy.Subscriber(self.mavros_state_topic, State, self.callback_state)
        rospy.Subscriber(self.mavros_local_position_topic, PoseStamped, self.callback_state_1)

    def callback_state(self, data):
        # Initialize start_time when lander first takes off. 
        if data.armed and not self.armed:
            self.takeoff = True
            self.armed = True
            self.init = time.time()
            self.start_time = self.init - self.init
            rospy.loginfo("LEAPFROG has taken off. Start time: ")
            rospy.loginfo(self.start_time)
            
        # Record end_time when lander disarms.
        if self.armed and not data.armed:
            self.landed = True
            self.armed = False
            
            # end time is less than start time
            self.final = time.time()
            self.end_time = self.final - self.init
            self.flight_dur = self.end_time - self.start_time
            rospy.loginfo("LEAPFROG has landed. Flight duration: ")
            rospy.loginfo(self.flight_dur)
            # rospy.loginfo(self.start_time)
            # rospy.loginfo(self.end_time)

    def callback_state_1(self, data):
        # Initialize start_pos when lander first takes off
        if self.takeoff:
            self.start_pos = data.pose
            rospy.loginfo("LEAPFROG has taken off. Position: ")
            rospy.loginfo(self.start_pos)
            self.takeoff = False
        
        #record end_pos when landed disarms
        if self.landed:
            self.land_pos = data.pose
            rospy.loginfo("LEAPFROG has landed. Position: ")
            rospy.loginfo(self.land_pos)
            self.landed = False

if __name__ == '__main__':
    try:
        PX4Listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
