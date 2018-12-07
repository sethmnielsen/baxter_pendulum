#!/usr/bin/env python
import a)
import numpy as np
from geometry_msgs.msg import PoseStamped

class PendulumStateSim:

    def __init__(self):
        rospy.Subscriber('/robot/pendulum/pose/NWU', PoseStamped, self.pose_cb)

        # ROS publishers
        self._pub = rospy.Publisher('', , queue_size=1)

    def pose_cb(self, msg):
        trans, rot = listener.lookupTransform('/leftpendulum', '/world', rospy.Time(0))
        pos_ee = np.array(trans)
        pos_pend = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        


        self._pub.publish()

if __name__ == '__main__':
    rospy.init_node("pendulum_state_sim", anonymous=True)
    aspd = PendulumStateSim()
    rate = rospy.Rate(60)  # Rate of main loop (Hz)

    while not rospy.is_shutdown():
        rospy.spin()
