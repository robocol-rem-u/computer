#!/usr/bin/env python
import rospy
from master_msgs.msg import traction_Orders, imu_Speed, imu_Magnetism, pots, current, rpm, arm_Orders, goal, connection

def node_interface():
    rospy.init_node('node_interface',anonymous=True)
    rospy.Subscriber('topic_traction_orders', traction_Orders, traction_Orders_Callback)
    rospy.Subscriber('topic_imu_speed', imu_Speed, IMU_Speed_Callback)
    rospy.Subscriber('topic_imu_magnetism', imu_Magnetism, IMU_Magnetism_Callback)
    rospy.Subscriber('topic_pots', pots, pots_Callback)
    rospy.Subscriber('topic_current', current, current_Callback)
    rospy.Subscriber('topic_rpm', rpm, RPM_Callback)
    rospy.Subscriber('topic_arm_orders', arm_Orders, arm_Orders_Callback)
    pub_Goal = rospy.Publisher('topic_goal', goal, queue_size=10)
    pub_Arm_Orders = rospy.Publisher('topic_arm_orders', arm_Orders, queue_size=10)
    pub_Connection = rospy.Publisher('topic_connection', connection, queue_size=10)
    pub_Traction_Orders = rospy.Publisher('topic_traction_orders', traction_Orders, queue_size=10)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()


def traction_Orders_Callback(param):
    pass

def IMU_Speed_Callback(param):
    pass

def IMU_Magnetism_Callback(param):
    pass

def pots_Callback(param):
    pass

def current_Callback(param):
    pass

def RPM_Callback(param):
    pass

def arm_Orders_Callback(param):
    pass


if __name__ == '__main__':
    try:

        node_interface()

    except rospy.ROSInterruptException:
        pass
