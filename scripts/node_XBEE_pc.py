#!/usr/bin/env python
import rospy
from master_msgs.msg import connection, arm_Orders, traction_Orders


def connection_Callback(parm):
    pass
def arm_Orders_Callback(parm):
    pass
def traction_Orders_Callback(param):
    pass

def node_XBEE_pc():

    rospy.init_node('node_XBEE_pc', anonymous=True)
    # Se suscribe a al topico de la informacion de la conexion sobre quien manda informacion
    rospy.Subscriber ('topic_Connection', connection, connection_Callback)
    # Se suscribe a al topico que indica ordenes sobre los brazos
    rospy.Subscriber ('topic_Arm_Orders', arm_Orders, arm_Orders_Callback)
    # Se suscribe a al topico que indica ordenes sobre ruedas
    rospy.Subscriber ('topic_Traction_Orders', traction_Orders, traction_Orders_Callback)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown ():
        rate.sleep ()


if __name__ == '__main__':
    try:

        node_XBEE_pc()
    except rospy.ROSInterruptException:
        pass
