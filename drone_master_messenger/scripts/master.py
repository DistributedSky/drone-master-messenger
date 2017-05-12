#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import State
from std_msgs.msg import String, UInt32
from messenger import messenger_mission_gen
from messenger import messenger_drone_free
import rospy, thread

adapters = []
states = {}

def set_mode(adapter, mode):
    rospy.wait_for_service(adapter+'/mavros/set_mode')
    try:
        service = rospy.ServiceProxy(adapter+'/mavros/set_mode', SetMode)
        service(0, mode)
        rospy.loginfo('{0} :: {1} mode activated'.format(adapter, mode))
    except e:
        print('Service call failed: {0}'.format(e))

def arming(adapter):
    rospy.wait_for_service(adapter+'/mavros/cmd/arming')
    try:
        arm = rospy.ServiceProxy(adapter+'/mavros/cmd/arming', CommandBool)
        arm(True)
        rospy.loginfo('{0} :: Armed'.format(adapter))
        rospy.sleep(3)
    except e:
        print('Service call failed: {0}'.format(e))

def takeoff(adapter):
    rospy.wait_for_service(adapter+'/mavros/cmd/takeoff')
    try:
        takeoff = rospy.ServiceProxy(adapter+'/mavros/cmd/takeoff', CommandTOL)
        takeoff(0,0,0,0,10)
        rospy.loginfo('{0} :: Takeoff'.format(adapter))
        rospy.sleep(10)
    except e:
        print('Service call failed: {0}'.format(e))

states = {}

if __name__ == '__main__':
    rospy.init_node('drone_master')

    def drone_adapter(msg):
        adapter = msg.data

        # Check for double reg
        if adapter in adapters:
            return
        else:
            adapters.append(adapter)
            messenger_drone_free(adapter)

        rospy.loginfo('Found adapter {0}'.format(adapter))

        def drone_state(msg):
            states[adapter] = msg

        rospy.Subscriber(msg.data+'/mavros/state', State, drone_state)
        rospy.loginfo('{0} :: State subscribed'.format(adapter))

        def drone_mission():
            rospy.loginfo('{0} :: Mission thread started'.format(adapter))
            for point in messenger_mission_gen(adapter):
                if point < 0:
                    set_mode(adapter, 'RTL')
                    messenger_drone_free(adapter)

                if not states[adapter].armed:
                    arming(adapter)
                    takeoff(adapter)

                if states[adapters].mode != 'AUTO':
                    set_mode(adapter, 'AUTO')

                if point >= 0:
                    rospy.wait_for_service(adapter+'/mavros/mission/set_current')
                    mission = rospy.ServiceProxy(adapter+'/mavros/mission/set_current',
                                                 WaypointSetCurrent)
                    mission(point)
                    rospy.loginfo('{0} :: Current mission point -> {1}'.format(adapter, point))

        thread.start_new_thread(drone_mission, ())

    rospy.Subscriber('/adapter', String, drone_adapter)
    rospy.spin()
