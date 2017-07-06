#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import State
from std_srvs.srv import SetBool
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
        takeoff(0,0,0,0,5)
        rospy.loginfo('{0} :: Takeoff'.format(adapter))
        rospy.sleep(10)
    except e:
        print('Service call failed: {0}'.format(e))

def record(enable):
    rospy.wait_for_service(adapter+'/camera/record')
    try:
        record = rospy.ServiceProxy(adapter+'/camera/record', SetBool)
        if record(enable).success:
            rospy.loginfo('{0} :: Record > {1} is success'.format(adapter, enable))
        else:
            rospy.logerr('{0} :: Record > {1} is fail'.format(adapter, enable))
    except e:
        print('Service call failed: {0}'.format(e))

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

        rospy.Subscriber('{0}/mavros/state'.format(adapter), State, drone_state)
        rospy.loginfo('{0} :: State subscribed'.format(adapter))

        def drone_video(msg):
            messenger_drone_video(adapter, msg.data)

        rospy.Subscriber('{0}/camera/video'.format(adapter), String, drone_video)
        rospy.loginfo('{0} :: Video subscribed'.format(adapter))

        def drone_thumbnail(msg):
            messenger_drone_thumbnail(adapter, msg.data)

        rospy.Subscriber('{0}/camera/thumbnail'.format(adapter), String, drone_thumbnail)
        rospy.loginfo('{0} :: Thumbnail subscribed'.format(adapter))

        def drone_mission():
            rospy.loginfo('{0} :: Mission thread started'.format(adapter))
            for point in messenger_mission_gen(adapter):
                if point < 0:
                    record(False)
                    set_mode(adapter, 'RTL')
                    messenger_drone_free(adapter)

                if not states[adapter].armed:
                    record(True)
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
