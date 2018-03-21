#!/usr/bin/env python
import rospy
import mavros
from mavros_msgs.srv import ParamSet, StreamRate, CommandBool
from mavros_msgs.msg import OverrideRCIn

def get_proxy(namespace, service, type, timeout=10):
    topic = mavros.get_topic(namespace, service)
    rospy.wait_for_service(topic, timeout)
    return rospy.ServiceProxy(topic, type)

def set_sys_id(id):
    success = True

    try:
        rospy.wait_for_service("mavros/param/set", 10)
        param_srv = rospy.ServiceProxy("mavros/param/set", ParamSet)
        param = param_srv("SYSID_MYGCS", id)
        if not param.success:
            success = False
    except:
        success = False

    return success

def set_stream_rate(rate):
    success = True

    try:
        rospy.wait_for_service("mavros/set_stream_rate", 10)
        stream_rate_srv = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)
        param = stream_rate_srv(stream_id=0, message_rate=rate, on_off=(rate != 0))
        if not param:
            success = False
    except:
        success = False

    return success

def arm(enable):
    success = True

    try:
        rosspy.wait_for_service("mavros/cmd/arming")
        arm_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        param = arm_srv(enable)
        if not param.success:
            success = False
    except:
        success = False

    return success

def set_override_message(roll, pitch, yaw, throttle, forward, lateral):
    PitchChan, RollChan, ThrottleChan, YawChan, ForwardChan, LateralChan = range(6)

    message = OverrideRCIn()
    message.channels[RollChan] = roll
    message.channels[PitchChan] = pitch
    message.channels[YawChan] = yaw
    message.channels[ThrottleChan] = throttle
    message.channels[ForwardChan] = forward
    message.channels[LateralChan] = lateral

    global override_message
    override_message = message

if __name__ == "__main__":
    override_message = None
    rospy.init_node("cnt")
    sys_id = rospy.get_param("~SYSID_MYGCS", 1)
    rate = rospy.get_param("~set_stream_rate", 10)

    set_sys_id(sys_id)
    set_stream_rate(rate)

    pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)

    while not rospy.is_shutdown():
        pub.publish(override_message)
        rospy.sleep(.045)
