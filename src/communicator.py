import rospy
from mavros.srv import ParamSet
from mavros.srv import StreamRate
from mavros.srv import CommandBool

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

if __name__ == "__main__":
    rospy.init_node("SYSID_MYGCS")
    rospy.init_node("set_stream_rate")

    sys_id = rospy.get_param("~SYSID_MYGCS", 1)
    rate = rospy.get_param("~set_stream_rate", 10)

    set_sys_id(sys_id)
    set_stream_rate(rate)
