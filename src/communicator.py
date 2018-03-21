import rospy
from mavros.srv import ParamSet
from mavros.srv import StreamRate

def set_sys_id(id):
    success=True

    try:
        rospy.wait_for_service("mavros/param/set", 10)
        paramSrv = rospy.ServiceProxy("mavros/param/set", ParamSet)
        param = paramSrv("SYSID_MYGCS", id)
        if not param.success:
            success=False
    except:
        success=False

    return success

def set_stream_rate(rate):
    success=True

    try:
        rospy.wait_for_service("mavros/set_stream_rate", 10)
        streamRateSrv = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)
        param = streamRateSrv(stream_id=0, message_rate=rate, on_off=(rate != 0))
        if not param:
            success=False
    except:
        success=False

    return success

if __name__ == "__main__":
    rospy.init_node("SYSID_MYGCS")
    rospy.init_node("set_stream_rate")

    sys_id = rospy.get_param("~SYSID_MYGCS", 1)
    rate = rospy.get_param("~set_stream_rate", 10)

    set_sys_id(sys_id)
    set_stream_rate(rate)
