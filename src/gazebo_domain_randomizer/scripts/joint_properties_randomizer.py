#!/usr/bin/env python
import numpy as np
import rospy
from gazebo_msgs.msg import ODEJointProperties
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from gazebo_msgs.srv import SetJointProperties, SetJointPropertiesRequest
import utils

class JointPropertiesRandomizer:
    def __init__(self, model_name, damping_range,joint_name=None,gazebo_ns='/gazebo'):
        self._model_name = model_name
        self._damping_range = damping_range
        res = utils.get_model_properties(self._model_name, gazebo_ns)
        self._joint_names = ["%s::%s" % (model_name, b) for b in res.joint_names]
        self._get_joint_prop = rospy.ServiceProxy(gazebo_ns + '/get_joint_properties', GetJointProperties)
        self._set_joint_prop = rospy.ServiceProxy(gazebo_ns + '/set_joint_properties', SetJointProperties)
        self._joint_name=joint_name

    def callback(self,event='timer'):
        req = SetJointPropertiesRequest()
        if len(self._joint_names) == 0:
            return
        if self._joint_name==None:
            rospy.logerr("Joint name is None" )
            self._joint_name = np.random.choice(self._joint_names)
        props = ODEJointProperties()
        props.damping = [np.random.uniform(*self._damping_range)]
        req.joint_name = self._joint_name
        req.ode_joint_config = props
        try:
            res = self._set_joint_prop(req)
            rospy.loginfo("Joint name:%s "%req.joint_name)
            rospy.loginfo("damping:%s "%props.damping)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Link properties randomizer')
    parser.add_argument('-m', '--model_name', type=str, default='mobile_base', help='Model name.')
    parser.add_argument('-j', '--joint_name', type=str, default='wheel_left_joint', help='Joint name.')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("joint_properties_randomizer")
    damping_range = rospy.get_param('~damping_range', {'min': 0.0, 'max': 2.0})
    rospy.loginfo("Load param damping_range: " + str(damping_range))
    randomizer = JointPropertiesRandomizer(args.model_name, [damping_range['min'], damping_range['max']],joint_name=args.joint_name,
                                           gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()