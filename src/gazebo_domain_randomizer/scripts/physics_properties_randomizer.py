#!/usr/bin/env python
import copy
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest

class PhysicsPropertiesRandomizer:
    def __init__(self, gravity_range, gazebo_ns='/gazebo'):
        self._gravity_range = gravity_range
        rospy.wait_for_service(gazebo_ns + '/get_physics_properties')
        get_phys_prop = rospy.ServiceProxy(gazebo_ns + '/get_physics_properties', GetPhysicsProperties)
        try:
            res = get_phys_prop()
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        self._default_props = res
        self._set_phys_prop = rospy.ServiceProxy(gazebo_ns + '/set_physics_properties', SetPhysicsProperties)
    def callback(self, event):
        req = SetPhysicsPropertiesRequest()
        req.time_step = self._default_props.time_step
        req.max_update_rate = self._default_props.max_update_rate
        req.gravity = Vector3(*[np.random.uniform(self._gravity_range['x']['min'], self._gravity_range['x']['max']),
                                np.random.uniform(self._gravity_range['y']['min'], self._gravity_range['y']['max']),
                                np.random.uniform(self._gravity_range['z']['min'], self._gravity_range['z']['max'])])
        req.ode_config = copy.deepcopy(self._default_props.ode_config)
        try:
            res = self._set_phys_prop(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Physics properties randomizer')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("physics_properties_randomizer")
    gravity_range = {}
    gravity_range['x'] = rospy.get_param("~gravity_range/x", {'min': -0.1, 'max': 0.1})
    gravity_range['y'] = rospy.get_param("~gravity_range/y", {'min': -0.1, 'max': 0.1})
    gravity_range['z'] = rospy.get_param("~gravity_range/z", {'min': -5.0, 'max': 5.0})
    rospy.loginfo("Load param gravity_ratio_range: " + str(gravity_range))
    randomizer = PhysicsPropertiesRandomizer(gravity_range, gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()
