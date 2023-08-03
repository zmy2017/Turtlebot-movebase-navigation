#!/usr/bin/env python
import numpy as np
import rospy
from gazebo_msgs.srv import GetLinkProperties, GetLinkPropertiesRequest
from gazebo_msgs.srv import SetLinkProperties, SetLinkPropertiesRequest
from gazebo_domain_randomizer import utils

class LinkPropertiesRandomizer:
    def __init__(self, model_name, mass_ratio_range, gazebo_ns='/gazebo'):
        self._model_name = model_name
        self._mass_ratio_range = mass_ratio_range
        res = utils.get_model_properties(self._model_name, gazebo_ns)
        link_names = ["%s::%s" % (model_name, b) for b in res.body_names]
        self._get_link_prop = rospy.ServiceProxy(gazebo_ns + '/get_link_properties', GetLinkProperties)
        self._set_link_prop = rospy.ServiceProxy(gazebo_ns + '/set_link_properties', SetLinkProperties)
        self._default_props = {}
        for l in link_names:
            try:
                self._default_props[l] = self._get_link_prop(GetLinkPropertiesRequest(link_name=l))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
    def callback(self, event):
        req = SetLinkPropertiesRequest()
        if len(self._default_props.keys()) == 0:
            return
        link_name = np.random.choice(self._default_props.keys())
        r_mass = np.random.uniform(*self._mass_ratio_range)
        req.link_name = link_name
        req.com = self._default_props[link_name].com
        req.gravity_mode = self._default_props[link_name].gravity_mode
        req.mass = self._default_props[link_name].mass * r_mass
        req.ixx = self._default_props[link_name].ixx
        req.ixy = self._default_props[link_name].ixy
        req.iyy = self._default_props[link_name].iyy
        req.iyz = self._default_props[link_name].iyz
        req.izz = self._default_props[link_name].izz
        try:
            res = self._set_link_prop(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Link properties randomizer')
    parser.add_argument('-m', '--model_name', type=str, default='', help='Model name.')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("link_properties_randomizer")
    mass_ratio_range = rospy.get_param('~mass_ratio_range', {'min': 0.8, 'max': 1.2})
    rospy.loginfo("Load param mass_ratio_range: " + str(mass_ratio_range))
    randomizer = LinkPropertiesRandomizer(args.model_name, [mass_ratio_range['min'], mass_ratio_range['max']],
                                          gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()
