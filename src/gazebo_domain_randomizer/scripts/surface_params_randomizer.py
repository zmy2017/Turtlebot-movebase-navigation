#!/usr/bin/env python
import numpy as np
import rospy
from gazebo_ext_msgs.srv import GetCollisionNames, GetCollisionNamesRequest
from gazebo_ext_msgs.srv import SetSurfaceParams, SetSurfaceParamsRequest
from gazebo_domain_randomizer import utils

class SurfaceParamsRandomizer:
    def __init__(self, model_name, mu_range, poisson_ratio_range, gazebo_ns='/gazebo'):
        self._model_name = model_name
        self._mu_range = mu_range
        self._poisson_ratio_range = poisson_ratio_range
        res = utils.get_model_properties(self._model_name, gazebo_ns)
        link_names = ["%s::%s" % (model_name, b) for b in res.body_names]
        rospy.wait_for_service(gazebo_ns + '/get_collision_names')
        get_col_names = rospy.ServiceProxy(gazebo_ns + '/get_collision_names', GetCollisionNames)
        try:
            res = get_col_names(GetCollisionNamesRequest(link_names=link_names))
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        self._collisions = res.link_collision_names
        self._set_surface = rospy.ServiceProxy(gazebo_ns + '/set_surface_params', SetSurfaceParams)

    def callback(self, event):
        if len(self._collisions) == 0:
            return
        req = SetSurfaceParamsRequest()
        req.link_collision_name = np.random.choice(self._collisions)
        mu = np.random.uniform(*self._mu_range)
        req.mu1 = mu
        req.mu2 = mu
        req.mu_torsion = mu
        req.poisson_ratio = np.random.uniform(*self._poisson_ratio_range)
        try:
            res = self._set_surface(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Link visual properties randomizer')
    parser.add_argument('-m', '--model_name', type=str, default='', help='Model name.')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("surface_params_randomizer")
    color_range = {}
    mu_range = rospy.get_param("~mu_range", {'min': 0.0, 'max': 10.0})
    poisson_ratio_range = rospy.get_param("~poisson_ratio_range", {'min': 0.0, 'max': 1.0})
    rospy.loginfo("Load param mu_range: " + str(mu_range))
    rospy.loginfo("Load param poisson_ratio_range: " + str(mu_range))
    randomizer = SurfaceParamsRandomizer(args.model_name, [mu_range['min'], mu_range['max']],
                                         [poisson_ratio_range['min'], poisson_ratio_range['max']],
                                         gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()