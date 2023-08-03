# import rospy
# from gazebo_msgs.srv import *


# MODEL_NAME = "mobile_base"

# def _get_model_properties_once(model_name, gazebo_ns="gazebo"):
#     rospy.wait_for_service(gazebo_ns + '/get_model_properties')
#     get_model_prop = rospy.ServiceProxy(gazebo_ns + '/get_model_properties', GetModelProperties)
#     try:
#         res = get_model_prop(GetModelPropertiesRequest(model_name=model_name))
#         if not res.success:
#             rospy.logwarn(res.status_message)
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)
#     return res

# def get_model_properties(model_name, gazebo_ns="gazebo", retry=5):
#     for _ in range(retry):
#         res = _get_model_properties_once(model_name, gazebo_ns)
#         if res.success:
#             break
#         rospy.sleep(0.5)
#     return res



# def getModelState():
#     rospy.wait_for_service("/gazebo/get_model_state")
#     get_state_service = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
#     model_req = GetModelStateRequest()
#     model_req.model_name = MODEL_NAME
#     try:
#         res = get_state_service(model_req)
#         if not res.success:
#             rospy.logwarn(res.status_message)
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)
#     return res 

# def goal_feedback(feedback):
#     # get exact pose from gazebo,not feedback
#     model_state =getModelState()
#     print(model_state.pose.position)