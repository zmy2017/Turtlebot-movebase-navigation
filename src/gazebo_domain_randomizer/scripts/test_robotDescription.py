import rospy
import os
from gazebo_msgs.srv import  SpawnModel,SpawnModelRequest
from gazebo_msgs.srv import  DeleteModel,DeleteModelRequest
from std_srvs.srv import Empty

import roslaunch
# robot_description = rospy.get_param("robot_description")
# print(robot_description)
fullpath =os.path.join(os.path.dirname(__file__),"robot_description.urdf")
# with open(fullpath,'a') as file:
#     file.write(robot_description)

rospy.init_node('spawn_urdf_model')
pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
pause.wait_for_service()
pause()
spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model",SpawnModel)
spawn.wait_for_service()
spawn_req = SpawnModelRequest()
delete =rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
delete.wait_for_service()
delete_req = DeleteModelRequest()
delete_req.model_name = "mobile_base"
delete(delete_req)
# rospy.sleep(1)
with open (fullpath,'r') as xml_file:
    model_xml = xml_file.read().replace('\n','')
    model_xml = model_xml.replace("wheel_left_link_mu","3.4")
    model_xml = model_xml.replace("wheel_right_link_mu","4.5")
    model_xml = model_xml.replace("base_link_mass","10")
spawn_req.model_xml = model_xml 
spawn_req.model_name ="mobile_base"
# spawn_req.reference_frame ="map"
try:
    res = spawn(spawn_req)
    if not res.success:
        rospy.logwarn(res.status_message)
except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s" % e)

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# tracking_launch = roslaunch.parent.ROSLaunchParent(
#     uuid, ["/home/zmy/turtlebot/src/gazebo_domain_randomizer/launch/amcl_simple.launch"])

# tracking_launch.start()
unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
unpause.wait_for_service()
unpause()
