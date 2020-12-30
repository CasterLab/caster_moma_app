#!/usr/bin/env python
import rospy

# from black_maine_app.msg import DistributionAction
import geometry_msgs.msg
from std_srvs.srv import SetBool

class MqttListener:

  def __init__(self):

    self.position_x = rospy.get_param('amcl_init_pose/position_x', 0.0)
    self.position_y = rospy.get_param('amcl_init_pose/position_y', 0.0)
    self.orientation_z = rospy.get_param('amcl_init_pose/orientation_z', 0.0)
    self.orientation_w = rospy.get_param('amcl_init_pose/orientation_w', 1.0)

    self.pub_ = rospy.Publisher("/initialpose", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10) 
    self.twist_pub_ = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)

    rospy.Service('init_pose', SetBool, self.InitPose)

  def InitPose(self, req):
    amcl_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    amcl_pose.header.frame_id = 'map'
    # amcl_pose.header.stamp = rospy.Time.now()
    amcl_pose.pose.pose.position.x = self.position_x
    amcl_pose.pose.pose.position.y = self.position_y
    amcl_pose.pose.pose.position.z = 0.0
    amcl_pose.pose.pose.orientation.x = 0.0
    amcl_pose.pose.pose.orientation.y = 0.0
    amcl_pose.pose.pose.orientation.z = self.orientation_z
    amcl_pose.pose.pose.orientation.w = self.orientation_w
    # amcl_pose.pose.covariance[0] = 0.25
    # amcl_pose.pose.covariance[7] = 0.25
    # amcl_pose.pose.covariance[35] =  0.06853892326654787
    self.pub_.publish(amcl_pose)

    rospy.sleep(1.0)

    twist_msg = geometry_msgs.msg.Twist()
    twist_msg.angular.z = 0.5
    count = 20
    while not rospy.is_shutdown():
      self.twist_pub_.publish(twist_msg)
      rospy.sleep(0.1)
      if count >= 0:
        count = count-1
      else:
        break

    twist_msg.angular.z = -0.5
    count = 20
    while not rospy.is_shutdown():
      self.twist_pub_.publish(twist_msg)
      rospy.sleep(0.1)
      if count >= 0:
        count = count-1
      else:
        break

    twist_msg.angular.z = 0.0
    # self.twist_pub_.publish(twist_msg)

    rospy.loginfo('reset amcl pose done')

    return [True, "init"]

def main():
  # pub_ = rospy.Publisher("initialpose", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10) 
  rospy.init_node('amcl_init_pose', anonymous=True)
  ML = MqttListener()
  # for i in range(0, 100):
  # rospy.sleep(2.0)
  # while not rospy.is_shutdown():   
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
