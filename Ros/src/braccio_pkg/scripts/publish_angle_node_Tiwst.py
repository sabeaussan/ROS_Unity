#!/usr/bin/env python
import rospy
import random
from braccio_pkg.msg import TwistArray
#from sensor_msgs.msg import JointState
#from std_msgs.msg import MultiArrayLayout,MultiArrayDimension,UInt8MultiArray

# Twist ROS to Unity
# z = x
# x = y
# y = z

def main():
#angles = rospy.getParam("joint_angles")
  angles = [0] *5
  pub = rospy.Publisher('joint_angles', TwistArray, queue_size=10)
  rospy.init_node('publish_angle_node')
  rate = rospy.Rate(0.5) # 2 Hz
  array = TwistArray()
  angles[0] = Twist()
  angles[1] = Twist()
  angles[2] = Twist()
  angles[3] = Twist()
  angles[4] = Twist()

  while not rospy.is_shutdown():
    
    array.t_array = []
    angles[0].angular.z = random.randrange(1,179)
    angles[1].angular.z = random.randrange(16,164)
    angles[2].angular.z = random.randrange(1,179)
    angles[3].angular.z = random.randrange(1,179)
    angles[4].angular.z = random.randrange(1,179)

    for i in range(5):
      array.t_array.append(angles[i])
   
    pub.publish(array);
    rate.sleep()
    a+=1

if __name__=="__main__" :
  try:
    main()
  except rospy.ROSInterruptException:
    pass
