#!/usr/bin/env python
import rospy
import random
from sensor_msgs.msg import JointState
from std_msgs.msg import MultiArrayLayout,MultiArrayDimension,UInt8MultiArray


def main():
#angles = rospy.getParam("joint_angles")
  angles = [0] *6
  pub = rospy.Publisher('joint_array', UInt8MultiArray, queue_size=10)
  rospy.init_node('publish_angle_node')
  rate = rospy.Rate(0.2) # 10hz
  a=0

  while not rospy.is_shutdown():
    array = UInt8MultiArray()
    array.data = []
    angles[0] = 90 #random.randrange(1,179)
    angles[1] = 90 #random.randrange(16,164)
    angles[2] = 90 #random.randrange(1,179)
    angles[3] = 90 #random.randrange(1,179)
    angles[4] = 90 #random.randrange(1,179)
    if a%2==0:
      angles[5] = 20 #random.randrange(11,72)
    else :
      angles[5] = 60 #random.randrange(11,72)
    for i in range(6):
      array.data.append(angles[i])
   
    pub.publish(array);
    rate.sleep()
    a+=1

if __name__=="__main__" :
  try:
    main()
  except rospy.ROSInterruptException:
    pass
