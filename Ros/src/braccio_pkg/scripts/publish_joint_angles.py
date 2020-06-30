#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import MultiArrayLayout,MultiArrayDimension,Float32MultiArray


def main():
  angles = [0] *5
  pub = rospy.Publisher('joint_angles', Float32MultiArray, queue_size=10)
  rospy.init_node('publish_joint_angles')
  rate = rospy.Rate(0.3) # 2 Hz
  array = Float32MultiArray()

  while not rospy.is_shutdown():
    array.data = []
    angles[0] = random.randrange(0,135)
    angles[1] = random.randrange(-35,45)
    angles[2] = random.randrange(-55,70)
    angles[3] = random.randrange(-55,70)
    angles[4] = random.randrange(-55,70)
    angles[5] = random.randrange(-55,0)
    for i in range(6):
      array.data.append(angles[i])
    pub.publish(array);
    rate.sleep()

if __name__=="__main__" :
  try:
    main()
  except rospy.ROSInterruptException:
    pass
