# import the necessary packages
from collections import deque
import numpy as np
import cv2
import time


# define the lower and upper boundaries of the "orange"
# ball in the HSV color space, then initialize the
# list of tracked points
orangeLower = (0,122,150)
orangeUpper = (25,194,246)

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,380)
# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
  # grab the current frame
  ret,frame = cam.read()
  # resize the frame, blur it, and convert it to the HSV
  # color space
  #frame = cv2.resize(frame, width=600)
  blurred = cv2.GaussianBlur(frame, (11, 11), 0)
  hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
  # construct a mask for the color "orange", then perform
  # a series of dilations and erosions to remove any small
  # blobs left in the mask
  mask = cv2.inRange(hsv, orangeLower, orangeUpper)
  mask = cv2.erode(mask, None, iterations=2)
  mask = cv2.dilate(mask, None, iterations=2)
  _,contours,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  # only proceed if at least one contour was found
  if len(contours) > 0:
  # find the largest contour in the mask, then use
  # it to compute the minimum enclosing circle and
  # centroid
    c = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    # only proceed if the radius meets a minimum size
    if radius > 10:
      # draw the circle and centroid on the frame,
      # then update the list of tracked points
      cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
      cv2.circle(frame, center, 5, (0, 0, 255), -1)

  # show the frame to our screen
  cv2.imshow("Frame", frame)
  key = cv2.waitKey(1) & 0xFF
  # if the 'q' key is pressed, stop the loop
  if key == ord("q"):
    break

cam.release()
cv2.destroyAllWindows()


