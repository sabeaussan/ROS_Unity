#!/usr/bin/python3
import cv2 as cv

img = cv.imread("balle_orange_2.jpg")
hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
mask = cv.inRange(hsv,(0,122,150),(25,194,246))
cv.imshow("orange",mask)
cv.waitKey()
cv.destroyAllWindows()
