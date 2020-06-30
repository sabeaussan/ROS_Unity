import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

# -- Id to find in the image
id_to_find = 6
marker_size = 2.45 # cm

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# Get the camera calibration parameters
path = "../camera/"
camera_matrix = np.loadtxt(path+"cam_mtx.txt",delimiter=",")
camera_distorsion = np.loadtxt(path+"disorsion_param.txt",delimiter=",")

print(camera_matrix)
print(camera_distorsion)

# flip 180 degrees matrix around the x axis
R_flip = np.zeros((3,3),dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0

# get image from webcam
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,380)

while True:

  # Read the camera frame
  # Ret est un boolean qui nous informe pour savoi rsi 
  ret, frame = cam.read()

  # Convert in gray scale
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
 
  # Find marker in the image
  corners,ids, _ = aruco.detectMarkers(image=gray, dictionary=ARUCO_DICT, parameters   =ARUCO_PARAMETERS, cameraMatrix = camera_matrix, distCoeff = camera_distorsion)

  if ids is not None and ids[0] == id_to_find :
    
    ret = aruco.estimatePoseSingleMarkers (corners,marker_size, camera_matrix,camera_distorsion)
    rot_vec,trans_vec = ret[0][0,0,:], ret[1][0,0,:]
    aruco.drawDetectedMarkers(frame,corners)
    aruco.drawAxis(frame,camera_matrix,camera_distorsion,rot_vec,trans_vec,10)
    str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(trans_vec[0], trans_vec[1], trans_vec[2])
    print(str_position)
    #cv2.putText(frame,str_position, (0,100), font,1,(0,255),2,cv2.LINE_AA)
  # Display the frame 
  cv2.imshow('frame',frame)
  
  # use q to quit
  key = cv2.waitKey(1) & 0xFF
  
  if key == ord('q'):
    cam.release()
    cv2.destroyAllWindows()
    break










