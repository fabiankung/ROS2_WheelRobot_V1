# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 14:12:25 2021
Display a video from file or webcam connected to the computer.
@author: user
"""

import cv2

#video = cv2.VideoCapture("SampleVideo.mp4") # Open a video file.
video = cv2.VideoCapture(0) # Open a camera connected to the computer.

if not video.isOpened():            # Check if video source is available.
    print("Cannot open camera or file")
    exit()
    
while True:                         # This is same as while (1) in C.
    successFlag, img = video.read() # Read 1 image frame from video.
    
    if not successFlag:             # Check if image frame is correctly read.
        print("Can't receive frame (stream end?). Exiting ...")    
        break
        
    cv2.imshow("Video",img)         # Display the image frame.
    if cv2.waitKey(1) & 0xFF == ord('q'): # Note: built-in function ord() scans the 
          break                           # keyboard for 1 msec, returns the integer value                                            
                                          # of a unicode character. Here we compare user key
                                          # press with 'q' 
                                          
# When everything done, release the capture resources.
video.release()
cv2.destroyAllWindows()                                          