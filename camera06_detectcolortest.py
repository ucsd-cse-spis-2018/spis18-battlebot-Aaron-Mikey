# This program illustrates how to capture frames in a video stream and
# and how to do extract pixels of a specific color
# It uses openCV

import picamera
import picamera.array                                   # This needs to be imported explicitly
import cv2
import time
import numpy as np                                      



# Define the range colors to filter; these numbers represent HSV
# old lowerColorThreshold = np.array([120, 161, 100])
# new upperColorThreshold = np.array([255, 255, 255])
lowerColorThreshold = np.array([160, 53, 57])
upperColorThreshold = np.array([181, 255, 255])


# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))


# Allow the camera to warm up
time.sleep(0.1)


if __name__ == '__main__':
    try:
        for frame in camera.capture_continuous(rawframe, format = 'bgr', use_video_port = True):

            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)  
            # Create a numpy array representing the image
            image = frame.array     

            #-----------------------------------------------------
            # We will use numpy and OpenCV for image manipulations
            #-----------------------------------------------------

            # Convert for BGR to HSV color space, using openCV
            # The reason is that it is easier to extract colors in the HSV space
            # Note: the fact that we are using openCV is why the format for the camera.capture was chosen to be BGR
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only colors in a range
            # The colors in range are set to white (255), while the colors not in range are set to black (0)
            ourmask = cv2.inRange(image_hsv, lowerColorThreshold, upperColorThreshold)

                   
            # Bitwise AND of the mask and the original image
            image_masked = cv2.bitwise_and(image, image, mask = ourmask)

            #counts pixels in left, mid, and right pixels
            maskLeft=ourmask[0 : 480, 0 : 213]
            maskMid=ourmask[0 : 480, 213 : 426]
            maskRight=ourmask[0 : 480, 426 : 640]
            numPixLeft=cv2.countNonZero(maskLeft)
            numPixMid=cv2.countNonZero(maskMid)
            numPixRight=cv2.countNonZero(maskRight)
            print("Number of pixels in the color range on the left part of the image:", numPixLeft)
            print("Number of pixels in the color range in the center part of the image:", numPixMid)
            print("Number of pixels in the color range on the right part of the image:", numPixRight)
            maxWhite=max(numPixLeft,numPixMid)
            maxWhite=max(maxWhite,numPixRight)
            isZero=maxWhite==0
            nonZero= maxWhite!=0
            if maxWhite==numPixLeft: #and nonZero:
                print("It's on the left")
            
            elif maxWhite==numPixMid:
                print("It's in the middle")

            else:
                print("It's on the right")
            # Show the frames
            # The waitKey command is needed to force openCV to show the image
            cv2.imshow("Frame", image)
            cv2.imshow("Mask", ourmask)
            cv2.imshow("Masked image", image_masked)  
            cv2.waitKey(1)
            #time.sleep(2)



    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        cv2.destroyAllWindows()
        # Clean up the camera resources
        camera.close()
        
