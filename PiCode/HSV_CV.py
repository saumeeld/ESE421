#!/usr/bin/env python2

# Code to detect Xo and PsiR from pictures of Penn Park roads taken from Pi Camera 
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
# import picamera

# UNCOMMENT ALL OF THIS IF YOU WANT TO ANALYZE PICTURES TAKEN BY THE PI
# # Set up the camera
# # default photo seems to be 1920 x 1080
# # half of that keeps things more manageable for on-screen debugging
# #
# camera = picamera.PiCamera()
# photoHeight = 540
# camera.resolution = (16*photoHeight/9, photoHeight)

# # Capture an image and read it back in
# # (Do this because picamera does not play nice with openCV?)
# camera.capture('piPicture.jpg')
# imgBGR = cv2.imread('piPicture.jpg')

# Read in image from directory of Penn Park images
# Change this to where the image 

# Read on
imgBGR = cv2.imread('/Users/adnanjafferjee/ESE421/PennParkImages/Picture 41.jpg')
imgGray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
height, width = imgGray.shape
ranges=(255,25,255) # Range of acceptable HSV values
frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
avgsize = 5 # number of pixels to average
blursize = 25

# Convert image to HSV
imghsv = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
# Blur image
imghsv = cv2.GaussianBlur(imghsv,(blursize,blursize),0)

## THIS CODE IS COURTESY OF RYAN KORTVELESY
# Get color of road
frontpoint = (int(imghsv.shape[0]*(1-frontpoint[1])), 
    int(imghsv.shape[1]*frontpoint[0]))
frontregion = imghsv[frontpoint[0]-avgsize//2:frontpoint[0]+avgsize//2, 
    frontpoint[1]-avgsize//2:frontpoint[1]+avgsize//2, 0:3]
avgcolor = np.average(np.average(frontregion,axis=1),axis=0).flatten()

# Create a mask of the image that separates all pixels that are within the roads
# color threshold
lowerthres = np.array([max(avgcolor[0]-ranges[0], 0),
    max(avgcolor[1]-ranges[1], 0), max(avgcolor[2]-ranges[2], 0)], dtype=np.uint8)
upperthres = np.array([min(avgcolor[1]+ranges[0], 255), 
    min(avgcolor[1]+ranges[1], 255), min(avgcolor[2]+ranges[2], 255)], dtype=np.uint8)
mask = cv2.inRange(imghsv, lowerthres, upperthres)
## END RYAN'S CODE

# Perform Canny edge detection on selected region of interest
# (Bottom right-hand corner of image)
heightOffset = height/2
widthOffset = width/2
edges = cv2.Canny(mask[heightOffset:height, widthOffset:width], 100,255)
cropHeight,cropWidth = edges.shape
print(cropHeight,cropWidth)
# Filter detected edges for potential road edges
minLineLength = 40
maxLineGap = 30
lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=20,
    minLineLength=minLineLength,maxLineGap=maxLineGap)

CAMERA_HEIGHT = 13
CAMERA_FOCAL_LENGTH = 1098.88

# WIP Code that will select the "most likely line" based on heuristics
# y2max = 0
# chosenLine = []
# for x in range(0, len(lines)):
#     for x1,y1,x2,y2 in lines[x]:
#         slope_image = (x2 - x1) * 1.0/ (y2 - y1)
#         if slope_image < 0:
#             if y2 > y2max:
#                 y2max = y2
#                 chosenLine = lines[x]

# x1 =chosenLine[0]
# x2 =chosenLine[1]
# y1 =chosenLine[3]
# y2 =chosenLine[4]

for x in range(0, len(lines)):
    for x1,y1,x2,y2 in lines[x]:
        # Draw the line on the original image
        cv2.line(imgBGR,(widthOffset+x1,heightOffset+y1),(widthOffset+x2,heightOffset+y2),(255,0,0),2) 
        slope_image = (x2 - x1) * 1.0/ (y2 - y1)
        x_intercept_computer_coords = x1 - (slope_image * y1)
        x_intercept_centered_coords = slope_image * (height/2) + x_intercept_computer_coords - (width/2)
        
        offset = slope_image * CAMERA_HEIGHT
        psi_r = math.degrees(math.atan(x_intercept_centered_coords/CAMERA_FOCAL_LENGTH))

print("Slope in image: {} Unshifted Intercept: {} Shifted Intercept: {} x1: {} x2: {} y1: {} y2: {}".format(slope_image, x_intercept_computer_coords, x_intercept_centered_coords, x1, x2, y1, y2))
print("Estimated Offset: {0:.2f}".format(offset))
print("Estimated Psi_r: {0:.2f}".format(psi_r))

# Plot debugging graphs
plt.imshow(imgBGR)
plt.title('Detected Road Edge')

fig = plt.figure()
a = fig.add_subplot(2, 2, 1)
# Original color image with detected line plotted on it
imgplot = plt.imshow(imgBGR)
a.set_title('RGB ')
a = fig.add_subplot(2, 2, 2)
# Image in HSV space
imgplot = plt.imshow(imghsv)
a.set_title('HSV')
a = fig.add_subplot(2, 2, 3)
# Thresholded HSV Mask
imgplot = plt.imshow(mask)
a.set_title('HSV Thresholded')
a = fig.add_subplot(2, 2, 4)
# Edges detected within region of interest of the thresholded mask
imgplot = plt.imshow(edges)
a.set_title('HSV Edges')
plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
                    wspace=0.35)
plt.show(block=True)

# TODO: Feedback "expected" value of PsiR and Xo to compare with detected (low pass filter)