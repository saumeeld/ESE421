
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
    
def perform_image_transformations(imgBGR):
    imgGray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
    
    ranges=(20,90,255) # Range of acceptable HSV values
    frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
    avgsize = 5 # number of pixels to average
    blursize = 25
    
    # Convert image to HSV
    imghsv = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
    # Blur image
    imghsv = cv2.GaussianBlur(imghsv,(blursize,blursize),0)
    
    return imgGray, imghsv


def get_CV_results(imgBGR, imgGray, imghsv,edges,heightOffset,widthOffset,height,width):
     # Print out dimensions of cropped image
    cropHeight,cropWidth = edges.shape
    # Filter detected edges for potential road edges
    minLineLength = 30
    maxLineGap = 30
    lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=20,
        minLineLength=minLineLength,maxLineGap=maxLineGap)

    # Empirically determined camera constants
    CAMERA_HEIGHT = 13
    CAMERA_FOCAL_LENGTH = 1098.88

    y2max = 0
    for x in range(0, len(lines)):
        for x1,y1,x2,y2 in lines[x]:
            x1 = x1 + widthOffset
            x2 = x2 + widthOffset
            y1 = y1 + heightOffset
            y2 = y2 + heightOffset
            slope_image = (x2 - x1) * 1.0/ (y2 - y1)
            x_intercept_computer_coords = x1 - (slope_image * y1)
            x_intercept_centered_coords = slope_image * (height/2) + x_intercept_computer_coords - (width/2)
            print("x1: {} x2: {} y1: {} y2: {}".format(x1, x2, y1, y2))
            if y2 > y2max:
                y2max = y2
                chosenLine = [x1,y1,x2,y2]
                # Calculate Xo and PsiR
                offset = slope_image * CAMERA_HEIGHT
                psi_r = math.degrees(math.atan(x_intercept_centered_coords/CAMERA_FOCAL_LENGTH))
            
            x1 = chosenLine[0]
            x2 = chosenLine[1]
            y1 = chosenLine[2]
            y2 = chosenLine[3]
            # Draw the line on the original image
            cv2.line(imgBGR,(x1,y1),(x2,y2),(255,0,0),4)
    
    return offset, psi_r, chosenLine

def init_camera(photoHeight):
    camera = picamera.PiCamera()
    photoHeight = 540
    camera.resolution = (16*photoHeight/9, photoHeight)
    return camera

def threshold_image(imgBGR,imgGray,imghsv):
    ranges=(0,10,100) # Range of acceptable HSV values
    frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
    avgsize = 5 # number of pixels to average
    blursize = 25
    height, width = imgBGR.shape[0:2]

    # Hard-coded thresholds for orange in hsv space
    lowerthres = np.array([0,100,100], dtype=np.uint8)
    upperthres = np.array([100,200,255], dtype=np.uint8)
    mask = cv2.inRange(imghsv, lowerthres, upperthres)

    # Perform Canny edge detection on selected region of interest
    # (Bottom right-hand corner of image)
    heightOffset = 0
    widthOffset = 0
    edges = cv2.Canny(mask[heightOffset:height, widthOffset:width], 100,255)
    return mask, edges,heightOffset,widthOffset,height,width

def capture_image(camera, camstore_filename):
    camera.capture(camstore_filename)
    imgBGR = cv2.imread(camstore_filename) 

# Plot debugging graphs
def plot_data(imgBGR, imghsv, mask, edges):
    b,g,r = cv2.split(imgBGR)       # get b,g,r
    imgRGB = cv2.merge([r,g,b])
    plt.imshow(imgRGB)
    plt.title('Detected Road Edge')

    fig = plt.figure()
    a = fig.add_subplot(2, 2, 1)
    # Original color image with detected line plotted on it
    imgplot = plt.imshow(imgRGB)
    a.set_title('RGB')
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

def debug_chosen_line(offset, psi_r, chosenLine, imgBGR):
    x1 = chosenLine[0]
    y1 = chosenLine[1]
    x2 = chosenLine[2]
    y2 = chosenLine[3]

    font                   = cv2.FONT_HERSHEY_SIMPLEX
    fontScale              = 1
    fontColor              = (255,255,255)
    lineType               = 2
    cv2.putText(imgBGR,"Estimated Offset: {0:.2f}".format(offset),(50,400),font,fontScale,fontColor,lineType)
    cv2.putText(imgBGR,"Estimated Psi_R: {0:.2f}".format(psi_r),(50,450),font,fontScale,fontColor,lineType)

    print("x1: {} x2: {} y1: {} y2: {}".format(x1, x2, y1, y2))
    print("Estimated Offset: {0:.2f}".format(offset))
    print("Estimated Psi_r: {0:.2f}".format(psi_r))
      
def main():
    CAMSTORE_FILENAME = 'piPicture.jpg'
    PHOTO_HEIGHT = 540
    # camera = init_camera(PHOTO_HEIGHT)
    # imgBGR = capture_image(camera, CAMSTORE_FILENAME)
    imgBGR = cv2.imread('/Users/adnanjafferjee/ESE421/PlatoonTestData/LinearOffset0".jpg')
    imgGray, imghsv = perform_image_transformations(imgBGR)
    edges,mask,heightOffset,widthOffset,height,width = threshold_image(imgBGR,imgGray,imghsv)
    offset, psi_r, chosenLine = get_CV_results(imgBGR,imgGray,imghsv,edges,heightOffset,widthOffset,height,width)
    debug_chosen_line(offset, psi_r, chosenLine, imgBGR)  
    plot_data(imgBGR, imghsv, mask, edges)
    return
    
if __name__ == "__main__":
    main()
    

# TODO: Feedback "expected" value of PsiR and Xo to compare with detected (low pass filter)

