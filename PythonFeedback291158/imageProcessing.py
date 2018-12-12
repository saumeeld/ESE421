import cv2
import numpy as np
import math
#

def filterColor(img,res,ranges=(255,30,255),ranges_rgb=(255,255,255)):
        # This code was adapted from the code posted on piazza a while back, and
        # also from this website: https://stackoverflow.com/questions/41879315/opencv-using-cv2-approxpolydp-correctly 
        
        blursize = 25
        refsize = 5

        refpoint = (0.5, 0.2)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
        #Convert to hsv (Hue, saturation, value) and threshold around saturation
        # (Using RGB gets messy)
        hsv = cv2.GaussianBlur(hsv,(blursize,blursize),0)

        
        # Find a reference area that determines the color of the road
        refpoint = (int(hsv.shape[0]*(1-refpoint[1])), int(hsv.shape[1]*refpoint[0]))
        refregion = hsv[refpoint[0]-refsize//2:refpoint[0]+refsize//2, refpoint[1]-refsize//2:refpoint[1]+refsize//2, 0:3]
        avgcolor = np.average(np.average(refregion,axis=1),axis=0).flatten()
        # Threshold the image based on that color
        lowerthres = np.array([max(avgcolor[0]-ranges[0], 0), max(avgcolor[1]-ranges[1], 0), max(avgcolor[2]-ranges[2], 0)], dtype=np.uint8)
        upperthres = np.array([min(avgcolor[1]+ranges[0], 255), min(avgcolor[1]+ranges[1], 255), min(avgcolor[2]+ranges[2], 255)], dtype=np.uint8)

        #################################################################
        # Get color of road in RGB (optional)
        refregion_rgb = img[refpoint[0]-refsize//2:refpoint[0]+refsize//2, refpoint[1]-refsize//2:refpoint[1]+refsize//2, 0:3]
        avgcolor_rgb = np.average(np.average(refregion_rgb,axis=1),axis=0).flatten()
        # Color Threshold
        lowerthres_rgb = np.array([max(avgcolor_rgb[0]-ranges_rgb[0], 0), max(avgcolor_rgb[1]-ranges_rgb[1], 0), max(avgcolor_rgb[2]-ranges_rgb[2], 0)], dtype=np.uint8)
        upperthres_rgb = np.array([min(avgcolor_rgb[1]+ranges_rgb[0], 255), min(avgcolor_rgb[1]+ranges_rgb[1], 255), min(avgcolor_rgb[2]+ranges_rgb[2], 255)], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lowerthres, upperthres)
        
        return mask

def findContours(self,res,mask):
        # Get contours

        mask.dtype = np.uint8
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Get largest contour
        maxarea = 0
        maxcontour = contours[0]
        for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > maxarea:
                        maxarea = area
                        maxcontour = contour
        # Simplify contour
        epsilon = .15*res[1]
        cnt = cv2.approxPolyDP(maxcontour,epsilon,True)
        return cnt

 

def chooseContour(hull, res):
        
        
        # Find best segment in the hull
        minlength = res[1]/2
        best = 0
        segment = None
        segangle = 0
        angle_threshold = 2 *math.pi/180
        for i in range(1,hull.shape[0]):
        # Check that line is not horizontal or vertical
                if (hull[i,0,0] != hull[i-1,0,0]) and (hull[i,0,1] != hull[i-1,0,1]):
        # Check that the line is above a threshold length
                        length = np.linalg.norm(hull[i,0,:]-hull[i-1,0,:])
                        
                        if length > minlength:
                        # Get angle
                                
                                angle = -math.atan2(hull[i,0,0] - hull[i-1,0,0], hull[i,0,1] - hull[i-1,0,1])
                                
                                if angle > math.pi/2:
                                        angle -= math.pi
                                elif angle < -math.pi/2:
                                        angle += math.pi

                                
                                
        # Choose best segment according to quality function
                                
                        quality = math.cos(angle/1.2)*length
                        #if (abs(angle) < angle_threshold):
                        #               quality = 0 #throw out very vertical roads
                        if quality > best:
                                best = quality
                                segment = (hull[i-1,0,:], hull[i,0,:])
                                segangle = angle
                                
        # returns tuple of 2 element np arrays in img coordinates. transform to xy with self.img2xy(points)
        return segment

def findBeta_Offset(roadEdge, res, imgOrigin):
        pt1 = roadEdge[0]
        pt2 = roadEdge[1]

        h = .013
        f = .00304
        fov = 62.2 * (math.pi/180)
        imagePlaneWidth = f*2*math.tan(fov/2)
        x1 = (pt1[0]-res[0]/2)*(imagePlaneWidth/res[0])
        x2 = (pt2[0]-res[0]/2)*(imagePlaneWidth/res[0])
        y1 = (pt1[1])*(imagePlaneWidth/res[0])
        y2 = (pt2[1])*(imagePlaneWidth/res[0])

        if (y1 == 0):
                y1 = 1*(imagePlaneWidth/res[0])
        if (y2 == 0):
                y2 = 1*(imagePlaneWidth/res[0])


        Z1 = f*h/y1
        X1 = Z1*x1/f

        Z2 = f*h/y2
        X2 = Z2*x2/f

        #n = (x2-x1)/(y2-y1)
        #C = x1-n*y1

        #X_0 = C/f
        #beta = math.atan(C/(f*n*h))*(180/math.pi)

        n = (X2-X1)/(Z2-Z1)
        X_0 = X1-n*Z1


        beta = -math.atan((X2-X1)/(Z2-Z1))*(180/math.pi)
        
        return (beta, X_0)

def processImage(img_filename):
        
        img = cv2.imread(img_filename) 
        height = np.size(img, 0)
        crop_height = int(math.floor(height/2))
        width = np.size(img, 1)
        res = [width, crop_height]
        imgOrigin = [int(math.floor(width/2)), 0]
        ranges = (255,30,250)
        ranges_rgb = (15,15,15)
        
        img =  img[res[1]:height, 0:res[0]]
        mask = filterColor(img,res,ranges,ranges_rgb)
        cnt = findContours(img,res,mask)
        roadEdge = chooseContour(cnt, res)
        pt1 = roadEdge[0]
        pt2 = roadEdge[1]
        ## If pi keeps crashing, comment these lines out:
        #####################################################
        cv2.line(img, (pt1[0],pt1[1]), (pt2[0],pt2[1]), (0,255,0), 10)
        cv2.imshow("Processed Image", img)
        k = cv2.waitKey(40)
        #####################################################
        beta_offset = findBeta_Offset(roadEdge,res,imgOrigin)
        return beta_offset

