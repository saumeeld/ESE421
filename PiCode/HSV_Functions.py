# Code to extract HSV convex hull courtesy of Ryan Kortvelesy 
def filterColor(self, img):
    ranges=(255,25,255)
    frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
    avgsize = 5 # number of pixels to average
    blursize = 25
    # Convert to HSV
    imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imghsv = cv2.GaussianBlur(imghsv,(blursize,blursize),0)
    # Get color of road
    frontpoint = (int(imghsv.shape[0]*(1-frontpoint[1])), int(imghsv.shape[1]*frontpoint[0]))
    frontregion = imghsv[frontpoint[0]-avgsize//2:frontpoint[0]+avgsize//2, frontpoint[1]-avgsize//2:frontpoint[1]+avgsize//2, 0:3]
    avgcolor = np.average(np.average(frontregion,axis=1),axis=0).flatten()
    # Color Threshold
    lowerthres = np.array([max(avgcolor[0]-ranges[0], 0), max(avgcolor[1]-ranges[1], 0), max(avgcolor[2]-ranges[2], 0)], dtype=np.uint8)
    upperthres = np.array([min(avgcolor[1]+ranges[0], 255), min(avgcolor[1]+ranges[1], 255), min(avgcolor[2]+ranges[2], 255)], dtype=np.uint8)
    mask = cv2.inRange(imghsv, lowerthres, upperthres)
    return mask

def hull(self,mask):
    # Get contours
    mask.dtype = np.uint8
    im2, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Get largest contour
    maxarea = 0
    maxcontour = contours[0]
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
    if area > maxarea:
        maxarea = area
        maxcontour = contour
    # Simplify contour
    epsilon = 0.06*self.res[0]
    simplecontour = cv2.approxPolyDP(maxcontour,epsilon,True)
    return simplecontour

# def segment(self,hull):
    # Find best segment in the hull
    minlength = self.res[0]/2.5
    best = 0
    segment = None
    segangle = 0
    for i in range(1,hull.shape[0]):
        # Check that line is not horizontal or vertical
        if (hull[i,0,0] != hull[i-1,0,0]) and (hull[i,0,1] != hull[i-1,0,1]):
        # Check that the line is above a threshold length
        length = np.linalg.norm(hull[i,0,:]-hull[i-1,0,:])
            if length > minlength:
            # Get angle
                angle = -atan2(hull[i,0,0] - hull[i-1,0,0], hull[i,0,1] - hull[i-1,0,1])
                if angle > pi/2:
                    angle -= pi
                elif angle < -pi/2:
                    angle += pi
                # Choose best segment according to quality function
                quality = cos(angle/2) * length
                if quality > best:
                best = quality
                segment = (hull[i-1,0,:], hull[i,0,:])
                segangle = angle
    # returns tuple of 2 element np arrays in img coordinates. transform to xy with self.img2xy(points)
    return segment


# Get contours
mask.dtype = np.uint8
im2, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Get largest contour
maxarea = 0
maxcontour = contours[0]
for i, contour in enumerate(contours):
    area = cv2.contourArea(contour)
if area > maxarea:
    maxarea = area
    maxcontour = contour
# Simplify contour
epsilon = 0.06*height
hull= cv2.approxPolyDP(maxcontour,epsilon,True)

minlength = height/2.5
best = 0
segment = None
segangle = 0
for i in range(1,hull.shape[0]):
    # Check that line is not horizontal or vertical
    if (hull[i,0,0] != hull[i-1,0,0]) and (hull[i,0,1] != hull[i-1,0,1]):
        # Check that the line is above a threshold length
        length = np.linalg.norm(hull[i,0,:]-hull[i-1,0,:])
        if length > minlength:
        # Get angle
            angle = math.atan2(hull[i,0,0] - hull[i-1,0,0], hull[i,0,1] - hull[i-1,0,1])
            if angle > math.pi/2:
                angle -= math.pi
            elif angle < -math.pi/2:
                angle += math.pi
            # Choose best segment according to quality function
            quality = math.cos(angle/2) * length
            if quality > best:
             best = quality
             segment = (hull[i-1,0,:], hull[i,0,:])
             segangle = angle

print(segment)
print(math.degrees(segangle))