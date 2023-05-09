# virtual line_follow
# made by robert o'brien

import math
import cv2
import numpy as np

class pid_controller:
    """ 
    Class for the pid 
    Can be initialized like:
    pid = pid_controller(20, 0, 40, 0.01, 0, 0)
    """

    def __init__(self, middlePoint, limMin, limMax, Kp, Ki, Kd):
        self.integrator = 0
        self.proportional = 0
        self.derivative = 0
        self.error = 0
        self.prevError = 0
        self.measurement = 0
        self.output = 6.6
        self.prevOutput = 0 
        self.middlepoint = middlePoint
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.limMax = limMax
        self.limMin = limMin

    def set_measurement(self, value):
        self.measurement = value

    def calculate_output(self):
        #calculate errors
        self.error = self.measurement


        #PROPORTIONAL
        self.proportional = self.Kp*self.error

        #DERIVATIVE
        self.derivative = (self.error - self.prevError)*self.Kd

        #OUTPUT CALCULATION
        self.output = self.middlepoint + (self.proportional)
    
        #output clamping
        if((int)(self.output) > (int)(self.limMax)):
            self.output = self.limMax
        elif((int)(self.output) < (int)(self.limMin)):
            self.output = self.limMin

        #set for next iteration
        self.prevError = self.error;

        return self.output
    
    def update_output(self, value):
        self.output = value


#finds the middle pixel between two pixels
# returns a point in form: tuple(int, int)
def get_middle_point(point1, point2):
    return ((point1[0]+point2[0]) // 2, (point1[1]+point2[1]) // 2 )

# takes in the corners of the car AprilTag, and the image with the line it should be following
# returns a value corresponding to the car in relation to the line
def get_car_to_path_distance(pid, car_corners, image):
    #car_corners: 
        # 0 = top left
        # 1 = top right
        # 2 = bottom left
        # 3 = bottom right

    # front left part of the car
    fl = car_corners[0]
    # front right corner of the car
    fr = car_corners[1]
    #back left
    bl = car_corners[2]
    #back right
    br = car_corners[3]

    # choosing l and r to be an extended version of the front of the car, as to anticipate movements
    l_dy = fl[1] - bl[1]
    l_dx = fl[0] - bl[0]
    r_dy = fr[1] - br[1]
    r_dx = fr[1] - br[1]
    
    l = (fl[0] + l_dx, fl[1] + l_dy)
    r = (fr[0] + r_dx, fr[1] + r_dy)

    dist = get_point_dist(l,r)

    p = [l[0], l[1]]

    #change in x and y pixel per step
    #normalizes the vector between the two points
    dx = ((r[0]-l[0]) / dist)
    dy = ((r[1]-l[1]) / dist)
    #print("dx:", dx," dy:", dy)

    while True:
        #iterates through every pixel between the left and right parts of the tag
        # the 0.5 is a trick to to round to make sure we round to nearest integer
        if dx > 0:
            p[0] = (int)(p[0] + dx + (0.5))
        else:
            p[0] = (int)(p[0] + dx - (0.5))
        
        if dy > 0:
            p[1] = (int)(p[1] + dy + (0.5))
        else: 
            p[1] = (int)(p[1] + dy - (0.5))

        

        # if the  pixel is pure red: 
        #print("     image r:", (int)(image[p[1],p[0]][2]))
        if (int)(image[p[1],p[0]][2]) == 255 and (int)(image[p[1],p[0]][1]) == 0 and (int)(image[p[1],p[0]][0]) == 0:
            #print("found red pixel")
            #print("output: ", math.dist(p,l) - math.dist(p,r))
            pid.measurement = math.dist(p,l) - math.dist(p,r)

        if p[1] >= image.shape[0] - 5 or p[1] < 0 :
            #print("didnt find red line, went outside of image")
            break
            
        if p[0] > image.shape[1] - 5 or p[0] < 0:
            #print("didnt find red line, went outside of image")
            break



#gets the distance between two tags
def get_tag_dist(tag1, tag2):
    return math.dist(tag1.center, tag2.center)

#gets the distance between two points
def get_point_dist(point1, point2):
    return math.dist(point1, point2)

def combine_images(background_fn, overlay_fn):
    background = cv2.imread(background_fn)
    overlay = cv2.imread(overlay_fn, cv2.IMREAD_UNCHANGED)  # IMREAD_UNCHANGED => open image with the alpha channel

    height, width = overlay.shape[0], overlay.shape[1]
    print(height)
    print(width)
    for y in range(height-3):
        for x in range(width-3):
            overlay_color = overlay[y, x, :3]  # first three elements are color (RGB)
            overlay_alpha = overlay[y, x, 3] / 255  # 4th element is the alpha channel, convert from 0-255 to 0.0-1.0

            # get the color from the background image
            background_color = background[y, x]

            # combine the background color and the overlay color weighted by alpha
            composite_color = background_color * (1 - overlay_alpha) + overlay_color * overlay_alpha

            # update the background image in place
            background[y, x] = composite_color

    return background


