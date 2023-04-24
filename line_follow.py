# vortual line_follow
#made by robert o'brien

import math
import cv2

class pid_controller:
    """ 
    Class for the pid 
    Can be initialized like:
    pid = pid_controller(20, 0, 40, 0.01, 0, 0)
    """

    def __init__(self, setpoint, limMin, limMax, Kp, Ki, Kd):
        self.integrator = 0
        self.proportional = 0
        self.derivative = 0
        self.error = 0
        self.prevError = 0
        self.measurement = 0
        self.output = 2
        self.prevOutput = 0 
        self.setPoint = setpoint
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.limMax = limMax
        self.limMin = limMin

    def set_measurement(self, value):
        self.measurement = value

    def calculate_output(self):
        #calculate errors
        self.error = self.setPoint - self.measurement;

        #PROPORTIONAL
        self.proportional = self.Kp*self.error;
        
        # INTEGRATOR
        self.integrator = self.Ki*(self.integrator + self.error);
        
        #DIFFERENTIATOR
        self.derivative = self.Kd*(self.error - self.prevError);

        #OUTPUT CALCULATION
        self.output = self.setPoint - (self.proportional + self.integrator + self.derivative);
    
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

#takes in the corners of the car AprilTag, and the image with the line it should be following
#returns some value corresponding to the car in relation to the line
def get_car_to_path_distance(car_corners, image):

    # left middle part of car tag
    l = get_middle_point(car_corners[0],car_corners[3])
    # right middle part of car tag
    r = get_middle_point(car_corners[1],car_corners[2])
    dist = get_point_dist(l,r)
    p = [l[0], l[1]]

    #print("r:", r)
    #print("l:", l)
    #change in x and y pixel per step
    dx = ((r[0]-l[0]) / dist)
    dy = ((r[1]-l[1]) / dist)

    #print("dx:", dx)
    #print("dy:", dy)
    #print("dist:", dist)
    #print("image shape: ", image.shape)


    while True:
        # if the  pixel is pure red: 
        if (int)(image[p[1],p[0]][2]) == 255 and (int)(image[p[1],p[0]][1]) == 0 and (int)(image[p[1],p[0]][0]) == 0:
            #print("found red pixel")
            break

        if p[1] >= image.shape[0] - 5 or p[1] < 0 :
            #print("didnt find red line, went outside of image")
            break
            
        if p[0] > image.shape[1] - 5 or p[0] < 0:
            #print("didnt find red line, went outside of image")
            break

        #iterates through every pixel between the left and right parts of the tag
        # the 0.5 is a trick to to round to make sure we round to nearest integer
        p[0] = (int)(p[0] + dx +(0.5))
        p[1] = (int)(p[1] + dy + (0.5))
        #print("p:", p)

    return get_point_dist(l, p) - get_point_dist(r, p)



#gets the distance between two tags
def get_tag_dist(tag1, tag2):
    return math.dist(tag1.center, tag2.center)

#gets the distance between two points
def get_point_dist(point1, point2):
    return math.dist(point1, point2)

#test: 
#pid = pid_controller(20, 0, 40, 0.01, 0, 0)


