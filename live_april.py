# live april
# R. O'Brien May 2023
import cv2
import apriltag
from phidl import Path, CrossSection, Device
import phidl.path as pp
import numpy as np
import a_star
import send_to_pi
import line_follow
import serial
import math

# Set up camera
cap = cv2.VideoCapture(0)

#set up april tag detector
options = apriltag.DetectorOptions()
options.families = "tag36h11" # "tag16h5"
detector = apriltag.Detector(options)

# Set all the parameters
tag_size = 6  # cm
focal_length = 60  # pixels
car_tag_id = 0 
A_tag_id = 8
B_id_tag = 2
C_id_tag = 4
target_tag_id = A_tag_id
destination_tag_id = 4
car_corners = () 
car_speed = 55 # speed is in duty cycle. 100 is min, 0 is max
pid = line_follow.pid_controller(6.6, 4.4, 8.8, 0.2, 0, 0)
fx, fy, cx, cy = (216.46208287856132, 199.68569189689305, 840.6661141370689, 518.01214031649) #found from calibrate_camera.py
camera_params = (fx, fy, cx, cy)
read_from_image = True # True to read from image; False to read from live camera
draw_path = False #draw your own path for the car to follow
jpg_fn = "calibration_3.jpg" # image filename
grid_size = (40, 30) # Define the size of the grid that we will run a* from 
car_path = [] # Initialize empty path for tag ID 0
drawing = False # true if mouse is pressed
pt1_x , pt1_y = None , None
in_front_of_car = ()

try:
    ser = serial.Serial(port = '/dev/tty.usbserial-DN062958', baudrate=115200,timeout=None) # sets up seial port
    send_to_pi.send_to_pi(ser,(str)("stop"))
    print("Serial found.")
except:
    print("ERROR!: no serial port found")

###############################################################################################################################################

print ("Now in setup mode. Set up the car and blocks to begin. Press A, B or C to make them the target blocks. Press 'q' to find a path.")
print()
while True:
    # Capture image from camera

    if read_from_image == True:
        ret = True
        frame = cv2.imread(jpg_fn)
        #overlay_fn = "track-1.png"
        #frame = line_follow.combine_images(jpg_fn, overlay_fn)
    else:
        ret, frame = cap.read()
        send_to_pi.send_to_pi(ser,(str)("stop"))
        print("sent stop...")

    # Detect AprilTags in image
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(frame_gray)

    # create an all black copy of the image
    blank = np.zeros((frame.shape[0] , frame.shape[1],3), np.uint8)
    
    # Draw bounding boxes around detected tags and calculate distance
    for tag in tags:
        # Get tag ID and corner coordinates
        tag_id = tag.tag_id
        corners = tag.corners.astype(int)
        # pose gives us direction and size data etc. could be useful later
        tag_pose = detector.detection_pose(detection=tag, camera_params=camera_params, tag_size=30, z_sign=1)

        # Draw bounding box with color based on tag ID
        if tag_id == car_tag_id:
            color = (0, 255, 0)  #green
            label = "Car"
            in_front_of_car = (tag.center.astype(int)[0] + 100, tag.center.astype(int)[1])
        elif tag_id == A_tag_id:
            color = (0, 0, 255)  # Red
            label = "A"
        elif tag_id is B_id_tag:
            color = (0, 0, 255)  # Red
            label = "B"
        elif tag_id is C_id_tag:
            color = (0, 0, 255)  # Red
            label = "C"
        elif tag_id is destination_tag_id:
            color = (0, 255, 0)  # green
            label = "Destination"
        else:
            color = color = (0, 0, 255)  # Red
            label = "obstacle"

        #overwrites whatever one is the target box to make it blue
        if tag_id is target_tag_id:
            color = (255, 0, 0) 
        
        #outlines the tag
        cv2.polylines(frame, [corners], True, color, 3)
        cv2.polylines(blank, [corners], True, color, 3)

        #fills in tag with its respective color
        cv2.fillPoly(blank, [corners], color)
        
        # Calculate distance to tag (non working)
        tag_size_in_pixels = max(abs(corners[:,0]-corners[:,1]))  # assume square tag
        distance = (tag_size * focal_length) / tag_size_in_pixels 

        # Put tag ID and distance on the image
        cv2.putText(frame, "{}".format(label), (corners[0][0], corners[0][1] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # shows frame
    cv2.imshow("AprilTag Tracking", frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("running A* and smoothing algorith...\n")
        break

    # Press 'a' to set a as the target
    if cv2.waitKey(1) & 0xFF == ord('a'):
        target_tag_id = A_tag_id

    # Press 'b' to set b as the target
    if cv2.waitKey(1) & 0xFF == ord('b'):
        target_tag_id = B_id_tag

    # Press 'c' to set c as the target
    if cv2.waitKey(1) & 0xFF == ord('c'):
        target_tag_id = C_id_tag

#initialize variables

# Getting location of obstacles and car in pixel space (original image)
obstacle_locs_pixels = []
car_loc_pixels = []
target_loc_pixels = []
destination_loc_pixels = []
a_star_path_pixels_to_target = []
a_star_path_pixels_to_destination = []


#Getting location of obstacles and car in grid space (less computing power required)
obstacle_locs_cell = []
car_loc_cell = []
target_loc_cell = []
destination_loc_cell = []
a_star_path_cell_to_target = []
grid = np.zeros(shape=grid_size, dtype=int)
center_helper = []

height, width, channels = blank.shape
cell_width = width // grid_size[1]
cell_height = height // grid_size[0]
cell_width_float = width / grid_size[1]
cell_height_float = height / grid_size[1]

# converts pixel to cell coordinates
def pixels_to_cell(pixel_coord):
    return (pixel_coord[1] // cell_height, pixel_coord[0] // cell_width)

# converts cell to pixel coordinates
def cells_to_pixels(cell_coord):
    return(cell[1] * (cell_width+1), cell[0] * (cell_height+1))

# converts pixel coordinates of target, destination, and car to cell locations
for tag in tags:
    center = tag.center.astype(int)
    if tag.tag_id is destination_tag_id:
        destination_loc_pixels = center
        destination_loc_cell = pixels_to_cell(center)
    elif tag.tag_id is target_tag_id:
        target_loc_pixels = center
        target_loc_cell = pixels_to_cell(center)
    elif tag.tag_id is car_tag_id:
        car_loc_pixels = center
        car_loc_cell = pixels_to_cell( ( center[0] + 100 , center[1] ) )
        center_helper = pixels_to_cell( center )
    else:
        obstacle_locs_pixels.append( center )
        obstacle_locs_cell.append( pixels_to_cell(center) )

# VISUALIZE GRID
#first seperate the image into a grid:
for i in range(grid_size[0]):
    for j in range(grid_size[1]):

        # Get the pixels in the current grid cell
        cell = blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width]

        #check if any pixel is blue
        if np.any(cell[:,:,0] == 255):
            # If so, set the entire cell to blue
            blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width] = (255, 0, 0) 
        
        # Check if any pixel in the cell is green
        if np.any(cell[:,:,1] == 255):
            # If so, set the entire cell to green
            blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width] = (0, 255, 0)

        #check if any pixel is red
        if np.any(cell[:,:,2] == 255):
            # If so, set the entire cell to red
            blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width] = (0, 0, 255)  

            #sets obstacles in the grid, or 'maze', to TRUE. This is representing that an obstacle is in that position
            #X+ 
            try:
               #try to fillq in conservatively
               grid[i:i+1,  j] = 1
               #try to fill in an expanded boundary
               grid[i:i+2,  j] = 1

               grid[i:i+3,  j] = 1

               grid[i:i+4,  j] = 1

               #grid[i:i+5,  j] = 1
            except:
                print("Expanded boundary failed")
                pass
            #X- 
            try:
               #try to fill in conservatively
               grid[i-1:i,  j] = 1
               #try to fill in an expanded boundary
               grid[i-2:i,  j] = 1

               grid[i-2:i,  j] = 1
               
               grid[i-3:i,  j] = 1

               #grid[i-4:i,  j] = 1
            except:
                print("Expanded boundary failed")
                pass
            #Y+ 
            try:
               #try to fill in conservatively
               grid[i,  j:j+1] = 1
               #try to fill in an expanded boundary
               grid[i,  j:j+2] = 1

               grid[i,  j:j+3] = 1
               
               grid[i,  j:j+4] = 1

               grid[i,  j:j+5] = 1

               grid[i,  j:j+6] = 1
            except:
                print("Expanded boundary failed")
                pass
            #Y- 
            try:
               #try to fill in conservatively
               grid[i,  j-1:j] = 1
               #try to fill in an expanded boundary
               grid[i,  j-2:j] = 1

               grid[i,  j-3:j] = 1

               grid[i,  j-4:j] = 1

               #grid[i,  j-5:j] = 1
            except:
                print("Expanded boundary failed")
                pass
        
        #artificially sets the outer borders to obstacles
        #if i is grid_size[0]-1 or i == 0:
        #    grid[i,  j] = 1
        #if j is grid_size[1]-1 or j == 0:
        #    grid[i,  j] = 1

#prints the full grid: 
def print_grid():
    with np.printoptions(threshold=np.inf):
        print(grid)

#print_grid()

#RUNS ASTAR ON "grid" matrix with car_loc_cell, target_loc_cell, obstacle_locs_cell
a_star_path_cell_to_target = a_star.astar(maze=grid, start=car_loc_cell, end=target_loc_cell, allow_diagonal_movement=True)


# converts cell coordinates back into pixel coordinates
for cell in a_star_path_cell_to_target:
    a_star_path_pixels_to_target.append( cells_to_pixels(cell) )

temp = []

#this is to make sure the line goes through the center
temp.append(car_loc_pixels)
#temp.append((car_loc_pixels[0] + 20, car_loc_pixels[1]))

for i in range(0, len(a_star_path_pixels_to_target)-2, 5):
    temp.append(a_star_path_pixels_to_target[i])
# add the end point
temp.append(a_star_path_pixels_to_target[-1])
temp[1] = in_front_of_car # might want to take this out
a_star_path_pixels_to_target = temp



rad = 1 #radius starts at 1
while True:
    try:
        P = pp.smooth(
            points = a_star_path_pixels_to_target,
            radius = rad,
            corner_fun = pp.euler, # Alternatively, use pp.arc
            use_eff = False,
            )
    except:
        print("p not exist")
        break

    if rad > 150:
        break

    rad = rad + 1
print("Max radius is found to be: " ,rad, "\n")

a_star_path_pixels_to_target = P.points

# add the intended path to the frame in pure RGB RED
for i in range(len(a_star_path_pixels_to_target)-1):
                cv2.line(blank, ((int)(a_star_path_pixels_to_target[i][0]), (int)(a_star_path_pixels_to_target[i][1])), ((int)(a_star_path_pixels_to_target[i+1][0]), (int)(a_star_path_pixels_to_target[i+1][1])), (0, 0, 255), 2)

########################################################################################################################

print ("Now showing the A * algoritm. This is the path the car will follow. Press 'q' to go back to live feed with the path overlayed.")
print()

drawn = []

# mouse callback function
def line_drawing(event,x,y,flags,param):
    global pt1_x,pt1_y,drawing

    if event==cv2.EVENT_LBUTTONDOWN:
        drawing=True
        pt1_x,pt1_y=x,y
        drawn.append((x,y))
    elif event==cv2.EVENT_MOUSEMOVE:
        if drawing==True:
            cv2.line(blank,(pt1_x,pt1_y),(x,y),color=(0,0,255),thickness=1)
            cv2.line(frame,(pt1_x,pt1_y),(x,y),color=(0,0,255),thickness=1)
            drawn.append((x,y))
            pt1_x,pt1_y=x,y
    elif event==cv2.EVENT_LBUTTONUP:
        drawing=False
        cv2.line(blank,(pt1_x,pt1_y),(x,y),color=(0,0,255),thickness=1)
        cv2.line(frame,(pt1_x,pt1_y),(x,y),color=(0,0,255),thickness=1)

cv2.namedWindow('AprilTag Tracking')
cv2.setMouseCallback('AprilTag Tracking',line_drawing)    

while True:
    
    #show the converted cell version of the image, including the motion plan and the obstacles to avoid
    #cv2.imshow("AprilTag Tracking", blank)
    if draw_path:  
        cv2.putText(frame,  "Draw path for the car to follow!", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("AprilTag Tracking", frame)
    else:
        cv2.imshow("AprilTag Tracking", blank)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #print(car_path)
        break

########################################################################################################################

# add the intended path to the frame in pure RGB RED
    
print("Now back in live feed. Press 'q' again and the car will begin its route:")
print()


# sets the motor speed to a nonzero value we set at the top of this file
# *NOTE remember its inverse, so 10 = 90% duty cycel, and 90 = 10% duty cycle
try:
    send_to_pi.send_to_pi(ser,"speed:"+(str)(car_speed))
    print("speed:"+(str)(car_speed))
except:
    pass

#initialize variables for running PID
target_center = []
car_center = [0,0]
car_speed_vector = [0,0]
old_car_center = [0,0]
car_speed_error = 0
ended = False

while True:

    if read_from_image == True:
        #read from image
        ret = True
        frame = cv2.imread(jpg_fn)
    else:
        # Capture image from camera
        ret, frame = cap.read()

    # Detect AprilTags in image
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(frame_gray)
    #make a copy of the image
    blank = np.zeros((frame.shape[0] , frame.shape[1],3), np.uint8)

    # Draw bounding boxes around detected tags and calculate distance
    for tag in tags:
        # Get tag ID and corner coordinates
        tag_id = tag.tag_id
        corners = tag.corners.astype(int)
        tag_pose = detector.detection_pose(detection=tag, camera_params=camera_params, tag_size=30, z_sign=1)

         # Draw bounding box with color based on tag ID
        if tag_id == car_tag_id:
            color = (0, 255, 0)  #blue
            label = "Car"
            
            # here we set up a very easy and simple speed control
            setpoint = 3 # in pixels / frame
            start_speed = 85 # in duty cycle out of 100 (0 is high, 100 is low)
            kp = 2
            
            car_center = tag.center.astype(int)
            car_speed_vector = (old_car_center[0] - car_center[0],old_car_center[1] - car_center[1])
            old_car_center = car_center

            car_speed_scalar = np.sqrt(car_speed_vector[0]**2 + car_speed_vector[1]**2)
            car_speed_error = car_speed_scalar - setpoint

            car_speed = start_speed + kp*car_speed_error;
            
            #sends over XBee to control car speed
            try:
                send_to_pi.send_to_pi(ser,"speed:"+(str)(car_speed))
                print("speed:"+(str)(car_speed))
            except:
                #print("Serial not found")
                pass
            #print("car speed: ", car_speed)
            #print("car speed error: ", car_speed_error)
            #print()
            
            #draws car speed error over car 
            cv2.putText(frame,  "Speed : %.1f" % car_speed_scalar, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame,  "Speed error: %.1f" % car_speed_error, (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        elif tag_id == A_tag_id:
            color = (0, 0, 255)  # Red
            label = "A"
        elif tag_id == B_id_tag:
            color = (0, 0, 255)  # Red
            label = "B"
        elif tag_id == C_id_tag:
            color = (0, 0, 255)  # Red
            label = "C"
        elif tag_id == destination_tag_id:
            color = (0, 255, 0)  # blue
            label = "Destination"
        else:
            color = color = (0, 0, 0)  # black
            label = "obstacle"

        #overwrites whatever one is the target box to make it green
        if tag_id == target_tag_id:
            color = (255, 0, 0)  # blue
            target_center = tag.center.astype(int)
        
        cv2.polylines(blank, [corners], True, color, 3)
        

        if draw_path:
            for i in range(len(drawn)-1):
                    cv2.line(frame, drawn[i], drawn[i+1], (0, 0, 255), 2)
        else:
            # draw the line we're following, start --> target, on the frame
            for i in range(len(a_star_path_pixels_to_target)-1):
                cv2.line(frame, ((int)(a_star_path_pixels_to_target[i][0]), (int)(a_star_path_pixels_to_target[i][1])), ((int)(a_star_path_pixels_to_target[i+1][0]), (int)(a_star_path_pixels_to_target[i+1][1])), (0, 0, 255), 2)
        
        # Update path for tag ID 
        if tag_id == car_tag_id:
            
            center = tag.center.astype(int)
            car_path.append(center)
            for i in range(len(car_path)-1):
                cv2.line(frame, tuple(car_path[i]), tuple(car_path[i+1]), (255, 0, 0), 2)
                cv2.line(blank, tuple(car_path[i]), tuple(car_path[i+1]), (255, 0, 0), 2)
            #USES LINEFOLLOW CLASs
            car_corners = tag.corners.astype(int)
            
    
    try:
        #puts the distance in pixels between the car and the final location
        cv2.putText(frame, "Dist: %.1f" % math.dist(car_center,target_center) , (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    except:
        break

    #Do PID update for steering:
    line_follow.get_car_to_path_distance(pid,car_corners, frame)
    #    cv2.line(frame, line_follow.get_middle_point(car_corners[0], car_corners[3]) , line_follow.get_middle_point(car_corners[1], car_corners[2]) , (0, 0, 255), 2)


    cv2.line(frame, car_corners[0],car_corners[1], (0, 0, 255), 2)
    op = pid.calculate_output()
    pid.update_output(op)
    cv2.putText(frame,  "Steering error: %.1f" % pid.error, (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame,  "Steering: %.1f" % op, (5, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    if ended:
        cv2.putText(frame,  "COMPLETE. CLAW MODE ACTIVATED.", (5, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # sends the steering direction to be sent to the pi
    try:
        send_to_pi.send_to_pi(ser,"steer:"+(str)(pid.output))
        print("steer:"+(str)(pid.output))
    except:
        #print("Serial not found")
        pass

    # the target and the car are less than X pixels away
    if math.dist(car_center,target_center) < 80 and ended is False:
        send_to_pi.send_to_pi(ser,(str)("claw"))
        print((str)("auto stop, and claw"))
        ended = True

    #shows the image: 
    cv2.imshow("AprilTag Tracking", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_to_pi.send_to_pi(ser,(str)("stop"))
        print((str)("stop"))
        break

######################################################################################################################


# Now completed path. Showing final image that the car drove, and saving that file:

print("\n\nShowing final image...")

while 1:
    send_to_pi.send_to_pi(ser,(str)("stop"))
    cv2.imshow("AprilTag Tracking", frame)
    photo_filename = f"final_frame.jpg"
    cv2.imwrite(photo_filename, frame)
    photo_filename = f"final_black.jpg"
    cv2.imwrite(photo_filename, blank)
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#Done
cap.release()
cv2.destroyAllWindows()
