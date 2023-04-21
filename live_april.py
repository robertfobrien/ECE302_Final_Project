import cv2
import apriltag
import numpy as np
import a_star
import send_to_pi

# how to use: 
# 1: hook up camera
# 2: press run in vs code in the correct directory where this file is. You will see all detected tags. Hold A B or C to set it as your target
# 3: press q to get just outlines of objects and a path drawn. This is our motion plan
# 4: press q again to get a live view of the objects and the motion plan
# 5: q again to exit

# Set up camera and AprilTag detector
cap = cv2.VideoCapture(0)

options = apriltag.DetectorOptions()
options.families = "tag36h11" # "tag16h5"
detector = apriltag.Detector(options)

# Set all thje parameters

tag_size = 6  # cm
focal_length = 60  # pixels

car_tag_id = 0
A_tag_id = 1
B_id_tag = 2
C_id_tag = 3
target_tag_id = A_tag_id
destination_tag_id = 4

fx, fy, cx, cy = (216.46208287856132, 199.68569189689305, 840.6661141370689, 518.01214031649) #found from calibrate_camera.py
camera_params = (fx, fy, cx, cy)

read_from_image = True # read from image, TRUE; read from live camera, FALSE

jpg_fn = "test_1.jpeg"

# Define the size of the grid that we will run a* from 
grid_size = (20, 30)

# Initialize empty path for tag ID 0
car_path = []

# END SETTINGs

print ("Now in setup mode. Set up the car and blocks to begin. Press A, B or C to make them the target blocks. Press 'q' to find a path.")
print()
while True:
    # Capture image from camera

    if read_from_image == True:
        ret = True
        frame = cv2.imread(jpg_fn)
    else:
        ret, frame = cap.read()

    # Detect AprilTags in image
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(frame_gray)

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

        #fills it in with its color
        cv2.fillPoly(blank, [corners], color)
        
        # Calculate distance to tag (this doesnt work right now, could be useful later)
        tag_size_in_pixels = max(abs(corners[:,0]-corners[:,1]))  # assume square tag
        distance = (tag_size * focal_length) / tag_size_in_pixels
        

        # Put tag ID and distance on the image
        cv2.putText(frame, "{}".format(label), (corners[0][0], corners[0][1] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        #cv2.putText(blank, "{}".format(label), (corners[0][0], corners[0][1] - 20), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        #cv2.putText(frame, "Distance: {:.2f} meters".format(distance), (corners[0][0], corners[0][1] - 60), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    

    cv2.imshow("AprilTag Tracking: Setup", frame)
    

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Press 'a to set a as the target
    if cv2.waitKey(1) & 0xFF == ord('a'):
        target_tag_id = A_tag_id

    # Press 'b to set a as the target
    if cv2.waitKey(1) & 0xFF == ord('b'):
        target_tag_id = B_id_tag

    # Press 'c to set a as the target
    if cv2.waitKey(1) & 0xFF == ord('c'):
        target_tag_id = C_id_tag


#getting location of obstacles and car in pixel space (original image)
obstacle_locs_pixels = []
car_loc_pixels = []
target_loc_pixels = []
destination_loc_pixels = []
a_star_path_pixels_to_target = []
a_star_path_pixels_to_destination = []


#getting location of obstacles and car in grid space (less computing power required)
obstacle_locs_cell = []
car_loc_cell = []
target_loc_cell = []
destination_loc_cell = []
a_star_path_cell_to_target = []
grid = np.zeros(shape=grid_size, dtype=int)

# getting some variables: 
height, width, channels = blank.shape
cell_width = width // grid_size[1]
cell_height = height // grid_size[0]
cell_width_float = width / grid_size[1]
cell_height_float = height / grid_size[1]

def pixels_to_cell(pixel_coord):
    return (pixel_coord[1] // cell_height, pixel_coord[0] // cell_width)

def cells_to_pixels(cell_coord):
    return(cell[1] * (cell_width+1), cell[0] * (cell_height+1))

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
        car_loc_cell = pixels_to_cell(center)
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
        # If so, set the entire cell to green
        if np.any(cell[:,:,1] == 255):
            blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width] = (0, 255, 0)

        #check if any pixel is red
        if np.any(cell[:,:,2] == 255):
            # If so, set the entire cell to red
           blank[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width] = (0, 0, 255)  

            #sets obstacles in the grid, or 'maze'
        #    print(i, j)
           grid[i:i+1,  j:j+1] = 1

#prints the full grid: 
#with np.printoptions(threshold=np.inf):
#    print(grid)

#RUNS ASTAR ON "grid" matrix with car_loc_cell, target_loc_cell, obstacle_locs_cell
a_star_path_cell_to_target = a_star.astar(maze=grid, start=car_loc_cell, end=target_loc_cell)
a_star_path_cell_to_destination = a_star.astar(maze=grid, start=target_loc_cell, end=destination_loc_cell)

# converts cell coordinates into pixel coordinates
for cell in a_star_path_cell_to_target:
    a_star_path_pixels_to_target.append( cells_to_pixels(cell) )
for cell in a_star_path_cell_to_destination:
    a_star_path_pixels_to_destination.append( cells_to_pixels(cell) )

# add path to the photo in red
for i in range(len(a_star_path_pixels_to_target)-1):
                cv2.line(blank, a_star_path_pixels_to_target[i], a_star_path_pixels_to_target[i+1], (0, 0, 255), 2)

# add path to the photo in blue
for i in range(len(a_star_path_pixels_to_destination)-1):
                cv2.line(blank, a_star_path_pixels_to_destination[i], a_star_path_pixels_to_destination[i+1], (0, 255, 0), 2)

print ("Now showing the A * algoritm. This is the path the car will follow. Press 'q' to go back to live feed with the path overlayed.")
print()

while True:
    cv2.imshow("AprilTag Tracking: Obstacles and motion plan", blank)
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #print(car_path)
        break

print("Now back in live feed. Press 'q' again and the car will begin its route:")
print()

while True:
    # Capture image from camera

    if read_from_image == True:
        ret = True
        frame = cv2.imread(jpg_fn)
    else:
        ret, frame = cap.read()

    # Detect AprilTags in image
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(frame_gray)

    	
    blank = np.zeros((frame.shape[0] , frame.shape[1],3), np.uint8)

    
    # Draw bounding boxes around detected tags and calculate distance
    for tag in tags:
        # Get tag ID and corner coordinates
        tag_id = tag.tag_id
        corners = tag.corners.astype(int)
        tag_pose = detector.detection_pose(detection=tag, camera_params=camera_params, tag_size=30, z_sign=1)
        #print(tag_pose[0])
        #print("")

         # Draw bounding box with color based on tag ID
        if tag_id == car_tag_id:
            color = (0, 255, 0)  #blue
            label = "Car"
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
        
        #cv2.polylines(frame, [corners], True, color, 3)
        #print(tag.center.astype(int))
        #print("")
        cv2.polylines(blank, [corners], True, color, 3)
        
        # Calculate distance to tag
        tag_size_in_pixels = max(abs(corners[:,0]-corners[:,1]))  # assume square tag
        distance = (tag_size * focal_length) / tag_size_in_pixels
        

        # Put tag ID and distance on the image
        #cv2.putText(frame, "{}".format(label), (corners[0][0], corners[0][1] - 20), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        #cv2.putText(blank, "{}".format(label), (corners[0][0], corners[0][1] - 20), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        #cv2.putText(frame, "Distance: {:.2f} meters".format(distance), (corners[0][0], corners[0][1] - 60), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Update path for tag ID 
        if tag_id == car_tag_id:
            center = tag.center.astype(int)
            car_path.append(center)
            for i in range(len(car_path)-1):
                cv2.line(frame, tuple(car_path[i]), tuple(car_path[i+1]), (255, 0, 0), 2)
                cv2.line(blank, tuple(car_path[i]), tuple(car_path[i+1]), (255, 0, 0), 2)
        
        #draw start --> target on window
        for i in range(len(a_star_path_pixels_to_target)-1):
                cv2.line(frame, tuple(a_star_path_pixels_to_target[i]), tuple(a_star_path_pixels_to_target[i+1]), (0, 0, 255), 2)

        #draw target --> dest on window
        for i in range(len(a_star_path_pixels_to_destination)-1):
                cv2.line(frame, tuple(a_star_path_pixels_to_destination[i]), tuple(a_star_path_pixels_to_destination[i+1]), (0, 255, 0), 2)


    cv2.imshow("AprilTag Tracking Live", frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# now we want some sort of PID controller to follow the line

cap.release()
cv2.destroyAllWindows()
