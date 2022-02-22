import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage.morphology import binary_dilation
import rospy
import sys

rect_list = np.array([[0, -9, 2.5, 2],
                    [8.75, -9.25, 2.75, 1.25],
                    [4.5, -5.5, 4, 2.75],
                    [-0.25, -1.0, 3.75, 2.0],
                    [-4.5, -5.5, 4, 2.75],
                    [-8.75, -9.25, 2.75, 1.25],
                    [-9.5, 1.5, 1.25, 2.75],
                    [-5, 1, 1.25, 3.5],
                    [-0.25, 1, 2.75, 3.5],
                    [5, 1, 1.25, 3.5],
                    [9.25, 1.5, 1.25, 2.75],
                    [5.5, 6.0, 3.75, 1.75],
                    [0.0, 6.0, 1.75, 3.0],
                    [-5.5, 6, 3.75, 1.75],
                    [-10, 0, 0.1, 20],
                    [10, 0, 0.1, 20],
                    [0, -10, 20, 0.1],
                    [0, 10, 20, 0.1]], dtype=float)

# real-life range of map in metres
xmin = -10.1
xmax = 10.1
ymin = -10.1
ymax = 10.1

DENIRO_width = 1 # width of DENIRO

scale = 16 # number of pixels per one metre in real-life

def generate_map():
    # Number of pixels in each direction
    N_x = int((xmax - xmin) * scale)
    N_y = int((ymax - ymin) * scale)
    
    # Initialise the map to be an empty 2D array
    img = np.zeros([N_x, N_y], dtype = np.float)

    for x1, y1, w, h in rect_list: # scaling rect_list
        x0 = int((x1 - w/2 - xmin) * scale)
        y0 = int((y1 - h/2 - ymin) * scale)
        x3 = int((x1 + w/2 - xmin) * scale)
        y3 = int((y1 + h/2 - ymin) * scale)
        
        # Fill the obstacles with 1s
        img[y0:y3, x0:x3] = 1
    return img, scale, scale
    
def create_circular_mask(h, w): # an alternative method for producing the circular map that does not use a nested for loop

    center = (int(w/2), int(h/2)) # finding centre of image
    radius = min(center[0], center[1], w-center[0], h-center[1]) # finding radius of image

    Y, X = np.ogrid[:h, :w] # creating one vector for the rows and one for the columns 
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2) # pythag

    mask = dist_from_center <= radius
    
    return mask

def expand_map(img, robot_width):
    robot_px = int(robot_width * scale)   # size of the robot in pixels x axis
    
    ############################################################### TASK A
    robot_mask_sqaure = np.ones((robot_px, robot_px)) # create a square array of ones of the size of the robot
    robot_mask_circular = create_circular_mask()# create a circular array of ones of the size of the robot
    
    expanded_img_square = binary_dilation(img, robot_mask_sqaure)
    expanded_img_circular = binary_dilation(img, robot_mask_circular)
    #return expanded_img_circular

    ############################################################### REPORT SECTION 2.2
    mask_width = 16
    robot_mask = np.ones((mask_width, mask_width))

    mask_radius = mask_width/2

    for y, v_y in enumerate(robot_mask):
        for x, v_x in enumerate(v_y):
            if (np.square(x - mask_radius + 0.5) + np.square(y - mask_radius + 0.5)) < 
                                        np.square(mask_radius):
                robot_mask[y, x] = 1
            else:
                robot_mask[y, x] = 0

    expanded_img = binary_dilation(img, robot_circular_mask)

    return expanded_img


def main(task):
    if task == 'view':
        print("============================================================")
        print("Generating the map")
        print("------------------------------------------------------------")
        img, xscale, yscale = generate_map()
        plt.imshow(img, vmin=0, vmax=1, origin='lower')
        plt.show()
    
    elif task == 'expand':
        print("============================================================")
        print("Generating the C-space map")
        print("------------------------------------------------------------")
        img, xscale, yscale = generate_map()
        c_img_circular = expand_map(img, DENIRO_width)
        #plot1 = plt.figure(1)
        #plt.imshow(c_img_square, vmin=0, vmax=1, origin='lower')
        
        #plot2 = plt.figure(2)
        plt.imshow(c_img_circular, vmin=0, vmax=1, origin='lower')
        plt.show()
        
    

if __name__ == "__main__":
    tasks = ['view', 'expand']
    if len(sys.argv) <= 1:
        print('Please include a task to run from the following options:\n', tasks)
    else:
        task = str(sys.argv[1])
        if task in tasks:
            print("Running Coursework 2 -", task)
            main(task)
        else:
            print('Please include a task to run from the following options:\n', tasks)



