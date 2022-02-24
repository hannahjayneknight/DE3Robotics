import numpy as np                                          # importing external modules
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage.morphology import binary_dilation
import rospy
import sys

rect_list = np.array([[0, -9, 2.5, 2],                      # coordinates for obstacles
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

# Real-life range of map in metres
xmin = -10.1
xmax = 10.1
ymin = -10.1
ymax = 10.1

DENIRO_width = 1                                                # width of DENIRO

scale = 16                                                      # number of pixels per one metre in real-life

def generate_map():
    N_x = int((xmax - xmin) * scale)                            # Number of pixels in each direction
    N_y = int((ymax - ymin) * scale)
    
    img = np.zeros([N_x, N_y], dtype = np.float)                # Initialise the map to be an empty 2D array

    for x1, y1, w, h in rect_list:                              # scaling the obstacles
        x0 = int((x1 - w/2 - xmin) * scale)
        y0 = int((y1 - h/2 - ymin) * scale)
        x3 = int((x1 + w/2 - xmin) * scale)
        y3 = int((y1 + h/2 - ymin) * scale)
        
        img[y0:y3, x0:x3] = 1                                   # Fill the obstacles with 1s
    return img, scale, scale

def expand_map(img, robot_width):
    robot_px = int(robot_width * scale)                         # size of the robot in pixels x axis
    
    ############################################################### REPORT SECTION 2.1
    robot_square_mask = np.ones((robot_px, robot_px))           # create a 16x16 array of ones as the square mask.
    #expanded_img = binary_dilation(img, robot_square_mask) #------ uncomment to apply the square mask

    ############################################################### REPORT SECTION 2.2
    robot_circular_mask = np.zeros((scale, scale))              # creating a 16x16 array to be filled
    mask_radius = scale/2                                       # finding radius of circle
    mask_centre = mask_radius - 0.5                             # centre of circle is at 7.5 (between two elements in the array)

    # Loop through each column and row of the array
    for y, v_y in enumerate(robot_circular_mask):               # y=index, v_y=value
        for x, v_x in enumerate(v_y):
            if (np.square(x - mask_centre) + np.square(y - mask_centre)) < np.square(mask_radius):
                robot_circular_mask[y, x] = 1                   # if element in the circle, make it equal to 1

    #print(robot_circular_mask)                                 #------ uncomment to see the array of 0s and circular 1s.
    expanded_img = binary_dilation(img, robot_circular_mask) # apply the circular mask to the image

    return expanded_img


def main(task):
    if task == 'view':
        print("============================================================")
        print("Generating the map")
        print("------------------------------------------------------------")
        img, xscale, yscale = generate_map()                    # generating the non-expanded map
        plt.imshow(img, vmin=0, vmax=1, origin='lower')         # showing the non-expanded map
        #plt.set_cmap("Wistia")                                 # ------ uncomment to change the colour of the map
        plt.show()                                              # showing the non-expanded map
    
    elif task == 'expand':
        print("============================================================")
        print("Generating the C-space map")
        print("------------------------------------------------------------")
        img, xscale, yscale = generate_map()                    # generating the non-expanded map
        c_img_circular = expand_map(img, DENIRO_width)          # expanding the map
        plt.imshow(c_img_circular, vmin=0, vmax=1, origin='lower') # showing the expanded map
        #plt.set_cmap("Wistia")                                 # ------ uncomment to change the colour of the map
        plt.show()
        
    

if __name__ == "__main__":                                      # if none of the options were chosen, the user is prompted to pick a task
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



