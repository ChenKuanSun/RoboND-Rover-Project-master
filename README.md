# Search and Sample Return Project
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=iPoCdc8nYBs" target="_blank"><img src="http://img.youtube.com/vi/iPoCdc8nYBs/0.jpg" 
alt="This My test video" width="960" height="540" border="10" /></a>

### This My test video

## In this project, we need to let Land Rover identify the terrain that can be taken and map the terrain we see to the map during the exploration process. In addition to drawing images with jupyterbook, we must also use the simulator and modify the code to let Rover explore the unknown world and, if possible, collect the samples back to the origin.

### Overview
---



[//]: # (Image References)

[image1]: ./mddata/data1.png "data1.png"
[image2]: ./calibration_images/example_grid1.jpg "example_grid1"
[image3]: ./calibration_images/example_rock1.jpg "example_rock1"
[image4]: ./mddata/data2.png "data2.png"
[image5]: ./mddata/data3.png "data3.png"
[image6]: ./mddata/IMG_0036.jpg "mindmap"
[image7]: ./mddata/confing.png "confing.png"

The Project

Config
![alt text][image7]

---
## Notebook Analysis

The goals / steps of this project are the following:

* Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
* Populate the process_image() function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap. Run process_image() on your test data using the moviepy functions provided to create video output of your result.

My code consisted of some steps.

For the Notebook Analysis section:

* 01.First open the simulator and manually manipulate the Rover and record the data.
* 02.Since the example has provided an image to correct the camera, the camera is first corrected.
* 03.The pre-processing action is performed on the recorded image, that is, the bird's-eye view transformation is performed.
* 04.Next, Color Thresholding is applied to the terrain portion to separate the walkable terrain from the non-walking terrain.
* 05.Because the sample part is yellow, I use the HSV channel for yellow sampling.
* 06.Next, rotate to a coordinate map with Rover as the origin.
* 07.Map to the map and render the color.


#### 01.First open the simulator and manually manipulate the Rover and record the data.

#### 02.Since the example has provided an image to correct the camera, the camera is first corrected.
#### 03.The pre-processing action is performed on the recorded image, that is, the bird's-eye view transformation is performed.

![alt text][image2]

![alt text][image3]

```python

def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    return warped

dst_size = 5 

bottom_offset = 6

source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
warped = perspect_transform(grid_img, source, destination)
plt.imshow(warped)

```

Result

![alt text][image1]

#### 04.Next, Color Thresholding is applied to the terrain portion to separate the walkable terrain from the non-walking terrain.

```python
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
```

Result
![alt text][image4]

#### 05.Because the sample part is yellow, I use the HSV channel for yellow sampling.

```python
def rock_thresh(img):
    HSV_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (20 < HSV_img[:,:,0]) \
                & (30 > HSV_img[:,:,0]) \
                & (80 < HSV_img[:,:,1]) \
                & (255 > HSV_img[:,:,1]) \
                & (80 < HSV_img[:,:,2]) \
                & (255 > HSV_img[:,:,2]) 
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
```

#### 06.Next, rotate to a coordinate map with Rover as the origin.

```python
def unwarp(img):
# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Grab another random image
idx = np.random.randint(0, len(img_list)-1)
image = mpimg.imread(img_list[idx])
warped = perspect_transform(image, source, destination)
threshed = color_thresh(warped)

# Calculate pixel values in rover-centric coords and distance/angle to all pixels
xpix, ypix = rover_coords(threshed)
dist, angles = to_polar_coords(xpix, ypix)
mean_dir = np.mean(angles)
```

Result

![alt text][image5]

#### 07.Map to the map and render the color.
I considered the mapping problem caused by Rover scrolling. I set a value and do not do mapping actions as long as the Rover roll and pitch angle exceed the value.

```python
if ((data.roll[data.count] < 1.5) | (data.roll[data.count] > 358.5) )& ((data.pitch[data.count] < 1.5) | (data.pitch[data.count] > 358.5)):
        data.worldmap[obs_y_world, obs_x_world, 0] = 255
        data.worldmap[rock_y_world, rock_x_world, 1] = 255
        data.worldmap[y_world, x_world, 2] = 255 
```

## Autonomous Navigation and Mapping

The goals / steps of this project are the following:

* Fill in the perception_step() (at the bottom of the perception.py script) and decision_step() (in decision.py) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
* Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might improve them in your writeup.

For the Autonomous Navigation and Mapping section:

* 01.In order to map the map in Autonomous mode, I transferred the code for Notebook analysis and corrected it to the format required by perception.py.
* 02.Then still avoid excessive pitching and scrolling resulting in incorrect mapping results.
* 03.I set the angle of the rock sample to the direction of travel for the rock sample I saw in Autonomous mode.

#### 01.In order to map the map in Autonomous mode, I transferred the code for Notebook analysis and corrected it to the format required by perception.py.

```python
def perception_step(Rover):
    #save Start position
    if Rover.start_pos == None:
        Rover.start_pos = Rover.pos
```
Considering that the sample is returned to the starting point after the sample is taken, the current position is stored first when starting the task.


```python
    original_img = Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[original_img.shape[1]/2 - dst_size, original_img.shape[0] - bottom_offset],
                      [original_img.shape[1]/2 + dst_size, original_img.shape[0] - bottom_offset],
                      [original_img.shape[1]/2 + dst_size, original_img.shape[0] - 2*dst_size - bottom_offset], 
                      [original_img.shape[1]/2 - dst_size, original_img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    unwarped_img = perspect_transform(original_img, src = source, dst = destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #terrain
    terrain_img = color_thresh(unwarped_img, rgb_thresh=(170, 170, 160))
    #rock samples
    rock_img = rock_thresh(unwarped_img)
    #obstacles
    obstacles_img = cv2.bitwise_not(terrain_img)-254
    #4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = obstacles_img*200
    Rover.vision_image[:, :, 1] = rock_img*200
    Rover.vision_image[:, :, 2] = terrain_img*200
    xpix, ypix = rover_coords(terrain_img)
    rock_xpix, rock_ypix = rover_coords(rock_img)
    obs_xpix, obs_ypix = rover_coords(obstacles_img)
    
    # 5) Convert rover-centric pixel values to world coords
    
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    yaw = Rover.yaw
    xpos, ypos = Rover.pos
    
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, xpos, ypos, yaw, world_size, scale)
```
The previous preprocessing is the same preprocessing analysis as Notebook.


#### 02.Then still avoid excessive pitching and scrolling resulting in incorrect mapping results.

```python 
    if ((Rover.roll < Rover.roll_limit) | (Rover.roll > (360 - Rover.roll_limit)) )& ((Rover.pitch < Rover.roll_limit) | (Rover.pitch > (360 - Rover.roll_limit))):
        Rover.worldmap[obs_y_world, obs_x_world, 0] = 255
        Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
        Rover.worldmap[y_world, x_world, 2] = 255    
```

#### 03.I set the angle of the rock sample to the direction of travel for the rock sample I saw in Autonomous mode.
```python 
    if Rover.hadpick == 0 :
        rover_dist, rover_angles = to_polar_coords(xpix, ypix)
        Rover.nav_dists = rover_dist
        Rover.nav_angles = rover_angles
    else:
        if len(rock_img.nonzero()[0]) > 10:
            rover_dist, rover_angles = to_polar_coords(rock_xpix, rock_ypix)
            Rover.samples_dists = np.sqrt((np.mean(rock_x_world)-Rover.pos[0])**2 + (np.mean(rock_y_world)-Rover.pos[1])**2) #rover_dist
            Rover.samples_angles = rover_angles
            Rover.prenav_dists = Rover.samples_dists
            Rover.prenav_angles = Rover.samples_angles
        else:
            Rover.samples_angles = Rover.prenav_angles
            Rover.samples_dists = Rover.prenav_dists
    if len(rock_img.nonzero()[0]) > 10 and not Rover.picking_up:
        Rover.hadpick = 1
    if len(rock_img.nonzero()[0]) < 10 and Rover.hadpick == 1:
        Rover.step_count += 1
        if Rover.step_count >= 100:
            Rover.hadpick = 0
    return Rover

```
### In order to complete the required conditions of the project and join the collection of rock samples, I designed the decision tree using mindmap.
![alt text][image6]

First of all, I aim to complete the task, so my first consideration is to check if the sample is collected.
If the collection is complete, you can stop when you return to the starting point.

```python 
if Rover.samples_collected == 6:
        if np.sqrt((Rover.pos[0]-Rover.start_pos[0])**2 + (Rover.pos[1]-Rover.start_pos[1])**2) < 5:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.mode = 'stop'
            return Rover
```

If you have not collected it, continue the task.
Next I'm thinking about the current state, because the task is to collect samples as the main goal, so consider whether there are rock samples in front of it, combined with the code I used in perception.py.

```python 
if len(rock_img.nonzero()[0]) > 10 and not Rover.picking_up:
    Rover.hadpick = 1
```

The current state is divided into two parts. If you don't find the rock, walk in the world. When you get stuck, turn right and move on.

```python
elif Rover.vel <= 0.2:
                    if Rover.loop_count < 50:
                        if len(Rover.nav_angles) < Rover.go_forward:
                            print("Turning Right")
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = -15
                        if len(Rover.nav_angles) >= Rover.go_forward:
                            print("Have Road to Go")
                            Rover.throttle = Rover.throttle_set
                            Rover.brake = 0
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                            Rover.mode = 'forward'
```

If you find rock, walk in the direction of the rock and check the distance. The shorter the distance, the slower the Rover will be until the rock is detected. Of course, I also set the foolproof mechanism. If it is stuck, Turn right and move on.

perception.py
```python 
if len(rock_img.nonzero()[0]) > 10:
            rover_dist, rover_angles = to_polar_coords(rock_xpix, rock_ypix)
            Rover.samples_dists = np.sqrt((np.mean(rock_x_world)-Rover.pos[0])**2 + (np.mean(rock_y_world)-Rover.pos[1])**2) #rover_dist
            Rover.samples_angles = rover_angles
            Rover.prenav_dists = Rover.samples_dists
            Rover.prenav_angles = Rover.samples_angles
```
decision.py
```python 
print("Find Sample! Dist=" + str(Rover.samples_dists))
                    if Rover.samples_dists > 20:
                        if Rover.vel < Rover.max_vel:
                            Rover.throttle = Rover.throttle_set
                        else:
                            Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                    elif Rover.samples_dists < 10:  
                        if Rover.vel < Rover.max_vel/5:
                            Rover.throttle = Rover.throttle_set
                            Rover.brake = 0
                        elif Rover.vel > Rover.max_vel/5:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set/2
                        else:
                            Rover.throttle = 0
                            Rover.brake = 0
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                        if Rover.near_sample and not Rover.picking_up:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                            Rover.mode = 'stop'
                            Rover.send_pickup = True
                            Rover.hadpick = 0
                            Rover.loop_count = 0
                            Rover.step_count = 0
                        if Rover.vel < 0.05 and not Rover.picking_up:
                            if Rover.loop_count < 50 :
                                Rover.loop_count +=1
                            else:
                                print('Turn Right')
                                Rover.throttle = 0
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'
                        else:
                            Rover.loop_count = 0

```


### Discussion

In this project, watching my video can reveal that Rover still has some situations where I can't navigate. I understand that because I haven't learned the path planning course yet, I am already learning how to use path planning.
In addition to navigation, I feel that I can make corrections when I get stuck. I don't have to go to the right. I can rotate according to the path planning or detecting whether the average angle of the currently movable terrain is left or right.

Thank you for reading