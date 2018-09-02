import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # Example:
    # Check if we have vision data to make decisions with 

    #Check the number of samples, stop the task if you reach six samples.
    if Rover.samples_collected == 6:

        #Check how far away from the starting point, stop if it is less than 10 meters.
        if np.sqrt((Rover.pos[0]-Rover.start_pos[0])**2 + (Rover.pos[1]-Rover.start_pos[1])**2) < 10:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.mode = 'stop'
            return Rover

    #Check if there is a sample in front, if any, go to the sample mode, if not, explore the world.
    if Rover.hadpick == 0 :
        if Rover.nav_angles is not None:

            # Check for Rover.mode status
            if Rover.mode == 'forward':

                #if too slow define its stuck
                if Rover.vel < 0.2 and not Rover.picking_up:

                    if Rover.loop_count < 50 :
                        Rover.loop_count +=1

                    else:
                        print('Stuck! Turn Right!!')
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
                #Speed up
                else:
                    Rover.loop_count = 0

                # Check the extent of navigable terrain
                if len(Rover.nav_angles) >= Rover.stop_forward:  
                    print("Moving Forward")
                    # If mode is forward, navigable terrain looks good
                    # and velocity is below max, then throttle
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15 left side to +10
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) + 10

                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                        print('No Road to go')
                        # Set mode to "stop" and hit the brakes!
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'

            # If we're already in "stop" mode then make different decisions
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking
                if Rover.vel > 0.2:
                    print("Stopping")
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0

                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2:

                    # Now we're stopped and we have vision data to see if there's a path forward路
                    if Rover.loop_count < 50:
                        if len(Rover.nav_angles) < Rover.go_forward:
                            print("Not have Road to go, Turning Right!!")
                            Rover.throttle = 0
                            # Release the brake to allow turning
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                            # Could be more clever here about which way to turn
                            Rover.steer = -15 

                        # If we're stopped but see sufficient navigable terrain in front then go!
                        if len(Rover.nav_angles) >= Rover.go_forward:
                            print("Have Road to Go!!")
                            # Set throttle back to stored value
                            Rover.throttle = Rover.throttle_set
                            # Release the brake
                            Rover.brake = 0
                            # Set steer to mean angle
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                            Rover.mode = 'forward'

                    else:
                        #if stuck
                        if Rover.loop_count < 90:
                            print("Stuck!! Turning Right!!!")
                            Rover.throttle = 0
                            # Release the brake to allow turning
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                            Rover.steer = -15 
                            Rover.loop_count += 1

                        else:
                            print("Finsh and Go")
                            # Set throttle back to stored value
                            Rover.throttle = Rover.throttle_set
                            # Release the brake
                            Rover.brake = 0
                            # Set steer to mean angle
                            Rover.steer = -15
                            Rover.loop_count = 0
                            Rover.mode = 'forward'

        # Just to make the rover do something
        # even if no modifications have been made to the code
        else:
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0

    elif Rover.hadpick == 1 :
        if not Rover.picking_up:
            if Rover.mode == 'forward':
                if Rover.samples_dists  is not None:
                    # Check the extent of navigable terrain
                    print("Find Sample! Dist=" + str(Rover.samples_dists))

                    #Check the rock sample distance, the shorter the slower the speed.
                    if Rover.samples_dists > 20:
                        # If mode is forward, navigable terrain looks good
                        # and velocity is below max, then throttle 
                        if Rover.vel < Rover.max_vel:
                            # Set throttle value to throttle setting
                            Rover.throttle = Rover.throttle_set
                        else: # Else coast
                            Rover.throttle = 0
                        Rover.brake = 0
                        # Set steering to average angle clipped to the range +/- 30
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                    # If there's a lack of navigable terrain pixels then go to 'stop' mode
                    elif Rover.samples_dists < 10:  
                        # If mode is forward, navigable terrain looks good  
                        if Rover.vel < Rover.max_vel/5:
                            # Set throttle value to throttle setting 
                            Rover.throttle = Rover.throttle_set
                            Rover.brake = 0
                        elif Rover.vel > Rover.max_vel/5:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set/2
                        else: # Else coast
                            Rover.throttle = 0
                            Rover.brake = 0
                        # and velocity is below max, then throttle
                        # Set steering to average angle clipped to the range +/- 30
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                        if Rover.near_sample and not Rover.picking_up:
                            # Set mode to "stop" and hit the brakes!
                            Rover.throttle = 0
                            # Set brake to stored brake value
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                            Rover.mode = 'stop'
                            Rover.send_pickup = True
                            Rover.hadpick = 0
                            Rover.loop_count = 0
                            Rover.step_count = 0
                            print("Picking UP~~~~~~~~~~~~~~~~~")

                        if Rover.vel < 0.05 and not Rover.picking_up:
                            if Rover.loop_count < 50 :
                                Rover.loop_count +=1
                            else:
                                print('Stuck~~~~~~~Turn Right!!!!')
                                Rover.throttle = 0
                                # Set brake to stored brake value
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'
                        else:
                            Rover.loop_count = 0

                # If we're already in "stop" mode then make different decisions
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking
                if Rover.vel > 0.2:
                        print("Stopping")
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if Rover.loop_count < 50:
                        if len(Rover.nav_angles) < Rover.go_forward:
                            print("Not have Road to go, Turning Right!!")
                            Rover.throttle = 0
                            # Release the brake to allow turning       
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                            Rover.steer = -15 # Could be more clever here about which way to turn

                        # If we're stopped but see sufficient navigable terrain in front then go!
                        if len(Rover.nav_angles) >= Rover.go_forward:
                            print("Have Road to Go")
                            # Set throttle back to stored value
                            Rover.throttle = Rover.throttle_set
                            # Release the brake
                            Rover.brake = 0
                            # Set steer to mean angle
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                            Rover.mode = 'forward'
                    else:
                        if Rover.loop_count < 80:
                            print("Stuck~~~~~~~Turn Right!!!!")
                            Rover.throttle = 0
                            # Release the brake to allow turning
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                            Rover.steer = -15 # Could be more clever here about which way to turn
                            Rover.loop_count += 1
                        # If we're stopped but see sufficient navigable terrain in front then go!
                        else:
                            print("Have Road to Go")
                            # Set throttle back to stored value
                            Rover.throttle = Rover.throttle_set
                            # Release the brake
                            Rover.brake = 0
                            # Set steer to mean angle
                            Rover.steer = -15
                            Rover.loop_count = 0
                            Rover.mode = 'forward'
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel < 0.2 and not Rover.picking_up:
        print("Picking UP~~~~~~~~~~~~~~~~~")
        Rover.send_pickup = True
        Rover.hadpick = 0
    return Rover