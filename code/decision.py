import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data         實施條件決定給定感知數據做什麼
    # Here you're all set up with some basic functionality but you'll need to   這裡你已經設置了一些基本的功能，但你需要
    # improve on this decision tree to do a good job of navigating autonomously!改進這個決策樹，做好自主導航！

    # Example:
    # Check if we have vision data to make decisions with 檢查我們是否有視力數據來做決定
    if Rover.samples_collected == 6:
        if np.sqrt((Rover.pos[0]-Rover.start_pos[0])**2 + (Rover.pos[1]-Rover.start_pos[1])**2) < 5:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.mode = 'stop'
            return Rover
    if Rover.hadpick == 0 :
        if Rover.nav_angles is not None:
            # Check for Rover.mode status   檢查Rover.mode狀態
            if Rover.mode == 'forward':
                if Rover.vel < 0.2 and not Rover.picking_up:
                    if Rover.loop_count < 50 :
                        Rover.loop_count +=1
                    else:
                        print('Turn Right0')
                        Rover.throttle = 0
                        # Set brake to stored brake value 將製動器設置為存儲的製動值
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
                else:
                    Rover.loop_count = 0
                # Check the extent of navigable terrain     檢查可導航地形的範圍
                if len(Rover.nav_angles) >= Rover.stop_forward:  
                    print("Moving Forward")
                    # If mode is forward, navigable terrain looks good  如果模式是向前的，可導航的地形看起來很好
                    # and velocity is below max, then throttle          和速度低於最大值，然後油門
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting        將油門值設置為油門設置
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15     ＃將轉向設置為平均夾角，範圍為+/- 15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) + 10
                # If there's a lack of navigable terrain pixels then go to 'stop' mode  如果缺少可導航地形像素，則轉到“停止”模式
                elif len(Rover.nav_angles) < Rover.stop_forward:
                        print('No Road to go')
                        # Set mode to "stop" and hit the brakes! 設置模式為“停止”並踩剎車！
                        Rover.throttle = 0
                        # Set brake to stored brake value 將製動器設置為存儲的製動值
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
            # If we're already in "stop" mode then make different decisions 如果我們已經處於“停止”模式，那麼做出不同的決定
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking 如果我們處於停止模式但仍在繼續制動
                if Rover.vel > 0.2:
                    print("Stopping")
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else 如果我們沒有移動（vel <0.2），那就做點別的
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward 現在我們停止了，我們有視覺數據，看看是否有前進的道路
                    if Rover.loop_count < 50:
                        if len(Rover.nav_angles) < Rover.go_forward:
                            print("Turning Right1")
                            Rover.throttle = 0
                            # Release the brake to allow turning        鬆開制動器以便轉動
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning      轉彎範圍為+/- 15度，當停止時，下一條線將導致4輪轉向
                            Rover.steer = -15 # Could be more clever here about which way to turn

                        # If we're stopped but see sufficient navigable terrain in front then go!  如果我們停下來但看到前面有足夠的可導航地形，那就去吧！
                        if len(Rover.nav_angles) >= Rover.go_forward:
                            print("Have Road to Go")
                            # Set throttle back to stored value     將油門設置回存儲值
                            Rover.throttle = Rover.throttle_set
                            # Release the brake 鬆開制動器
                            Rover.brake = 0
                            # Set steer to mean angle 將轉向設置為平均角度
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                            Rover.mode = 'forward'
                    else:
                        if Rover.loop_count < 80:
                            print("Turning Right2")
                            Rover.throttle = 0
                            # Release the brake to allow turning        鬆開制動器以便轉動
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning      轉彎範圍為+/- 15度，當停止時，下一條線將導致4輪轉向
                            Rover.steer = -15 # Could be more clever here about which way to turn
                            Rover.loop_count += 1
                        # If we're stopped but see sufficient navigable terrain in front then go!  如果我們停下來但看到前面有足夠的可導航地形，那就去吧！
                        else:
                            print("Have Road to Go")
                            # Set throttle back to stored value     將油門設置回存儲值
                            Rover.throttle = Rover.throttle_set
                            # Release the brake 鬆開制動器
                            Rover.brake = 0
                            # Set steer to mean angle 將轉向設置為平均角度
                            Rover.steer = -15
                            Rover.loop_count = 0
                            Rover.mode = 'forward'
        # Just to make the rover do something  只是為了讓流浪者做點什麼 即使沒有對代碼進行任何修改
        # even if no modifications have been made to the code
        else:
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0
    elif Rover.hadpick == 1 :
        if not Rover.picking_up:
            if Rover.mode == 'forward':
                if Rover.samples_dists  is not None:
                    # Check the extent of navigable terrain     檢查可導航地形的範圍
                    print("Find Sample! Dist=" + str(Rover.samples_dists))
                    if Rover.samples_dists > 20:
                        # If mode is forward, navigable terrain looks good  如果模式是向前的，可導航的地形看起來很好
                        # and velocity is below max, then throttle          和速度低於最大值，然後油門
                        if Rover.vel < Rover.max_vel:
                            # Set throttle value to throttle setting        將油門值設置為油門設置
                            Rover.throttle = Rover.throttle_set
                        else: # Else coast
                            Rover.throttle = 0
                        Rover.brake = 0
                        # Set steering to average angle clipped to the range +/- 15     ＃將轉向設置為平均夾角，範圍為+/- 15
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                    # If there's a lack of navigable terrain pixels then go to 'stop' mode  如果缺少可導航地形像素，則轉到“停止”模式
                    elif Rover.samples_dists < 10:  
                        # If mode is forward, navigable terrain looks good  如果模式是向前的，可導航的地形看起來很好
                        if Rover.vel < Rover.max_vel/5:
                            # Set throttle value to throttle setting        將油門值設置為油門設置
                            Rover.throttle = Rover.throttle_set
                            Rover.brake = 0
                        elif Rover.vel > Rover.max_vel/5:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set/2
                        else: # Else coast
                            Rover.throttle = 0
                            Rover.brake = 0
                        # and velocity is below max, then throttle          和速度低於最大值，然後油門
                        # Set steering to average angle clipped to the range +/- 15     ＃將轉向設置為平均夾角，範圍為+/- 15
                        Rover.steer = np.clip(np.mean(Rover.samples_angles * 180/np.pi), -30, 30)
                        if Rover.near_sample and not Rover.picking_up:
                            # Set mode to "stop" and hit the brakes! 設置模式為“停止”並踩剎車！
                            Rover.throttle = 0
                            # Set brake to stored brake value 將製動器設置為存儲的製動值
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                            Rover.mode = 'stop'
                            Rover.send_pickup = True
                            Rover.hadpick = 0
                            Rover.loop_count = 0
                            Rover.step_count = 0
                            print("111111111")

                        if Rover.vel < 0.05 and not Rover.picking_up:
                            if Rover.loop_count < 50 :
                                Rover.loop_count +=1
                            else:
                                print('Turn Right4')
                                Rover.throttle = 0
                                # Set brake to stored brake value 將製動器設置為存儲的製動值
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'
                        else:
                            Rover.loop_count = 0

                # If we're already in "stop" mode then make different decisions 如果我們已經處於“停止”模式，那麼做出不同的決定
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking 如果我們處於停止模式但仍在繼續制動
                if Rover.vel > 0.2:
                        print("Stopping")
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    # If we're not moving (vel < 0.2) then do something else 如果我們沒有移動（vel <0.2），那就做點別的
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward 現在我們停止了，我們有視覺數據，看看是否有前進的道路
                    if Rover.loop_count < 50:
                        if len(Rover.nav_angles) < Rover.go_forward:
                            print("Turning Right5")
                            Rover.throttle = 0
                            # Release the brake to allow turning        鬆開制動器以便轉動
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning      轉彎範圍為+/- 15度，當停止時，下一條線將導致4輪轉向
                            Rover.steer = -15 # Could be more clever here about which way to turn

                        # If we're stopped but see sufficient navigable terrain in front then go!  如果我們停下來但看到前面有足夠的可導航地形，那就去吧！
                        if len(Rover.nav_angles) >= Rover.go_forward:
                            print("Have Road to Go")
                            # Set throttle back to stored value     將油門設置回存儲值
                            Rover.throttle = Rover.throttle_set
                            # Release the brake 鬆開制動器
                            Rover.brake = 0
                            # Set steer to mean angle 將轉向設置為平均角度
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                            Rover.mode = 'forward'
                    else:
                        if Rover.loop_count < 80:
                            print("Turning Right6")
                            Rover.throttle = 0
                            # Release the brake to allow turning        鬆開制動器以便轉動
                            Rover.brake = 0
                            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning      轉彎範圍為+/- 15度，當停止時，下一條線將導致4輪轉向
                            Rover.steer = -15 # Could be more clever here about which way to turn
                            Rover.loop_count += 1
                        # If we're stopped but see sufficient navigable terrain in front then go!  如果我們停下來但看到前面有足夠的可導航地形，那就去吧！
                        else:
                            print("Have Road to Go")
                            # Set throttle back to stored value     將油門設置回存儲值
                            Rover.throttle = Rover.throttle_set
                            # Release the brake 鬆開制動器
                            Rover.brake = 0
                            # Set steer to mean angle 將轉向設置為平均角度
                            Rover.steer = -15
                            Rover.loop_count = 0
                            Rover.mode = 'forward'
        
    # If in a state where want to pickup a rock send pickup command 如果在想要拾取岩石的狀態下發送拾取命令
    if Rover.near_sample and Rover.vel < 0.2 and not Rover.picking_up:
        print("2222222222")
        Rover.send_pickup = True
        Rover.hadpick = 0
    return Rover