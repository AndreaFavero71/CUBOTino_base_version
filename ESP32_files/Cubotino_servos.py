"""
#############################################################################################################
# Andrea Favero April 2022
#
# This script relates to "CUBOTino", a simpler (and much cheaper) Rubik's cube solver robot than my first one:
# (https://www.youtube.com/watch?v=oYRXe4NyJqs)
#
# This script controls two servos based on the movements string from Cubotino_moves.py
# This script also interacts with the uart, to feedback the solving progress and to check if stop requests.
# 
# Possible moves with this robot
# 1) Spins the complete cube ("S") laying on the bottom face: 1 means CW 90deg turns, while 3 means 90CCW turn
# 2) Flips the complete cube ("F") by "moving" the Front face to Bottom face: Only positive values are possible
# 3) Rotates the bottom layer ("R") while costraining the 2nd and 3rd layer.
# 4) The order of S, F has to be strictly followed
# 5) Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (Down) layer, 1x (90deg) CCW cube Spin
#
# For Rotations, the bottom servo makes a little extra rotation than target, before coming back to target; This
# is needed to recover the gaps between cube hoder - cube - top cover, and still getting a decent cube layers alignment.
#
#
# Spin and Rotation moovements don't take into account the next movement; This means each and every
# spin or rotation stops after 90deg rotation, also when followed by another 90deg spin (or rotation), despite
# having the same direction: Optimization is possible in these cases....
#
#############################################################################################################
"""


from machine import Pin, PWM, TouchPad
from utime import sleep, sleep_ms, time
import sys, select



t_servo= PWM(Pin(22), freq=50)  # top servo, connected to Pin 22
sleep_ms(500)

b_servo= PWM(Pin(23), freq=50)  # bottom servo, connected to Pin 23
sleep_ms(500)

settings_status=False           # boolean to track the servos settings uploading status
robot_init_status=False         # boolean to track the servos inititialization status
stop_servos=True                # boolean to stop the servos during solving proces: It is set true at the start, servos cannot operate
fun_status=False                # boolean to track the robot fun status, it is True after solving the cube :-)


def init_servo(debug=False):
    """ Function to initialize the robot (servos position) and some global variables."""
    
    global b_servo_CCW, b_home, b_servo_CW, b_servo_CCW_rel, b_servo_CW_rel, b_home_from_CW, b_home_from_CCW
    global b_rotate_time, b_rel_time, b_spin_time
    global t_servo_flip, t_servo_open, t_servo_close, t_servo_rel, t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    global robot_init_status, stop_servos, fun_status
    
    settings_status = upload_settings()  # call the function that uploads the servos settings
    if settings_status:
        stop_servos=False             # boolean to stop the servos during solving process is set False: servos can be operated
        b_servo_operable=False        # variable to block/allow bottom servo operation
        t_servo.duty(t_servo_open)    # top servo is positioned in open position at the start
        sleep_ms(700)                 # bottom servo is positioned at home, the starting position
        t_top_cover = 'open'          # variable to track the top cover/lifter position
        
        b_servo_operable=True         # variable to block/allow bottom servo operation
        b_servo.duty(b_home)          # bottom servo is positioned at home, the starting position   
        sleep_ms(700)                 # bottom servo is positioned at home, the starting position
        b_servo_stopped = True        # boolean of bottom servo at location the lifter can be operated
        b_servo_home=True             # boolean of bottom servo at home
        b_servo_CW_pos=False          # boolean of bottom servo at full CW position
        b_servo_CCW_pos=False         # boolean of bottom servo at full CCW position
        robot_init_status = True      # boolean to track the inititialization status of the servos is set true
    else:
        robot_init_status = False     # boolean to track the inititialization status of the servos is set false
    
    fun_status=False                  # boolean to track the robot fun status, it is True after solving the cube :-)
    
    return robot_init_status






def upload_settings():
    """ Function to upload the servos settings."""
    
    global t_servo_flip, t_servo_open, t_servo_close, t_servo_rel, t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time
    global b_servo_CCW, b_home, b_servo_CW, b_servo_CCW_rel, b_servo_CW_rel, b_home_from_CW, b_home_from_CCW
    global b_rotate_time, b_rel_time, b_spin_time
    
    
    # import settings from the text file
    with open("Cubotino_settings.txt", "r") as f:  # text file is opened
        data = f.readline()                        # data is on first line
        data = data.replace(' ','')                # empty spaces are removed
        if '(' in data and ')' in data:            # case the dat contains open and close parenthesis
            data_start = data.find('(')            # position of open parenthesys in data
            data_end = data.find(')')              # position of close parenthesys in data
            data = data[data_start+1:data_end]     # data in between parenthesys is assigned, to the same string variable
            data_list=data.split(',')              # data is split by comma, becoming a list of strings 
                        
            settings=[]                            # empty list to store the list of numerical settings
            for setting in data_list:              # iteration over the list of strings
                settings.append(int(setting))      # each string setting is changed to integer and appended to the list of settings
                            
            t_servo_flip = settings[0]      # top servo position to flip the cube on one of its horizontal axis
            t_servo_open = settings[1]      # top servo position to free up the top cover from the cube, and keep the lifter out of the way
            t_servo_close = settings[2]     # top servo position to constrain the top cover on cube mid and top layer
            t_servo_rel = t_servo_close - settings[3]  # top servo position to release tension
            t_flip_to_close_time = settings[4]         # time needed to the servo to lower the cover/flipper from flip to close position
            t_close_to_flip_time = settings[5]         # time needed to the servo to raise the cover/flipper from close to flip position
            t_flip_open_time = settings[6]             # time needed to the servo to raise/lower the flipper between open and flip positions
            t_open_close_time = settings[7]            # time needed to the servo to raise/lower the flipper between open and close positions
            
            b_servo_CCW = settings[8]      # bottom servo position when fully CW 
            b_home = settings[9]           # bottom servo home position
            b_servo_CW = settings[10]      # bottom servo position when fully CCW
            b_extra_sides = settings[11]   # bottom servo position extra rotation at CW and CCW
            b_extra_home = settings[12]    # bottom servo position extra rotation at home
            b_spin_time = settings[13]     # time needed to the bottom servo to spin about 90deg
            b_rotate_time = settings[14]   # time needed to the bottom servo to rotate about 90deg
            b_rel_time = settings[15]      # time needed to the servo to rotate slightly back, to release tensions

            b_servo_CCW_rel=b_servo_CCW + b_extra_sides   # bottom servo position to rel tensions when fully CW
            b_servo_CW_rel=b_servo_CW - b_extra_sides     # bottom servo position to rel tensions when fully CCW
            b_home_from_CW=b_home - b_extra_home          # bottom servo extra home position, when moving back from full CW
            b_home_from_CCW=b_home + b_extra_home         # bottom servo extra home position, when moving back from full CCW
            
            settings_status=True           # boolean to track the servos settings uploading status
            return settings_status
        
        else:
            print("not a valid Cubotino_settings.txt file, Cubotino_servos.py cannot proceed") # feedback is returned
            settings_status=False          # boolean to track the servos settings uploading status is set false
            return settings_status






def stopping_servos(debug):
    """ Function to stop the servos."""
    global stop_servos
    
    if debug:
        print("\ncalled the servos stopping function\n")
    stop_servos=True            # boolean to stop the servos during solving process, is set true: Servos are stopped
    init_servo(debug)           # after stopping the solving process, servos are positioned back to start postion






def stop_release(debug):
    """ Function to relelease the stop from servos."""
    
    global stop_servos
    
    if debug:
        print("\ncalled the stop release function\n")
    stop_servos=False            # boolean to stop the servos during solving process, is set false: Servo can be operated






def flip_test():
    """ Function to test the flipping function (toggle)."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if t_top_cover != 'flip':   # case the servo is not on flip position
        flip_up()               # servo is positioned to flip the cube
    else:                       # case the servo is on flip position
        flip_to_open()          # servo is positioned to flip the cube






def flip_up():
    """ Function to raise the flipper to the upper position, to flip the cube around its horizontal axis."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                          # case there is not a stop request for servos
        if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False               # variable to block/allow bottom servo operation
            if t_top_cover == 'close':           # cover/lifter position variable set to close
                t_servo.duty(t_servo_flip)       # servo is positioned to flip the cube
                sleep_ms(t_close_to_flip_time)   # time for the servo to reach the flipping position from close position
            elif t_top_cover == 'open':          # cover/lifter position variable set to open
                t_servo.duty(t_servo_flip)       # servo is positioned to flip the cube
                sleep_ms(t_flip_open_time)       # time for the servo to reach the flipping position                
            t_top_cover='flip'                   # cover/lifter position variable set to flip 






def flip_to_open():
    """ Function to raise the top cover to the open position. The cube is not contrained by the top cover or the flipper."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                        # case there is not a stop request for servos
        if b_servo_stopped==True:              # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False             # variable to block/allow bottom servo operation
#             t_servo.duty(t_servo_close)        # servo is positioned to constrain the mid and top cube layers
#             sleep_ms(t_flip_to_close_time)     # time for the servo to reach the close position from the flip position
            t_servo.duty(t_servo_open)         # top servo is positioned in open top cover position, from close position
            sleep_ms(t_flip_open_time)         # time for the top servo to reach the open top cover position
            t_top_cover='open'                 # variable to track the top cover/lifter position
            b_servo_operable=True              # variable to block/allow bottom servo operation






def flip_to_close():
    """ Function to lower the flipper to the close position, position that contrains the cube with the top cover."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                           # case there is not a stop request for servos
        if b_servo_stopped==True:                 # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False                # variable to block/allow bottom servo operation
            t_servo.duty(t_servo_close)           # servo is positioned to constrain the mid and top cube layers
            if t_top_cover == 'flip':             # cover/lifter position variable set to flip
                sleep_ms(t_flip_to_close_time)    # time for the servo to reach the close position
            elif t_top_cover == 'open':           # cover/lifter position variable set to open
                sleep_ms(t_open_close_time)       # time for the servo to reach the flipping position
            t_servo.duty(t_servo_rel)             # servo is positioned to release the tention from top of the cube (in case of contact)
            t_top_cover='close'                   # cover/lifter position variable set to close
            b_servo_operable=True                 # variable to block/allow bottom servo operation






def open_cover():
    """ Function to open the top cover from the close position, to release the contrain from the cube."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                        # case there is not a stop request for servos
        if b_servo_stopped==True:              # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False             # variable to block/allow bottom servo operation
            t_servo.duty(t_servo_open)         # servo is positioned to open
            sleep_ms(t_open_close_time)        # time for the servo to reach the open position
            t_top_cover='open'                 # variable to track the top cover/lifter position
            b_servo_operable=True              # variable to block/allow bottom servo operation






def close_cover():
    """ Function to close the top cover, to contrain the cube."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                        # case there is not a stop request for servos
        if b_servo_stopped==True:              # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False             # variable to block/allow bottom servo operation
            t_servo.duty(t_servo_close)        # servo is positioned to open
            sleep_ms(t_open_close_time)        # time for the servo to reach the open position
            t_servo.duty(t_servo_rel)          # servo is positioned to release the tention from top of the cube (in case of contact)
            t_top_cover='close'                # cover/lifter position variable set to close
            b_servo_operable=True              # variable to block/allow bottom servo operation






def spin_out(direction):
    """ Function that spins the cube holder toward CW or CCW.
        During the spin the cube is not contrained by the top cover.
        The cube holder stops to the intended position, without making extra rotation."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:                            # case there is not a stop request for servos
        if b_servo_operable==True:                 # variable to block/allow bottom servo operation
            if b_servo_home==True:                 # boolean of bottom servo at home
                b_servo_stopped=False              # boolean of bottom servo at location the lifter can be operated
                b_servo_CW_pos=False               # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False              # boolean of bottom servo at full CCW position
                
                if direction=='CCW':               # case the set direction is CCW
                    b_servo.duty(b_servo_CCW+1)    # bottom servo moves to the most CCW position
                    sleep_ms(b_spin_time)          # time for the bottom servo to reach the most CCW position
                    b_servo_CCW_pos=True           # boolean of bottom servo at full CCW position
                
                elif direction=='CW':              # case the set direction is CW
                    b_servo.duty(b_servo_CW-1)     # bottom servo moves to the most CCW position
                    sleep_ms(b_spin_time)          # time for the bottom servo to reach the most CCW position
                    b_servo_CW_pos=True            # boolean of bottom servo at full CW position
                
                b_servo_stopped=True               # boolean of bottom servo at location the lifter can be operated
                b_servo_home=False                 # boolean of bottom servo at home
                b_servo_stopped=True               # boolean of bottom servo at location the lifter can be operated






def spin_home():
    """ Function that spins the cube holder to home position.
        During the spin the cube is not contrained by the top cover.
        The cube holder stops to home position, without making extra rotation."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:                       # case there is not a stop request for servos
        if b_servo_operable==False:           # variable to block/allow bottom servo operation
            open_cover()                      # top servo is moved to open position
        elif b_servo_operable==True:          # variable to block/allow bottom servo operation
            if b_servo_home==False:           # boolean of bottom servo at home
                b_servo_stopped = False       # boolean of bottom servo at location the lifter can be operated
                b_servo.duty(b_home)          # bottom servo moves to the home position, releasing then the tensions
                sleep_ms(b_spin_time)         # time for the bottom servo to reach the extra home position
                b_servo_stopped=True          # boolean of bottom servo at location the lifter can be operated
                b_servo_home=True             # boolean of bottom servo at home
                b_servo_CW_pos=False          # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False         # boolean of bottom servo at full CCW position






def rotate_out(direction):
    """ Function that rotates the cube holder toward CW or CCW position; During the rotation the cube is contrained by the top cover.
        The cube holder makes first an extra rotation, and later it comes back to the intended position; This approach
        is used for a better facelets alignment to the faces, and to relese the friction (cube holder - cube - top cover)."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:                            # case there is not a stop request for servos
        if b_servo_operable==True:                 # variable to block/allow bottom servo operation
            if b_servo_home==True:                 # boolean of bottom servo at home
                
                if t_top_cover!='close':           # case the top cover is not in close position
                    flip_to_close()                # top cover is lowered in close position
                
                b_servo_stopped=False              # boolean of bottom servo at location the lifter can be operated
                if direction=='CCW':               # case the set direction is CCW
                    b_servo.duty(b_servo_CCW)      # bottom servo moves to the most CCW position
                    sleep_ms(b_rotate_time)        # time for the bottom servo to reach the most CCW position
                    b_servo.duty(b_servo_CCW_rel)  # bottom servo moves slightly to release the tensions
                    sleep_ms(b_rel_time)           # time for the servo to release the tensions
                    b_servo_CCW_pos=True           # boolean of bottom servo at full CCW position
                    
                elif direction=='CW':              # case the set direction is CW
                    b_servo.duty(b_servo_CW)       # bottom servo moves to the most CCW position
                    sleep_ms(b_rotate_time)        # time for the bottom servo to reach the most CCW position
                    b_servo.duty(b_servo_CW_rel)   # bottom servo moves slightly to release the tensions
                    sleep_ms(b_rel_time)           # time for the servo to release the tensions
                    b_servo_CW_pos=True            # boolean of bottom servo at full CW position
                    
                b_servo_stopped=True               # boolean of bottom servo at location the lifter can be operated
                b_servo_home=False                 # boolean of bottom servo at home
                
                if t_top_cover=='close':           # case the top cover is in close position
                    open_cover()                   # top cover is raised in open position






def rotate_home(direction):
    """ Function that rotates the cube holder to home position; During the rotation the cube is contrained by the top cover.
        The cube holder makes first an extra rotation, and later it comes back to the home position; This approach
        is used for a better facelets alignment to the faces, and to relese the friction (cube holder - cube - top cover)."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:                                # case there is not a stop request for servos
        if b_servo_operable==True:                     # variable to block/allow bottom servo operation
            if b_servo_home==False:                    # boolean of bottom servo at home
                
                if t_top_cover!='close':               # case the top cover is not in close position
                    flip_to_close()                    # top cover is lowered in close position
                
                if direction=='CCW':                   # case the set direction is CCW
                    if b_servo_CW_pos==True:           # boolean of bottom servo at full CW position
                        b_servo.duty(b_home_from_CW)   # bottom servo moves to the extra home position, from CCW
                
                elif direction=='CW':                  # case the set direction is CW
                    if b_servo_CCW_pos==True:          # boolean of bottom servo at full CW position
                        b_servo.duty(b_home_from_CCW)  # bottom servo moves to the extra home position, from CW
                
                sleep_ms(b_rotate_time)                # time for the bottom servo to reach the extra home position
                b_servo.duty(b_home)                   # bottom servo moves to the home position, releasing then the tensions
                sleep_ms(b_rel_time)                   # time for the servo to release the tensions
                b_servo_stopped=True                   # boolean of bottom servo at location the lifter can be operated
                b_servo_home=True                      # boolean of bottom servo at home
                b_servo_CW_pos=False                   # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False                  # boolean of bottom servo at full CCW position
                
                open_cover()                           # top cover is raised in open position






def rotate_CCW_CW_test(direction):
    """ Function that rotates the cube holder toward CCW or CCW position for test purpose (fine tuning servo settings from GUI).
        After the main rotation, the cube holder comes back to the intended position; This approach is used to release
        the friction (cube holder - cube - top cover).
        The cube might be contrained by the top cover, if the top cover has been lowered via the GUI."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:
        if b_servo_operable==True:             # variable to block/allow bottom servo operation              
            b_servo_stopped=False              # boolean of bottom servo at location the lifter can be operated
            if b_servo_home==False:            # case the cube holder is not in home position
                rotate_home_test()             # the cube holder is rotated to home
            
            if direction=='CCW':               # case the set direction is CCW
                b_servo.duty(b_servo_CCW)      # bottom servo moves to the most CCW position
                sleep_ms(b_rotate_time)        # time for the bottom servo to reach the most CCW position
                b_servo.duty(b_servo_CCW_rel)  # bottom servo moves slightly to release the tensions
                sleep_ms(b_rel_time)           # time for the servo to release the tensions
                b_servo_CCW_pos=True           # boolean of bottom servo at full CCW position
                
            elif direction=='CW':              # case the set direction is CW
                b_servo.duty(b_servo_CW)       # bottom servo moves to the most CCW position
                sleep_ms(b_rotate_time)        # time for the bottom servo to reach the most CCW position
                b_servo.duty(b_servo_CW_rel)   # bottom servo moves slightly to release the tensions
                sleep_ms(b_rel_time)           # time for the servo to release the tensions
                b_servo_CW_pos=True            # boolean of bottom servo at full CW position
                
            b_servo_stopped=True               # boolean of bottom servo at location the lifter can be operated
            b_servo_home=False                 # boolean of bottom servo at home
 





def rotate_home_test():
    """ Function that spins the cube holder to home position for test purpose (fine tuning servo settings from GUI)
        The cube holder stops to home position, without making extra rotation.
        The cube might be contrained by the top cover, if the top cover has been lowered via the GUI."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    current_pos=b_servo.duty()                         # retrieve current servo position (last duty-cycle value sent...)
    if current_pos < b_home:                           # case current position has PWM < home, meaning it is at CCW
        direction='CW'                                 # direction is set to CW
    elif current_pos > b_home:                         # case current position has PWM > home, meaning it is atCCW
        direction='CCW'                                # direction is set to CCW
    else:                                              # case current position has PWM == home, meaning it is at home
        return                                         # the function is terminated
    
    if not stop_servos:                                # case there is not a stop request for servos
        if b_servo_operable==True:                     # variable to block/allow bottom servo operation
            if b_servo_home==False:                    # boolean of bottom servo at home
                
                if direction=='CCW':                   # case the set direction is CCW
                    if b_servo_CW_pos==True:           # boolean of bottom servo at full CW position
                        b_servo.duty(b_home_from_CW)   # bottom servo moves to the extra home position, from CCW
                
                elif direction=='CW':                  # case the set direction is CW
                    if b_servo_CCW_pos==True:          # boolean of bottom servo at full CW position
                        b_servo.duty(b_home_from_CCW)  # bottom servo moves to the extra home position, from CW
                
                sleep_ms(b_rotate_time)                # time for the bottom servo to reach the extra home position
                b_servo.duty(b_home)                   # bottom servo moves to the home position, releasing then the tensions
                sleep_ms(b_rel_time)                   # time for the servo to release the tensions
                b_servo_stopped=True                   # boolean of bottom servo at location the lifter can be operated
                b_servo_home=True                      # boolean of bottom servo at home
                b_servo_CW_pos=False                   # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False                  # boolean of bottom servo at full CCW position






def check_moves(moves, debug):
    """ Function that counts the total servo moves, based on the received moves string.
        This function also verifies if the move string is compatible with servo contrained within 180 deg range (from -90 to 90 deg):
        Not possible to rotate twice time +90deg (or -90deg) from the center, nor 3 times +90deg (or -90deg) from one of the two extremes."""
    
    servo_angle=0                                                 # initial angle is set to zero, as this is the starting condition at string receival
    servo_angle_ok=True                                           # boolean to track the check result
    tot_moves=0                                                   # counter for the total amount of servo moves (1x complete flip, 1x each 90 deg cube spin or 1st layer rotation)
    
    for i in range(len(moves)):                                   # iteration over all the string characters
        if moves[i]=='1':                                         # case direction is CW 
            if moves[i-1] == 'R' or moves[i-1] == 'S':            # case the direction refers to cube spin or layer rotation
                servo_angle+=90                                   # positive 90deg angle are added to the angle counter
                tot_moves+=1                                      # counter is increased

        elif moves[i]=='3':                                       # case direction is CW 
            if moves[i-1] == 'R' or moves[i-1] == 'S':            # case the direction refers to cube spin or layer rotation
                servo_angle-=90                                   # negative 90deg angle are subtracted from the angle counter
                tot_moves+=1                                      # counter is increased

        elif moves[i]=='F':                                       # case there is a flip on the move string
            tot_moves+=int(moves[i+1])                            # counter is increased
        
        if servo_angle<-90 or servo_angle>180:                    # case the angle counter is out of range
            if debug:
                print(f'servo_angle out of range at string pos:{i}')  # info are printed
            servo_angle_ok=False                                  # bolean of results is updated
            break                                                 # for loop is interrupted
    
    if servo_angle_ok==True:                                      # case the coolean is still positive
        if debug:
            print('servo_angle within range')                     # positive result is printed
        pass                                                      # this command in case of commenting out the print command

    return servo_angle_ok, tot_moves # total counter is increased






def check_uart(debug, stop_btn, btn_ref):
    """ Function that checks if there are info received by the uart.
        When the robot is solving, this function is frequently called to check whether a STOP request has been received."""
    
    global stop_servos
    
    strMsg = ''                                      # Create a local variable to hold the receive data in serial
    
    if stop_btn.read()<btn_ref:                      # case the touch button is touched
        stop_servos= True                            # boolean for stopping the servo is set true
        return stop_servos                           # function is interrupted
        
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   # case there is data on the uart
        ch = sys.stdin.read(1).strip('\n')           # received character
        if '[' in ch:                                # case the received character is an open square bracket                     
            strMsg=''                                # the string variable for the full message is emptied
        elif ']' in ch:                              # case the received character is a close square bracket  
            if 'stop' in strMsg:                     # case the message string variable contain the stop word
                stop_servos= True                    # boolean for stopping the servo is set true
                if debug:
                    print("received stop command")
                return stop_servos                           # function is interrupted
        else:                                        # case the received character is not a square bracket
            strMsg=strMsg+ch                         # the received character is added to the message






def update_moves(start_moves, remaining_moves, index, debug=False):
    """ Function that keeps track of the number of remaining movement to solve the cube.
        This info is feedback to the uart, so that the GUI updates accordingly."""
    
    remaining_moves-=1            # counter is decreased
    print('i_'+ str(index))       # message to UART (via print function) on cube solving progress
    return remaining_moves        # the function is terminated by returning the remaining moves quantity






def fun(debug):
    """ Cube holder spins, to make some vittory noise once the cube is solved."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos, fun_status
    
    
    if t_top_cover != 'open':                        # variable to track the top cover/lifter position
        if not stop_servos:                          # case there is not a stop request for servos
            if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
                b_servo_operable=False               # variable to block/allow bottom servo operation
                t_servo.duty(t_servo_open)           # top servo is positioned in open top cover position, from close position
                sleep_ms(t_flip_open_time)           # time for the top servo to reach the open top cover position
                t_top_cover='open'                   # variable to track the top cover/lifter position
                b_servo_operable=True                # variable to block/allow bottom servo operation
        
    if b_servo_home==False:                          # case bottom servo is not home
        if not stop_servos:                          # case there is not a stop request for servos
            if b_servo_operable==True:               # variable to block/allow bottom servo operation
                if b_servo_CW_pos==True:             # boolean of bottom servo at full CW position
                    b_servo.duty(b_home_from_CW)     # bottom servo moves to the extra home position, from CCW
            
                elif b_servo_CCW_pos==True:          # boolean of bottom servo at full CCW position
                    b_servo.duty(b_home_from_CCW)    # bottom servo moves to the extra home position, from CW
                
                sleep_ms(b_spin_time)                # time for the bottom servo to reach the extra home position
                b_servo_home=True                    # boolean bottom servo is home

    
    sleep_ms(250)                                    # little delay, to timely separate from previous robot movements
    runs=8                                           # number of sections
    b_delta=int((b_home-b_servo_CCW)/runs)           # PWM amplitute per section
    t_delta=int(b_spin_time/runs)                    # time amplitude per section
    k=1                                              # coefficient that treats differently the first rotation, as it start from home
    
    for i in range(4,runs):                          # iteration over the sections, starting somehome from the middle
        b_target_CCW=b_home+int(b_delta*(runs-i))    # PWM target calculation for CCW postion
        delay_time=int(t_delta*(runs-i))             # time calculation for the servo movement
        if not stop_servos:                          # case there is not a stop request for servos
            b_servo_stopped=False                    # boolean of bottom servo at location the lifter can be operated
            b_servo.duty(b_target_CCW)               # bottom servo moves to the target_CCW position
            sleep_ms(k*(delay_time+i*10))            # time for the bottom servo to reach the target position
            k=2                                      # coefficient to double the time, at each move do not start from home anymore
            b_target_CW=b_home-int(b_delta*(runs-i)) # PWM target calculation for CW postion
            delay_time=int(t_delta*(runs-i))         # time calculation for the servo movement
        if not stop_servos:                          # case there is not a stop request for servos
            b_servo_operable=False                   # variable to block/allow bottom servo operation
            b_servo.duty(b_target_CW)                # bottom servo moves to the target_CW position
            sleep_ms(k*(delay_time+i*10))            # time for the bottom servo to reach the target position
            b_servo_stopped=True                     # boolean of bottom servo at location the lifter can be operated
    
    if not stop_servos:                              # case there is not a stop request for servos
        b_servo_stopped=False                        # boolean of bottom servo at location the lifter can be operated
        b_servo.duty(b_home)                         # bottom servo moves to home position
        sleep_ms(k*(delay_time+i*10))                # time for the bottom servo to reach home position
        b_servo_stopped=True                         # boolean of bottom servo at location the lifter can be operated
        b_servo_home=True                            # boolean bottom servo is home
    
    fun_status=True                                  # boolean to track the robot fun status, it is True after solving the cube :-)
    stopping_servos(debug)
    
    return fun_status
        
    




def servo_solve_cube(moves, debug, stop_btn, btn_ref):
    """ Function that translates the received string of moves, into servos sequence activations.
        This is substantially the main function."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, stop_servos
    start_time=time()
    end_time=time()
    
    # the received string is analyzed if compatible with servo rotation contraints, and amount of movements
    servo_angle_ok, tot_moves = check_moves(moves, debug)    
    if debug:
        print(f'total amount of servo movements: {tot_moves}\n')    
    
    start_moves=tot_moves                                    # start moves is the calculates movements prior starting
    remaining_moves=tot_moves                                # at start the remaining moves are obviosly all the moves
    
    string_len=len(moves)                                    # number of characters in the moves string
    for i in range(string_len):                              # iteration over the characters of the moves string
        stop_servos=check_uart(debug, stop_btn, btn_ref)     # check is a stop request has been received at uart
        if stop_servos:                                      # case there is a stop request for servos
            break                                            # the foor loop in interrupted
        
        
        if moves[i]=='F':                                    # case there is a flip on the move string
            flips=int(moves[i+1])                            # number of flips
            if debug:
                print(f'To do F{flips}')                     # for debug
            
            for flip in range(flips):                        # iterates over the number of requested flips
                stop_servos=check_uart(debug, stop_btn, btn_ref)     # check is a stop request has been received at uart           
                if stop_servos:                              # case there is a stop request for servos
                    break                                    # the foor loop in interrupted
                
                flip_up()                                    # lifter is operated to flip the cube
                remaining_moves = update_moves(start_moves, remaining_moves, i, debug)       # counter is decreased, and remaining moves sent to uart

                if flip<(flips-1):                           # case there are further flippings to do
                    flip_to_open()                           # lifter is lowered stopping the top cover in open position (cube not constrained)

# alternative choice
#                     flip_to_close()   # lifter is lowered stopping the top cover in close position (cube constrained, for better facelets alignment)
                
                if flip==(flips-1) and string_len-(i+2)>0:   # case it's the last flip and there is a following command on the move string
                    if moves[i+2]=='R':                      # case the next action is a 1st layer cube rotation
                        flip_to_close()                      # top cover is lowered to close position
                    elif moves[i+2]=='S':                    # case the next action is a cube spin
                        flip_to_open()                       # top cover is lowered to open position
        


        elif moves[i]=='S':                        # case there is a cube spin on the move string
            direction=int(moves[i+1])              # rotation direction is retrived
            if debug:
                print(f'To do S{direction}')       # for debug

            if direction==3:                       # case the direction is CCW
                set_dir='CCW'                      # CCW directio is assigned to the variable
            else:                                  # case the direction is CW
                set_dir='CW'                       # CW directio is assigned to the variable
            
            if b_servo_home==True:                 # case bottom servo is at home
                spin_out(set_dir)                  # call to function to spin the full cube to full CW or CCW
                remaining_moves = update_moves(start_moves, remaining_moves, i, debug)      # counter is decreased, and remaining moves sent to uart
            
            elif b_servo_CW_pos==True or b_servo_CCW_pos==True:   # case the bottom servo is at full CW or CCW position
                    spin_home()                                   # call to function to spin the full cube toward home position
                    remaining_moves = update_moves(start_moves, remaining_moves, i, debug)  # counter is decreased, and remaining moves sent to uart



        elif moves[i]=='R':                        # case there is a cube 1st layer rotation
            direction=int(moves[i+1])              # rotation direction is retrived   
            if debug:
                print(f'To do R{direction}')       # for debug

            if direction==3:                       # case the direction is CCW
                set_dir='CCW'                      # CCW directio is assigned to the variable
            else:                                  # case the direction is CW
                set_dir='CW'                       # CW directio is assigned to the variable
            
            if b_servo_home==True:                 # case bottom servo is at home
                rotate_out(set_dir)                # call to function to rotate cube 1st layer on the set direction, moving out from home
                remaining_moves=update_moves(start_moves, remaining_moves, i, debug)        # counter is decreased, and remaining moves sent to uart               
            
            elif b_servo_CW_pos==True:             # case the bottom servo is at full CW position
                if set_dir=='CCW':                 # case the set direction is CCW
                    rotate_home(set_dir)           # call to function to spin the full cube toward home position
                    remaining_moves = update_moves(start_moves, remaining_moves, i, debug)  # counter is decreased, and remaining moves sent to uart
                
            elif b_servo_CCW_pos==True:            # case the bottom servo is at full CCW position
                if set_dir=='CW':                  # case the set direction is CW
                    rotate_home(set_dir)           # call to function to spin the full cube toward home position
                    remaining_moves = update_moves(start_moves, remaining_moves, i, debug)  # counter is decreased, and remaining moves sent to uart
    
    if stop_servos:                                # case there is a stop request for servos 
        if debug:
            print("\nRobot stopped")
        robot_status='Robot_stopped'               # string variable indicating how the servo_solve_cube function has ended
        stopping_servos(debug)                     # call the stop servo function
        
    elif not stop_servos:                          # case there is not a stop request for servos
        if debug:
            print(f"\nCompleted all the servo movements")
        robot_status='Cube_solved'                 # string variable indicating how the servo_solve_cube function has ended

    robot_time=(time()-start_time)
    
    return robot_status, robot_time



     
     




if __name__ == "__main__":
    """ example of robot movements for a given moves string:
        F2R1S3R1S3S3F1R1F2R1S3S3F1R1S3R1F3R1S3R1S3S3F3R1S3F1R1S3R1F3R1S3R1S3F3R1S3R1
        F1R3S1F1R3F1S1R3S3F1R1S3R1F3S1R3F1R1S3S3F3R1S3R1F3R1S3R1S3F1S1R3S1F3R3F1R1S3'  """
    
    
    moves='F2R1S3R1S3S3F1R1F2R1S3S3F1R1S3R1F3R1S3R1S3S3F3R1S3F1R1S3R1F3R1S3R1S3F3R1S3R1F1R3S1F1R3F1S1R3S3F1R1S3R1F3S1R3F1R1S3S3F3R1S3R1F3R1S3R1S3F1S1R3S1F3R3F1R1S3'
    # the complete the moves of this string CUBOTino takes 1m:12secs (02/04/2022)
    # settings (54,68,76,0,450,500,400,150,51,76,101,2,3,550,600,50)
    
    
    debug=False
    
    stop_btn=TouchPad(Pin(32))                # create the touch pad button object in GPIO 13
    stop_btn_value=stop_btn.read()            # create a global variable for the touchPad button variable
    btn_ref = 0                               # button threshold reference is set initially to zero
    n=1000                                    # number of touch buttong reading to perform as initial reference
    for i in range(n):                        # iteration for 1000 readings
        btn_ref+=stop_btn_value               # touch pad value is added to the reference at each iteration                                
    btn_ref=btn_ref//n//2                     # button threshold reference is set at 50% the average value during previous iteration    if debug:
    if debug:
        print("ref_stop_btn_value:",btn_ref)
    
    
    
    if init_servo(debug):                                                # servos are initialized
        print('servo init done\n')                                       # print feedback
        
        robot_status, robot_time = servo_solve_cube(moves, debug, stop_btn, btn_ref)   # robot solver is called                                                     # time after the robot movements

        if robot_status == 'Cube_solved':                                # case the robot solver returns Cube_solved in the robot_status
            print("\nCube is solved")                                    # print the status as feedback
            print(f"Solving time: {robot_time} secs")                    # print the solving time as feedback
            fun(debug)                                                        # fun function ... some cube_holder movements to get attention
            
        
        elif robot_status == 'Robot_stopped':                            # case the robot solver returns Robot_stopped in the robot_status
            print(f"\nRobot has been stopped, after {robot_time} secs")  # print the status as feedback
