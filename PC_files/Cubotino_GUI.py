#!/usr/bin/env python
# coding: utf-8

"""
#############################################################################################################
# Andrea Favero          Rev. 17 January 2024
# 
# GUI for CUBOTino, a very small and simple Rubik's cube solver robot.
# 
# The GUI is built over the one from Mr. Kociemba: https://github.com/hkociemba/RubiksCube-TwophaseSolver;
# Credits to Mr. Kociemba for the GUI, in addition to the TwoPhasesSolver!
# 
# This GUI aims to get the cube status, to communicate with the Kociemba TwophaseSolver (getting the cube solving
#  manoeuvres) and finally to interact with the Cubotino robot;
# During the cube solving process by the robot, the GUI updates the cube status sketch accordingly.
# 
# The cube status can be entered manually, or it can be acquired via a webcam by manually presenting the cube faces.
# The GUI has a setting page, with the most relevant settings for the robot, and the webcam
# 
# Manoeuvres to CUBOTino robot are sent as a tipical cube solution string, after removing empty spaces,
#  with "<" and ">" characters used as conwtrols to ensure the full string is received by the robot UART.
# Other commands to the robot are sent in between square brackets, to only process complete strings
#
#############################################################################################################
"""

# __version__ variable
version = '4.6'


################  setting argparser ####################################################################################
import argparse

# argument parser object creation
parser = argparse.ArgumentParser(description='Arguments for Cubotino_GUI')

# --version argument is added to the parser
parser.add_argument('-v', '--version', help='Display version.', action='version',
                    version=f'%(prog)s ver:{version}')

# --debug argument is added to the parser
parser.add_argument("-d", "--debug", action='store_true',
                    help="Activates printout of settings, variables and info for debug purpose.")

# --estimate argument is added to the parser
parser.add_argument("-e", "--estimate", action='store_true',
                    help="Activates ehe estimation of last two cube facelets position/contour.")

# --delay argument is added to the parser
parser.add_argument("--delay", type=int,
                    help=f"Enter the time (2 to 30s) to delay facelets detection at cube face change."
                    " Default 10s if this argument is not used.")

args = parser.parse_args()   # argument parsed assignement
# ######################################################################################################################




################  processing arguments  #################################################################################
print('\n\n\n===================  Cubotino_GUI AF (17 January 2024)  ============================')
debug = False                     # flag to enable/disable the debug related prints
if args.debug != None:            # case 'debug' argument exists
    if args.debug:                # case the Cubotone.py has been launched with 'debug' argument
        debug = True              # flag to enable/disable the debug related prints is set True
        print('Debug prints activated')   # feedback is printed to the terminal

estimate_fclts = False            # flag to enable/disable the estimation of last two cube facelets position/contour
if args.estimate != None:         # case 'estimate' argument exists
    if args.estimate:             # case the Cubotino_GUI.py has been launched with 'estimate' argument
        estimate_fclts = True     # flag to estimate last two cube facelets position/contour is set True
        print('Facelets position estimation activated')  # feedback is printed to the terminal

delay = 10                        # delay to facelets detection at faces change 
if args.delay != None:            # case 'delay' argument exists
    delay = abs(int(args.delay))  # delay to facelets detection at faces change e
    if delay < 2:                 # case the provided time is smaller than 2 seconds
        delay = 2                 # 2 seconds are assigned
    elif delay > 30:              # case the provided time is bigger than 30 seconds
        delay = 30                # 30 seconds are assigned
if debug:                         # case debug has been activate
    print(f'Delay of {delay}s to start reading the facelets after a cube face change')
print()
# ######################################################################################################################




# ################################## Imports  ##########################################################################
# custom libraries
import Cubotino_webcam as cam           # recognize cube status via a webcam (by Andrea Favero)
import Cubotino_moves as cm             # translate a cube solution into CUBOTino robot moves (by Andrea Favero)


import sys                                            # library import to check if another lybrary is already imported
if 'solver' in sys.modules or 'cm' in globals():      # case the library 'solver'is already imported
    solver_already_imported = True                    # booleand variable is set true
else:                                                 # case the library 'solver'is not already imported
    solver_already_imported = False                   # booleand variable is set true

try:                                                  # attempt
    import solver as sv                               # import Kociemba solver, copied in robot folder
    import face, cubie                                # import other Kociemba solver library parts, copied in robot folder
    if not solver_already_imported:                   # case the solver was not already imported                                 
        print('imported the installed twophase solver...')  # feedback is printed to the terminal
    solver_found = True                               # boolean to track no exception on import the copied solver
except:                                               # exception is raised if no library in folder or other issues
    solver_found = False                              # boolean to track exception on importing the copied solver
    
if not solver_found:                                  # case the library was not in folder
    try:                                              # attempt
        import twophase.solver as sv                  # import Kociemba solver installed
        import twophase.face as face                  # import face Kociemba solver library part, installed
        import twophase.cubie as cubie                # import cubie Kociemba solver library part, installed
        if not solver_already_imported:               # case the solver was not already imported 
            print('imported the copied twophase solver...') # feedback is printed to the terminal
        twophase_solver_found = True                  # boolean to track no exception on import the installed solver
    except:                                           # exception is raised if no library in venv or other issues
        twophase_solver_found = False                 # boolean to track exception on importing the installed solver

if not solver_found and not twophase_solver_found:    # case no one solver has been imported
    if solver_already_imported:                       # case the solver was not already imported 
        print('\n(Kociemba) twophase solver not found')    # feedback is printed to the terminal
    quit()

# print()


# python libraries, normally distributed with python
import tkinter as tk                 # GUI library
from tkinter import ttk              # GUI library
import datetime as dt                # date and time library used as timestamp on a few situations (i.e. data log)
import threading                     # threading library, to parallelize uart data 
import time                          # time library is imported
import os                            # os is imported to ensure the file presence, check/make

# python library, to be installed (pyserial)
import serial.tools.list_ports

########################################################################################################################





# #################### couple of functions to load settings to global variables  #############################

def get_settings(settings):
    """ Function to assign the servo settings to individual variables, based on a tupple in argument."""
    
    global t_servo_flip, t_servo_open, t_servo_close, t_servo_release, b_rotate_time, b_spin_time, b_rel_time
    global t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time
    global b_servo_CCW, b_home, b_servo_CW, b_extra_sides, b_extra_home
    global t_srv_pw_range, b_srv_pw_range, last_t_srv_pw_range, last_b_srv_pw_range
    global s_pwm_flip_min, s_pwm_flip_max, s_pwm_open_min, s_pwm_open_max
    global s_pwm_close_min, s_pwm_close_max, s_pwm_ccw_min, s_pwm_ccw_max
    global s_pwm_home_min, s_pwm_home_max, s_pwm_cw_min, s_pwm_cw_max
    global robot_settings
    
    t_servo_flip = settings[0]          # top servo pos to flip the cube on one of its horizontal axis
    t_servo_open = settings[1]          # top servo pos to free up the top cover from the cube
    t_servo_close = settings[2]         # top servo pos to constrain the top cover on cube mid and top layer            
    t_servo_release = settings[3]       # top servo rotation bacl from close to release tension
    t_flip_to_close_time = settings[4]  # time to lower the cover/flipper from flip to close position
    t_close_to_flip_time = settings[5]  # time to raise the cover/flipper from close to flip position
    t_flip_open_time = settings[6]      # time to raise/lower the flipper between open and flip positions
    t_open_close_time = settings[7]     # time to raise/lower the flipper between open and close positions
    
    b_servo_CCW = settings[8]           # bottom servo position when fully CW 
    b_home = settings[9]                # bottom servo home position
    b_servo_CW = settings[10]           # bottom servo position when fully CCW
    b_extra_sides = settings[11]        # bottom servo position extra rotation at CW and CCW
    b_extra_home = settings[12]         # bottom servo position extra rotation at home
    b_spin_time = settings[13]          # time needed to the bottom servo to spin about 90deg
    b_rotate_time = settings[14]        # time needed to the bottom servo to rotate about 90deg
    b_rel_time = settings[15]           # time needed to the servo to rotate slightly back, to release tensions
       
    t_srv_pw_range = settings[16] # top servo pulse width range (string variable)
    b_srv_pw_range = settings[17] # bottom servo pulse width range (string variable)
    
    if t_srv_pw_range.lower() == 'small':
        s_pwm_flip_min = 45       # min slider value for flip PWM setting (unit/1024*20 ms)
        s_pwm_flip_max = 60       # max slider value for flip PWM setting (unit/1024*20 ms)
        s_pwm_open_min = 60       # min slider value for open PWM setting (unit/1024*20 ms)
        s_pwm_open_max = 75       # man slider value for open PWM setting (unit/1024*20 ms)
        s_pwm_close_min = 65      # min slider value for close PWM setting (unit/1024*20 ms)
        s_pwm_close_max = 90      # max slider value for close PWM setting (unit/1024*20 ms)
        
    elif t_srv_pw_range.lower() == 'large':
        s_pwm_flip_min = 13       # min slider value for flip PWM setting (unit/1024*20 ms)
        s_pwm_flip_max = 43       # max slider value for flip PWM setting (unit/1024*20 ms)
        s_pwm_open_min = 43       # min slider value for open PWM setting (unit/1024*20 ms)
        s_pwm_open_max = 73       # man slider value for open PWM setting (unit/1024*20 ms)
        s_pwm_close_min = 53      # min slider value for close PWM setting (unit/1024*20 ms)
        s_pwm_close_max = 103     # max slider value for close PWM setting (unit/1024*20 ms)
    
    if b_srv_pw_range.lower() == 'small':
        s_pwm_ccw_min = 40        # min slider value for CCW PWM setting (unit/1024*20 ms)
        s_pwm_ccw_max = 62        # max slider value for CCW PWM setting (unit/1024*20 ms)
        s_pwm_home_min = 63       # min slider value for Home PWM setting (unit/1024*20 ms)
        s_pwm_home_max = 87       # max slider value for Home PWM setting (unit/1024*20 ms)
        s_pwm_cw_min = 88         # min slider value for CW PWM setting (unit/1024*20 ms)
        s_pwm_cw_max = 110        # max slider value for CW PWM setting (unit/1024*20 ms)
    
    elif b_srv_pw_range.lower() == 'large':
        s_pwm_ccw_min = 3         # min slider value for CCW PWM setting (unit/1024*20 ms)
        s_pwm_ccw_max = 47        # max slider value for CCW PWM setting (unit/1024*20 ms)
        s_pwm_home_min = 49       # min slider value for Home PWM setting (unit/1024*20 ms)
        s_pwm_home_max = 97       # max slider value for Home PWM setting (unit/1024*20 ms)
        s_pwm_cw_min = 99         # min slider value for CW PWM setting (unit/1024*20 ms)
        s_pwm_cw_max = 143        # max slider value for CW PWM setting (unit/1024*20 ms)

    last_t_srv_pw_range = t_srv_pw_range  # top servos pulse width setting is assigned as last one used
    last_b_srv_pw_range = b_srv_pw_range  # bottom servos pulse width setting is assigned as last one used
    
    # tuple with all the servos and webcam settings saved on a txt file
    robot_settings=(t_servo_flip,t_servo_open,t_servo_close,t_servo_release, t_flip_to_close_time,
                    t_close_to_flip_time, t_flip_open_time,t_open_close_time,
                    b_servo_CCW,b_home,b_servo_CW,b_extra_sides,b_extra_home,b_spin_time,b_rotate_time,b_rel_time,
                    t_srv_pw_range, b_srv_pw_range, last_t_srv_pw_range, last_b_srv_pw_range,
                    s_pwm_flip_min, s_pwm_flip_max, s_pwm_open_min, s_pwm_open_max,
                    s_pwm_close_min, s_pwm_close_max, s_pwm_ccw_min, s_pwm_ccw_max,
                    s_pwm_home_min, s_pwm_home_max, s_pwm_cw_min, s_pwm_cw_max)
    
#     print(robot_settings)







def get_cam_settings(cam_settings):
    """ Function to assign the cam related settings into individual variables, based on the tupple in argument."""
    
    global cam_number, cam_width, cam_height, cam_crop, facelets_in_width #, cam_settings
    
    cam_number = cam_settings[0]         # cam number 
    cam_width = cam_settings[1]          # cam width (pixels) 
    cam_height = cam_settings[2]         # cam height (pixels)
    cam_crop = cam_settings[3]           # amount of pixel to crop at the frame right side 
    facelets_in_width = cam_settings[4]  # min number of facelets side, in frame width, to filter out too smal squares






def read_settings(file):
    """ Function to read text files with the parameters, and to return a list of them.
        All the settings are separated bty comma, and contained between a couple of brackets."""
    
    settings=[]                                        # empty list to store the list of settings    
    try:                                               # tentative
        with open(file, "r") as f:                     # open the file text file in read mode
            data = f.readline()                        # data is on first line
            data = data.replace(' ','')                # empty spaces are removed
            if '(' in data and ')' in data:            # case the dat contains open and close parenthesis
                data_start = data.find('(')            # position of open parenthesys in data
                data_end = data.find(')')              # position of close parenthesys in data
                data = data[data_start+1:data_end]     # data in between parenthesys is assigned, to the same string variable
                data_list=data.split(',')              # data is split by comma, becoming a list of strings        
                for setting in data_list:              # iteration over the list of strings
                    setting.lower().strip()            # setting string is lowered and stripped
                    if setting.isdigit():              # case the setting looks like a digit
                        settings.append(int(setting))  # each str setting changed to int and appended to the list of settings
                    else:                              # case the setting does not look like a digit
                        if 'small' in setting:         # case the setting is equal to 'small'
                            settings.append('small')   # string setting appends 'small' to the list of settings
                        elif 'large' in setting:       # case the setting is equal to 'large'
                            settings.append('large')   # string setting appends 'large' to the list of settings
                if len(data)==0:                       # case no one setting has been added to the list
                    print("File", file, "does not contain settings")  # print a feedback to the terminal
    except Exception as ex:                            # case the tentative did not succeeded
        print("Something is wrong with file:", file, " or file missed:\r\n", ex)  # print a feedback to the terminal
    return settings                                    # returns the list of settings






def settings_update(data):
    """starting from the revision 4.1 the settings has 18 parameters instead of 16.
        When the project is updated via git pull update, the settings file are not overwritten.
        This function adds the 17th and 18th parameter to the setting files.
        The added parameters are those implicitly used up to the revision 4.0 """
    
    if str(data[-1]).isnumeric():           # check if last parameter is a number (situation up to ver 4.0)
        data.append('small')                # 17th parameter is added to the setting files
        data.append('small')                # 18th parameter is added to the setting files
        data_save = str(data)               # list of parameters is converted to string
        data_save=data_save.replace(" ","")                    # eventual empty spaces are removed from the data_save string
        data_save=data_save.replace("[","(").replace("]",")")  # square parentheses are exchanged with round ones
        try: 
            with open("Cubotino_settings_backup.txt", "w") as f:  # open the servos settings text backup file in write mode
                f.write(data_save)                                # data_save is saved
            print("updated Cubotino_settings_backup.txt to be compatible with 4.1 and later versions")
        except:
            print("Something went wrong while updating Cubotino_settings_backup.txt file")
        
        try: 
            with open("Cubotino_settings.txt", "w") as f:         # open the servos settings text file in write mode
                f.write(data_save)                                # data_save is saved
            print("updated Cubotino_settings.txt to be compatible with 4.1 and later versions")
        except:
            print("Something went wrong while updating Cubotino_settings.txt file")
        return data    # updated parameters are returned
        
    else:              # this case should be hardly possible to get
        print("ATTENTION: The code fails because of a Cubotino_GUI version change")
        print("           and it look like the update to a version >4.1 encounters problems\n")
        print("TIP: take note of the your settings at Cubotino_settings.txt, and")
        print("     add at the end two times the parameter 'small' with comma separation")





def wrong_settings_feedback(fname):
    """ Printout feedback to the terminal when the settings file parameters are not correct and cannot be
        corrected by the cose in a simple way."""
    
    print("\n\n\n===============================  ATTENTION  ==================================")
    print("===============================  ATTENTION  ==================================")
    print("  the file: ", fname)
    print("  does not contain valid parameters")
    print("  or there are too many or too less parameters\n")
    print("  try to correct them, otherwise the code will keep crashing\n")
    print("  the file should have between parenthese a list of 18 elements, similar to:")
    print("  (54,68,76,0,900,1000,800,300,51,76,101,2,3,1100,1200,100,'small','small')")
    print("==============================================================================")
    print("==============================================================================\n\n\n")

########################################################################################################################





# ################################## global variables and constants ###################################################

# width of a facelet in pixels, to draw the unfolded cube sketch
width = 70                     # many other widgets and positions, are based to this variable
gap = width//8                 # graphical gap between faces (to increase the gap reduces the denominator)

facelet_id = [[[0 for col in range(3)] for row in range(3)] for fc in range(6)]    # list for the facelets GUI id
colorpick_id = [0 for i in range(6)]                           # list for the colors GUI id
faceletter_id = [0 for i in range(6)]                          # list for the faces letters GUI id

t = ("U", "R", "F", "D", "L", "B")                                  # tuple with faces identifier and related order
base_cols = ["white", "red", "green", "yellow", "orange", "blue"]   # list with colors initially associated to the cube
gray_cols = ["gray50", "gray52", "gray54", "gray56", "gray58", "gray60"]  # list with gray nuances for cube scrambling
# gray_cols = ["white", "red", "green", "purple", "orange", "blue"]

cols = base_cols.copy()        # list with colors initially associated to the cube

curcol = None                  # current color, during colorpick function and sketch color assignment
last_col=5                     # integer variable to track last color used while scrolling over the sketch
last_facelet_id = 0            # integer variable to track last facelet colored uwith mouse wheel scroll
gui_read_var=""                # string variable used for the radiobuttons, on cube status detection mode
gui_webcam_num=0               # integer variable used for the webcam number at radiobutton
cam_number=0                   # integer variable used for the webcam id number in CV
cam_width=640                  # integer variable used for the webcam width (pixels)
cam_height=360                 # integer variable used for the webcam height (pixels)
cam_crop=0                     # integer variable used to crop the right frame side (pixels)
facelets_in_width =11          # integer variable for min amount of facelets in frame width, to filter small squares
mainWindow_ontop=False         # boolean to track the main window being (or not) the raised one

cube_solving_string=""         # string variable holding the cube solution manoeuvres, as per Kociemba solver
cube_solving_string_robot=""   # string variable holding the string sent to the robot
gui_buttons_state="active"     # string variable used to activate/deactivate GUI buttons according to the situations
robot_working=False            # boolean variable to track the robot working condition, initially False
serialData=False               # boolean variable to track when the serial data can be exchanged, initially False
robot_moves=""                 # string variable holding all the robot moves (robot manoeuvres)
cube_status={}                 # dictionary variable holding the cube status, for GUI update to robot permutations
left_moves={}                  # dictionary holding the remaining robot moves

timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')      # timestamp used on logged data and other locations


# read settings from text file, giving priority to personal settings (prevent issues after a git update)
fname = os.path.join('.','Cubotino_settings_backup.txt')     # backup file with servos settings
if os.path.isfile(fname):         # case the settings backup files exist (user have made personal settings)
    data = read_settings('Cubotino_settings_backup.txt')     # from servos backup txt file to list of settings
    datalen = len(data)
    if datalen > 0:               # case the file reading returned a list of settings
        if datalen <= 15 or datalen == 17 or datalen >18: # case the valid parameters found are <=15, ==17, >18
            wrong_settings_feedback(fname)  # a warning feedback is printed to the terminal
        elif datalen == 16:       # case the file has 16 elements it likely belongs to ver < 4.1
            data = settings_update(data)    # call a funtion that updates 'Cubotino_settings_backup.txt'
        if len(data) == 18:       # case the file has 18 valid parameters, as it should be from V4.1
           get_settings(data)     # call to the function that makes global these servos settings

else:             # case the settings backup files do not exist (user has not made personal settings yet)
    data = read_settings('Cubotino_settings.txt') # from servos settings txt file to list of settings
    datalen = len(data)
    if datalen > 0:               # case the file reading returned a list of settings
        if datalen <= 15 or datalen == 17 or datalen >18: # case the valid parameters found are <=15, ==17, >18
            wrong_settings_feedback('./Cubotino_settings.txt')  # a warning feedback is printed to the terminal
        elif datalen == 16:       # case the file has 16 elements it likely belongs to ver < 4.1
            data = settings_update(data)    # call a funtion that updates 'Cubotino_settings.txt'
        if len(data) == 18:       # case the file has 18 valid parameters, as it should be from V4.1
           get_settings(data)     # call to the function that makes global these servos settings
            

# read cam settings from text files, giving priority to personal settings (prevent issues after a git update)
fname = os.path.join('.','Cubotino_cam_settings_backup.txt') # backup file with cam settings
if os.path.isfile(fname):         # case the cam settings backup files exist (user have made personal settings)
    data = read_settings('Cubotino_cam_settings_backup.txt') # from cam backup txt file to list of settings
    if len(data) > 0:             # case the file reading returned a list of settings
        get_cam_settings(data)    # call to the function that makes global these cam settings 
else:             # case the settings backup files do not exist (user has not made personal settings yet)
    data = read_settings('Cubotino_cam_settings.txt') # from cam settings txt file to list 
    if len(data) > 0:             # case the file reading returned a list of settings
        get_cam_settings(data)    # call to the function that makes global these cam settings 

########################################################################################################################






# ################################################ Functions ###########################################################

def write_backup_settings(data):
    timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')    # timestamp used on logged data and other locations
    backup=timestamp+"_backup_"+data
    try: 
        with open("Cubotino_settings_backup.txt", "w") as f:  # open the servos settings text backup file in read mode
            f.write(backup)                              # data is on first line
        print("saved last settings to Cubotino_settings_backup.txt file")
    except:
        print("Something went wrong while saving Cubotino_settings_backup.txt file")






def show_window(window):
    """Function to bring to the front the window in argument."""
    
    global mainWindow_ontop, app_width, app_height
    
    if window==settingWindow:                  # case the request is to show the settings windows
        settingWindow.tkraise()                # settings windows is raised on top
        root.minsize(int(1*max(app_width,1250)), int(1.05*app_height))  # min GUI size, limiting resizing, for setting_window
        root.maxsize(int(1.2*max(app_width,1250)), int(1.3*app_height))  # max GUI size, limiting resizing, for setting_window
        root.resizable(True, True)                         # allowing the root windows to be resizeable
        mainWindow_ontop=False                 # boolean of main windows being on top is set false
        gui_sliders_update('update_sliders')   # calls the function to update the sliders on the (global) variables
        return                                 # function in closed
    
    elif window==mainWindow:                   # case the request is to show the main windows          
        window.tkraise()                       # main windows is raised on top
        root.minsize(int(0.9*app_width), int(0.9*app_height))    # min GUI size, limiting the resizing on screen
        root.maxsize(int(1.05*app_width), int(1.05*app_height))  # max GUI size, limiting the resizing on screen
        root.resizable(True, True)             # allowing the root windows to be resizeable
        mainWindow_ontop=True                  # boolean of main windows being on top is set true






def show_text(txt):
    """Display messages on text window."""
    
    gui_text_window.insert(tk.INSERT, txt)      # tk function for text insert
    gui_canvas.update_idletasks()               # canvas is re-freshed






def create_facelet_rects(a):
    """ Initialize the facelet grid on the canvas."""
    
    offset = ((1, 0), (2, 1), (1, 1), (1, 2), (0, 1), (3, 1))  # coordinates (in cube face units) for cube faces position
    
    for f in range(6):                                   # iteration over the six cube faces
        for row in range(3):                             # iteration over three rows, of cube facelests per face
            y = 20 + offset[f][1] * 3 * a + row * a + offset[f][1]*gap      # top left y coordinate to draw a rectangule
            for col in range(3):                         # iteration over three columns, of cube facelests per face
                x = 20 + offset[f][0] * 3 * a + col * a + offset[f][0]*gap  # top left x coordinate to draw a rectangule
                
                # the list of graphichal facelets (global variable) is populated, and initially filled in grey color
                facelet_id[f][row][col] = gui_canvas.create_rectangle(x, y, x + a, y + a, fill="grey65", width=2)
    
    for f in range(6):  # iteration over the 6 faces
        gui_canvas.itemconfig(facelet_id[f][1][1], fill=cols[f]) # centers face facelets are colored as per cols list
    
    face_letters(a)       # call the function to place URFDLB letters on face center facelets
    draw_cubotino()       # calls the funtion to draw Cubotino sketch






def face_letters(a):
    """ Add the face letter on central facelets."""
    
    offset = ((1, 0), (2, 1), (1, 1), (1, 2), (0, 1), (3, 1))  # coordinates (in cube face units) for cube faces position
    for f in range(6):                                         # iteration over the six cube faces
        y = 20 + offset[f][1] * 3 * a + a + offset[f][1]*gap   # y coordinate for text placement
        x = 20 + offset[f][0] * 3 * a + a + offset[f][0]*gap   # x coordinate for text placement
        
        # each of the URFDLB letters are placed on the proper cuvbe face
        faceletter_id[f]=gui_canvas.create_text(x + width // 2, y + width // 2, font=("", 18), text=t[f], fill="black")






def create_colorpick(a):
    """Initialize the "paintbox" on the canvas."""
    
    global curcol, cols
    
    cp = tk.Label(gui_canvas, text="color picking")      # gui text label informing the color picking concept
    cp.config(font=("Arial", 18))                        # gui text font is set
    
    # gui text label for color picking info is placed on the canvas
    hp_window = gui_canvas.create_window(2*gap+int(8.25 * width), 2*gap+int(6.45 * width), anchor="nw", window=cp)
    
    for i in range(6):                                   # iteration over the six cube faces
        x = 2*gap + int((i % 3) * (a + 15) + 7.65 * a)   # x coordinate for a color palette widget
        y = 2*gap + int((i // 3) * (a + 15) + 7 * a)     # y coordinate for a color palette widget
        
        # round widget, filled with color as per cols variable, and with tick border (20) of same gui background color
        colorpick_id[i] = gui_canvas.create_oval(x, y, x + a, y + a, fill=cols[i], width=20, outline="#F0F0F0")
        
        # the first widget is modified by reducing the border width and changing the borger color to a different gray
        gui_canvas.itemconfig(colorpick_id[0], width=5, outline="Grey55" ) 
        curcol = cols[0]    # the first color of cols tupple is assigned to the (global) current color variable






def draw_cubotino():
    """ Draw a cube and a Cubotino robot.
        This has a decorative purpose, aiming to clearly suggest the initial cube orientation on Cubotino
        (when the faces aren"t detected directly by the robot).
        Graphical widgets are all hard coded, and all referring to a single starting "(s) coordinate."""
    
    s= 5,5      # starting point coordinates to the gui_canvas2 origin
  
    # Cubotino frame
    # tuple with three tuples, holding the coordinated for the three Cubotino frame panels 
    frm_faces=((s[0],s[1]+140, s[0]+80, s[1]+190, s[0]+80, s[1]+125,s[0], s[1]+80),
               (s[0]+80, s[1]+190, s[0]+190, s[1]+105, s[0]+190, s[1]+80, s[0]+80,s[1]+125),
               (s[0]+80, s[1]+125, s[0]+190, s[1]+80, s[0]+110, s[1]+47, s[0],s[1]+80))    
    for i in range(3):      # iteration over the three tuples
        pts=frm_faces[i]    # coordinates
        gui_canvas2.create_polygon(pts, outline = "black", fill = "burlywood3", width = 1) # Cubotino frame panels drawn
    
    
    # cube on Cubotino
    # tuple with three tuples, holding the coordinated for the three faces of the cube on Cubotino sketch
    cube_faces=((s[0]+30, s[1]+80, s[0]+86, s[1]+111, s[0]+86, s[1]+42, s[0]+30, s[1]+16),
                (s[0]+86, s[1]+111, s[0]+150, s[1]+88, s[0]+150,s[1]+24, s[0]+86, s[1]+42),
                (s[0]+86, s[1]+42, s[0]+150, s[1]+24, s[0]+96, s[1]+1, s[0]+30, s[1]+16))
    for i in range(3):         # iteration over the three tuples with coordinates
        pts=cube_faces[i]      # coordinates
        gui_canvas2.create_polygon(pts, outline = "black", fill = "grey65", width = 2)  # Cube faces are drawn on Cubotino
    
    # draw cube lines, in between the facelets, on the Cubotino 
    thck=2                                                           # line thickness
    fclt_lines_pts=((s[0]+46, s[1]+24, s[0]+112, s[1]+9),    # U     # couple of coordinates per line, located at U face
                    (s[0]+65, s[1]+33, s[0]+131, s[1]+16),   # U
                    (s[0]+54, s[1]+12, s[0]+108, s[1]+36),   # U
                    (s[0]+73, s[1]+6, s[0]+131, s[1]+32),    # U
                    (s[0]+86, s[1]+65, s[0]+150, s[1]+46),   # R     # couple of coordinates per line, located at R face
                    (s[0]+86, s[1]+88, s[0]+150, s[1]+67),   # R
                    (s[0]+108, s[1]+36, s[0]+108, s[1]+104), # R
                    (s[0]+131, s[1]+32, s[0]+131, s[1]+97),  # R
                    (s[0]+30, s[1]+38, s[0]+86, s[1]+65),    # F     # couple of coordinates per line, located at F face
                    (s[0]+30, s[1]+59, s[0]+86, s[1]+88),    # F
                    (s[0]+47, s[1]+23, s[0]+47, s[1]+91),    # F
                    (s[0]+65, s[1]+32, s[0]+65, s[1]+100))   # F
    
    for i in range(len(fclt_lines_pts)):   # iteration over the tuple with coordinates for facelets lines
        gui_canvas2.create_line(fclt_lines_pts[i], width=thck)   # lines are drawn
        
    draw_cubotino_center_colors() # draw the cube center facelets with related colors






def draw_cubotino_center_colors():
    """ Fills the color on center facelets at cube on the Cubotino sketch (three visible sides)."""
    
    s= 5,5                # starting point coordinates to the gui_canvas2 origin
    fclt_col=[]           # emplty list to be populated with center faces colors on the unfolded cube sketch
    for i in range(3):    # iteration stops on three, as only the first three faces are presented on the Cubotino sketch
        fclt_col.append(gui_canvas.itemcget(facelet_id[i][1][1], "fill"))  # center faces colors are retrieved, and listed
    
    # tuple with three tuples, holding the coordinated for center faces of the three visible cube faces on Cubotino sketch 
    fclt_pts=((s[0]+70, s[1]+19, s[0]+89, s[1]+28, s[0]+107, s[1]+21, s[0]+90, s[1]+14),
              (s[0]+109, s[1]+80, s[0]+130, s[1]+74, s[0]+130, s[1]+53, s[0]+109, s[1]+59),
              (s[0]+47, s[1]+67, s[0]+64, s[1]+76, s[0]+64, s[1]+55, s[0]+47, s[1]+47))
    
    for i in range(3):  # iteration over the three tuples, for the center facelets of Cubotino sketch
        # Cube center faces colored on Cubotino
        gui_canvas2.create_polygon(fclt_pts[i], outline = "black", fill = fclt_col[i], width = 1) 






def get_definition_string():
    """Generate the cube definition string, from the facelet colors."""
    
    color_to_facelet = {}           # dict to store the color of the 54 facelets
    for i in range(6):              # iteration over the six cube faces
        # populate the dict connecting the face descriptors (UFRDLB) to center face colors (keys, 'white', 'red', etc)
        color_to_facelet.update({gui_canvas.itemcget(facelet_id[i][1][1], "fill"): t[i]})
    s = ""                          # empty string variable to be populated with the cube status
    for f in range(6):              # iteration over the six cube faces
        for row in range(3):        # iteration over the three rows of facelets 
            for col in range(3):    # iteration over the three columns of facelets
                
                # cube status string is generated by adding the colors retrieved from the 54 facelets (cube sketch)
                s += color_to_facelet[gui_canvas.itemcget(facelet_id[f][row][col], "fill")]
    return s    






def solve():
    """Connect to Kociemba solver to get the solving maneuver."""
    
    global cols, sv, b_read_solve, cube_solving_string, cube_defstr
    global cube_status, robot_moves, tot_moves, previous_move
    
    b_robot["state"] = "disable"                 # GUI robot button is disabled at solve() function start
    b_robot["relief"] = "sunken"                 # GUI robot button is sunk at solve() function start
    
    if gui_scramble_var.get():                   # case the scramble check box is checked
        if cols != gray_cols.copy():             # case the cube sketch is not made with gray colored facelets
            cols = gray_cols.copy()              # list with gray nuances is used instead of the cube colors
            try:
                for f in range(6):               # iteration over the six cube faces
                    for row in range(3):         # iteration over the three rows of facelets 
                        for col in range(3):     # iteration over the three columns of facelets
                            color=gray_cols[base_cols.index(gui_canvas.itemcget(facelet_id[f][row][col], "fill"))]
                            gui_canvas.itemconfig(facelet_id[f][row][col], fill=color)
            except:
                print("exception 1 at Cubotino_GUI.solve()")

    elif not gui_scramble_var.get():             # case the scramble check box is not checked
        if cols == gray_cols.copy():             # case the cube sketch is made with gray colored facelets
            cols = base_cols.copy()              # list with colors initially associated to the cube
            try:
                for f in range(6):               # iteration over the six cube faces
                    for row in range(3):         # iteration over the three rows of facelets 
                        for col in range(3):     # iteration over the three columns of facelets
                            color=base_cols[gray_cols.index(gui_canvas.itemcget(facelet_id[f][row][col], "fill"))]
                            gui_canvas.itemconfig(facelet_id[f][row][col], fill=color)
            except:
                print("exception 2 at Cubotino_GUI.solve()")
                
    for i in range(6):                   # iteration on six center facelets
        cols[i]= gui_canvas.itemcget(facelet_id[i][1][1], "fill")  # colors list updated as per center facelets on schreen
    draw_cubotino()                     # updates Cubotino cube sketch, with URF centers facelets colors
    
    gui_text_window.delete(1.0, tk.END)  # clears output window
    cube_defstr=""                       # cube status string is set empty
    cube_solving_string=""               # cube solving string is set empty
    
    try:
        cube_defstr = get_definition_string()+ "\n"      # cube status string is retrieved
        if not gui_scramble_var.get():                   # case the scramble check box is not checked
            show_text(f'Cube status: {cube_defstr}\n')   # cube status string is printed on the text window
            if debug:                                    # case debug has been activate
                if len(cube_defstr)>=54:                 # case the solution string has at least 54 characters
                    print(f'cube status, from sketch on screen: {cube_defstr}')
        else:                                            # case the scramble check box is checked
            show_text('Cube status: Random\n')           # random cube status is printed on the text window
            if debug:                                    # case debug has been activate
                print(f'cube status, from gray sketch on screen: {cube_defstr}')


                
    except:                                              # case the cube definition string is not returned 
        show_text("Invalid facelet configuration.\nWrong or missing colors.")  # feedback to user
        return  # function is terminated
    
    # Kociemba TwophaseSolver, running locally, is called with max_length=18 or timeout=2s and best found within timeout
    cube_solving_string = sv.solve(cube_defstr.strip(), 18 , 2)
    
    if debug:   # case debug has been activate
        print(f'cube solution string: {cube_solving_string}\n')     # feedback is printed to the terminal
        
    
    if cube_defstr=="":                                             # case there is no cube status string
        show_text("Invalid facelet configuration.\nWrong or missing colors.")  # feedback to user
    else:                                                           # case there is a cube status string
        if not gui_scramble_var.get():                              # case the scramble check box is not checked 
            show_text(f'Cube solution: {cube_solving_string}\n\n')  # solving string is printed on GUI text windows
        if gui_scramble_var.get():                                  # case the scramble check box is checked
            s = cube_solving_string                                 # shorter local variable name     
            s_start = s.find('(')                                   # position of open parenthesys in data
            s_end = s.find(')')                                     # position of close parenthesys in data
            manoeuvres = s[s_start+1:s_end-1]                       # cube manoeuvres are sliced from the cube solution
            show_text(f'Cube manoeuvres: {manoeuvres}\n\n')         # number of manoeuvres is printed on GUI text windows   
        
    if not 'Error' in cube_solving_string and len(cube_solving_string)>4:   # case there is a cube to be solved
        pos=cube_solving_string.find('(')      # position of the "(" character in the string
        solution=cube_solving_string[:pos]     # string is sliced, by removing the additional info from Kociemba solver
        solution=solution.replace(" ","")      # empty spaces are removed
        
        # robot moves dictionary, and total robot moves, are retrieved from the imported Cubotino_moves script
        robot_moves_dict, robot_moves, tot_moves = cm.robot_required_moves(solution, "")
        if not gui_scramble_var.get():                       # case the scramble check box is not checked
            show_text(f'Robot moves: {robot_moves}\n')       # robot moves string is printed on the text window
            if debug:                                        # case the debug checkcutton is selected
                print(f'Robot moves: {robot_moves}\n')       # feedback is printed to the terminal
                
        else:                                                # case the scramble check box is checked
            show_text(f'Robot moves: As per random cube\n')  # robot moves string is printed on the text window

        for key in range(len(cube_defstr.strip())):          # iteration over the cube status string
            cube_status[key]=cube_defstr[key]                # dict generation
        previous_move=0                                      # previous move set to zero

    gui_f2.update()                     # GUI f2 part is updated, to release eventual clicks on robot button
    b_robot["state"] = "active"         # GUI robot button is activated after solve() function
    b_robot["relief"] = "raised"        # GUI robot button is raised after solve() function
    gui_robot_btn_update()              # updates the cube related buttons status






def clean():
    """Restore the cube to a clean cube."""
    
    global cols, cube_solving_string
    
    cube_solving_string=""               # empty string variable to later hold the cube solution
    gui_text_window.delete(1.0, tk.END)  # clears the text window
    gui_scramble_var.set(0)
    
    cols = base_cols.copy()              # list with colors initially associated to the cube
    create_facelet_rects(width)          # cube sketch is refreshed
    
    for f in range(6):                   # iteration over the six cube faces
        for row in range(3):             # iteration over the three rows of facelets 
            for col in range(3):         # iteration over the three columns of facelets
                gui_canvas.itemconfig(facelet_id[f][row][col], fill=gui_canvas.itemcget(facelet_id[f][1][1], "fill"))
    
    draw_cubotino_center_colors()        # draw the cube center facelets with related colors, at Cubotino sketch
    gui_read_var.set("screen sketch")    # "screen sketch" activated at radiobutton, as of interest when clean()
    gui_robot_btn_update()               # updates the cube related buttons status           






def empty():
    """Remove the facelet colors except the center facelets colors."""
    
    global cols, cube_solving_string
    
    cube_solving_string=""                # empty string variable to later hold the cube solution
    gui_text_window.delete(1.0, tk.END)   # clears the text window
    
    gui_scramble_var.set(0)
    cols = base_cols.copy()               # list with colors initially associated to the cube
    create_facelet_rects(width)           # cube sketch is refreshed
    
    for f in range(6):                    # iteration over the six cube faces
        for row in range(3):              # iteration over the three rows of facelets 
            for col in range(3):          # iteration over the three columns of facelets
                if row != 1 or col != 1:  # excluded the center facelets of each face
                    gui_canvas.itemconfig(facelet_id[f][row][col], fill="grey65") # facelets are colored by gray

    draw_cubotino_center_colors()         # draw the cube center facelets with related colors, at Cubotino sketch
    gui_robot_btn_update()                # updates the cube related buttons status






def random():
    """Generate a random cube and sets the corresponding facelet colors."""
    
    global gui_read_var, cube_solving_string, cols, gui_buttons_state
    
    cube_solving_string=""                   # cube solving string is set empty
    gui_text_window.delete(1.0, tk.END)      # clears the text window
    gui_buttons_state = gui_buttons_for_cube_status("disable")   # GUI buttons (cube-status) are disabled
    
    cc = cubie.CubieCube()                   # cube in cubie reppresentation
    cc.randomize()                           # randomized cube in cubie reppresentation 
    fc = cc.to_facelet_cube()                # randomized cube is facelets reppresentation string
    
    if gui_scramble_var.get():               # case the scramble check box is checked
        cols = gray_cols.copy()              # list with gray nuances is used instead of the cube colors
    
    elif not gui_scramble_var.get():         # case the scramble check box is not checked
        cols = base_cols.copy()              # list with colors initially associated to the cube
    
    create_facelet_rects(width)              # cube sketch is refreshed to the colors 
    
    for i in range(6):                       # iteration on six center facelets
        cols[i]= gui_canvas.itemcget(facelet_id[i][1][1], "fill")  # colors list updated as center facelets on screen

    idx = 0                                  # integer index, set to zero
    for f in range(6):                       # iteration over the six cube faces
        for row in range(3):                 # iteration over the three rows of facelets 
            for col in range(3):             # iteration over the three columns of facelets

                # facelet idx, at the cube sketch, is colored as per random cube (and colors associated to the 6 faces) 
                gui_canvas.itemconfig(facelet_id[f][row][col], fill=cols[fc.f[idx]]) 
                idx += 1                     # index is increased

    redraw(str(fc))                          # cube sketch is re-freshed on GUI
    gui_read_var.set("screen sketch")        # "screen sketch" activated at radiobutton, as of interest when random()
    print('\n\n\n\n\n\n\n\n')
    if gui_scramble_var.get():               # case the scramble check box is checked
            # feeback is printed to the terminal
            print('=============================   random cube for scrambling   ==============================')
    else:                                    # case the scramble check box is not checked
        # feeback is printed to the terminal
        print('==========================   random cube on the screen sketch   ===========================')

    solve()                                  # solve function is called, because of the random() cube request
    draw_cubotino_center_colors()            # draw the cube center facelets with related colors, at Cubotino sketch
    gui_buttons_state = gui_buttons_for_cube_status("active")    # GUI buttons (cube-status) are actived






def redraw(cube_defstr):
    """Updates sketch cube colors as per cube status string."""
    
    cube_defstr=cube_defstr.strip()      # eventual empty spaces at string start/end are removed
    idx = 0                              # integer index, set to zero
    for f in range(6):                   # iteration over the six cube faces
        for row in range(3):             # iteration over the three rows of facelets 
            for col in range(3):         # iteration over the three columns of facelets
                # facelet idx, at the cube sketch, is colored as per cube_defstr in function argument
                gui_canvas.itemconfig(facelet_id[f][row][col], fill=cols[t.index(cube_defstr[idx])])
                idx += 1                 # index is increased






def click(event):
    """Define how to react on left mouse clicks."""
    
    global curcol
    
    if gui_scramble_var.get():                               # case the scramble check box is checked
        return                                               # function is returned without real actions
    
    idlist = gui_canvas.find_withtag("current")              # return the widget at pointer click
    if len(idlist) > 0:                                      # case the pointer is over a widged
        if idlist[0] in colorpick_id:                        # case the widget is one of the six color picking palette
            curcol = gui_canvas.itemcget("current", "fill")  # color selected at picking palette assigned "current color"
            for i in range(6):                               # iteration over all the six color picking palette widgets
                # the circle widget is set to thick border with same color of the GUI background
                gui_canvas.itemconfig(colorpick_id[i], width=20, outline="#F0F0F0")
            
            # the selected circle widget gets thinner borger, colored with a visible gray color
            gui_canvas.itemconfig("current", width=5, fill=curcol, outline="Grey55")
        
        elif idlist[0] not in faceletter_id:                 # case the widget is not one of the six color picking palette
            gui_canvas.itemconfig("current", fill=curcol)    # that widget is filled with the "current color"
    
    draw_cubotino_center_colors()         # draw the cube center facelets with related colors, at Cubotino sketch






def scroll(event):
    """Changes the facelets color on the schetch by scroll the mouse wheel over them."""
    
    global mainWindow_ontop, last_col, last_facelet_id
    
    
    if mainWindow_ontop:                                    # case the main windows is the one on top
        
        if gui_scramble_var.get():                          # case the scramble check box is checked
            return                                          # function is returned without real actions
    
        if len(gui_canvas.find_withtag("current"))>0:       # case scrolling over a widget
            facelet=gui_canvas.find_withtag("current")[0]   # widget id is assigned to facelet variable
            
            # case the facelet (widget id) is not a color picking and not a cube face letter
            if facelet not in colorpick_id and facelet not in faceletter_id : 
                delta = -1 if event.delta > 0 else 1        # scroll direction
                if facelet != last_facelet_id:              # case the facelet is different from the lastest one changed
                    last_col=5 if delta>0 else 0            # way to get the first color in cols list at scroll start
                    last_facelet_id=facelet                 # current facelet is asigned to the latest one changed
                
                last_col=last_col+delta                     # color number is incremented/decrement by the scroll
                last_col=last_col%6                         # scroll limited within the range of six
                gui_canvas.itemconfig("current", fill=cols[last_col]) # current facelet is filled with scrolled color

            if facelet in (5,14,23):            # case the facelet is a URF face center
                draw_cubotino_center_colors()   # draw the cube center facelets with related colors, at Cubotino sketch







def gui_buttons_for_cube_status(status):
    """Changes the button states, on those related to cube-status GUI part."""
    
    global b_read_solve, b_clean, b_empty, b_random
    
    try:
        if status == "active":                   # case the function argument is "active"
            b_read_solve["relief"] = "raised"    # button read&solve is raised
            gui_f2.update()                      # frame2 gui part is updated
        b_read_solve["state"] = status           # button read&solve activated, or disabled, according to args
        b_clean["state"] = status                # button clean activated, or disabled, according to args
        b_empty["state"] = status                # button empty activated, or disabled, according to args
        b_random["state"] = status               # button random activated, or disabled, according to args

        if status=="disable":                    # case the function argument is "disable"
            b_read_solve["relief"] = "sunken"    # button read&solve is lowered
            gui_f2.update()                      # frame2 gui part is updated
            return "disable"                     # string "disable" is returned
    except:
        return "error"
        pass






def robot_solver():
    """Sends the cube manouvres to the robot
       The solving string for the robot is without space characters, and contained within <> characters
       When the robot is working, the same button is used to stop the robot."""
    
    global cube_solving_string, cube_solving_string_robot, ser
    
    s = cube_solving_string                               # shorter local variable name
    sr = cube_solving_string_robot                        # shorter local variable name
    
    if b_robot["text"] == "Send\ndata\nto\nrobot":        # case the button is ready to send solving string to the robot
        if s != None and len(s)>1 and "f)" in s:          # case there is useful data to send to the robot
            
            sr = s.strip().strip("\r\n").replace(" ","")  # empty, CR, LF, cgaracters are removed
            if sr[0]!="<" and sr[-1:]!=">":               # case the string isn't contained by '<' and '>' characters
                sr = "<" + sr +">"                        # starting '<' and ending '>' chars are added
            cube_solving_string_robot = sr                # global variable is updated
            
            try:
                ser.write((sr+"\n").encode())             # attempt to send the solving string to the robot      
            except:
                pass
        
        else:                                             # case the cube_solving_string doesn't fit pre-conditions
            print("not a proper string...")               # feedback is printed to the terminal
        
    elif b_robot["text"] == "STOP\nROBOT":                # case the button is in stop-robot mode
        stop_robot()                                      # calls the stopping robot function
        
    gui_robot_btn_update()                                # updates the cube related buttons status
    draw_cubotino_center_colors()         # draw the cube center facelets with related colors, at Cubotino sketch






def left_Cubotino_moves(robot_moves):
    """ Generates dict with the remaining servo moves, based on the moves string.
        This is later used to keep track of the robot solving progress."""
    
    global tot_moves, left_moves
    
    left_moves={}                                       # empty dict to store the left moves 
    remaining_moves=tot_moves                           # initial remaining moves are all the moves
    for i in range(len(robot_moves)):                   # iteration over all the string characters
        if robot_moves[i]=='R' or robot_moves[i]=='S':  # case the move is cube spin or layer rotation               
            remaining_moves-=1                          # counter is decreased by one
            left_moves[i]=remaining_moves               # the left moves are associated to the move index key                           
        elif robot_moves[i]=='F':                       # case there is a flip on the move string
            remaining_moves-=int(robot_moves[i+1])      # counter is decreased by the amount of flips
            left_moves[i]=remaining_moves               # the left moves are associated to the move index key






def start_robot():
    """Function that sends the starting command to the robot. Start command is in between square brackets."""
    
    global robot_working, robot_moves
    
    if gui_scramble_var.get():                          # case the scramble check box is checked
        task = "scrambling"
    else:
        task = "solving"
    
#     print("\n====================================================================================")
    print(f"{time.ctime()}: request the robot to start {task} the cube") # print to the terminal
    
    exception=False                                     # boolean to track the exceptions, set to false
    try:
        ser.write(("[start]\n").encode())               # attempt to send the start command to the robot
    except:
        exception=True                                  # boolean to track the exceptions is set true cause exception
        pass
    
    if not exception:                                   # case there are no exceptions
        robot_working=True                              # boolean that tracks the robot working is set True
        left_Cubotino_moves(robot_moves)                # left moves of the robot are calculated / stored
        gui_prog_bar["value"]=0                         # progress bar is set to zero
        gui_f2.update()                                 # frame2 of the gui is updated
        gui_f2.after(1000, gui_robot_btn_update())      # updates the cube related buttons status, with 1 sec delay






def stop_robot():
    """Function that sends the stopping command to the robot. Stop command is in between square brackets."""
    
    global robot_working 
    
    print("\nstopping the robot from GUI")              # print to the terminal 
    try:
        ser.write(("[stop]\n").encode())                # attempt to send the stop command to the robot
    except:
        print("\nexception raised while stopping the robot from GUI")     # print to the terminal 
        pass
#     print("\n====================================================================================")
    gui_robot_btn_update()                              # updates the cube related buttons status






def gui_robot_btn_update():
    """Defines the Robot buttons state, for the robot related GUI part, according to some global variables """
                             
    global serialData, cube_solving_string, robot_working, gui_buttons_state
        
    if not robot_working:                                 # case the robot is not working
        gui_buttons_state = gui_buttons_for_cube_status("active")    # buttons for cube status are set active
        
        if not serialData:                                # case there is not serial communication set
            b_robot["relief"] = "sunken"                  # large robot button is lowered
            b_robot["state"] = "disable"                  # large robot button is disabled
            b_robot["bg"] = "gray90"                      # large robot button is gray colored
            b_robot["activebackground"] = "gray90"        # large robot button is gray colored
            if not "f)" in cube_solving_string:           # case the cube solution string has not robot moves
                b_robot["text"] = "Robot:\nNo connection\nNo data" # large robot button text, to feedback the status
            
            elif "f)" in cube_solving_string:             # case the cube solution string has not robot moves
                b_robot["text"] = "Robot:\nNot\nConnected" # large robot button text, to feedback the status
        
        # case there serial communication is set, and there are no robot moves on cube solving string
        if serialData and (not "f)" in cube_solving_string or "(0" in cube_solving_string):
            b_robot["text"] = "Robot:\nConnected\nNo data" # large robot button text, to feedback the status
            b_robot["relief"] = "sunken"                  # large robot button is lowered
            b_robot["state"] = "disable"                  # large robot button is disabled
            b_robot["bg"] = "gray90"                      # large robot button is gray colored
            b_robot["activebackground"] = "gray90"        # large robot button is gray colored
        
        # case there serial communication is set, and there are robot moves on cube solving string
        elif serialData and "f)" in cube_solving_string and not "(0" in cube_solving_string:
            b_robot["text"] = "Send\ndata\nto\nrobot"     # large robot button text, to feedback the status
            b_robot["relief"] = "raised"                  # large robot button is raised
            b_robot["state"] = "active"                   # large robot button is activated
            b_robot["bg"] = "OliveDrab1"                  # large robot button is green colored
            b_robot["activebackground"] = "OliveDrab1"    # large robot button is green colored

    if robot_working:                                     # case the robot is working
        b_robot["text"] = "STOP\nROBOT"                   # large robot button text, to feedback the status
        b_robot["relief"] = "raised"                      # large robot button is raised
        b_robot["state"] = "active"                       # large robot button is activated
        b_robot["bg"] = "orange red"                      # large robot button is red colored
        b_robot["activebackground"] = "orange red"        # large robot button is red colored
        
        if gui_buttons_state!="disable":                  # case the robot is not disabled
            gui_buttons_state = gui_buttons_for_cube_status("disable") # buttons for cube status part are disabled






def cube_facelets_permutation(cube_status, move_type, direction):
    """Function that updates the cube status, according to the move type the robot does
       The 'ref' tuples provide the facelet current reference position to be used on the updated position.
       As example, in case of flip, the resulting facelet 0 is the one currently in position 53 (ref[0])."""
    
    if move_type == 'flip':      # case the robot move is a cube flip (complete cube rotation around L-R horizontal axis) 
        ref=(53,52,51,50,49,48,47,46,45,11,14,17,10,13,16,9,12,15,0,1,2,3,4,5,6,7,8,18,
             19,20,21,22,23,24,25,26,42,39,36,43,40,37,44,41,38,35,34,33,32,31,30,29,28,27)
    
    elif move_type == 'spin':    # case the robot move is a spin (complete cube rotation around vertical axis)
        if direction == '1':     # case spin is CW
            ref=(2,5,8,1,4,7,0,3,6,18,19,20,21,22,23,24,25,26,36,37,38,39,40,41,42,43,44,
                 33,30,27,34,31,28,35,32,29,45,46,47,48,49,50,51,52,53,9,10,11,12,13,14,15,16,17)
        elif direction == '3':      # case spin is CCW
            ref=(6,3,0,7,4,1,8,5,2,45,46,47,48,49,50,51,52,53,9,10,11,12,13,14,15,16,17,
                 29,32,35,28,31,34,27,30,33,18,19,20,21,22,23,24,25,26,36,37,38,39,40,41,42,43,44)
    
    elif move_type == 'rotate':  # case the robot move is a rotation (lowest layer rotation versus mid and top ones) 
        if direction == '1':     # case 1st layer rotation is CW
            ref=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,24,25,26,18,19,20,21,22,23,42,43,44,
                 33,30,27,34,31,28,35,32,29,36,37,38,39,40,41,51,52,53,45,46,47,48,49,50,15,16,17)
        elif direction == '3':   # case 1st layer rotation is CCW
            ref=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,51,52,53,18,19,20,21,22,23,15,16,17,
                 29,32,35,28,31,34,27,30,33,36,37,38,39,40,41,24,25,26,45,46,47,48,49,50,42,43,44)
    
    new_status={}                # empty dict to generate the cube status, updated according to move_type and direction
    for i in range(54):                    # iteration over the 54 facelets
        new_status[i]=cube_status[ref[i]]  # updated cube status takes the facelet from previous status at ref location
    
    return new_status                      # updated cube status is returned






def animate_cube_sketch(move_index):
    """Function that keeps updating the cube sketch colors on screen, according to the robot move."""
    
    global cube_status, robot_moves, previous_move
    
    if move_index >= previous_move or move_index==0:    # case there is a new move (or the first one)
        i=move_index                     # shorther variable name
        if robot_moves[i]=='F':          # case there is a flip on the move string
            cube_status=cube_facelets_permutation(cube_status, 'flip', None)               # cube status after a flip
            
        elif robot_moves[i]=='S':        # case there is a cube spin on the move string
            cube_status=cube_facelets_permutation(cube_status, 'spin', robot_moves[i+1])   # cube status after a spin
        
        elif robot_moves[i]=='R':        # case there is a cube 1st layer rotation
            cube_status=cube_facelets_permutation(cube_status, 'rotate', robot_moves[i+1]) # cube status after a rotate

        for k in range(54):              # iteration over the 54 facelets
            f=k//9                       # cube face
            row=(k%9)//3                 # face row
            col=(k%9)%3                  # face column
            gui_canvas.itemconfig(facelet_id[f][row][col], fill=cols[t.index(cube_status[k])])  # color filling
        
        if move_index > previous_move:   # case the move index is larger than previous (not the case on multui flips)
            previous_move +=2            # previous move index is increased (it goes with step of two)
    
    if gui_scramble_var.get():           # case the scramble check box is checked
        return                           # function is returned, by skipping Cubotino sketch update
    
    else:                                # case the scramble check box is not checked
        draw_cubotino_center_colors()    # update of center facelets colors on cube at Cubotino sketch  






def cube_read_solve():
    """GUI button retrieve the cube status from the sketch on screen, and to return the solving string."""

    global cols, gui_buttons_state  
    
    if not robot_working:                          # case the robot is not working
        gui_text_window.delete(1.0, tk.END)        # clears the text window
        gui_buttons_state = gui_buttons_for_cube_status("disable")     # disable the buttons on the cube-status GUI part
        cube_solving_string=""                     # set to empty the cube solving string
        cube_defstr=""                             # set to empty the cube status string
        var=gui_read_var.get()                     # get the radiobutton selected choice
        
        if gui_scramble_var.get():                 # case the scramble check box is checked
            gui_read_var.set("screen sketch")      # set the radiobutton to the screen sketch, as scrambling checkbutton               
        var=gui_read_var.get()                     # get the radiobutton selected choice

        print('\n\n\n\n\n\n\n\n')
        if "webcam" in var:                        # case the webcam is selected as cube status detection method
            # feeback is printed to the terminal
            print('=============================   cube status via the webcam   ==============================\n')

            try:
                empty()                            # empties the cube sketch on screen
                cam_num = gui_webcam_num.get()     # webcam number retrieved from radiobutton
                cam_wdth = s_webcam_width.get()    # webcam width is retrieved from the slider
                cam_hght = s_webcam_height.get()   # webcam heigth is retrieved from the slider
                cam_crop = s_webcam_crop.get()     # pixels quantity to crop at the frame right side
                w_fclts = s_facelets.get()         # max number of facelets in frame width (for the contour area filter)


                webcam_cols=[]                     # empty list to be populated with the URFDLB colors sequence 
                webcam_cube_status_string=''       # string, to hold the cube status string returned by the webcam app

                # cube color sequence and cube status are returned via the webcam application
                webcam_cols, webcam_cube_status_string = cam.cube_status(cam_num, cam_wdth, cam_hght, cam_crop,\
                                                                         w_fclts,debug, estimate_fclts, delay)

                if len(webcam_cols)==6 and len(webcam_cube_status_string)>=54:  # case the app return is valid
                    cols = webcam_cols                        # global variable URFDLB colors sequence is updated
                    cube_defstr = webcam_cube_status_string   # global variableod cube status is updated
                    cube_defstr = cube_defstr+"\n"            # cube status string in completed by '\n'
                    redraw(cube_defstr)                       # cube sketch on screen is updated to cube status string
                    draw_cubotino_center_colors()             # draw the cube center facelets with related colors
            except:
                if debug:
                    show_text(" Cube status not defined")     # cube status undefined is printed on the text window
                pass

        elif "screen" in var:                                 # case the screen sketch is selected on the radiobuttons
            if gui_scramble_var.get():                        # case the scramble check box is checked
                # feeback is printed to the terminal
                print('=============================   random cube for scrambling   ==============================\n')    
            else:                                             # case the scramble check box is not checked
                # feeback is printed to the terminal
                print('======================   cube status defined via the screen sketch   ======================\n')    
                    
                    
            try:
                cube_defstr = get_definition_string()+"\n"    # cube status string is returned from the sketch on screen
            except:
                show_text(" Cube status not defined")         # cube status undefined is printed on the text window
                pass

        elif "robot" in var:                     # case the robot is selected on the radiobuttons
            empty()                              # empties the cube sketch on screen
            draw_cubotino_center_colors()        # draw the cube center facelets with related colors
            
        if len(cube_defstr)>=54:                 # case the cube solution string has min amount of characters
            solve()                              # solver is called
 
        draw_cubotino_center_colors()            # draw the cube center facelets with related colors
        gui_buttons_state = gui_buttons_for_cube_status("active")    # activate the buttons on the cube-status GUI part
        gui_robot_btn_update()                   # updates the cube related buttons status






def progress_percent(move_index):
    """Returns the robot solving progress, in percentage."""
    
    global tot_moves, left_moves
    
    remaining_moves= left_moves[move_index]             # remaining moves are retrived from the left moves dict
    return str(int(100*(1-remaining_moves/tot_moves)))  # returns a string with the integer of the solving percentage






def progress_update(received):
    """Function that updates the robot progress bar and the progress label
       Argument is the robot_move string index of the last move.
       As example, 'i_12', means the robot is doing the 12th move from finish."""
    
    global gui_prog_label_text, gui_prog_label
    
    if not 'end' in received:                             # case the robot is still running                    
        move_index=int(received[2:])                      # string part with the progress value
        percent=progress_percent(move_index)              # percentage is calclated
        try:
            gui_prog_bar["value"]=percent                 # progress bar is set to the percentage value
            gui_prog_label_text.set(percent+" %")         # progress label is updated with percentage value and simbol  
            if percent=="100":                            # case the solving percentage has reached 100
                gui_prog_bar["value"]='0'                 # progress bar is set to zero
                gui_prog_label_text.set("")               # progress label is set empty
        except:
            pass
    
        animate_cube_sketch(move_index)  # cube facelets sketch updates according to the robot move in execution

    elif 'end' in received:                               # case the robot has been stopped                  
        gui_prog_bar["value"]='0'                         # progress bar is set to zero
        gui_prog_label_text.set("")                       # progress label is set empty

    gui_prog_bar.update_idletasks()                       # gui widget (progress bar) is updated
    gui_prog_label.update()                               # gui widget (progress label) is updated






def robot_received_settings(received):
    """Servo settings returned by the robot."""
    
    if '(' in received and ')' in received:        # case the data contains open and close round parenthesis
        received = received.replace(' ','')        # empty spaces are removed
        data_start = received.find('(')            # position of open parenthesys in data
        data_end = received.find(')')              # position of close parenthesys in data
        print(f'servos settings sent by the robot: {received[data_start:data_end+1]}')
        data = received[data_start+1:data_end]     # data in between parenthesys is assigned, to the same string variable
        data_list=data.split(',')                  # data is split by comma, becoming a list of strings 

        settings=[]                                # empty list to store the list of numerical settings
        for setting in data_list:                  # iteration over the list of strings
            setting.lower().strip()                # setting string is lowered and stripped
            if setting.isdigit():                  # case the setting looks like a digit
                settings.append(int(setting))      # each str setting changed to int and appended to the list of settings
            else:                                  # case the setting does not look like a digit
                if 'small' in setting:             # case the setting is equal to 'small'
                    settings.append('small')       # string setting appends 'small' to the list of settings
                elif 'large' in setting:           # case the setting is equal to 'large'
                    settings.append('large')       # string setting appends 'large' to the list of settings
       
               
        get_settings(settings)                                 # function that updates the individual global variables
        gui_sliders_update('update_sliders')                   # function that updates the gui sliders
        with open("Cubotino_settings.txt", 'w') as f:          # open the servos setting text file in write mode
            f.write(timestamp+received[data_start:data_end+1]) # save the received servos settings

    else:                                          # case the data does not contains open and close round parenthesis
        print("not a valid settings string")       # feedback is returned






def gui_sliders_update(intent):
    """depending on the argument, this function updates the global variable based on the sliders
        or updates the sliders based on the global variables."""
    
    global t_servo_flip, t_servo_open, t_servo_close, t_servo_release, b_rotate_time, b_spin_time, b_rel_time
    global t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time
    global b_servo_CCW, b_home, b_servo_CW, b_extra_sides, b_extra_home
    global t_srv_pw_range, b_srv_pw_range, robot_settings
    
    if intent == 'read_sliders':             # case the intention is to get sliders values to update the global variables
        t_servo_flip = s_top_srv_flip.get()
        t_servo_open = s_top_srv_open.get()
        t_servo_close = s_top_srv_close.get()
        t_servo_release = s_top_srv_release.get()
        t_flip_to_close_time = s_top_srv_flip_to_close_time.get()
        t_close_to_flip_time = s_top_srv_close_to_flip_time.get()
        t_flip_open_time = s_top_srv_flip_open_time.get()
        t_open_close_time = s_top_srv_open_close_time.get()
        
        b_servo_CCW = s_btm_srv_CCW.get()
        b_home = s_btm_srv_home.get()
        b_servo_CW = s_btm_srv_CW.get()
        b_extra_sides = s_btm_srv_extra_sides.get()
        b_extra_home = s_btm_srv_extra_home.get()
        b_spin_time = s_btm_srv_spin_time.get()
        b_rotate_time = s_btm_srv_rotate_time.get()
        b_rel_time = s_btm_srv_rel_time.get()
        
        # global tuple variable with the servos related settings
        robot_settings=(t_servo_flip,t_servo_open,t_servo_close,t_servo_release, t_flip_to_close_time,
                    t_close_to_flip_time, t_flip_open_time,t_open_close_time,
                    b_servo_CCW,b_home,b_servo_CW,b_extra_sides,b_extra_home,b_spin_time,b_rotate_time,b_rel_time,
                    t_srv_pw_range, b_srv_pw_range)

    elif intent == 'update_sliders':          # case the intention is to update sliders from the global variables values
        s_top_srv_flip.set(t_servo_flip)
        s_top_srv_open.set(t_servo_open)
        s_top_srv_close.set(t_servo_close)
        s_top_srv_release.set(t_servo_release)
        s_top_srv_flip_to_close_time.set(t_flip_to_close_time)
        s_top_srv_close_to_flip_time.set(t_close_to_flip_time)
        s_top_srv_flip_open_time.set(t_flip_open_time)
        s_top_srv_open_close_time.set(t_open_close_time)
        
        s_btm_srv_CCW.set(b_servo_CCW)
        s_btm_srv_home.set(b_home)
        s_btm_srv_CW.set(b_servo_CW)
        s_btm_srv_extra_sides.set(b_extra_sides)
        s_btm_srv_extra_home.set(b_extra_home)
        s_btm_srv_spin_time.set(b_spin_time)
        s_btm_srv_rotate_time.set(b_rotate_time)
        s_btm_srv_rel_time.set(b_rel_time)

    




def log_data():
    """ Data logging, just for fun."""
    
    global timestamp, defStr, cube_solving_string, cube_solving_string_robot, end_method
    global tot_moves, robot_time
           

    import os                                        # os is imported to ensure the folder check/make
    folder = os.path.join('.','data_log_folder')     # folder to store the collage pictures
    if not os.path.exists(folder):                   # if case the folder does not exist
        os.makedirs(folder)                          # folder is made

    fname = os.path.join(folder,'Cubotino_log.txt')  # folder+filename
    
    if not os.path.exists(fname):     # case the file does not exist (file with headers is generated)
        print(f'\ngenerated Cubotino_log.txt file with headers')
        
        # columns headers
        headers = ['Date', 'CubeStatusEnteringMethod', 'CubeStatus', 'CubeSolution',
                   'RobotoMoves', 'TotCubotinoMoves', 'EndingReason', 'RobotTime(s)']
        
        s=''                                     # empty string to hold the headers, separated by tab                            
        for i, header in enumerate(headers):     # iteration over the headers list
            s+=header                            # header is added to the string
            if i < len(headers)-1:               # case there are other headers in list
                s= s+'\t'                        # tab is added to the headers string
            elif i == len(headers)-1:            # case there are no more headers in list
                s= s+'\n'                        # LF at string end

        # 'a' means the file will be generated if it does not exist, and data will be appended at the end
        with open(fname,'a') as f:               # the text file is opened in generate/edit mode
            f.write(s)                           # headers are added to the file

    
    reading_method=gui_read_var.get()            # gets the radiobutton selected choice

    # info to log
    a=str(timestamp)                             # date and time
    b=str(reading_method)                        # method used to enter the cube status
    c=str(cube_defstr.strip('\n'))               # cube status detected
    d=str(cube_solving_string.strip('n'))        # solution returned by Kociemba solver
    e=str(robot_moves)                           # robot moves string
    f=str(tot_moves)                             # total amount of Cubotino moves 
    g=str(end_method)                            # cause of the robot stop
    h=str(robot_time)                            # robot solving time (not the color reading part)
    s = a+'\t'+b+'\t'+c+'\t'+d+'\t'+e+'\t'+f+'\t'+g+'\t'+h+'\n'  # tab separated string with all the info to log

    # 'a'means the file will be generated if it does not exist, and data will be appended at the end
    fname = os.path.join(folder,'Cubotino_log.txt')  # folder+filename
    with open(fname,'a') as f:                   # the text file is opened in edit mode  
        f.write(s)                               # data is appended   






def debug_check():
    """update the global variable debug according to the checkbutton at settings window."""
    global debug
    if checkVar1.get()==1:
        debug = True
        print("debug printout active\n")
    elif checkVar1.get()==0:
        debug = False
        print("debug printout not active\n")
    else:
        print("error on debug_check function\n")






def estimate_fclts_check():
    """update the global variable debug according to the checkbutton at settings window."""
    global estimate_fclts
    if checkVar2.get()==1:
        estimate_fclts = True
        print("estimated facelets position active\n")        
    elif checkVar2.get()==0:
        estimate_fclts = False
        print("estimated facelets position not active")
    else:
        print("error on estimate_fclts_check function\n")


# ################################### serial comunication related functions #############################################
""" the serial communication part and tkinter gui approach is largely based on tutorial
https://www.youtube.com/watch?v=x_5VbOOskw0 """


def connect_check(args):
    """Function that activates the Connect button only when a serial port has been selected on the drop down menu."""
    
    if "-" in clicked_com.get():         # case no serial port selected
        b_connect["state"] = "disable"   # Connect button is disabled
    else:                                # case a serial port selected
        b_connect["state"] = "active"    # Connect button is activated






def update_coms():
    """Function that updates the serial ports connected to the PC."""
    
    global clicked_com, b_drop_COM
    
    ports = serial.tools.list_ports.comports()      # all com ports are retrieved
    coms = [com[0] for com in ports]                # list of the serial ports
    coms.insert(0, "-")                             # first position on drop down menu is not a serial port
    try:
        b_drop_COM.destroy()                        # previous drop down menu is destroyed
    except:
        pass
    clicked_com = tk.StringVar()                    # string variable used by tkinter for the selection
    clicked_com.set(coms[0])                        # activates first drop down menu position (not a serial port)
    b_drop_COM = tk.OptionMenu(gui_robot_label, clicked_com, *coms, command=connect_check) # populated drop down menu
    b_drop_COM.config(width=7, font=("Arial", "10"))        # drop down menu settings
    b_drop_COM.grid(column=0, row=8, sticky="e", padx=10)   # drop down menu settings
    connect_check(0)                                        # updates the button Connect status
    gui_robot_btn_update()                                  # updates the cube related buttons status






def connection():
    """Function to open / close the serial communication.
    When a serial is opened, a thread is initiated for the communication."""
    
    global ser, serialData, gui_prog_bar, robot_working, cube_solving_string
    
    if "Disconnect" in b_connect["text"]:           # case the conection button shows Disconnect
        stop_robot()                                # robot is requested to stop
        robot_working=False                         # boolean tracking the robot working is set to False
        serialData = False                          # boolean enabling serial comm data analysis is set False
        try: 
            print("closing COM")                    # feedback print to the terminal
            ser.write(("[led_off]\n").encode())     # ESP32 blue led is set off
            ser.close()                             # serial port (at PC side) is closed
        except:
            pass
        b_connect["text"] = "Connect"               # conection button label is changed to Connect
        b_refresh["state"] = "active"               # refresch com button is activated
        b_drop_COM["state"] = "active"              # drop down menu for ports is activated
        b_settings["state"] = "disable"             # settings button is disabled
        gui_robot_btn_update()                      # updates the cube related buttons status

    else:                                           # case the conection button shows Connect
        serialData = True                           # boolean enabling serial comm data analysis is set True
        gui_prog_bar["value"]=0                     # progress bar widget is set to zero
        gui_prog_bar.update_idletasks()             # progress bar widget is updated
        gui_prog_label_text.set("")                 # progress label is set to empty
        gui_prog_label.update_idletasks()           # progress label widget is updated
        gui_robot_btn_update()                      # updates the cube related buttons status
        b_connect["text"] = "Disconnect"            # conection button label is changed to Disconnect
        b_refresh["state"] = "disable"              # refresch com button is disables
        b_drop_COM["state"] = "disable"             # drop down menu for ports is disabled
        port = clicked_com.get()                    # serial port is retrieved from the selection made on drop down menu
        print(f"selected port: {port} \n")          # feedback print to the terminal
        
        try:                                        # serial port opening
            ser = serial.Serial(port,
                                baudrate=115200,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=None,
                                xonxoff=False,
                                rtscts=False,
                                dsrdtr=False,     
                               )
#             print(ser)
            time.sleep(1)

        except:
            text_info="check if ESP32 is connected to the IDE"        # text of possible connection fail reason
            print(text_info)                                          # print text of possible connection fail reason
            if (text_info not in gui_text_window.get(1.0, tk.END)):   # case the text_info is not displayed at GUI
                show_text("\nCheck if ESP32 is connected to the IDE\n") # text_info is displayed at GUI
                
            serialData = False                               # boolean enabling serial comm data analysis is set True
            b_connect["text"] = "Connect"                    # conection button label is changed to Connect
            b_refresh["state"] = "active"                    # refresch com button is activated
            b_drop_COM["state"] = "active"                   # drop down menu for ports is activated
            gui_robot_btn_update()                           # updates the cube related buttons status
            
            return

        if ser.isOpen():                                          # case the serial is succesfully opened
            text_info="Check if ESP32 is connected to the IDE"    # text of possible connection fail reason                                       # print text of possible connection fail reason
            if (text_info in gui_text_window.get(1.0, tk.END)):   # case the text_info is displayed at GUI
                gui_text_window.delete(1.0, tk.END)               # clears GUI text window
                
            try:
                ser.write(("[led_on]\n").encode())   # ESP32 blue led is set on (fix) 
            except:                                  # case the first write attempt goes wrong
                print("could not write on ser")      # feedback print to the terminal

        b_settings["state"] = "active"               # settings button is activated
            
        t1 = threading.Thread(target=readSerial)     # a thread is associated to the readSerial function
        t1.deamon = True                             # thread is set as deamon (runs in background with low priority)
        t1.start()                                   # thread is started






def readSerial():
    """Functon, called by a daemon thread, that keeps reading the serial port."""
    
    global serialData, robot_working, gui_prog_bar, cube_solving_string, cube_solving_string_robot
    global end_method, robot_time
    
    while serialData and ser.isOpen():    # script has set the conditions for Serail com and serial port is found open
        try:
            data = ser.readline()                                     # serial dat is read by lines
        except:
            pass

        if len(data) > 0:                                             # case there are characters read from the serial
            try:
                received = data.decode()                              # data is decoded
                received=received.strip().strip("\n")                 # empty space and LF characters removed
            except:
                pass

            if "conn" in received:                                    # case 'conn' is in received: ESP32 is connectd
                print("established connection with ESP32\n")          # feedback is printed to the terminal


            elif "<" in received and ">" in received:                 # robot replies with the received solving string
                if received==cube_solving_string_robot.strip("\r\n"): # check if the robot received string is ok
                    start_robot()                                     # call the robot start function
                else:
                    print(f"cube_solving_string_robot received by the robot differs from :{cube_solving_string_robot}")
                    print("====================================================================================")


            elif "stop" in received and robot_working==True:          # case 'stop' is in received: Robot been stopped
                print("\nstop command has been received by the robot\n")  # feedback is printed to the terminal
                print("====================================================================================")
                robot_working=False                                   # boolean trcking the robot working is set False
                end_method="stopped"                                  # variable tracking the end method
                if '(' in received and ')' in received:               # case the dat contains open and close parenthesis
                    data_start = received.find('(')                   # position of open parenthesys in received
                    data_end = received.find(')')                     # position of close parenthesys in received
                    robot_time = received[data_start+1:data_end]      # data between parenthesys is assigned
                log_data()                                            # log the more relevant data
                progress_update('i_end')                              # progress feedback is ise to end
                gui_text_window.delete(1.0, tk.END)                   # clears the text window
                cube_solving_string=""                                # cube solving string is set empty
                gui_robot_btn_update()                                # updates the cube related buttons status


            elif "start" in received:                                 # case 'start' is in received: Robot is solving
                print("start command has been received by the robot") # feedback is printed to the terminal


            elif "i_" in received:                                    # case 'i_' is received: Robot progress index
                progress_update(received)                             # progress function is called


            elif "solved" in received:                                # case 'solved' is in received: Robot is solving                          
                robot_time=0.0
                robot_working=False                                   # boolean trcking the robot working is set False
                if gui_scramble_var.get():                            # case the scramble check box is checked
                    end_method="scrambled"                            # variable tracking the end method
                elif not gui_scramble_var.get():                      # case the scramble check box is not checked
                    end_method="solved"                               # variable tracking the end method
                if '(' in received and ')' in received:               # case the dat contains open and close parenthesis
                    data_start = received.find('(')                   # position of open parenthesys in received
                    data_end = received.find(')')                     # position of close parenthesys in received
                    robot_time = received[data_start+1:data_end]      # data between parenthesys is assigned
                log_data()                                            # log the more relevant data
                gui_text_window.delete(1.0, tk.END)                   # clears the text window
                cube_solving_string=""                                # cube solving string is set empty
                
                show_text(f"\n Cube {end_method} in: {robot_time} secs")  # feedback is showed on the GUI                
                print(f"\nCube {end_method}, in: {robot_time} secs")      # feedback to the terminal
                print("\n===========================================================================================")
                gui_robot_btn_update()                                    # updates the cube related buttons status


            elif "current_settings" in received:                      # case 'current_settings' is in received 
                print("\nservos settings request has been received by the robot")  # feedback is printed to the terminal
                robot_received_settings(received)                     # robot_received_settings function is called
            

            elif "new_settings" in received:                          # case 'new_settings' is in received 
                print("new servos settings has been received by the robot")   # feedback is printed to the terminal

            
            else:                                                     # case not expected data is received
                if data=='\n':
                    pass
                else:
                    print(f"unexpected data received by the robot: {received}") # feedback is printed to the terminal

        else:                                                 # case there not countable characters read from the serial
            if data!=b"":                                     # case the character is not an empty binary
                print(f"len data not >0")                     # feedback is printed to the terminal
                print(f"undecoded data from robot: {data}")   # feedback is printed to the terminal
                break                                         # while loop is interrupted






def close_window():
    """Function taking care to properly close things when the GUI is closed."""
    
    global root, serialData, ser
    
    try:
        if ser.isOpen():                           # case the serial port (at PC) is open
            print("closing COM")                   # feedback is printed to the terminal
            ser.write(("[led_off]\n").encode())    # ESP32 blue led is switched off
            ser.close()                            # serial port (at PC) is closed
    except:
        pass
    serialData = False                             # boolean tracking serial comm conditions is set False
    root.destroy()                                 # GUI is closed






# ################################### functions to get the slider values  ##############################################

def servo_CCW(val):
    b_servo_CCW = int(val)     # bottom servo position when fully CW

def servo_home(val):
    b_home = int(val)          # bottom servo home position

def servo_CW(val):
    b_servo_CW = int(val)      # bottom servo position when fully CCW
    
def servo_extra_sides(val):
    b_extra_sides = int(val)   # bottom servo position small rotation back from CW and CCW, to release tension
    
def servo_extra_home(val):
    b_extra_home = int(val)    # bottom servo position extra rotation at home, to releasetension

def servo_rotate_time(val):
    b_rotate_time = int(val)   # time needed to the bottom servo to rotate about 90deg

def servo_rotate_time(val):
    b_rel_time = int(val)      # time to rotate slightly back, to release tensions

def servo_spin_time(val):
    b_spin_time = int(val)     # time needed to the bottom servo to spin about 90deg

def servo_flip(val):
    t_servo_flip = int(val)    # top servo pos to flip the cube on one of its horizontal axis

def servo_open(val):
    t_servo_open = int(val)    # top servo pos to free up the top cover from the cube

def servo_close(val):
    t_servo_close = int(val)   # top servo pos to constrain the top cover on cube mid and top layer

def servo_release(val):
    t_servo_release = int(val)       # top servo release position after closing toward the cube

def flip_to_close_time(val):
    t_flip_to_close_time = int(val)  # time to lower the cover/flipper from flip to close position     

def close_to_flip_time(val):
    t_close_to_flip_time = int(val)  # time to raise the cover/flipper from close to flip position

def flip_open_time(val):
    t_flip_open_time = int(val)      # time to raise/lower the flipper between open and flip positions

def open_close_time(val):
    t_open_close_time = int(val)     # time to raise/lower the flipper between open and close positions





# ######################## functions to test the servos positions  #####################################################
def flip_cube():
    try:
        ser.write(("[test(flip)]\n").encode()) # send the flip_test request to the robot
    except:
        pass
    
def close_top_cover():
    try:
        ser.write(("[test(close)]\n").encode()) # send the close_cover settings request to the robot
    except:
        pass

def open_top_cover():
    try:
        ser.write(("[test(open)]\n").encode()) # send the open_cover settings request to the robot
    except:
        pass

def ccw():
    try:
        ser.write(("[test(ccw)]\n").encode()) # send the spin/rotate to CCW request to the robot
    except:
        pass
    
def home():
    try:
        ser.write(("[test(home)]\n").encode()) # send the spin/rotate to HOME request to the robot
    except:
        pass

def cw():
    try:
        ser.write(("[test(cw)]\n").encode()) # send the spin/rotate to CW request to the robot
    except:
        pass






def get_current_servo_settings():
    """Request robot to send the current servos settings."""
        
    try:
        ser.write(("[current_settings]\n").encode())      # send the request to the robot for current servos settings
    except:
        pass






def send_new_servo_settings():
    """Send new servos settings (defined via the sliders) to the robot."""
    
    global robot_settings
    
    gui_sliders_update('read_sliders')                     # sliders positions are read                    
    data=str(robot_settings)                               # tuple with servos settings is changed in string
    data=data.replace(" ","")                              # eventual empty spaces are removed from the data string
    print(f'\nservos settings sent to the robot: {data}')  # feedback is print to the terminal, as tuning reference
    try:
        ser.write(("[new_settings"+data+"]\n").encode())   # send the new servos settings to the robot
        write_backup_settings(data)                        # call to the function to save a backup file od the settings
    except:
        pass






def angle2slider_value(angle, min_pw, max_pw, min_angle=-90):
    """Returns the servo duty signal for a target angle of the servo;
    This considers a servo having 180deg rotation, determined by min_pw and max_pw."""
    
    slider_val = 0 # arbitray slider value outside the expected range
    
    # pulse width is calculated in microseconds
    pulse_with = (max_pw-min_pw)/180*(angle-min_angle+(180*min_pw/(max_pw-min_pw)))
    
    # converts the pulse width (microseconds) to servo duty units
    servo_freq = 50
    period = 1000//servo_freq
    slider_val = int(round(pulse_with*1.024/period,0))   
    
    if slider_val == 0:
        print("angle2slider_value conversion error")
        
    return slider_val






def slider2angle_value(slider_val, srv_pw_range):
    """Returns the servo angle associated to a slider cursor value and the servo pulse width range;
    This considers a servo having 180deg rotation."""
       
    servo_freq = 50
    period = 1000//servo_freq
    pulse_width = slider_val/1024*period
    
    angle = 1000                    # arbitray angle value outside the expected range
    
    if srv_pw_range == 'small':     # case the servo has a pulse width range from 1 to 2 ms
        angle = 180*pulse_width-270
    
    elif srv_pw_range == 'large':   # case the servo has a pulse width range from 1 to 2 ms
        angle = 90*pulse_width-135
    
    if angle == 1000:
        print("slider2angle_value conversion error")
    
    return angle




def pw_update():
    """update the range and cursor value for the sliders, according to the servos pulse width range.
    This function is only called when the 'change confirmed' button is pressed."""
    
    global t_srv_pw_range, b_srv_pw_range, last_t_srv_pw_range, last_b_srv_pw_range
    
    # case the 'update' button has been pressed without changing the servos pulse width
    if gui_var_t_srv_pw.get() == last_t_srv_pw_range and gui_var_b_srv_pw.get() == last_b_srv_pw_range:
        return  # the function returns without any action
    
    global t_servo_flip, t_servo_open, t_servo_close
    global b_servo_CCW, b_home, b_servo_CW
    global s_pwm_flip_min, s_pwm_flip_max, s_pwm_open_min, s_pwm_open_max
    global s_pwm_close_min, s_pwm_close_max, s_pwm_ccw_min, s_pwm_ccw_max
    global s_pwm_home_min, s_pwm_home_max, s_pwm_cw_min, s_pwm_cw_max 
    
    if gui_var_t_srv_pw.get() != last_t_srv_pw_range:
        if last_t_srv_pw_range.lower() == 'small':   # case the previous setting was 'small' pulse width (1 to 2 ms)
            min_pw = 1000             # min pulse width is 1000us
            max_pw = 2000             # max pulse width is 2000us
            new_min_pw = 500          # new min pulse width is 500us
            new_max_pw = 2500         # mew max pulse width is 2500us
            srv_pw_range = 'small'    # last pulse width used in string is 'small'
            t_srv_pw_range = 'large'  # new pulse width for top servo in string'is 'large'
            srv_pw_label = '0.5 - 2.5 ms' # pulse width label for the chosen large setting

        elif last_t_srv_pw_range.lower() == 'large': # case the previous setting was 'large' pulse width (0.5 to 2.5 ms)
            min_pw = 500              # min pulse width is 500us
            max_pw = 2500             # max pulse width is 2500us
            new_min_pw = 1000         # new min pulse width is 1000us
            new_max_pw = 2000         # mew max pulse width is 2000us
            srv_pw_range = 'large'    # last pulse width used in string is 'large'
            t_srv_pw_range = 'small'  # new pulse width for top servo is string is 'small'
            srv_pw_label = '1 - 2 ms' # pulse width label for the chosen small setting
        
        print('changed the pulse width range of top servo to', srv_pw_label)
       
       
        # current and new sliders cursors values
        t_servo_flip = s_top_srv_flip.get()                               # current slider positions is read
        angle = slider2angle_value(t_servo_flip, srv_pw_range)            # current angle positions
        t_servo_flip = angle2slider_value(angle, new_min_pw, new_max_pw)  # new slider position for the same angle

        t_servo_open = s_top_srv_open.get()                               # current slider positions is read
        angle = slider2angle_value(t_servo_open, srv_pw_range)            # current angle positions
        t_servo_open = angle2slider_value(angle, new_min_pw, new_max_pw)  # new slider position for the same angle

        t_servo_close = s_top_srv_close.get()                             # slider positions is read
        angle = slider2angle_value(t_servo_close, srv_pw_range)           # current angle positions
        t_servo_close = angle2slider_value(angle, new_min_pw, new_max_pw) # new slider position for the same angle
 
        # top_cover flip postion, slider extremes values
        s_pwm_flip_min = angle2slider_value(-112, new_min_pw, new_max_pw) # min slider value
        s_pwm_flip_max = angle2slider_value(-59, new_min_pw, new_max_pw)  # max slider value
        s_top_srv_flip.configure(from_ = s_pwm_flip_min)                  # min slider setting
        s_top_srv_flip.configure(to = s_pwm_flip_max)                     # max slider setting

        # top_cover open position, slider extremes values
        s_pwm_open_min = angle2slider_value(-59, new_min_pw, new_max_pw)  # min slider value
        s_pwm_open_max = angle2slider_value(-6, new_min_pw, new_max_pw)   # max slider value
        s_top_srv_open.configure(from_ = s_pwm_open_min)                  # min slider setting
        s_top_srv_open.configure(to = s_pwm_open_max)                     # max slider setting

        # top_cover close position, slider extremes values
        s_pwm_close_min = angle2slider_value(-41, new_min_pw, new_max_pw) # min slider value
        s_pwm_close_max = angle2slider_value(46, new_min_pw, new_max_pw)  # max slider value
        s_top_srv_close.configure(from_ = s_pwm_close_min)                # min slider setting
        s_top_srv_close.configure(to = s_pwm_close_max)                   # max slider setting

        last_t_srv_pw_range = t_srv_pw_range  # the new top servos pulse width is now the last one used

    
    if gui_var_b_srv_pw.get() != last_b_srv_pw_range: 
        if last_b_srv_pw_range.lower() == 'small':   # case the previous setting was 'small' pulse width (1 to 2 ms)
            min_pw = 1000             # min pulse width is 1000us
            max_pw = 2000             # max pulse width is 2000us
            new_min_pw = 500          # new min pulse width is 500us
            new_max_pw = 2500         # mew max pulse width is 2500us
            srv_pw_range = 'small'    # last pulse width used in string is 'small'
            b_srv_pw_range= 'large'   # new pulse width for bottom servo in string is 'large'
            srv_pw_label = '0.5 - 2.5 ms' # pulse width label for the chosen large setting

        elif last_b_srv_pw_range.lower() == 'large': # case the previous setting was 'large' pulse width (0.5 to 2.5 ms)
            min_pw = 500              # min pulse width is 500us
            max_pw = 2500             # max pulse width is 2500us
            new_min_pw = 1000         # new min pulse width is 1000us
            new_max_pw = 2000         # mew max pulse width is 2000us
            srv_pw_range = 'large'    # last pulse width used in string is 'large'
            b_srv_pw_range= 'small'   # new pulse width for bottom servo is string is 'small'
            srv_pw_label = '1 - 2 ms' # pulse width label for the chosen small setting
        
        print('changed the pulse width range of top servo to', srv_pw_label)
        
        # current and new sliders cursors values 
        b_servo_CCW = s_btm_srv_CCW.get()                                 # current slider positions is read
        angle = slider2angle_value(b_servo_CCW, srv_pw_range)             # current angle positions
        b_servo_CCW = angle2slider_value(angle, new_min_pw, new_max_pw)   # new slider position for the same angle

        b_home = s_btm_srv_home.get()                                     # current slider positions is read
        angle = slider2angle_value(b_home, srv_pw_range)                  # current angle positions
        b_home = angle2slider_value(angle, new_min_pw, new_max_pw)        # new slider position for the same angle

        b_servo_CW = s_btm_srv_CW.get()                                   # current slider positions is read
        angle = slider2angle_value(b_servo_CW, srv_pw_range)              # current angle positions
        b_servo_CW = angle2slider_value(angle, new_min_pw, new_max_pw)    # new slider position for the same angle

        # bottom servo CCW position, slider extremes values
        s_pwm_ccw_min = angle2slider_value(-129, new_min_pw, new_max_pw)  # min slider value
        s_pwm_ccw_max = angle2slider_value(-52, new_min_pw, new_max_pw)   # max slider value
        s_btm_srv_CCW.configure(from_ = s_pwm_ccw_min)                    # min slider setting
        s_btm_srv_CCW.configure(to = s_pwm_ccw_max)                       # max slider setting

        # bottom servo Home position, slider extremes values
        s_pwm_home_min = angle2slider_value(-49, new_min_pw, new_max_pw)  # min slider value
        s_pwm_home_max = angle2slider_value(36, new_min_pw, new_max_pw)   # max slider value
        s_btm_srv_home.configure(from_ = s_pwm_home_min)                  # min slider setting
        s_btm_srv_home.configure(to = s_pwm_home_max)                     # max slider setting

        # bottom servo CW position, slider extremes values
        s_pwm_cw_min = angle2slider_value(39, new_min_pw, new_max_pw)     # min slider value
        s_pwm_cw_max = angle2slider_value(117, new_min_pw, new_max_pw)    # max slider value
        s_btm_srv_CW.configure(from_ = s_pwm_cw_min)                      # min slider setting
        s_btm_srv_CW.configure(to = s_pwm_cw_max)                         # max slider setting

        last_b_srv_pw_range = b_srv_pw_range  # the new bottom servos pulse width is now the last one used        

    print()
    gui_sliders_update('update_sliders')  # calls the function to update the sliders on the (global) variables
    
    
    
    






# ############################ functions for the webcam related settings ###############################################

def save_webcam():
    """Function to save the webcam related settings to a text file."""
    
    global timestamp
    
    cam_number=gui_webcam_num.get()     # webcam number retrieved from radiobutton
    cam_width=s_webcam_width.get()      # webcam width is retrieved from the slider
    cam_height=s_webcam_height.get()    # webcam heigth is retrieved from the slider
    cam_crop=s_webcam_crop.get()        # pixels quantity to crop at the frame right side
    facelets_in_width=s_facelets.get()  # max number of facelets in frame width (for the contour area filter)    
    
    cam_settings=(cam_number, cam_width, cam_height, cam_crop, facelets_in_width) # tuple with the settings
    data=timestamp + str(cam_settings)  # string with timestamp and string of webcam settings
    
    try: 
        with open("Cubotino_cam_settings.txt", 'w') as f:        # open the wbcam settings text file in write mode
            f.write(data)                                        # write the string and save/close the file
        print(f'\nsaving the webcam settings: {cam_settings}')   # feedback is printed to the terminal

    except:
        print("Something is wrong with Cubotino_cam_settings.txt file")

    backup = timestamp + "_cam_backup_" + str(cam_settings)      # string with timestamp and string of webcam settings
    try: 
        with open("Cubotino_cam_settings_backup.txt", "w") as f: # open the servos settings text backup file in read mode
            f.write(backup)                                      # data is on first line
        print("saved last settings to Cubotino_cam_settings_backup.txt file")
    except:
        print("Something went wrong while saving Cubotino_cam_settings_backup.txt file")
     
        

def webcam_width(val):
    cam_width = int(val)          # width of the webacam setting in pixels

def webcam_height(val):
    cam_height = int(val)         # height of the webacam setting in pixels

def webcam_crop(val):
    cam_crop = int(val)           # crop quantity of pixels to the right frame side

def facelets_width(val):
    facelets_in_width = int(val)  # min number of facelets side in frame width (to filer out too small squares)

########################################################################################################################





# ####################################################################################################################
# ############################### GUI high level part ################################################################
root = tk.Tk()                                     # initialize tkinter as root 
root.title("CUBOTino: Rubik's cube solver robot")  # name is assigned to GUI root
try:
    root.iconbitmap("Rubiks-cube.ico")             # custom icon is assigned to GUI root
except:
    pass


app_width = 12*width+3*gap+40+320               # GUI width is defined via the facelet width
app_height = max(9*width+2*gap+40,740)          # GUI height is defined via the facelet width, with a minimum size 670 pixels
root.minsize(int(0.9*app_width), int(0.9*app_height))       # min GUI size, limiting the resizing on screen
root.maxsize(int(1.2*app_width), int(1.2*app_height))       # max GUI size, limiting the resizing on screen

# calculate x and y coordinates for the Tk root window starting coordinate
ws = root.winfo_screenwidth()             # width of the screen
hs = root.winfo_screenheight()            # height of the screen
x = int((ws/2) - (app_width/2))           # top left x coordinate to center on the screen the GUI at its opening
y = int((hs/2) - (app_height/2))          # top left y coordinate to center on the screen the GUI at its opening


root.geometry(f'{app_width}x{app_height}+{x}+{y}') # setting the GUI dimension, and its centering to the screen
root.resizable(True, True)                         # allowing the root windows to be resizeale

root.rowconfigure(0, weight=1)                 # root is set to have 1 row of  weight=1
root.columnconfigure(0,weight=1)               # root is set to have 1 column of weight=1

# two windows
mainWindow=tk.Frame(root)                      # a first windows (called mainWindow) is derived from root
settingWindow=tk.Frame(root)                   # a second windows (called settingWindow) is derived from root
for window in (mainWindow, settingWindow):     # iteration over the two defined windows
    window.grid(row=0,column=0,sticky='nsew')  # each window goes to the only row/column, and centered

show_window(mainWindow)                        # the first window (mainWindow) is the one that will initially appear

# the first window (mainWindow) is devided in 2 frames, a graphical one on the left and an interactibve one on the right'
gui_f1 = tk.Frame(mainWindow, width= 12*width+3*gap+20, height= 9*width+2*gap+40)  # first frame (gui_f1), dimensions
gui_f2 = tk.Frame(mainWindow, width= 3 * width, height= 9 * width + 40)        # second frame (gui_f2), dimensions
gui_f1.grid(row=0, column=0,sticky="ns")      # frame1 takes the left side
gui_f2.grid(row=0, column=1,sticky="ns")      # frame2 takes the right side
gui_f2.grid_rowconfigure(15, weight=1)        # frame2 uses 15 rows
gui_f2.grid_columnconfigure(0, weight=1)      # frame2 uses 1 column

# a canvas is made and positioned to fully cover frame gui_f1, in the mainWindow
gui_canvas = tk.Canvas(gui_f1, width=12*width+3*gap+20, height=9*width+2*gap+40, highlightthickness=0)  # gui_canvas for most of the "graphic"
gui_canvas.pack(side="top", fill="both", expand="true")   # gui_canvas is packed in gui_f1
   
root.bind("<Button-1>", click)                # pressing the left mouse button calls the click function
root.bind("<MouseWheel>", scroll)             # scrol up of the mouse wheel calls the scroll function 
########################################################################################################################






# ############################### GUI low level part ###################################################################
# ############################### Main windows widget ##################################################################

# gui text windows for feedback messages
gui_text_window = tk.Text(gui_canvas,highlightthickness=0)
gui_text_window.place(x=20+6*width+10+2*gap, y=20, height=3*width-10, width=6*width-10)


# cube status and solve buttons
cube_status_label = tk.LabelFrame(gui_f2, text="Cube status", labelanchor="nw", font=("Arial", "12"))
cube_status_label.grid(column=0, row=0, columnspan=2, sticky="w", padx=10, pady=10)


# radiobuttons for cube status source
read_modes=["webcam","screen sketch"]  #,"robot color sens"]
gui_read_var = tk.StringVar()
for i, read_mode in enumerate(read_modes):
    rb=tk.Radiobutton(cube_status_label, text=read_mode, variable=gui_read_var, value=read_mode)
    rb.configure(font=("Arial", "10"))
    rb.grid(column=0, row=i, sticky="w", padx=10, pady=0)
gui_read_var.set("webcam")


# buttons for the cube status part
b_read_solve = tk.Button(cube_status_label, text="Read &\nsolve", height=3, width=11, command=cube_read_solve)
b_read_solve.configure(font=("Arial", "12"), bg="gray90", activebackground="gray90")
b_read_solve.grid(column=1, row=0, sticky="w", rowspan=3, padx=10, pady=5)

b_empty = tk.Button(cube_status_label, text="Empty", height=1, width=12, command=empty)
b_empty.configure(font=("Arial", "11"))
b_empty.grid(column=0, row=3, sticky="w", padx=10, pady=5)

b_clean = tk.Button(cube_status_label, text="Clean", height=1, width=11, command=clean)
b_clean.configure(font=("Arial", "11"))
b_clean.grid(column=1, row=3, sticky="w",padx=10, pady=5)

b_random = tk.Button(cube_status_label,text="Random", height=1, width=12, command=random)
b_random.configure(font=("Arial", "11"))
b_random.grid(column=0, row=4, padx=10, pady=10, sticky="w")

# checkbuttons for cube scrambling
gui_scramble_var = tk.BooleanVar()
cb_scramble=tk.Checkbutton(cube_status_label, text="scramble", variable=gui_scramble_var)
cb_scramble.configure(font=("Arial", "10"))
cb_scramble.grid(column=1, row=4, sticky="ew", padx=5, pady=5)
gui_scramble_var.set(0)


# robot related buttons
gui_robot_label = tk.LabelFrame(gui_f2, text="Robot", labelanchor="nw", font=("Arial", "12"))
gui_robot_label.grid(column=0, row=6, rowspan=11, columnspan=2, sticky="n", padx=10, pady=10)

b_robot = tk.Button(gui_robot_label, text="Robot", command=robot_solver, height=6, width=11)
b_robot.configure(font=("Arial", "12"), relief="sunken", state="disable")
b_robot.grid(column=1, row=7, sticky="w", rowspan=3, padx=10, pady=5)

b_refresh = tk.Button(gui_robot_label, text="Refresh COM", height=1, width=12, command=update_coms)
b_refresh.configure(font=("Arial", "11"))
b_refresh.grid(column=0, row=7, sticky="w", padx=10, pady=5) 

b_connect = tk.Button(gui_robot_label, text="Connect", height=1, width=12, state="disable", command=connection)
b_connect.configure(font=("Arial", "11"))
b_connect.grid(column=0, row=9, sticky="w", padx=10, pady=5)

gui_canvas2=tk.Canvas(gui_robot_label,width=200, height=200)  # a second canvas, for the Cubotino sketch
gui_canvas2.grid(column=0, row=11, columnspan=2, pady=5)

gui_prog_bar = ttk.Progressbar(gui_robot_label, orient="horizontal", length=175, mode="determinate")
gui_prog_bar.grid(column=0, row=12, sticky="w", padx=10, pady=10, columnspan=2)

gui_prog_label_text = tk.StringVar()
gui_prog_label = tk.Label(gui_robot_label, height=1, width=5, textvariable=gui_prog_label_text, font=("arial", 12), bg="#E6E6E6")
gui_prog_label.grid(column=1, sticky="e", row=12, padx=10, pady=10)

b_settings = tk.Button(gui_robot_label, text="Settings window", height=1, width=26, state="disable",
                       command= lambda: show_window(settingWindow))
b_settings.configure(font=("Arial", "11"))
b_settings.grid(column=0, row=13, columnspan=2,  padx=10, pady=5)






# ############################### Settings windows widgets #############################################################

#### general settings related widgets ####   
b_back = tk.Button(settingWindow, text="Main page", #fg='red', activeforeground= 'red',
                   height=1, width=12, state="active", command= lambda: show_window(mainWindow))
b_back.configure(font=("Arial", "12"))
b_back.grid(row=9, column=4, sticky="e", padx=20, pady=20)



#### getting and sending settings from/to the robot ####
b_get_settings = tk.Button(settingWindow, text="Get current CUBOTino settings", height=1, width=26,
                           state="active", command= get_current_servo_settings)
b_get_settings.configure(font=("Arial", "11"))
b_get_settings.grid(row=0, column=0, sticky="w", padx=20, pady=10)


b_send_settings = tk.Button(settingWindow, text="Send new settings to CUBOTino", height=1, width=26,
                           state="active", command= send_new_servo_settings)
b_send_settings.configure(font=("Arial", "11"))
b_send_settings.grid(row=0, column=1, sticky="w", padx=20, pady=10)


#### estimate facelets check button ####
checkVar2 = tk.IntVar()
c_estimate = tk.Checkbutton(settingWindow, text = "estimate facelets \n(beta version)", variable = checkVar2,
                            command=estimate_fclts_check, onvalue = 1, offvalue = 0)
c_estimate.configure(font=("Arial", "11"))
c_estimate.grid(row=0, column=3, sticky="w", padx=20, pady=10)


#### debug check button ####
checkVar1 = tk.IntVar()
c_debug = tk.Checkbutton(settingWindow, text = "debug print-out\n(webcam)", variable = checkVar1,
                         command=debug_check, onvalue = 1, offvalue = 0)
c_debug.configure(font=("Arial", "11"))
c_debug.grid(row=0, column=4, sticky="w", padx=20, pady=10)


#### servos min and max pulse width widgets ####

# overall label frame for the servos pulse width section
srv_pw_label = tk.LabelFrame(settingWindow, text="Servos - pulse width range",
                             labelanchor="nw", font=("Arial", "12"))
srv_pw_label.grid(row=1, column=0, rowspan=2, columnspan=5, sticky="w", padx=20, pady=15)

servos_modes=[("1-2 ms","small"),("0.5-2.5ms","large")]

# label frame and radiobutton for the top servo pulse width
t_srv_pw_label = tk.LabelFrame(srv_pw_label, text="Top servo", labelanchor="nw", font=("Arial", "12"))
t_srv_pw_label.grid(row=1, column=0, rowspan=2, columnspan=2, sticky="w", padx=20, pady=15)
gui_var_t_srv_pw = tk.StringVar()
pos=0
for servos_mode, servos_pw in servos_modes:
    rb_srv=tk.Radiobutton(t_srv_pw_label, text=servos_mode, variable=gui_var_t_srv_pw, value=servos_pw)
    rb_srv.configure(font=("Arial", "10"))
    rb_srv.grid(row=2, column=pos, sticky="w", padx=12, pady=5)
    pos+=1
gui_var_t_srv_pw.set(t_srv_pw_range)

# label frame and radiobutton for the bottom servo pulse width
b_srv_pw_label = tk.LabelFrame(srv_pw_label, text="Bottom servo", labelanchor="nw", font=("Arial", "12"))
b_srv_pw_label.grid(row=1, column=2, rowspan=2, columnspan=2, sticky="w", padx=20, pady=15)
servos_modes=[("1-2 ms","small"),("0.5-2.5ms","large")]
gui_var_b_srv_pw = tk.StringVar()
pos=0
for servos_mode, servos_pw in servos_modes:
    rb_srv=tk.Radiobutton(b_srv_pw_label, text=servos_mode, variable=gui_var_b_srv_pw, value=servos_pw)
    rb_srv.configure(font=("Arial", "10"))
    rb_srv.grid(row=2, column=pos, sticky="w", padx=12, pady=5)
    pos+=1
gui_var_b_srv_pw.set(b_srv_pw_range)


# button to process the servos pulse width choice
pw_update_btn = tk.Button(srv_pw_label, text="confirm\nchanges", height=2, width=18, state="active", command= pw_update)
pw_update_btn.configure(font=("Arial", "12"))
pw_update_btn.grid(row=2, column=5, sticky="w", padx=15, pady=10)




#### top servo related widgets ####
top_srv_label = tk.LabelFrame(settingWindow, text="Top cover - servo settings",
                                   labelanchor="nw", font=("Arial", "12"))
top_srv_label.grid(row=3, column=0, rowspan=3, columnspan=4, sticky="w", padx=20, pady=0)

s_top_srv_flip = tk.Scale(top_srv_label, label="PWM flip", font=('arial','11'), orient='horizontal',
                               length=170, from_=s_pwm_flip_min, to_=s_pwm_flip_max, command=servo_flip)
s_top_srv_flip.grid(row=4, column=0, sticky="w", padx=12, pady=5)
s_top_srv_flip.set(t_servo_flip)


s_top_srv_open = tk.Scale(top_srv_label, label="PWM open", font=('arial','11'), orient='horizontal',
                               length=170, from_=s_pwm_open_min, to_=s_pwm_open_max, command=servo_open)
s_top_srv_open.grid(row=4, column=1, sticky="w", padx=12, pady=5)
s_top_srv_open.set(t_servo_open)


s_top_srv_close = tk.Scale(top_srv_label, label="PWM close", font=('arial','11'), orient='horizontal',
                              length=170, from_=s_pwm_close_min, to_=s_pwm_close_max, command=servo_close)
s_top_srv_close.grid(row=4, column=2, sticky="w", padx=12, pady=5)
s_top_srv_close.set(t_servo_close)


s_top_srv_release = tk.Scale(top_srv_label, label="PWM release from close", font=('arial','11'), orient='horizontal',
                              length=170, from_=0, to_=10, command=servo_release)
s_top_srv_release.grid(row=4, column=3, sticky="w", padx=12, pady=5)
s_top_srv_release.set(t_servo_close)


flip_btn = tk.Button(top_srv_label, text="FLIP  (toggle)", height=1, width=18, state="active", command= flip_cube)
flip_btn.configure(font=("Arial", "12"))
flip_btn.grid(row=5, column=0, sticky="w", padx=15, pady=10)

open_btn = tk.Button(top_srv_label, text="OPEN", height=1, width=18, state="active", command= open_top_cover)
open_btn.configure(font=("Arial", "12"))
open_btn.grid(row=5, column=1, sticky="w", padx=15, pady=10)

close_btn = tk.Button(top_srv_label, text="CLOSE", height=1, width=18, state="active", command= close_top_cover)
close_btn.configure(font=("Arial", "12"))
close_btn.grid(row=5, column=2, sticky="w", padx=15, pady=10)


s_top_srv_flip_to_close_time = tk.Scale(top_srv_label, label="TIME: flip > close (ms)", font=('arial','11'),
                                        orient='horizontal', length=170, from_=200, to_=1000,
                                        resolution=50, command=flip_to_close_time)
s_top_srv_flip_to_close_time.grid(row=6, column=0, sticky="w", padx=12, pady=5)
s_top_srv_flip_to_close_time.set(t_flip_to_close_time)


s_top_srv_close_to_flip_time = tk.Scale(top_srv_label, label="TIME: close > flip (ms)", font=('arial','11'),
                                        orient='horizontal', length=170, from_=200, to_=1000,
                                        resolution=50, command=close_to_flip_time)
s_top_srv_close_to_flip_time.grid(row=6, column=1, sticky="w", padx=12, pady=5)
s_top_srv_close_to_flip_time.set(t_close_to_flip_time)


s_top_srv_flip_open_time = tk.Scale(top_srv_label, label="TIME: flip <> open (ms)", font=('arial','11'),
                                    orient='horizontal', length=170, from_=200, to_=1000,
                                    resolution=50, command=flip_open_time)
s_top_srv_flip_open_time.grid(row=6, column=2, sticky="w", padx=10, pady=5)
s_top_srv_flip_open_time.set(t_flip_open_time)


s_top_srv_open_close_time = tk.Scale(top_srv_label, label="TIME: open <> close(ms)", font=('arial','11'),
                                     orient='horizontal', length=170, from_=100, to_=700,
                                     resolution=50, command=open_close_time)
s_top_srv_open_close_time.grid(row=6, column=3, sticky="w", padx=12, pady=5)
s_top_srv_open_close_time.set(t_open_close_time)





#### bottom servo related widgets ####
b_srv_label = tk.LabelFrame(settingWindow, text="Cube holder - servo settings",
                                   labelanchor="nw", font=("Arial", "12"))
b_srv_label.grid(row=7, column=0, columnspan=5, sticky="w", padx=20, pady=10)


s_btm_srv_CCW = tk.Scale(b_srv_label, label="PWM CCW", font=('arial','11'), orient='horizontal',
                              length=170, from_=s_pwm_ccw_min, to_=s_pwm_ccw_max, command=servo_CCW)
s_btm_srv_CCW.grid(row=8, column=0, sticky="w", padx=13, pady=5)
s_btm_srv_CCW.set(b_servo_CCW)


s_btm_srv_home = tk.Scale(b_srv_label, label="PWM home", font=('arial','11'), orient='horizontal',
                               length=170, from_=s_pwm_home_min, to_=s_pwm_home_max, command=servo_home)
s_btm_srv_home.grid(row=8, column=1, sticky="w", padx=12, pady=5)
s_btm_srv_home.set(b_home)


s_btm_srv_CW = tk.Scale(b_srv_label, label="PWM CW", font=('arial','11'), orient='horizontal',
                               length=170, from_=s_pwm_cw_min, to_=s_pwm_cw_max, command=servo_CW)
s_btm_srv_CW.grid(row=8, column=2, sticky="w", padx=12, pady=5)
s_btm_srv_CW.set(b_servo_CW)


s_btm_srv_extra_sides = tk.Scale(b_srv_label, label="PWM release CW/CCW", font=('arial','11'), orient='horizontal',
                               length=170, from_=0, to_=11, command=servo_extra_sides)
s_btm_srv_extra_sides.grid(row=8, column=3, sticky="w", padx=12, pady=5)
s_btm_srv_extra_sides.set(b_extra_sides)


s_btm_srv_extra_home = tk.Scale(b_srv_label, label="PWM release at home", font=('arial','11'), orient='horizontal',
                               length=170, from_=0, to_=11, command=b_extra_home)
s_btm_srv_extra_home.grid(row=8, column=4, sticky="w", padx=12, pady=5)
s_btm_srv_extra_home.set(b_extra_home)


CCW_btn = tk.Button(b_srv_label, text="CCW", height=1, width=18, state="active", command= ccw)
CCW_btn.configure(font=("Arial", "12"))
CCW_btn.grid(row=9, column=0, sticky="w", padx=15, pady=10)


close_btn = tk.Button(b_srv_label, text="HOME", height=1, width=18, state="active", command= home)
close_btn.configure(font=("Arial", "12"))
close_btn.grid(row=9, column=1, sticky="w", padx=15, pady=10)


CW_btn = tk.Button(b_srv_label, text="CW", height=1, width=18, state="active", command= cw)
CW_btn.configure(font=("Arial", "12"))
CW_btn.grid(row=9, column=2, sticky="w", padx=15, pady=10)


s_btm_srv_spin_time = tk.Scale(b_srv_label, label="TIME: spin (ms)", font=('arial','11'), orient='horizontal',
                               length=170, from_=300, to_=1200,  resolution=50, command=b_spin_time)
s_btm_srv_spin_time.grid(row=10, column=0, sticky="w", padx=12, pady=5)
s_btm_srv_spin_time.set(b_spin_time)


s_btm_srv_rotate_time = tk.Scale(b_srv_label, label="TIME: rotate (ms)", font=('arial','11'), orient='horizontal',
                               length=170, from_=300, to_=1300,  resolution=50, command=b_rotate_time)
s_btm_srv_rotate_time.grid(row=10, column=1, sticky="w", padx=12, pady=5)
s_btm_srv_rotate_time.set(b_rotate_time)


s_btm_srv_rel_time = tk.Scale(b_srv_label, label="TIME: release (ms)", font=('arial','11'), orient='horizontal',
                               length=170, from_=0, to_=400,  resolution=50, command=b_rel_time)
s_btm_srv_rel_time.grid(row=10, column=2, sticky="w", padx=12, pady=5)
s_btm_srv_rel_time.set(b_rel_time)





#### webcam  ####
webcam_label = tk.LabelFrame(settingWindow, text="Webcam", labelanchor="nw", font=("Arial", "12"))
webcam_label.grid(row=9, column=0, columnspan=6, sticky="w", padx=20, pady=10)

# radiobuttons for webcam source
webcam_nums=[0,1] #,2]
gui_webcam_num = tk.IntVar()
for i, webcam_num in enumerate(webcam_nums):
    rb=tk.Radiobutton(webcam_label, text=webcam_num, variable=gui_webcam_num, value=webcam_num)
    rb.configure(font=("Arial", "10"))
    rb.grid(row=10, column=0+i, sticky="w", padx=6, pady=0)
gui_webcam_num.set(cam_number)


s_webcam_width = tk.Scale(webcam_label, label="cam width", font=('arial','11'), orient='horizontal',
                               length=120, from_=640, to_=1280,  resolution=20, command=webcam_width)
s_webcam_width.grid(row=10, column=3, sticky="w", padx=15, pady=5)
s_webcam_width.set(cam_width)


s_webcam_height = tk.Scale(webcam_label, label="cam height", font=('arial','11'), orient='horizontal',
                               length=120, from_=360, to_=720,  resolution=20, command=webcam_height)
s_webcam_height.grid(row=10, column=4, sticky="w", padx=8, pady=5)
s_webcam_height.set(cam_height)


s_webcam_crop = tk.Scale(webcam_label, label="right crop", font=('arial','11'), orient='horizontal',
                               length=120, from_=0, to_=300,  resolution=20, command=webcam_crop)
s_webcam_crop.grid(row=10, column=5, sticky="w", padx=8, pady=5)
s_webcam_crop.set(cam_crop)


s_facelets = tk.Scale(webcam_label, label="distance", font=('arial','11'), orient='horizontal',
                               length=120, from_=10, to_=15,  command=facelets_width)
s_facelets.grid(row=10, column=6, sticky="w", padx=8, pady=5)
s_facelets.set(facelets_in_width)


save_cam_num_btn = tk.Button(webcam_label, text="save cam settings", height=1, width=16, state="active",
                    command= save_webcam)
save_cam_num_btn.configure(font=("Arial", "12"))
save_cam_num_btn.grid(row=10, column=8, sticky="w", padx=10, pady=10)

########################################################################################################################








# ############################### general GUI  #########################################################################

create_facelet_rects(width)                       # calls the function to generate the cube sketch
create_colorpick(width)                           # calls the function to generate the color-picking palette
update_coms()                                     # calls the function to generate the cube sketch
root.protocol("WM_DELETE_WINDOW", close_window)   # the function close_function is called when the windows is closed
root.mainloop()                                   # tkinter main loop

########################################################################################################################



