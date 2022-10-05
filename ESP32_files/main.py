"""
##############################################################################################################################
# Andrea Favero April 2022
#
# This script relates to "CUBOTino", a simpler (and much cheaper) Rubik's cube solver robot than my first one:
# (https://www.youtube.com/watch?v=oYRXe4NyJqs)
#
# This script has the function to comunicate with the PC, and with the CUBOTino modules
# Via UART communication, the cube solution string is received.
# Via the Cubotino_moves.py module, the cube solution string is converted in a string of robot moves.
# Via the Cubotino_servos.py module, the robot servos are controlled according to the string of robot moves.
#
#
# Note: If the cube status has been entered manually, or detected via a webcam apart from the robot,
# the cube should be positioned with the Front face facing the viewer, and Upper facing upward; The CUBOTino sketch on GUI
# suggests the right cube orientation
#
##############################################################################################################################
"""

from machine import Pin, Timer, TouchPad
from utime import sleep_ms
import sys, select



def initialize_robot(debug):
    """ Function to initialize the servos parameters and positions."""
    
    import Cubotino_servos as servo               # converts the robot moves string to servos actions
    robot_init_status = servo.init_servo(debug)   # call the function that operates the servos
    return robot_init_status                      # returned the boolean variable, indicating success or failure of servos inititialization






def robot_fun(debug):
    """ Function to initialize the servos parameters and positions."""
    
    import Cubotino_servos as servo               # converts the robot moves string to servos actions
    fun_status=servo.fun(debug)                   # call the function that operates the servos, for a fun dance
    return fun_status                             # returned the boolean variable, indicating success or failure of fun dance






def save_new_settings(strMsg):
    """ Function to save the received new settings, and to update the variableshelp the servos positions settings."""
    
    data_start=strMsg.find('(')                     # location of the ( char on the string, that is where the settings content start
    data_end=strMsg.find(')')                       # location of the ) char on the string, that is where the settings content stop
    data=strMsg[data_start:data_end+1]              # settings content is retrived from the string
    with open("Cubotino_settings.txt", "w") as f:   # the text file is opened in write mode
        f.write(data)                               # the new settings are written to the text file (by over writing the previous settings)
    print("new_settings:" + str(data))              # message to UART with settings
    
    import Cubotino_servos as servo                 # module that manages the servos actions
    servo.upload_settings()                         # requests the robot to upload the servos settings






def test_robot(data,debug):
    """ Function to test the servos positions settings."""
    
    import Cubotino_servos as servo          # converts the robot moves string to servos actions
    upload_status = servo.upload_settings()  # forces the robot to upload settings, to ensure testing the latest settings changes

    if upload_status:                        # case the settings uploading process has been successfull
        servo.stop_release(debug)            # release the stop flag in case the robot has been previously stopped, and not re-initialized yet
        if 'flip' in data:
            servo.flip_test()                # flip test function
        elif 'close' in data:
            servo.close_cover()              # close_cover function
        elif 'open' in data:
            servo.open_cover()               # open_cover function
        elif 'ccw' in data:
            servo.rotate_CCW_CW_test('CCW')  # rotate cube holder to CCW function
        elif 'home' in data:
            servo.rotate_home_test()         # rotate cube holder to home function
        elif 'cw' in data:
            servo.rotate_CCW_CW_test('CW')   # rotate cube holder to CW function
        else:
            return




    

def robot_solver(solution, debug, stop_btn, btn_ref):
    """ Function that convert strings:
            - From Kociemba solver solution string to robot moves string
            - From robot moves string to servo action string
        and activates the servos based on the created string."""
    
    import Cubotino_moves as cubotino                                   # converts the solution string from Kociemba solver to robot moves string
    import Cubotino_servos as servo                                     # module that manages the servos actions
    start_t=0
    end_t=0
    
    flash.init(period=100, mode=Timer.PERIODIC, callback=flash_led)     # keeps the ESP blue led flashing when the robot is solving the cube
    
    # solution (from Kociemba solver) is converted in robot moves dict, string and amount of movement
    robot, moves, robot_tot_moves = cubotino.robot_required_moves(solution,"")

    if debug:
        print(f'robot moves:{moves}')                                        # print for debug

    robot_status, robot_time = servo.servo_solve_cube(moves, debug, stop_btn, btn_ref)   # call the function that operates the servos
    
    if 'stopped' in robot_status or 'solved'in robot_status:                 # cases the robot is stopped or it has finished the cube solving 
        flash.deinit()       # timer for the led flashing is stopped
        led.on()             # led is forced on, expecting the UART being still communicating properly
        
    return robot_status, robot_time     # returned a string with the robot status, and an integer with robot time in secs






def solution_string(strMsg):
    """Sanity check on the received cube solution string."""
    if debug:
        solution = strMsg.decode()      # received string is decoded
        solution = solution.strip()     # empty spaces are removed
    
    elif not debug:
        solution = strMsg
    
    pos=solution.find('(')              # position of the "(" character in the string
    solution=solution[:pos]             # string is sliced, by removing the additional info from Kociemba solver
    
    for char in solution:               # iteration over the characters of the string
        if char not in ['U','R','F','D','L','B','1','2','3']:   # case the character differs from those listed (expected characters!)
            solution=solution.replace(char,"")                  # the character is removed
    sol_string_ready=True                                       # boolean variable used to track the readiness of the cube solution string                                 
    return solution, sol_string_ready                           # cube solution string, and its readiness status, are returned






def flash_led(timer):
    """ function to alternation on/off the blue led."""
    
    led.value(not led.value())          # led status changes each time this function is called





############################################# MAIN FUNCTION ####################################################

def main_func(debug, robot_init_status, robot_status, stop_btn, btn_ref, connect_status):
    """ This is substantially the main functio, with the largest communication part with the PC.
        When the cube solution string is available, then the robot solving modules are called
        For the communication it is used the same port the ESP32 uses for its programming, forcing a
        slightly creative mode to use the uart."""
    
    strMsg=''                             # string variable used to generate the received string
    
    while True:                           # infinite loop
        sleep_ms(20)                      # fix little delay, to prevent using much resorcuces from the microcontroller
        if connect_status:                # case the conenction with the PC is established
            led.on()                      # ESP32 led is turned on when the serial connection is established
        elif not connect_status:          # case the conenction with the PC is not established
            led.off()                     # ESP32 led is turned off when the serial connection is dropped
        
        if 'solved' in robot_status:                        # case robot status includes the solved word  
            print(f'solved_({robot_time})')                 # message to UART that cube is solved, and related solving time
            robot_fun(debug)                                # fun function ... some cube_holder movements to get attention                                   
            robot_status=''                                 # cube status is set empty
    
        elif 'stopped' in robot_status:                     # case robot status includes the stopped word  
            print(f'stopped_({robot_time})')                # message to UART on cube stopped, and the tiime the robot has worked
            robot_status=''                                 # cube status is set empty
  
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:  # while case there is data on the uart buffer
            ch = sys.stdin.read(1).strip('\n')                        # one character is retrieved

            if '[' in ch:                                             # case the character is an open square bracket (start of a message content)
                strMsg=''                                             # string message variable used to generate the received string is emptied
            
            elif ']' in ch:                                           # case the character is a close square bracket (end of a message content)
                if 'current_settings' in strMsg:                      # case the message string includes the 'current_settings' word 
                    with open("Cubotino_settings.txt", "r") as f:     # txt file with settings is opened in read mode
                        data=f.readline()                             # first line is returned
                    print("current_settings" + str(data))             # message to UART with settings
                         
                elif 'new_settings' in strMsg:                        # case the message string includes the 'new_settings' word 
                    save_new_settings(strMsg)                         # function to save the new settings is called
                
                elif 'test' in strMsg:                                # case the message string includes the 'test' word 
                    data_start=strMsg.find('(')                       # string position for the open round square character (start of data content)
                    data_end=strMsg.find(')')                         # string position for the open round square character (end of data content)
                    data=strMsg[data_start:data_end+1]                # string is liced to only keep the data content
                    test_robot(data, debug)                           # function to test robot functions is called
                    
                elif 'start' in strMsg:                               # case the message string includes the 'start' word 
                    print('start')                                    # message to UART that the start has been received
                    if sol_string_ready:                              # case the cube solution string is ready
                         robot_status, robot_time  = robot_solver(solution, debug, stop_btn, btn_ref)   # robot solver function is called
                
                elif 'led_on' in strMsg:                              # case the message string includes the 'led_on' word
                    connect_status=True                               # the boolean variable that tracks the connection status is set true
                    if debug:
                        print('ESP 32 is connect_status')             # message to UART the connection is on
                
                elif 'led_off' in strMsg:                             # case the message string includes the 'led_off' word
                    connect_status=False                              # the boolean variable that tracks the connection status is set flase
                    if debug:
                        print('ESP 32 is disconnect_status')          # message to UART the connection is on

                strMsg=''  # string message variable is set empty, as the acceptable words were is cased above

                    
            elif '<' in ch:                      # case the character is a ">" (start of cube solution string)
                strMsg='<'

            elif '>' in ch and '<' in strMsg:    # case the character is an ">" and the message includes the character "<"
                strMsg = strMsg + ch             # the string message add the lates character ">"
                if 'f)' in strMsg:               # case the string message includes "f)" (means it has the cube solution from Kociemba solver)   
                    solution, sol_string_ready = solution_string(strMsg)  # function to split info from the Kociemba cube solution string
                    print(strMsg)    # the Kociemba cube solution string is printed to the uart, for communication sanity check at GUI 
                    strMsg=''        # string message variable is set empty

            else:                    # case the caracter is not "[", nor "]", nor "<" and not ">"
                strMsg=strMsg+ch     # the received character is added to the string message variable
     





############################################# MAIN PROGRAM ####################################################
# global variables
debug=False                   # boolean variable that enable/disable prints for debug purpose           

robot_init_status=False       # boolean to track the robot initialization status is initially set false
sol_string_ready=False        # boolean to track the cube solution string readiness is initially set false
robot_status=''               # string to track the robot status is initially set empty
connect_status=False          # boolean to track the connection status with the uart is initially set false


led = Pin(2, Pin.OUT)         # create the led object in GPIO 2  
stop_btn=TouchPad(Pin(32))    # create the touch pad button object in GPIO 13
flash=Timer(0)                # create a flash object on hardware timer
flash.deinit()                # flash object is set disable


if not robot_init_status:                          # case the robot is not initialized 
    robot_init_status=initialize_robot(debug)      # robot initialization function is called
    for i in range(3):                             # iteration of three times
        led.on()                                   # ESP32 blue led is switched on
        sleep_ms(100)                              # delay of 100ms
        led.off()                                  # ESP32 blue led is switched off
        sleep_ms(100)                              # delay of 100ms

    stop_btn_value=stop_btn.read()                 # create a global variable for the touchPad button variable
    btn_ref = 0                                    # button threshold reference is set initially to zero
    for i in range(1000):                          # iteration for 1000 readings
        btn_ref+=stop_btn_value                    # touch pad value is added to the reference at each iteration                                    
    btn_ref=btn_ref//2000                          # button threshold reference is set at 50% the average value during previous iteration
    if debug:
        print("ref_stop_btn_value:",btn_ref)

# main loop
main_func(debug, robot_init_status, robot_status, stop_btn, btn_ref, connect_status)


