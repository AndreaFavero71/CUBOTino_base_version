def swipe_and_center():
    """ Function that spins the cube holder to both end positions before stopping to the middle one.
        This is meant to:
        1) check the rotation range of the servo before assembling the robot.
        2) set the servos to their mid position, so to properly connect the arms."""
        
    from machine import Pin, PWM
    from utime import sleep_ms

    t_servo= PWM(Pin(22), freq=50)  # top servo, connected to Pin 22
    sleep_ms(50)

    b_servo= PWM(Pin(23), freq=50)  # bottom servo, connected to Pin 23
    sleep_ms(50)
    
    max_1000to2000us = 106          # Servo duty value for angle -100deg, beyond limit of a 1 to 2ms servo
    max_500to2500us = 134           # Servo duty value for angle -100deg, beyond limit of a 0.5 to 2.5ms servo
    min_1000to2000us = 48           # Servo duty value for angle 100deg, beyond limit of a 1 to 2ms servo
    min_500to2500us = 20            # Servo duty value for angle 100deg, beyond limit of a 0.5 to 2.5ms servo
    mid_pos = 76                    # Servo duty value for angle 0deg, for 1to2ms and 0.5to2.5ms servo
    
    b_servo.duty(mid_pos)           # servo is set to mid position
    t_servo.duty(mid_pos)           # servo is set to mid position
    print(f"\nservos set to their middle position")
    sleep_ms(1200)                  # time for the servos to reach the mid position
    
    # because of different resolution of servos and accepted value:
    # a 1to2ms servo will stop rotating earlier, and each step will take larger rotation
    # while a 500to2500us servo will rotate for longer time, and each step will take smaller rotation
    
    print(f"\nservos rotating in steps toward one rotation extreme (CCW, from servo point of view)")
    for i in range(mid_pos, min_500to2500us, -1):        # swipe from mid to min position
        b_servo.duty(i)
        t_servo.duty(i)
        sleep_ms(80)
    sleep_ms(1000)
    
    print(f"\nservos rotating in steps toward the other rotation extreme (CW, from servo point of view)")
    for i in range(min_500to2500us, max_500to2500us, 1): # swipe from min to max position
        b_servo.duty(i)
        t_servo.duty(i)
        sleep_ms(80)
    sleep_ms(1000)
    
    print(f"\nservos rotating in steps back to their middle position")
    for i in range(max_500to2500us, mid_pos, -1):        # swipe from max to mid position
        b_servo.duty(i)
        t_servo.duty(i)
        sleep_ms(80)
    
    print(f"\nservos are back to their middle position")
    

        



if __name__ == "__main__":
    swipe_and_center()