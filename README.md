# Disc Robot
The server code is flashed onto the microcontroller on the arm that controls the small motor. The client is the microcontroller on the base that interfaces with the user, controls the big motor, and the release solenoid.

The information exchanged over bluetooth is below. For simplicity, a write or a notify event can be thought of as an interrupt to the receiver.\
Client writing to the server: goal speed, uint16_t\
Server notifying the client: motor state, enum {stopped, pending, done}

## Server-arm
Connect to the server board using a client device such as the STM32 BLE Toolbox mobile app or the client board. Send a speed to the server in hexidecimal.\
Most of the bluetooth code that will be modified is in Server_arm/STM32_WPAN/App/p2p_server_app.c.\
Other functions can be placed in Server_arm/Core/Src/arm_utilities.c\
Any gets or sets to certain system perhiperials such as timers should be done in main.c

## Server-arm-toolbox
Runs a simulation of the arm motor. You can connect to the board using the STM32 BLE Toolbox mobile app and directly control it.
Press a button on the board to send to the client the motor state {STOPPED, PENDING, DONE}. From the app, send a 16 bit hex integer as the desired motor speed to the 
board. LEDs will light up depending on the range. (0-300, Blue), (301-600, Green), (600+, Red). 

## big_motor
For testing the big motor in isolation.

## small_motor
For testing the small motor in isolation.

## solenoid
For testing the solenoid in isolation. Both standard GPIO output and PWM signals can be used.
