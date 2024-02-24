# Disc Robot
The server code is flashed onto the microcontroller on the arm that controls the small motor. The client is the microcontroller on the base that interfaces with the user, controls the big motor, and the release solenoid.

The information exchanged over bluetooth is below. For simplicity, a write or a notify event can be thought of as an interrupt to the receiver.\
Client writing to the server: goal speed, uint16_t\
Server notifying the client: motor state, enum {stopped, pending, done}

## Server-arm
Most of the bluetooth code that will be modified is in Server_arm/STM32_WPAN/App/p2p_server_app.c.\
Other functions can be placed in Server_arm/Core/Src/arm_utilities.c
