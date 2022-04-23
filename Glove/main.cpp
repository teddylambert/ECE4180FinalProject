#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"
#include "PinDetect.h"

//IMU definitions
#define DIR_THRESH 4000
#define IMU_AX 1
#define IMU_AY 2

//IMU sensor pins
#define IMU_SDA p28
#define IMU_SCL p27

//Communication pins
#define RADIO_TX p13
#define RADIO_RX p14

//Flex pot pins
#define FLEX_IN p20

//5-Way Switch pins
#define NAV_LEFT p18
#define NAV_RIGHT p17
#define NAV_FIRE p19

//Command arrays
volatile char move_cmd[2]; //Used to send forward/backward/left/right + speed
volatile char old_move_cmd[2]; 

//Thread activators
volatile bool send_turn_l = false;
volatile bool send_turn_r = false;
volatile bool send_horn = false;

//Declare I/O objects
LSM9DS1 imu(IMU_SDA, IMU_SCL, 0xD6, 0x3C);
RawSerial coordinator(RADIO_TX, RADIO_RX);
AnalogIn speed(p20);
PinDetect NavLeft(NAV_LEFT);
PinDetect NavRight(NAV_RIGHT);
PinDetect NavFire(NAV_FIRE);

//Function to determine which IMU command to send
char imu_threshold(int direction)
{
    imu.readAccel();
    switch(direction) 
    {
        case IMU_AX:  
            if(imu.ax > DIR_THRESH) return 'B';
            else if(imu.ax < -DIR_THRESH) return 'F';
            else return '0';
        case IMU_AY:
            if(imu.ay > DIR_THRESH) return 'L';
            else if(imu.ay < -DIR_THRESH) return 'R';
            else return '0';
        default:
            return '0'; 
    }
}

//Function to determine which speed value to send
//Lower value = more flex
char speed_check(float spd)
{
    if(spd > 0.27) return '0';
    else if(spd > 0.21) return '1';
    else if(spd > 0.15) return '2';
    else return '3';
}

void t_read_movement()
{
    while(1) {
        //Get speed and direction
        char speedVal = speed_check(speed.read());
        char imu_x = imu_threshold(IMU_AX);
        char imu_y = imu_threshold(IMU_AY);
        //If either no speed or no direction, send the stop command
        if (speedVal == '0' || (imu_x == '0' && imu_y == '0')) {
            move_cmd[0] = '0';
            move_cmd[1] = '0';
        }
        else {
            move_cmd[0] = (imu_x != '0') ? imu_x : imu_y; //We know  one of these isn't zero, favor imu_x
            move_cmd[1] = speedVal;
        }
        Thread::wait(50);
    }   
}

void navLeft_cb() {
    send_turn_l = true;
}

void navRight_cb() {
    send_turn_r = true;
}

void navFire_cb() {
    send_horn = true;
}

int main() {    
    //Start I/O
    if (!imu.begin()) {
        while(1) {}
    }
    imu.calibrate();
    
    coordinator.baud(9600);

    //Start buttons
    NavLeft.mode(PullUp);
    NavRight.mode(PullUp);
    NavFire.mode(PullUp);
    wait(0.1); //Wait for pullup
    
    NavLeft.attach_deasserted(&navLeft_cb);
    NavRight.attach_deasserted(&navRight_cb);
    NavFire.attach_deasserted(&navFire_cb);
    
    NavLeft.setSampleFrequency(25000); //25ms
    NavRight.setSampleFrequency(25000); //25ms
    NavFire.setSampleFrequency(25000); //25ms
    
    //Start threads
    Thread move_thread(t_read_movement);
      
    //Main thread
    while(1) {
        //Figure out whether or not to send command
        if(move_cmd[0] == old_move_cmd[0] && move_cmd[1] == old_move_cmd[1]) {} //If commands equal, do nothing
        else { //Otherwise, we have a new movement command to send!
            if(move_cmd[0] != old_move_cmd[0] && move_cmd[0] != '0') //We are changing directions, so we should momentarily stop the motors
            {
                coordinator.putc('0');
                coordinator.putc('0');
            }
            old_move_cmd[0] = move_cmd[0];
            old_move_cmd[1] = move_cmd[1];
            //Send out command
            coordinator.putc(move_cmd[0]);
            coordinator.putc(move_cmd[1]);
        }
        //Figure out whether to send alerts
        if(send_turn_l)
        {
            coordinator.putc('T');
            coordinator.putc('L');
            send_turn_l = false;   
        }
        if(send_turn_r)
        {
            coordinator.putc('T');
            coordinator.putc('R');
            send_turn_r = false;  
        }
        if(send_horn)
        {
            coordinator.putc('H');
            coordinator.putc('1');
            send_horn = false;  
        }
        Thread::yield();
    }
}
