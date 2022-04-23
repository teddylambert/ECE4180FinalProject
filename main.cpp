#include "mbed.h"
#include "rtos.h"
#include "motordriver.h"
#include "XNucleo53L0A1.h"
#include <stdio.h>

//I2C sensor pins
#define VL53L0_I2C_SDA   p28
#define VL53L0_I2C_SCL   p27

//Communication pins
#define RADIO_TX p13
#define RADIO_RX p14

//Motor objects
Motor A(p22, p29, p21, 1); // pwm, fwd, rev , can brake - Left Wheel Motor
Motor B(p26, p7, p8, 1);  // pwm, fwd, rev , can brake - Rights Wheel Motor

//LED objects
DigitalOut LeftTurn(p15);
DigitalOut RightTurn(p16);
DigitalOut ReverseLights(p12);
DigitalOut HeadLights(p19);
DigitalOut BrakeLights(p18);
AnalogIn   LowLight(p20);

//Communication objects
RawSerial xbee(RADIO_TX, RADIO_RX);

//RPG objects
InterruptIn RPG_A(p23,PullUp);//encoder A and B pins/bits use interrupts
InterruptIn RPG_B(p24,PullUp);

//ToF objects
DigitalOut shdn(p30);

//Speaker objects
PwmOut speaker(p25);

// This VL53L0X board test application performs a range measurement in polling mode
// Use 3.3(Vout) for Vin, p9 for SDA, p10 for SCL, P29 for shdn on mbed LPC1768
static XNucleo53L0A1 *board=NULL;

//Thread activators
volatile bool leftturn_thread = false;
volatile bool rightturn_thread = false;
volatile bool horn_thread = false;
volatile bool brakelights_thread = true;
volatile bool reverselights_thread = false;

//RPG vars
volatile int old_enc = 0;
volatile int new_enc = 0;
volatile int enc_count = 200;
const int lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

//Communication variables
enum statetype {waiting = 0, got_off, got_turn, got_horn, got_for, got_back, got_left, got_right};
statetype state = waiting;

void Enc_change_ISR(void)
{
    new_enc = RPG_A<<1 | RPG_B;//current encoder bits
    //check truth table for -1,0 or +1 added to count
    enc_count = enc_count + lookup_table[old_enc<<2 | new_enc];
    old_enc = new_enc;
}

void parse_message()
{
    char ch;
    int spd; 
    switch (state) {
        case waiting:
        {
            ch = xbee.getc(); //Parse first char
            if (ch == 'T') state = got_turn;
            else if (ch == 'H') state = got_horn;
            else if (ch == 'F') state = got_for;
            else if (ch == 'B') state = got_back;
            else if (ch == 'L') state = got_left;
            else if (ch == 'R') state = got_right;
            else if (ch == '0') state = got_off;
            else state = waiting;
            break;
        }
        case got_for:
        {
            ch = xbee.getc(); //Get 1, 2, or 3 value
            spd = ch - '0'; //Convert speed char to int
            brakelights_thread = false; //Turn off brakelights
            switch(spd)
            {
             case 1:
                A.speed(0.33);
                B.speed(0.33); 
             break;
             case 2:
                A.speed(0.66);
                B.speed(0.66);
             break; 
             case 3:
                A.speed(1);
                B.speed(1);
             break;
            }  
            state = waiting;
            break;
        }
        case got_back:
        {
            ch = xbee.getc(); //Get 1, 2, or 3 value
            spd = ch - '0';
            reverselights_thread = true; //Turn on reverselights
            brakelights_thread = false;
            switch(spd)
            {
             case 1:
                A.speed(-0.33);
                B.speed(-0.33); 
             break;
             case 2:
                A.speed(-0.66);
                B.speed(-0.66);
             break; 
             case 3:
                A.speed(-1);
                B.speed(-1);
             break;
            }  
            state = waiting;
            break;
        }
        case got_left:
        {
            ch = xbee.getc(); //Get 1, 2, or 3 value
            spd = ch - '0';
            brakelights_thread = false;
            switch(spd)
            {
             case 1:
                A.speed(-0.33); //left motor
                B.speed(0.33); //right motor
             break;
             case 2:
                A.speed(-0.66);
                B.speed(0.66);
             break; 
             case 3:
                A.speed(-1);
                B.speed(1);
             break;
            }  
            state = waiting;
            break;
        }
        case got_right:
        {
            ch = xbee.getc(); //Get 1, 2, or 3 value
            spd = ch - '0';
            brakelights_thread = false;
            switch(spd)
            {
             case 1:
                A.speed(0.33); //left motor
                B.speed(-0.33); //right motor
             break;
             case 2:
                A.speed(0.66);
                B.speed(-0.66);
             break; 
             case 3:
                A.speed(1);
                B.speed(-1);
             break;
            }  
            state = waiting;
            break;
        }
        case got_turn:
        {
            char dir = xbee.getc();
            if (dir == 'L') leftturn_thread = true; //turn on left signal
            else if (dir == 'R') rightturn_thread = true; //turn on left signal
            state = waiting;
            break;
        }
        case got_horn:
        {
            xbee.getc(); //Clear out '1' char
            horn_thread = true; //turn on horn
            state = waiting;
            break;
        }
        case got_off:
        {
            xbee.getc(); //Clear out second '0' char
            reverselights_thread = false; //Make sure reverselights are off
            brakelights_thread = true; //turn brakelights back on
            A.speed(0); //Turn off motors
            B.speed(0); 
            state = waiting;
            break;
        }
        default:
        {
            xbee.getc();
            state = waiting;
        }
    }
}

void LeftTurn_Thread() {
    while(1) {
        if(leftturn_thread) {
            for(int i = 0; i < 6; i++) { //Blink turn signal for 3 seconds
                LeftTurn = !LeftTurn;
                Thread::wait(0.5*1000);
            }
            LeftTurn = 0;
            leftturn_thread = false;
        }
        Thread::wait(100);
    }
}

void RightTurn_Thread() {
    while(1) {
        if(rightturn_thread) {
            for(int i = 0; i < 6; i++) { //Blink turn signal for 3 seconds
                RightTurn = !RightTurn;
                Thread::wait(0.5*1000);
            }
            RightTurn = 0;
            rightturn_thread = false;
        }
        Thread::wait(100);
    }
}

void Horn_Thread() {
    while(1) {
        if(horn_thread) {
            speaker = 0.5; //Turn horn on for 0.5 seconds
            Thread::wait(0.5*1000);
            speaker = 0.0;
            horn_thread = false;
        }
        Thread::wait(100);
    }   
}

void Headlights_Thread() {
    while(1) {
        if(LowLight < 0.65) HeadLights = 1; //Threshold may change based on environment
        else HeadLights = 0;
        Thread::wait(3000);
    }    
}

//Use thread to actually access brakelight and reverselight objects since multiple threads access them
void MovementLights_Thread() {
    while(1) {
        BrakeLights = (int)brakelights_thread;
        ReverseLights = (int)reverselights_thread;
        Thread::wait(100);
    }
}

   
int main()
{
    //Start XBee and attach serial RX interrupt
    xbee.baud(9600);
    xbee.attach(&parse_message,Serial::RxIrq);
    
    //Start ToF sensor
    volatile int status;
    DevI2C *device_i2c = new DevI2C(VL53L0_I2C_SDA, VL53L0_I2C_SCL);
    board = XNucleo53L0A1::instance(device_i2c, A2, D8, D2); /* creates the 53L0A1 expansion board singleton obj */
    shdn = 0; //must reset sensor for an mbed reset to work
    wait(0.1); //Normal wait since threads haven't started yet
    shdn = 1;
    wait(0.1);
    status = board->init_board(); /* init the 53L0A1 board with default values */
    while (status) {
        status = board->init_board(); //Continue to try to start board until it does
    }
    
    //Init lights to off (except brake lights, since we're stopped)
    LeftTurn = 0;
    RightTurn = 0;
    HeadLights = 0;
    ReverseLights = 0;
    BrakeLights = 1;
    
    //Ensure motors off
    A.speed(0);
    B.speed(0);
    
    //Set speaker frequency
    speaker.period(1.0/300.0); // 300hz period
    
    //Start threads
    Thread left_turn_thread(LeftTurn_Thread);
    Thread right_turn_thread(RightTurn_Thread);
    Thread honk_thread(Horn_Thread);
    Thread headlights_thread(Headlights_Thread);
    Thread movelights_thread(MovementLights_Thread);
    
    //Set up RPG
    RPG_A.rise(&Enc_change_ISR);
    RPG_A.fall(&Enc_change_ISR);
    RPG_B.rise(&Enc_change_ISR);
    RPG_B.fall(&Enc_change_ISR);
    
    //Start main thread
    while(1)
    {
        uint32_t distance;
        //Get ToF reading
        //If it doesn't get a good reading, just set to max (1000mm)
        status = board->sensor_centre->get_distance(&distance);
        if (distance <= 150) distance = 1000;
        
        //Scale RPG and check against distance
        if (enc_count >= 800) enc_count = 800; //Set max RPG value just under ToF max
        if (enc_count <= 200) enc_count = 200; //Set min RPG value just under ToF min
        if (distance <= enc_count) {
            //Mutexes can't be called inside IRQs
            //Therefore, instead of "locking" this thread
            //to only move the motors, we'll just momentarily
            //disable the Serial IRQ, which means glove will no longer
            //be able to control motors
            xbee.attach(NULL, Serial::RxIrq); //Any chars read will just get cleared by the NULL func
            A.speed(0); //Stop motors, we're too close to obstacle!
            B.speed(0);
            brakelights_thread = true;
            Thread::wait(0.5*1000);
            brakelights_thread = false;
            reverselights_thread = true;
            A.speed(-0.5); //Back robot up a bit so we can direct it away from obstacle
            B.speed(-0.5);
            Thread::wait(1*1000);
            A.speed(0);
            B.speed(0);
            brakelights_thread = true;
            reverselights_thread = false;
            //Re-enable the Serial IRQ
            xbee.attach(&parse_message,Serial::RxIrq);
        }
        Thread::yield(); //Let other threads run if needed
    }
}