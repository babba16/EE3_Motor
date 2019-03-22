#include "mbed.h"
#include "SHA256.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdlib>
#include "ctype.h"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5
#define KS_P 0.08
#define KS_I 0.001

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/


PwmOut motor(D9);


/* Mail */
typedef struct {
    int ID;
} mail_t;
Mail<mail_t, 16> mail_box;

Thread printThread;
Thread decodeThread;
Thread motorCtrlT (osPriorityNormal,1024);
Thread melodyThread(osPriorityLow);

Mutex newKey_mutex;
Mutex motor_mutex;
Mutex maxVelocity_mutex;
Mutex rotation_mutex;
Mutex melody_mutex;

Timer timer;
Timer velocityTime;

uint32_t nonceVal;
volatile float duty;
float hashRate;
float hashCount;
volatile uint64_t newKey = 0;
int8_t intState = 0;
int8_t intStateOld = 0;
int errorInState =0;
int counterRev;
float velocity;
int8_t position;
float maxVelocity = 0.0;
float maxRotations = 0.0;
volatile float torque;
//-----Melody Variables -----//
float period_out = 2000.0;
int duration[8];
string melody_string = "";
string notes[8] = {"  ","  ","  ","  ","  ","  ","  ","  "};
//---------------------- -----//

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);
Queue<void,8> inCharQ;
void serialISR()
{
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState)
{

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}


//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState()
{
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome()
{
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}
int8_t orState = motorHome();

void playMelody(float freq, int duration);


void sendToMail (int ID)
{
    mail_t *mail = mail_box.alloc();
    mail->ID = ID;
    mail_box.put(mail);
    wait(1);
}

void getHashRate(float hashCount, float timePassed)
{
    hashRate = hashCount/timePassed;
    sendToMail(3);
}




float note_frequency(char note, char type)
{
    float freq;
    switch(note) {
        case 'A':
            switch(type) {
                case '^':
                    freq = 415.30;
                case ' ':
                    freq = 440.00;
                case '#':
                    freq = 466.16;
            }
        case 'B':
            switch(type) {
                case '^':
                    freq = 466.16;
                case ' ':
                    freq = 493.88;
            }
        case 'C':
            switch (type) {
                case ' ':
                    freq = 523.25;
                case '#':
                    freq = 554.37;
            }
        case 'D':
            switch (type) {
                case '^':
                    freq = 554.37;
                case ' ':
                    freq = 587.33;
                case '#':
                    freq = 622.25;
            }
        case 'E':
            switch (type) {
                case '^':
                    freq = 622.25;
                case ' ':
                    freq = 659.25;
            }
        case 'F':
            switch (type) {
                case ' ':
                    freq = 698.46;
                case '#':
                    freq = 739.99;
            }
        case 'G':
            switch (type) {
                case '^':
                    freq = 739.99;
                case ' ':
                    freq = 783.99;
                case '#':
                    freq = 830.61;
            }
        default:
            freq = 500;
    }
    return freq;
}


void parse_melody(void)
{
    int period_index;
    int notes_index;
    while(1) {
        if(melody_string != "") {
            period_index = 0;  // use to index the duration of note vector
            notes_index = 0; // used to index notes vector
            for(int i=0; i<int(melody_string.length()); i++) {
                if (isdigit(melody_string[i])) {
                    duration[period_index] = int(melody_string[i])-48; //minus 48 fo ASCII
                    period_index++;
                } else if((int(melody_string[i])>=65)&&(int(melody_string[i])<=71)) { //Find a in the ASCII range A-G
                    notes[0][notes_index] = melody_string[i];
                    notes[1][notes_index] = ' '; // set to normal note First
                    notes_index++;
                } else if((melody_string[i]=='^')||(melody_string[i]=='#')) { // an flat or a sharp
                    if(notes[0][notes_index-1]!=' ') {
                        notes[1][notes_index-1] = melody_string[i];
                    }
                } else {
                    notes_index++;
                }

                while(period_index<8) {
                    duration[period_index] = 0;
                    notes[0][notes_index] = ' ';
                    notes[1][notes_index] = ' ';
                    period_index++;
                    notes_index++;
                }

                for(int i = 0; i<8; i++) {
                    playMelody(note_frequency(notes[0][i],notes[1][i]),duration[i]);
                    if(i == 8) {
                        melody_string = "";
                    }
                }
            }
        }
    }
}

void executeCommand(char* command)
{
    switch(command[0]) {
        case 'K':
            newKey_mutex.lock();
            sscanf(command,"K%llx",&newKey);
            newKey_mutex.unlock();
            sendToMail(1);
            break;

        case 'M':
            motor_mutex.lock();
            sscanf(command,"M%f",&duty);
            motor_mutex.unlock();
            motor = duty;
            sendToMail(2);
            break;

        case 'H':
            float timePassed = timer.read();
            getHashRate(hashCount, timePassed);
            break;

        case 'S':
            sendToMail(4);
            break;

        case 'V':
            maxVelocity_mutex.lock();
            sscanf(command,"V%f",&maxVelocity);
            maxVelocity_mutex.unlock();
            sendToMail(5);
            break;

        case 'R':
            rotation_mutex.lock();
            sscanf(command,"R%f",&maxRotations);
            rotation_mutex.unlock();
            sendToMail(6);
            break;

        case 'T':
            melody_mutex.lock();
            sscanf(command,"T%s",melody_string);
            melody_mutex.unlock();
            sendToMail(7);
            break;
    }
}


void print_thread (void)
{
    while(true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            switch(mail->ID) {
                case 0:
                    pc.printf("nonce: %lu\n\r", nonceVal);
                    break;

                case 1:
                    pc.printf("new key: %llu\n\r", newKey);
                    break;

                case 2:
                    pc.printf("duty cycle: %f\n\r", duty);
                    break;

                case 3:
                    pc.printf("hash rate: %f\n\r", hashRate);
                    break;

                case 4:
                    pc.printf("velocity: %f\n\r", velocity);
                    break;

                case 5:
                    pc.printf("max velocity: %f\n\r", maxVelocity);
                    break;

                case 6:
                    pc.printf("Rotations to be done: %f\n\r", maxRotations);
                    break;
                case 7:
                    pc.printf("Melody to be player: %s\n\r", melody_string);
                    break;
            }
            mail_box.free(mail);
        }
       }
}

void decode_thread(void)
{
 
    char* cmdIn = new char[18];
    int  n = 0;
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if (newChar != '\r' && newChar != '\n') {
            cmdIn[n] = newChar;
            n++;
        } else {
            n = 0;
            executeCommand(cmdIn);
        }
 
       
    }
}

void motorCtrlTick()
{
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(void)
{
    counterRev = 0;
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    int counter = 0;
    float oldRevs = 0.0;
    float revChange;
    float revs;
    //float torque;
    float er =0.0;
    float erold =0.0;
    float erderiv =0.0;
    float es;
    float esTotal = 0.0;
    float oldRotation= 0.0;
    float mv;
    float mr;
    float sgn_v;
    float torque_V;
    float torque_R;

    while(1) {
        velocityTime.start();
        motorCtrlT.signal_wait(0x1);
        revs = counterRev/6.0;
        revChange = revs - oldRevs;
        velocity = revChange * 10.0;
        oldRevs = revs;
        
        if(counter == 10) {
            //core_util_critical_section_enter();
            
                                 // we have velocity every second
            sendToMail(4);
            counter = 0;
            //velocityTime.reset();
            //core_util_critical_section_exit();
            //oldRevs = revs;
        }
        //vel control

        if (maxRotations != oldRotation) {
            //core_util_critical_section_enter();
            counterRev =0.0;
            //core_util_critical_section_exit();
            oldRotation = maxRotations;
        }

        mv = maxVelocity;


        es = mv-abs(velocity);
        //core_util_critical_section_enter();
        //velocity error
        er = maxRotations - counterRev/6.0;            //position error // 5 because one not working
        //core_util_critical_section_exit();
        (velocity < 0) ? sgn_v = -1 : sgn_v = 1;
        erderiv = er - erold;                         //deriv or position error
        erold = er;                                    // set er previous version


        if(mv==0.0) {

            if(maxRotations==0.0) {    //default case no inputs (to be set to duty cycle later)
                torque=1.0;
            } else {
                //rotation alg
                torque=30*er +20*erderiv;
            }
        } else { //maxVelocity is set
            if(maxRotations==0.0) {
                // vel alg
                torque = (0.08*es+0.62)* sgn_v  ;
            } else {
                //combined case
                torque_V = (0.08*es+0.62)* sgn_v;
                torque_R = 30*er +20*erderiv;
                if(sgn_v > 0) {
                    if(torque_V > torque_R) {
                        torque = torque_R;
                    } else {
                        torque = torque_V;
                    }
                } else {
                    if(torque_V > torque_R) {
                        torque = torque_V;
                    } else {
                        torque = torque_R;
                    }
                }
            }
        }

        if(torque < 0) {
            torque = -torque; // negative values need to be made positive
            lead = -2; // backwards rotation
        } else {
            lead = 2; // forwards rotation
        }

        // this needs to have the constant added
        //printf("torque: %f\n\r", torque);


        if(torque >1.0) {  //cap in all cases
            torque = 1.0;
        }


        motor = torque;





        counter++;
        //printf ("rrs rev: %i\n\r", intState);
    }
   

}



//void spinRotor(){
//   int8_t intState = readRotorState();
//    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
//            //pc.printf("%d\n\r",intState);
//}
void newPhISR()
{
    intStateOld = intState;
    intState = readRotorState();
    position = readRotorState();   // should give position? out of 6 choices
    if(intState < intStateOld) {
        counterRev--;
    } else if (intState> intStateOld) {
        counterRev++;
    }

    if (intState != intStateOld) {
        motorOut((intState-orState+lead+6)%6);
    }
}

void playMelody(float freq, int duration)
{
    for(int i = 0; i < duration - 1; i++) {
        period_out = (1000000/freq);
        //motor=0.5;
        motor.period_us(period_out);
        newPhISR();
        wait(1);
    }
}



//Main
int main()
{
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    I1.rise(&newPhISR);
    I1.fall(&newPhISR);
    I2.rise(&newPhISR);
    I2.fall(&newPhISR);
    I3.rise(&newPhISR);
    I3.fall(&newPhISR);

    motor.period(0.002f);
    motor.write(1.0f);

    SHA256 sha;
    timer.start();
   
    decodeThread.start(callback(decode_thread));
    melodyThread.start(callback(parse_melody));

    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                          0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                          0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                          0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                          0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                          0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
                         };
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];

    printThread.start(callback(print_thread));
    motorCtrlT.start(callback(motorCtrlFn));

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        sha.computeHash(hash,sequence,64);

        if ((hash[0] == 0) && (hash[1] == 0)) {
            nonceVal = (uint32_t) *nonce;
            sendToMail(0);
        }
        (*nonce)++;
        hashCount++;
    }
}
