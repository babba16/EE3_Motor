#include "mbed.h"
#include "SHA256.h"
#include <iostream>   
#include <string>
#include <stdio.h>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

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

Mutex newKey_mutex;
Mutex motor_mutex;

Timer timer;
Timer velocityTime;

uint32_t nonceVal;
float duty;
float hashRate;
float hashCount;
uint64_t newKey = 0;
int intState = 0;
int intStateOld = 0;
int errorInState =0;
int counterRev;
float velocity;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);
Queue<void,8> inCharQ;
void serialISR(){
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
void motorOut(int8_t driveState){
    
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
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
int8_t orState = motorHome();

void sendToMail (int ID) {
    mail_t *mail = mail_box.alloc();
    mail->ID = ID;
    mail_box.put(mail);
    wait(1);
}

void getHashRate(float hashCount, float timePassed){
    hashRate = hashCount/timePassed;
    sendToMail(3);    
}    

void executeCommand(char* command){
    switch(command[0]){
        case 'K': newKey_mutex.lock();
                  sscanf(command,"K%llx",&newKey);
                  newKey_mutex.unlock(); 
                  sendToMail(1);
                  break;
                  
        case 'M': motor_mutex.lock();
                  sscanf(command,"M%f",&duty); 
                  motor_mutex.unlock(); 
                  motor = duty;  
                  sendToMail(2); 
                  break;
                
        case 'H': float timePassed = timer.read(); 
                  getHashRate(hashCount, timePassed);
                  break;
        
        case 'V': sendToMail(4);
    }        
}    
 
void print_thread (void) {
    while(true){
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            switch(mail->ID){
                case 0: pc.printf("nonce: %lu\n\r", nonceVal);
                          break;
                          
                case 1: pc.printf("new key: %llu\n\r", newKey);
                          break;
                          
                case 2: pc.printf("duty cycle: %f\n\r", duty);
                          break;
                          
                case 3: pc.printf("hash rate: %f\n\r", hashRate);
                          break;
                
                case 4: pc.printf("velocity: %f\n\r", velocity);
            }    
            mail_box.free(mail);    
        }
    }    
} 

void decode_thread(void){
    char* cmdIn = new char[18];
    int  n = 0;
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if (newChar != '\r' && newChar != '\n'){
            cmdIn[n] = newChar;
            n++;
        }
        else {
            n = 0;
            executeCommand(cmdIn);
        }
    }
} 

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn(void){
    counterRev = 0;
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    int counter = 0;
    float oldRevs = 0.0;
    float revChange;
    float revs;
    while(1){
        velocityTime.start();
        motorCtrlT.signal_wait(0x1);
        if(counter == 10){
            core_util_critical_section_enter();  
            revs = counterRev/6.0;
            revChange = revs - oldRevs;
            float timePassed = velocityTime.read();
            velocity = revChange/timePassed;
            //pc.printf("time passed: %f\n\r", timePassed);
            counter = 0;
            velocityTime.reset();
            core_util_critical_section_exit();
            oldRevs = revs;
        }
        counter++;  
    }     
}     

        

//void spinRotor(){
 //   int8_t intState = readRotorState();
//    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
//            //pc.printf("%d\n\r",intState);
//}
void newPhISR() {
    intStateOld = intState;
    intState = readRotorState();
    if (intState != intStateOld) {
        //errorInState = intState - intStateOld;
        }
    if(intState < intStateOld){ 
        lead = -2;
        counterRev--;
    }
    else if (intState > intStateOld){
        //printf("intstate: %i\n\r", intState);
        lead = 2;
        counterRev++;
    }
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
         //pc.printf("%d\n\r",intState);
    
}
//Main
int main() {   
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
    
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
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
            
            if ((hash[0] == 0) && (hash[1] == 0)){
                    nonceVal = (uint32_t) *nonce;
                    sendToMail(0);
                 }
                 (*nonce)++;
                 hashCount++;
        }
}
