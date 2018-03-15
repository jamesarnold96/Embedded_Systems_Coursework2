#include "mbed.h"
#include "Crypto_light.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

Thread commOutT;
Thread commInT;
Thread motorCtrlT(osPriorityNormal,1024);
RawSerial pc(SERIAL_TX, SERIAL_RX);
// Serial pc(SERIAL_TX, SERIAL_RX);
Queue<void,8> inCharQ;
volatile uint64_t newKey;
volatile float newRev;
volatile float maxSpeed;
uint32_t newTorque;

Mutex newKey_mutex;

// --------------------------------------------------------------------------------------------------------------------
// Serial communication functions 
typedef struct{
    uint8_t code;
    uint32_t data;
    } message_t;
    
Mail<message_t,16> outMessages;
    
// Handles serial input interrupts
void serialISR(){
 uint8_t newChar = pc.getc();
 inCharQ.put((void*)newChar);
 }  
 
// uses Mail to queue messages for serial output
void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage -> code = code;
    pMessage -> data = data;
    outMessages.put(pMessage);
}

void commInFn(){
    // array to hold each command 
    uint8_t N = 50;
    char newCmd[N];
    uint8_t idx = 0;
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        newCmd[idx] = newChar;
        if(idx == N-1) pc.printf("Incoming string is too long!\n\r");
        idx++;
        if(newChar == '\r'){
             newCmd[idx] = '\0';
             idx = 0;
             if (newCmd[0] == 'K'){
                newKey_mutex.lock();
                sscanf(newCmd, "K%x", &newKey); //Decode the command
                newKey_mutex.unlock();
                //putMessage(3,124); 
            }
            else if(newCmd[0] == 'R'){
                    sscanf(newCmd, "R%f", &newRev);            
            }
            else if(newCmd[0] == 'V'){
                    sscanf(newCmd, "V%f", &maxSpeed);
                    //newTorque = /newSpeed
                }
            //set motor torque
            else if(newCmd[0] == 'T'){
                    sscanf(newCmd, "T%d", &newTorque);
                }
        }
    }
}

void commOutFn(){   
    while (1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
		switch(pMessage->code){
			case 1:
				pc.printf("Hash rate 0x%016x\n\r",
				pMessage->data);
				break;
        	case 2:
				pc.printf("Hash computed 0x%016x\n\r",
				pMessage->data);
				break;
			case 3:
				//pc.printf("Motor position %d\n\r",
				//pMessage->data);
				break;
			case 4:
				//pc.printf("Motor velocity %d\n\r",
				//pMessage->data);
				break;
			default:
				//pc.printf("Message %d with data 0x%016x\n\r",
				//pMessage-> code, pMessage->data);
        }
        outMessages.free(pMessage);
    }
}
    
// --------------------------------------------------------------------------------------------------------------------------------
// Motor control functions
int32_t motorPosition;
int64_t motorVelocity;
Mutex motorVelocity_mutex;
int32_t counter=0;

void motorCtrlTick(){
 motorCtrlT.signal_set(0x1);
 }

void motorCtrlFn(){
    static int32_t oldmotorPosition;
	// Timer to count time passed between ticks to enable accurate velocity calculation
	//Timer t;
	//t.start();
	// local copy of motorPosition to avoid concurrent access
	int32_t motorPos = motorPosition;
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    while(1){
       /* motorCtrlT.signal_wait(0x1);
		motorVelocity_mutex.lock();
        motorVelocity = abs(oldmotorPosition - motorPos)*10;//t.read_us(); 
        motorVelocity_mutex.unlock();
		oldmotorPosition = motorPos;
		// t.reset();
        counter++;
        if(counter == 10){
            counter = 0;
			putMessage(3,motorPos);
			motorVelocity_mutex.lock();
			putMessage(4,motorVelocity);
			motorVelocity_mutex.unlock();
        }*/
    }
	motorPosition = motorPos;
}
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
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState, uint32_t motorTorque){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(motorTorque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(motorTorque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(motorTorque);
    if (driveOut & 0x20) L3H = 0;
}
    
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
	L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);
    motorOut(0, 1000);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

//orState is subtracted from future rotor state inputs to align rotor and motor states   
int8_t orState = motorHome();
//int8_t intState = 0;
//int32_t motorPosition;
void motorISR() {
    static int8_t oldRotorState;
    //intState = readRotorState();
    int8_t rotorState = readRotorState();
    motorOut((rotorState-orState+lead+6)%6,newTorque); //+6 to make sure the remainder is positive
    if (rotorState - oldRotorState == 5 ) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}

// ------------------------------------------------------------------------------------
// Bitcoin mining code 
int16_t hashCount = 0;

void calcHashRate() {
    putMessage (1, hashCount);
    hashCount = 0;
}


/*char command;
char buffer[32];
void rxCallback() {
    command = pc.getc();
    if (command == 'R'){
        printf("%c\n\r", command);
    }
    if (command == 'V'){
        printf("%c\n\r", command);
    }
    if (command == 'K'){
        printf("%c\n\r", command);
    }
    if (command == 'T'){
        printf("%c\n\r", command);
    }
}*/

//void serialISR(){
//        uint8_t newChar = pc.getc();
//        inCharQ.put((void*)newChar);
//    }

// -------------------------------------------------------------------------------------------------------------------------
//Main
int main() {
	//set up pwm period
    commOutT.start(commOutFn);
    commInT.start(commInFn);
    
    //pc.attach(&rxCallback);
    
    // Run the motor synchronisation
    pc.printf("Rotor origin: %x\n\r", orState);
    
    // motor controlling interrupt routines
    I1.rise(&motorISR);
    I1.fall(&motorISR);
    I2.rise(&motorISR);
    I2.fall(&motorISR);
    I3.rise(&motorISR);
    I3.fall(&motorISR);
	//motorCtrlT.start(motorCtrlFn);
    // mining bitcoins
    SHA256 mine;
    uint8_t sequence[] = {
        0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    
    
   //  uint64_t* key = (uint64_t*)((int)sequence + 48);
   newKey_mutex.lock();
   uint64_t key = newKey;
   newKey_mutex.unlock();
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    
    // timer for hash rate
    Ticker t;
    t.attach(&calcHashRate, 1.0);
    
    //uint32_t dummyhash = 0;
    while (1) {
        mine.computeHash(hash, sequence, 64);
        hashCount = hashCount + 1;
        if (hash[0] == 0 && hash[1] == 0){
            putMessage(2, *nonce);
        }
        *nonce = *nonce + 1;
        /*dummyhash = dummyhash + 1;
        if (dummyhash % 2000000 == 0){
            putMessage (2,123);
            dummyhash = 0;
        }*/
    }
}