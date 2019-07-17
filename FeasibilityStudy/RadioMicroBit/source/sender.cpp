#include "MicroBit.h"

MicroBit uBit;
uint8_t data = 1;

int iterate = 0;

void sendData(void) {
    if (iterate < 300) {
        uBit.radio.datagram.send(&data, 1);
        uBit.serial.printf("Sent data %d\n", iterate);
        iterate++;
    }
}

extern void (*callBack)(void);

int main() {

    callBack = &sendData;

    uBit.init();
    uBit.radio.enable();

    uBit.serial.printf("Microbit sender running.\n");

    uBit.io.P0.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
    
    while(1);
    release_fiber();
}