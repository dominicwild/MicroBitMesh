#include "MicroBit.h"

MicroBit uBit;

int main() {
    
    uBit.init();
    uBit.serial.printf("Manager running");
    uBit.display.print(1);
    while(!uBit.buttonA.isPressed()){
        uBit.io.P0.setDigitalValue(0);
        uBit.sleep(300);
        uBit.io.P0.setDigitalValue(1);
        uBit.sleep(300);
    }

    release_fiber();
}