#include "MicroBit.h"
#define LENGTH 300

MicroBit uBit;
uint8_t data = 0;
uint8_t* rec[LENGTH];
int i = 0;

void onRecieve(MicroBitEvent e) {
    uBit.serial.printf("MicroBitRec got packet: %d\n", i);
    rec[i] = uBit.radio.datagram.recv().getBytes();

    i++;
}

int main() {

    uBit.init();
    uBit.radio.enable();
    uBit.messageBus.listen(MICROBIT_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, onRecieve);
    
    uBit.serial.printf("Microbit receiver running.\n");
    
    release_fiber();
}