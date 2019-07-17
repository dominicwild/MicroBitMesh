#include "MicroBit.h"
#include "ManagedString.h"
#include <stdlib.h>
#include <string>

MicroBit uBit;
uint8_t g[] = {70,71,72,73,74,75,76,77,78,79,80};
void onRecieve(MicroBitEvent e) {
    FrameBuffer* result = uBit.radio.datagram.recvBuf();
    uBit.serial.printf("----------------Result: %d %d %d %d %d %d\r\n",result->protocol,result->relayCounter,result->length,result->group,sizeof(result),result->senderName[5]);
    
    char* c = (char*)result;
    for(int i=0;i<32;i++){
        uBit.serial.printf("%d ",c[i]);
    }
    uBit.serial.printf("\r\n");
    
    delete result;
}

void triggerSend(MicroBitEvent e){
    uBit.radio.n++;
    uBit.radio.datagram.meshSend(g,3,"Dom222");
}

void printline(MicroBitEvent e){
    uBit.serial.printf("----------------\r\n");
    uBit.radio.n = 0;
   
}

uint64_t before = 0;
uint64_t after = 0;

void tic(void) {
    before = system_timer_current_time_us();
}

void toc(void) {
    uBit.serial.printf("%d\r\n", system_timer_current_time_us() - before);
}

void output_(char* s) {
    uBit.serial.printf(s);
}

void printInt_(uint8_t s){
    uBit.serial.printf("%d",s);
}

void printInt64_(uint64_t s){
    uBit.serial.printf("%d",s);
}

void printChar_(char c){
    uBit.serial.putc(c);
}

void printBuf_(FrameBuffer* result){
    uBit.serial.printf("Result: %d %d %d %d ",result->length,result->version,result->group,result->protocol);
    for(int i=0;i<6;i++){
        uBit.serial.printf("%d ",result->senderName[i]);
    }
    for(int i=0;i<6;i++){
        uBit.serial.printf("%d ",result->recieverName[i]);
    }
    uBit.serial.printf(" %d %d",result->sequenceNumber,result->relayCounter);
    for(int i=0;i<32;i++){
        uBit.serial.printf(" %d",result->payload[i]);
    }
    uBit.serial.printf("\r\n");
}

extern void (*tic_)(void);
extern void (*toc_)(void);
extern void (*output)(char* s);
extern void (*printInt)(uint8_t s);
extern void (*printInt64)(uint64_t s);
extern void (*printChar)(char c);
extern void (*printBuf) (FrameBuffer*);
void (*s)(void);
int k = 0;
int main() {

    tic_ = &tic;
    toc_ = &toc;
    output = &output_;
    printInt = &printInt_;
    printInt64 = &printInt64_;
    printChar = &printChar_;
    printBuf = &printBuf_;
    uBit.init();
    
    uBit.messageBus.listen(MICROBIT_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, onRecieve);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, triggerSend);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, printline);
    
    uBit.display.print("R");
    
    uBit.radio.setName("Dom3");
    uBit.radio.enable();

//    uBit.serial.printf("RoundNumber,N,R,Collisions,Wake Time\r\n");
//    uBit.radio.setInitiator(true);
//    uBit.radio.sendSyncPacket();
        
    
    uBit.serial.printf("Name: %d %d %d %d %d %d\r\n",uBit.radio.radioName[0],uBit.radio.radioName[1],uBit.radio.radioName[2],uBit.radio.radioName[3],uBit.radio.radioName[4],uBit.radio.radioName[5]);
    
    uBit.serial.printf("PacketNumber,N,R,Send Attempts,Time Taken\r\n");
    
    int i = 0;
    while(1){
        uBit.serial.printf("%d,%d,%d,",i+1,uBit.radio.N,DIAMETER);
        tic();
        uBit.radio.datagram.meshSend(g, 9, "Dom1");
        uBit.serial.printf(",");
        toc();
        uBit.sleep(10);
        i++;
    }
    
    release_fiber();
}


