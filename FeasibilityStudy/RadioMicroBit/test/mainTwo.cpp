#include "MicroBit.h"
//#include "ManagedString.h"
//#include <stdlib.h>
//#include <string>
//
//
//#include <stdbool.h>
//#include "nrf.h"
//#include "nrf_gpiote.h"
//#include "app_error.h"
//#include "TimedInterruptIn.h"


MicroBit uBit;
uint8_t data = 1;

FrameBuffer *rxBuf = NULL;

int iterate = 0;



void sendData(void) {
    if (iterate < 300) {
//        send(&data, 1);
        uBit.radio.datagram.send(&data,1);
        uBit.serial.printf("Sent data %d\n", iterate);
        iterate++;
    }
}

void onRecieve(MicroBitEvent e) {
    sendData();
}

void (*callBack)(void);

int main() {
    
    callBack = &sendData;
    
    

    uBit.init();
    uBit.radio.setFrequencyBand(0);
    uBit.radio.enable();
    

    uBit.serial.printf("Microbit sender running.\n");
    //    MyMicroBitPin p0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_ALL, &uBit);

    //    uBit.io.P0 = p0;
        uBit.io.P0.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);

    //    uBit.messageBus.listen(MICROBIT_ID_IO_P0,MICROBIT_PIN_EVT_RISE,onRecieve,MESSAGE_BUS_LISTENER_IMMEDIATE);

    
    // Turn off the transceiver.
//    NRF_RADIO->EVENTS_DISABLED = 0;
//    NRF_RADIO->TASKS_DISABLE = 1;
//    while(NRF_RADIO->EVENTS_DISABLED == 0);
    
    // Turn on the transmitter, and wait for it to signal that it's ready to use.
//    NRF_RADIO->EVENTS_READY = 0;
//    NRF_RADIO->TASKS_TXEN = 1;
//    while (NRF_RADIO->EVENTS_READY == 0);
    
    
//    while(NRF_RADIO->EVENTS_DISABLED == 0);
        
    
    while(1);
    release_fiber();

}

#include <stddef.h>
#include "cmsis.h"

#include "gpio_irq_api.h"
#include "mbed_error.h"


#define CHANNEL_NUM    31

static uint32_t channel_ids[CHANNEL_NUM] = {0}; //each pin will be given an id, if id is 0 the pin can be ignored.
static uint8_t channel_enabled[CHANNEL_NUM] = {0};
static uint32_t portRISE = 0;
static uint32_t portFALL = 0;
static gpio_irq_handler irq_handler;

#ifdef __cplusplus
extern "C" {
#endif
    
   
    
//     void (*callBack)(void) = NULL;
    
void GPIOTE_IRQHandler(void)
{
    
    
        callBack();
    
    volatile uint32_t newVal = NRF_GPIO->IN;

    if ((NRF_GPIOTE->EVENTS_PORT != 0) && ((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_PORT_Msk) != 0)) {
        NRF_GPIOTE->EVENTS_PORT = 0;

        for (uint8_t i = 3; i<=3; i++) {
            if (channel_ids[i]>0) {
                if (channel_enabled[i]) {
                    if( ((newVal>>i)&1)  && ( ( (NRF_GPIO->PIN_CNF[i] >>GPIO_PIN_CNF_SENSE_Pos) & GPIO_PIN_CNF_SENSE_Low) != GPIO_PIN_CNF_SENSE_Low) && ( (portRISE>>i)&1) ){
                        irq_handler(channel_ids[i], IRQ_RISE);
                    } else if ((((newVal >> i) & 1) == 0) &&
                               (((NRF_GPIO->PIN_CNF[i] >> GPIO_PIN_CNF_SENSE_Pos) & GPIO_PIN_CNF_SENSE_Low) == GPIO_PIN_CNF_SENSE_Low) &&
                               ((portFALL >> i) & 1)) {
                        irq_handler(channel_ids[i], IRQ_FALL);
                    }
                }

                if (NRF_GPIO->PIN_CNF[i] & GPIO_PIN_CNF_SENSE_Msk) {
                    NRF_GPIO->PIN_CNF[i] &= ~(GPIO_PIN_CNF_SENSE_Msk);

                    if (newVal >> i & 1) {
                        NRF_GPIO->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
                    } else {
                        NRF_GPIO->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
                    }
                }
            }
        }
    }
}

#ifdef __cplusplus
}
#endif

int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id)
{
    if (pin == NC) {
        return -1;
    }

    irq_handler             = handler;
    obj->ch                 = pin;
    NRF_GPIOTE->EVENTS_PORT = 0;
    channel_ids[pin]        = id;
    channel_enabled[pin]    = 1;
    NRF_GPIOTE->INTENSET    = GPIOTE_INTENSET_PORT_Set << GPIOTE_INTENSET_PORT_Pos;

    NVIC_SetPriority(GPIOTE_IRQn, 3);
    NVIC_EnableIRQ  (GPIOTE_IRQn);
    return 0;
}

void gpio_irq_free(gpio_irq_t *obj)
{
    channel_ids[obj->ch] = 0;
}

void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable)
{
    NRF_GPIO->PIN_CNF[obj->ch] &= ~(GPIO_PIN_CNF_SENSE_Msk);
    if (enable) {
        if (event == IRQ_RISE) {
            portRISE |= (1 << obj->ch);
        } else if (event == IRQ_FALL) {
            portFALL |= (1 << obj->ch);
        }
    } else {
        if (event == IRQ_RISE) {
            portRISE &= ~(1 << obj->ch);
        } else if (event == IRQ_FALL) {
            portFALL &= ~(1 << obj->ch);
        }
    }

    if (((portRISE >> obj->ch) & 1) || ((portFALL >> obj->ch) & 1)) {
        if ((NRF_GPIO->IN >> obj->ch) & 1) {
            NRF_GPIO->PIN_CNF[obj->ch] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);    // | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
        } else {
            NRF_GPIO->PIN_CNF[obj->ch] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);     //| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
        }
    }
}

void gpio_irq_enable(gpio_irq_t *obj)
{
    channel_enabled[obj->ch] = 1;
}

void gpio_irq_disable(gpio_irq_t *obj)
{
    channel_enabled[obj->ch] = 0;
}















































































void program2() {
    //sender
    uBit.init();

    uBit.radio.enable();

    uBit.serial.printf("Microbit sender running.\n");
    
    for (int i = 0; i < 100; i++) {
        uBit.radio.datagram.send(&data, 1);
        uBit.serial.printf("Microbit sent packet %d.\n" + i);
        uBit.display.print(i);
        uBit.sleep(1000);
    }

    release_fiber();
}

void program1(){
    string aString;
    char countString[4];
    int count = 0;
    bool up = true;

        while (1) {
        if (up) {
            count++;
            if (count >= 255) {
                up = false;
            }
        } else {
            count--;
            if (count < 1) {
                up = true;
            }
        }
        itoa(count, countString);
        aString = string(countString);
        aString.append(aString + "," + aString + "," + aString + "," + aString + "," + aString + "\n" + aString + "," + aString + "," + aString + "," + aString + "," + aString + "\n" + aString + "," + aString + "," + aString + "," + aString + "," + aString + "\n" + aString + "," + aString + "," + aString + "," + aString + "," + aString + "\n" + aString + "," + aString + "," + aString + "," + aString + "," + aString + "\n" );
        uBit.display.image.paste(MicroBitImage(aString.c_str()));
        uBit.sleep(500);
        uBit.serial.printf("Something\n");
        uBit.display.setBrightness(count);
//    }
};
}