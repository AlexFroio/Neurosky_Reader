#include "mbed.h"
#include "max32630fthr.h"
#include "USBKeyboard.h"
#include "USBSerial.h"
#include "Brain.h"

//const uint32_t BLINK_DELAY = 250;

void blink_led(DigitalOut &led, const uint32_t delay);
// Hardware serial port over DAPLink
Serial daplink(USBTX, USBRX);          // Use USB debug probe for serial link
Serial uart(P2_1, P2_0);
RawSerial EEG(P3_1, P3_0);
DigitalOut rLed(LED_RED, 1);
DigitalOut gLed(LED_GREEN, 1);
DigitalOut bLed(LED_BLUE, 1);

Brain brain(EEG);

// Virtual serial port over USB
USBSerial microUSB;
USBKeyboard keyB;
bool ready = false;
uint32_t msTicks = 0;

Ticker ms_tick;
void onMillisecondTicker(void)
{
    msTicks++;
}

void onCharReceived()
{
    ready = brain.update();
}

void init()
{
    MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);
    //DigitalOut led_array[] = {rLed, gLed};
    uart.baud(115200);
    daplink.baud(57600);
    EEG.baud(57600);
    EEG.attach(&onCharReceived);
    ms_tick.attach_us(onMillisecondTicker,1000);
}

int main()
{
    init();
    while(1)
    {
        
        if (!ready)
        {
            rLed = 1;
            microUSB.printf("still\r\n");
            rLed = 0;
        }
        else
        {
            gLed = 1;
            if ((brain.readNotifications() >> brain.PKT_EEG) & 1U)
            {
                microUSB.printf(brain.readEEGPower());
            }
            gLed = 0;
        }

    }
}

void blink_led(DigitalOut &led, const uint32_t delay)
{
    led = !led;
    wait_ms(delay);
    led = !led;
}
