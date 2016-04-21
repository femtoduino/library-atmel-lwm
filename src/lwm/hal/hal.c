#include "hal.h"

void HAL_Init(void)
{
    pinPeripheral(PIN_RF1, PIO_TIMER_ALT);
    pinPeripheral(PIN_RF1, PIO_TIMER_ALT);
    pinMode(PIN_RF1, OUTPUT);
    pinMode(PIN_RF2, OUTPUT);


    // Enable RFCTRL (AT86RF233 is internally connected to SERCOM4 SPI)
    PM->APBCMASK.reg |=     PM_APBCMASK_RFCTRL;
    
    // Change amount of shift.
    RFCTRL_FECTRL = (0 << 4/*DIG1*/) | (1 << 2/*DIG2*/);

    /* setup GPIOs */

    pinMode(PIN_SPI_SLEEP_TR,  OUTPUT);
    pinMode(PIN_SPI_RESET,  OUTPUT);
    pinMode(PIN_SPI_IRQ,    INPUT);
    pinMode(PIN_SPI_SS,     OUTPUT);


    /* initialise SPI - pinPerhipheral call required for Arduino. Dunno why variant.cpp settings are discarded. Manually doing this gets it working*/
    pinPeripheral(PIN_SPI_SCK, PIO_TIMER_ALT);
    pinPeripheral(PIN_SPI_MISO, PIO_TIMER_ALT);
    pinPeripheral(PIN_SPI_MOSI, PIO_TIMER_ALT);

    // SPI.usingInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ));
    
    // SPI.beginTransaction(
    //     SPISettings(
    //         MODULE_AT86RF233_CLOCK, 
    //         MSBFIRST, 
    //         SPI_MODE0
    //     )
    // );

    // attachInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ), HAL_IrqHandlerSPI, RISING);
    // /*  wait for SPI to be ready  */
    // delay(10);

    /*  initialize GPIOs */
    digitalWrite(PIN_SPI_SLEEP_TR, LOW); // Wake up
    digitalWrite(PIN_SPI_RESET, HIGH);
    digitalWrite(PIN_SPI_SS, HIGH);
}