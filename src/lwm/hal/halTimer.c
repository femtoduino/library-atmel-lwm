#include <stdint.h>
#include "halTimer.h"

/*- Variables --------------------------------------------------------------*/
volatile uint8_t halTimerIrqCount;

void HAL_IrqHandlerSPI(void)
{
    // halTimerIrqCount++;
}
