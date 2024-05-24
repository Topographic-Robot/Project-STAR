// If wanting to use msp.h
#include <ti/devices/msp432p4xx/inc/msp.h>

// If wanting to use driverlib.h
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

int main(void) {
  /* Stop Watchdog Timer */
  WDT_A_holdTimer();

  while (1) {
    /* Go into Low Power Mode */
    PCM_gotoLPM0();
  }

  return 0;
}
