// ok no namespace fl
#ifndef __LED_SYSDEFS_ARM_MAX32
#define __LED_SYSDEFS_ARM_MAX32

// #define LED_TIMER NRF_TIMER1
// #define FASTLED_NO_PINMAP
// #define FL_CLOCKLESS_CONTROLLER_DEFINED

// #define FASTLED_SPI_BYTE_ONLY

#ifndef FASTLED_ARM
#error "FASTLED_ARM must be defined before including this header. Ensure platforms/arm/is_arm.h is included first."
#endif

#ifndef F_CPU
#define F_CPU 60000000
#endif

#ifndef INTERRUPT_THRESHOLD
#define INTERRUPT_THRESHOLD 1
#endif

#include "fl/stdint.h"

#define PROGMEM
#define NO_PROGMEM
#define NEED_CXX_BITS

// Default to NOT using PROGMEM here
#ifndef FASTLED_USE_PROGMEM
#define FASTLED_USE_PROGMEM 0
#endif

#ifndef FASTLED_ALLOW_INTERRUPTS
#define FASTLED_ALLOW_INTERRUPTS 1
#endif

#define cli()  __disable_irq();
#define sei() __enable_irq();

typedef volatile uint32_t RoReg;
typedef volatile uint32_t RwReg;

#include "mxc_delay.h"
#include "rtc.h"

#include <Arduino.h>

// definitions for "generic_pin.h" to compile properly, not functioning as our own pin implementation will be used
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define pinMode(pin, mode) {}
#define digitalWrite(pin, level) {}
#define digitalRead(pin) (0)

#endif
