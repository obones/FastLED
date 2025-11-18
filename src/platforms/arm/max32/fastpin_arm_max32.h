// ok no namespace fl
#ifndef __FASTPIN_ARM_MAX32_H
#define __FASTPIN_ARM_MAX32_H

#if defined(FASTLED_FORCE_SOFTWARE_PINS)
#warning "Software pin support forced, pin access will be slightly slower."
#define NO_HARDWARE_PIN_SUPPORT
#undef HAS_HARDWARE_PIN_SUPPORT
#else

#include "fl/force_inline.h"
#include "fl/fastpin_base.h"

#include "gpio.h"

namespace fl {


template<uint8_t PIN, mxc_gpio_regs_t* PORT(), uint32_t MASK> class _ARMPIN: public ValidPinBase {
public:
    // MAX32 uses 32 bit registers aligned to 32 bit boundaries
	typedef volatile uint32_t * port_ptr_t;
	typedef uint32_t port_t;

	inline static void setPinFunction(mxc_gpio_func_t function) {
		mxc_gpio_cfg_t pinConfig = {PORT(), MASK, function, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_3};
		MXC_GPIO_Config(&pinConfig);
	}
	inline static void setOutput() { setPinFunction(MXC_GPIO_FUNC_OUT); }
	inline static void setInput() { setPinFunction(MXC_GPIO_FUNC_IN); }

    inline static void hi() __attribute__ ((always_inline)) { MXC_GPIO_OutSet(PORT(), MASK); /* PORT()->out_set = MASK;*/ }
	inline static void lo() __attribute__ ((always_inline)) { MXC_GPIO_OutClr(PORT(), MASK); /*PORT()->out_clr = MASK;*/ }
	inline static void set(FASTLED_REGISTER port_t val) __attribute__ ((always_inline)) { MXC_GPIO_OutPut(PORT(), MASK, val); /*PORT()->out = (PORT()->out & ~MASK) | (val & MASK);*/ }

	inline static void strobe() __attribute__ ((always_inline)) { toggle(); toggle(); }

	inline static void toggle() __attribute__ ((always_inline)) { MXC_GPIO_OutToggle(PORT(), MASK); /*PORT()->out ^= MASK;*/ }

	inline static void hi(FASTLED_REGISTER port_ptr_t port) __attribute__ ((always_inline)) { hi(); }
	inline static void lo(FASTLED_REGISTER port_ptr_t port) __attribute__ ((always_inline)) { lo(); }
	inline static void fastset(FASTLED_REGISTER port_ptr_t port, FASTLED_REGISTER port_t val) __attribute__ ((always_inline)) { *port = (*port & ~MASK) | (val & MASK); }

    inline static port_ptr_t port() __attribute__ ((always_inline)) { return &PORT()->out; }
    inline static port_t mask() __attribute__ ((always_inline)) { return MASK; }
};

#define _FL_DEFPIN(PIN, PORT, MASK) constexpr mxc_gpio_regs_t* _Func##PORT() { return PORT; }; template<> class FastPin<PIN> : public _ARMPIN<PIN, _Func##PORT, MASK> {};

#if defined(MAX32665)
#include "max32665.h"

_FL_DEFPIN(12, MXC_GPIO0, MXC_GPIO_PIN_12);

//constexpr mxc_gpio_regs_t* MXC_GPIO0Func() { return MXC_GPIO0; }; template<> class FastPin<12> : public _ARMPIN<12, MXC_GPIO0Func, MXC_GPIO_PIN_12> {};

#define HAS_HARDWARE_PIN_SUPPORT 1

#else
#error "Platform not supported"
#endif

#endif  // FASTLED_FORCE_SOFTWARE_PINS
};
#endif // __FASTPIN_ARM_MAX32_H
