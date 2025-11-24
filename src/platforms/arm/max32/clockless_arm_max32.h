#ifndef __INC_CLOCKLESS_ARM_RENESAS
#define __INC_CLOCKLESS_ARM_RENESAS

#include "fl/chipsets/timing_traits.h"
#include "fastled_delay.h"
#define FL_CLOCKLESS_CONTROLLER_DEFINED 1

#include "../../shared/clockless_blocking.h"

namespace fl {
#if MAX32_DEFAULT_CLOCKLESS
template <int DATA_PIN, typename TIMING, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 0>
	using ClocklessController = ClocklessBlockingGeneric<DATA_PIN, TIMING, RGB_ORDER, XTRA0, FLIP, WAIT_TIME>;
#elif MAX32_TMR4_CLOCKLESS
// Definition for a single channel clockless controller for MAX32666 (Cortex M4)
// We would have loved to use the same controller as the one for RA4M1 as it's using the same Cortex M4 core
// but sadly, CYCCNT is never incremented on this controller.
// Indeed, the DWT is optional and bits 24 to 27 are all set to 1, telling us that nothing is available!
// As a workaround, we use TIMER4 with the smallest possible divider, which at 96MHz gives us 20.8333ns per
// count increment (ticks)
// See clockless.h for detailed info on how the template parameters are used.

#define CLOCKLESS_TIMER MXC_TMR4
#define CURRENT_TICKS CLOCKLESS_TIMER->cnt

template <int DATA_PIN, typename TIMING, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 280>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {
	// Extract timing values from ChipsetTiming struct and convert from nanoseconds to timer ticks
	// Formula: ticks = (nanoseconds * CPU_MHz + 500) / 1000
	// The +500 provides rounding to nearest integer
	#define NS_TO_TICKS(ns) ((ns/2 * (F_CPU/2 / 1000000) + 500) / 1000)
	// #define TICKS_PER_US NS_TO_TICKS(1000)

	static constexpr uint32_t T1 = NS_TO_TICKS(TIMING::T1);
	static constexpr uint32_t T2 = NS_TO_TICKS(TIMING::T2);
	static constexpr uint32_t T3 = NS_TO_TICKS(TIMING::T3);
	typedef typename FastPin<DATA_PIN>::port_ptr_t data_ptr_t;
	typedef typename FastPin<DATA_PIN>::port_t data_t;

	data_t mPinMask;
	data_ptr_t mPort;
	CMinWait<WAIT_TIME> mWait;

public:
	virtual void init() {
		FastPin<DATA_PIN>::setOutput();
		mPinMask = FastPin<DATA_PIN>::mask();
		mPort = FastPin<DATA_PIN>::port();

		// prepare timer in continuous mode
		/*#ifdef MSDK_NO_GPIO_CLK_INIT
		MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER4);
		while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER4) {}
		MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T4);
		#endif
		mxc_tmr_cfg_t config =
		{
			pres: MXC_TMR_PRES_1,
			mode: MXC_TMR_MODE_CONTINUOUS,
			cmp_cnt: 0xFFFFFFFF
		};
		MXC_TMR_Init(CLOCKLESS_TIMER, &config);*/
	}

	virtual uint16_t getMaxRefreshRate() const { return 400; }

protected:
	virtual void showPixels(PixelController<RGB_ORDER> & pixels) {
        // Wait for minimum time since last frame
        mWait.wait();

        // Disable interrupts to ensure timing accuracy
        // (LED protocols are very timing-sensitive on most platforms)
        cli();

        // Send all pixel data
        if (pixels.mLen > 0) {
            sendPixelData(pixels);
        }

        // Re-enable interrupts
        sei();  // Re-enable interrupts on AVR

        // Mark that we've sent data
        mWait.mark();
	}

private:
    /// Send raw pixel data with precise timing
    FASTLED_FORCE_INLINE static void sendPixelData(PixelController<RGB_ORDER> & pixels)
    {
        // Get color component order
        uint16_t pixel_count = pixels.mLen;
        uint8_t *data = (uint8_t *)pixels.mData;

		// reset nanoseconds timer
		MXC_TMR_Stop(CLOCKLESS_TIMER);
		CURRENT_TICKS = 0;
		MXC_TMR_Start(CLOCKLESS_TIMER);

        // Iterate through all bytes in all pixels
        for (uint16_t i = 0; i < pixel_count * 3; ++i) {
            uint8_t byte = data[i];
            sendByte(byte);
        }

        // Send reset code: line low for at least 50µs
        // (WS2812/SK6812 datasheet requirement)
        fl::FastPin<DATA_PIN>::lo();
        delayTicks(NS_TO_TICKS(280000));  // 280 microseconds
    }

    /// Send a single byte with bit-by-bit timing
    FASTLED_FORCE_INLINE static void sendByte(uint8_t byte)
    {
        // Send bits MSB first (standard for most LED protocols)
        for (int bit = 7; bit >= 0; --bit) {
            bool is_one = (byte & (1 << bit)) != 0;

            if (is_one) {
                sendBit1();
            } else {
                sendBit0();
            }
        }
    }

    /// Send a '1' bit with correct WS2812 timing
    /// T1H (high time for 1-bit) = T1+T2, T1L (low time) = T3
    FASTLED_FORCE_INLINE static void sendBit1()
    {
        // Set line HIGH
        fl::FastPin<DATA_PIN>::hi();
        // Hold high for T1+T2 nanoseconds (e.g., 875ns for WS2812)
        delayTicks(T1 + T2);

        // Set line LOW
        fl::FastPin<DATA_PIN>::lo();
        // Hold low for T3 nanoseconds (e.g., 375ns for WS2812)
        delayTicks(T3);
    }

    /// Send a '0' bit with correct WS2812 timing
    /// T0H (high time for 0-bit) = T1, T0L (low time) = T2+T3
    FASTLED_FORCE_INLINE static void sendBit0()
    {
        // Set line HIGH
        fl::FastPin<DATA_PIN>::hi();
        // Hold high for T1 nanoseconds (e.g., 250ns for WS2812)
        delayTicks(T1);

        // Set line LOW
        fl::FastPin<DATA_PIN>::lo();
        // Hold low for T2+T3 nanoseconds (e.g., 1000ns for WS2812)
        delayTicks(T2 + T3);
    }

	FASTLED_FORCE_INLINE static void delayTicks(uint32_t ticks)
	{
		uint32_t startTicks = CURRENT_TICKS;
		uint32_t endTicks = startTicks;
		while (endTicks - startTicks < ticks)
			endTicks = CURRENT_TICKS;
	}
}; // template class
#elif MAX32_I2S_CLOCKLESS
void prepareBitPatterns(uint32_t T1ns, uint32_t T2ns, uint32_t T3ns);
void sendPixelData(PixelIterator& pixelIterator);//uint8_t *data, uint16_t pixel_count);

template <int DATA_PIN, typename TIMING, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 280>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {
	CMinWait<WAIT_TIME> mWait;
public:
	virtual void init() {
        // prepareAudio();

        // prepare bit patterns
        prepareBitPatterns(TIMING::T1, TIMING::T2, TIMING::T3);
    }

protected:
	virtual void showPixels(PixelController<RGB_ORDER> & pixels) {
        // Wait for minimum time since last frame
        mWait.wait();

        // Disable interrupts to ensure timing accuracy
        // (LED protocols are very timing-sensitive on most platforms)
        //cli();

        // Send all pixel data
        if (pixels.mLen > 0)
        {
            //uint8_t *data = (uint8_t *)pixels.mData;
            //uint16_t pixel_count = pixels.mLen;
            auto pixel_iterator = pixels.as_iterator(this->getRgbw());

            sendPixelData(pixel_iterator); //data, pixel_count);
        }

        // Re-enable interrupts
        //sei();  // Re-enable interrupts on AVR

        // Mark that we've sent data
        mWait.mark();
	}

private:
};
#else
    #error "No clockless controller selected"
#endif
}  // namespace fl
#endif
