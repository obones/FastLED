#ifndef __INC_CLOCKLESS_ARM_RENESAS
#define __INC_CLOCKLESS_ARM_RENESAS

#include "fl/chipsets/timing_traits.h"
#include "fastled_delay.h"
#define FL_CLOCKLESS_CONTROLLER_DEFINED 1

#include "../../shared/clockless_blocking.h"

namespace fl {
void prepareBitPatterns(uint32_t T1ns, uint32_t T2ns, uint32_t T3ns);
void prepareAudioSubsystem();
void sendPixelData(PixelIterator& pixelIterator);//uint8_t *data, uint16_t pixel_count);

template <int DATA_PIN, typename TIMING, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 280>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {
	CMinWait<WAIT_TIME> mWait;
public:
	virtual void init() {
        prepareBitPatterns(TIMING::T1, TIMING::T2, TIMING::T3);

        prepareAudioSubsystem();
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
}  // namespace fl
#endif
