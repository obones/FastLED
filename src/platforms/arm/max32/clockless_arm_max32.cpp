#include <stdio.h>
#include <mxc_device.h>
#include <mxc_delay.h>
#include "audio_regs.h"
#include <max32665.h>
#include <math.h>

// don't include it, it embarks most of the FastLED framework
//#include "clockless_arm_max32.h"

int pgcd(int smallest, int precision, int a, int b, int c) {
    int pgc_ = 1;
    for (int i = smallest; i > 0; --i) {

        if (a % i <= precision && b % i <= precision && c % i <= precision) {
            pgc_ = i;
            break;
        }
    }
    return pgc_;
}

void printf_binary(uint32_t value)
{
    for (int index = sizeof(value) * 8 - 1; index >= 0; index--)
    {
        uint8_t byte = (value >> index) & 1;
        printf("%u", byte);
        if (index % 8 == 0 && index != 0)
            printf("'");
    }
}

void printf_binary(uint32_t* value, uint32_t len)
{
    for (uint32_t index = 0; index < len; index++)
    {
        printf_binary(*(value++));
        if (index % 2 == 1)
            printf("\n");
        else
            printf(" ");
    }
}

static constexpr uint8_t I2S_MAX_PULSE_PER_BIT = 20; // put it higher to get more accuracy but it could decrease the refresh rate without real improvement
static constexpr uint8_t USABLE_BITS = 24; // we'd love for it to be 32 -> https://ez.analog.com/microcontrollers/ultra-low-power-microcontrollers/f/q-a/600942/why-does-max32666-i2s-pcm-always-truncate-samples-to-24-bits
static constexpr int8_t WORD_BIT_LENGTH = sizeof(uint32_t) * 8;
static constexpr int8_t LOWEST_USABLE_BIT_INDEX = WORD_BIT_LENGTH - USABLE_BITS;
static constexpr int8_t FIFO_DEPTH = 4; // per datasheet

int gPulsesPerBit = 0;
uint32_t gOneBit = 0;
uint32_t gZeroBit = 0;

namespace fl
{
    void prepareBitPatterns(uint32_t T1ns, uint32_t T2ns, uint32_t T3ns)
    {
        /*
        We calculate the best pcgd to the timing
        ie
        WS2811 77 77 154 => 1  1 2 => nb pulses = 4
        WS2812 60 150 90 => 2 5 3 => nb pulses =10
        */
        uint32_t smallest = 0;
        if (T1ns > T2ns)
            smallest = T2ns;
        else
            smallest = T1ns;
        if (smallest > T3ns)
            smallest = T3ns;
        double freq = (double)1 / (double)(T1ns + T2ns + T3ns);
        printf("chipset frequency: %f Khz\n", 1000000L*freq);
        printf("smallest: %d\n",smallest);

        int pgc_ = 1;
        int precision = 0;
        pgc_ = pgcd(smallest, precision, T1ns, T2ns, T3ns);
        // Serial.printf("%f\n",I2S_MAX_CLK/(1000000000L*freq));
        while (
            pgc_ == 1 ||
            (T1ns / pgc_ + T2ns / pgc_ + T3ns / pgc_) >
                I2S_MAX_PULSE_PER_BIT) // while(pgc_==1 ||  (T1ns/pgc_ +T2ns/pgc_
                                    // +T3ns/pgc_)>I2S_MAX_CLK/(1000000000L*freq))
        {
            ++precision;
            pgc_ = pgcd(smallest, precision, T1ns, T2ns, T3ns);
            // Serial.printf("%d %d\n",pgc_,(a+b+c)/pgc_);
        }
        pgc_ = pgcd(smallest, precision, T1ns, T2ns, T3ns);
        printf("pgcd: %d\nprecision:%d\n", pgc_, precision);

        int T1Pulses = (int)T1ns / pgc_;
        int T2Pulses = (int)T2ns / pgc_;
        int T3Pulses = (int)T3ns / pgc_;

        gPulsesPerBit = T1Pulses + T2Pulses + T3Pulses;
        printf("nb pulse per bit: %d\n", gPulsesPerBit);
        printf("    T1Pulses: %d\n", T1Pulses);
        printf("    T2Pulses: %d\n", T2Pulses);
        printf("    T3Pulses: %d\n", T3Pulses);

        freq = 1000000000L * freq * gPulsesPerBit;
        printf("needed frequency (nbPulse per bit) * (chipset // frequency): %f Mhz\n", freq / 1000000);
        printf("SystemCoreClock: %d\n", SystemCoreClock);
        printf("needed BitCLK divider: %d\n", SystemCoreClock / (uint32_t)round(freq));

        uint32_t bit_mask = ~(0xFFFFFFFF << gPulsesPerBit);
        int ones_for_one = T1Pulses + T2Pulses;
        gOneBit = 0xFFFFFFFF << (gPulsesPerBit - ones_for_one);
        gOneBit &= bit_mask;
        printf("gOneBit:  ");
        printf_binary(gOneBit);
        printf("\n");

        int ones_for_zero = T1Pulses;
        gZeroBit = 0xFFFFFFFF << (gPulsesPerBit - ones_for_zero);
        gZeroBit &= bit_mask;
        printf("gOneBit:  ");
        printf_binary(gZeroBit);
        printf("\n");
    }

    void addBitPulses(uint32_t bitPulses, uint16_t& currentWord, int8_t& currentPulsePos, uint32_t* pixelsPulses)
    {
        if (currentPulsePos >= LOWEST_USABLE_BIT_INDEX)
        {
            pixelsPulses[currentWord] |= bitPulses << currentPulsePos;
        }
        else
        {
            // here, we must split the bit pulses over two consecutive words in the buffer
            int8_t remainingBits = gPulsesPerBit + currentPulsePos - LOWEST_USABLE_BIT_INDEX;
            printf("  remainingBits: %d - gPulsesPerBit: %d - currentPulsePos: %d, LOWEST_USABLE_BIT_INDEX: %d\n", remainingBits, gPulsesPerBit, currentPulsePos, LOWEST_USABLE_BIT_INDEX);
            //   remainingBits: 0 - gPulsesPerBit: 10 - currentPulsePos: 2, LOWEST_USABLE_BIT_INDEX: 8

            pixelsPulses[currentWord] |= (bitPulses >> (gPulsesPerBit - remainingBits)) << LOWEST_USABLE_BIT_INDEX;
            currentWord++;
            currentPulsePos = WORD_BIT_LENGTH - gPulsesPerBit + remainingBits;
            pixelsPulses[currentWord] |= bitPulses << currentPulsePos;
        }

        currentPulsePos -= gPulsesPerBit;
    }

    void addByte(uint8_t byte, uint16_t& currentWord, int8_t& currentPulsePos, uint32_t* pixelsPulses)
    {
        // Send bits MSB first (standard for most LED protocols)
        for (int bit = 7; bit >= 0; --bit)
        {
            bool is_one = (byte & (1 << bit)) != 0;

            if (is_one)
                addBitPulses(gOneBit, currentWord, currentPulsePos, pixelsPulses);
            else
                addBitPulses(gZeroBit, currentWord, currentPulsePos, pixelsPulses);
        }
    }

    void sendPixelData(uint8_t *data, uint16_t pixel_count)
    {
        uint16_t pixelsPulsesSize =
            ceil(
                (
                    pixel_count
                    * 3               // 3 bytes per pixel
                    * 8               // 8 bits per pixel
                    * gPulsesPerBit   // n pulses per bit
                )
                / (float)USABLE_BITS  // m usable bits in each "cell" of the array
            );
        pixelsPulsesSize +=
            pixelsPulsesSize % 2      // ensure we have an even number of uint32_t elements for our two PCM channels
            + FIFO_DEPTH * 4;         // full 0 words to "clear the line" after the bits have been sent
        uint32_t pixelsPulses[pixelsPulsesSize] = {};
        printf("pixelsPulsesSize: %d\n", pixelsPulsesSize);

        // Iterate through all bytes in all pixels to add their "pulse" bit patterns in the buffer
        uint16_t currentWord = 2;
        int8_t currentPulsePos = WORD_BIT_LENGTH - gPulsesPerBit;
        for (uint16_t i = 0; i < pixel_count * 3; ++i)
        {
            uint8_t byte = data[i];
            addByte(byte, currentWord, currentPulsePos, pixelsPulses);
        }

        const int zeroPulsesInFront = 6;
        const int zeroPulsesAtEnd = 2;
        for (int i = 0; i < zeroPulsesInFront; i++)
            pixelsPulses[i] = 0b10000000000000000000001000000000 | ((uint32_t)1 << (i+2 + LOWEST_USABLE_BIT_INDEX));
        for (int i = zeroPulsesInFront; i < pixelsPulsesSize - zeroPulsesAtEnd; i++)
            pixelsPulses[i] = 0xFFFFFE00 & ~((uint32_t)1 << (i-zeroPulsesInFront+2 + LOWEST_USABLE_BIT_INDEX));
        for (int i = pixelsPulsesSize - zeroPulsesAtEnd; i < pixelsPulsesSize; i++)
            pixelsPulses[i] = 0b10000000000000000000001000000000 | ((uint32_t)1 << (WORD_BIT_LENGTH - (pixelsPulsesSize - i)));

        printf("pixelsPulses:\n");
        printf_binary(pixelsPulses, pixelsPulsesSize);
        printf("\n");

        // disable TX and saturate the FIFOs
        MXC_AUDIO->global_en = 1;
        MXC_AUDIO->pcm_tx_enables_byte0 = 0;
        uint16_t wordIndex = 0;
        printf("wordIndex before saturate: %d\n", wordIndex);
        printf("int_pcm_tx_status: ");
        printf_binary(MXC_AUDIO->int_pcm_tx_status);
        printf("\n");

        for (int fifoIndex = 0; (fifoIndex < FIFO_DEPTH) && (wordIndex < pixelsPulsesSize); fifoIndex++)
        {
            MXC_AUDIO->tx_pcm_ch0_addr = pixelsPulses[wordIndex++];
            MXC_AUDIO->tx_pcm_ch1_addr = pixelsPulses[wordIndex++];
        }
        printf("wordIndex after saturate: %d\n", wordIndex);
        printf("int_pcm_tx_status: ");
        printf_binary(MXC_AUDIO->int_pcm_tx_status);
        printf("\n");

        // Enable TX and send the buffer, two words at a time, only if there is room in the FIFOs (ie, not almost full)
        MXC_AUDIO->pcm_tx_enables_byte0 = (MXC_F_PCM_TX_CH0_EN | MXC_F_PCM_TX_CH1_EN);
        //MXC_AUDIO->global_en = 1;
        while (wordIndex < pixelsPulsesSize)
        {
            while ((MXC_AUDIO->int_pcm_tx_status & MXC_F_PDM_TX_FIFO_CH1_ALMOST_FULL) != 0)
                printf("%d ", wordIndex);

            MXC_AUDIO->tx_pcm_ch0_addr = pixelsPulses[wordIndex++];
            MXC_AUDIO->tx_pcm_ch1_addr = pixelsPulses[wordIndex++];
        }

        // Words are placed in the FIFOs, wait for transmission end (ie, FIFOs are empty) then disable TX
        while ((MXC_AUDIO->int_pcm_tx_status & MXC_F_PDM_TX_FIFO_CH1_ALMOST_EMPTY) == 0)
            ;
        printf("\n");

        printf("wordIndex at end: %d\n", wordIndex);
        printf("int_pcm_tx_status: ");
        printf_binary(MXC_AUDIO->int_pcm_tx_status);
        printf("\n");

        //MXC_Delay(1);
        MXC_AUDIO->pcm_tx_enables_byte0 = 0;
        //MXC_AUDIO->global_en = 0; // settings pcm_tx_enables_byte0 to 0 leaves 900mv on the DOUT pin!
    }
}