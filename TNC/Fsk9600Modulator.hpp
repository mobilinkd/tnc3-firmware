// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Modulator.hpp"
#include "AudioInput.hpp"

#include "stm32l4xx_hal.h"
#include <arm_math.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

extern IWDG_HandleTypeDef hiwdg;

namespace mobilinkd { namespace tnc {

/**
 * Generate an array of FIR filter coefficients for a Gaussian filter.
 *
 * @tparam TS is the symbol period, typically 1.
 * @tparam FS is the samples per symbol (this is the upsampling factor).
 * @tparam NSYMS is the pulse duration in symbols.
 *
 * @param b is the 3dB bandwidth.
 *
 * The filter's BT (bandwidth-time product) is TS * b.
 *
 * @note ARM's interpolating FIR filter requires that the number of
 * filter coefficients be a multiple of the upsampling factor, FS.
 */
template <int TS, int FS, int NSYMS>
constexpr std::array<float, TS*FS*NSYMS + FS> gaussian_filter(float b)
{
    // Gaussian filter.
    std::array<float, TS*FS*NSYMS> h;

    float sigma = std::sqrt(std::log(2.0f)) / (2*std::numbers::pi*b*TS);
    float sqrt_2pi = std::sqrt(2.0f*std::numbers::pi);

    for (int t = (NSYMS * TS * FS / -2), i = 0; t < (NSYMS * TS * FS / 2); ++t, ++i) {
        float tx = t / float(FS);
        h[i] = (1.0f / FS) / (sqrt_2pi * sigma * TS) * exp(-tx * tx / (2.0f * sigma * sigma * TS * TS));
    }

    // Rectangle of width equal to the upsampling factor, FS.
    std::array<float, FS> rect;
    rect.fill(1.0f);

    // Convolve the Gaussian pulse with rectangle for interpolation.
    std::array<float, TS*FS*NSYMS + FS> g;
    g.fill(0.0f);
    for (size_t i = 0; i != g.size() - 1; ++i) {
        const size_t j_min = (i >= h.size() - 1) ? i - (h.size() - 1) : 0;
        const size_t j_max = (i < rect.size() - 1) ? i : rect.size() - 1;
        for (size_t j = j_min; j <= j_max; ++j) {
            g[i] += (rect[j] * h[i - j]);
        }
    }

    return g;
}


struct Scrambler
{
    uint32_t state{0};

    bool operator()(bool bit)
    {
        bool result = (bit ^ (state >> 16) ^ (state >> 11)) & 1;
        state = ((state << 1) | result) & 0x1FFFF;
        return result;
    }
};

struct Fsk9600Modulator : Modulator
{

    static constexpr uint8_t UPSAMPLE = 10;
    static constexpr uint8_t BLOCKSIZE = 8;
    static constexpr uint8_t NSYMS = 5;
    static constexpr uint8_t TRANSFER_LEN = BLOCKSIZE * UPSAMPLE;
    static constexpr uint16_t DAC_BUFFER_LEN = TRANSFER_LEN * 2;

    static constexpr auto gaussian = gaussian_filter<1, UPSAMPLE, NSYMS>(0.5);

    static constexpr uint32_t STATE_SIZE = (gaussian.size() / UPSAMPLE) + BLOCKSIZE - 1;
    // Number of bits to flush the FIR filter.
    static constexpr uint8_t FLUSH_LEN = 1;
    static constexpr uint16_t VREF = 4095;

    enum class Level { ZERO, HIGH, LOW };
    enum class State { STOPPED, STARTING, RUNNING, STOPPING };

    arm_fir_interpolate_instance_f32 fir_interpolator;
    std::array<float, STATE_SIZE> fir_state;
    std::array<float, BLOCKSIZE> symbols;
    float tmp[TRANSFER_LEN];

    osMessageQId dacOutputQueueHandle_{0};
    PTT* ptt_{nullptr};
    uint16_t volume_{4096};
    std::array<uint16_t, DAC_BUFFER_LEN> buffer_;
    Level level{Level::HIGH};
    State state{State::STOPPED};
    Scrambler lfsr;
    uint8_t input_buffer_;
    int8_t input_index_ = 0;
    int8_t stop_count_ = 0;

    Fsk9600Modulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {
        arm_fir_interpolate_init_f32(
            &fir_interpolator, UPSAMPLE, gaussian.size(),
            (float32_t*) gaussian.data(), fir_state.data(), BLOCKSIZE);
    }

    ~Fsk9600Modulator() override {}

    void init(const kiss::Hardware& hw) override;

    void deinit() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);
        ptt_->off();
        INFO("Fsk9600Modulator::deinit");
    }

    void set_gain(uint16_t level) override
    {
        auto v = std::max<uint16_t>(256, level);
        v = std::min<uint16_t>(4096, v);
        volume_ = v;
    }

    void set_ptt(PTT* ptt) override
    {
        if (state != State::STOPPED)
        {
            ERROR("PTT change while not stopped");
            CxxErrorHandler();
        }
        ptt_ = ptt;
        ptt_->off();
    }

    PTT* get_ptt() const override { return ptt_; }

    void send(uint8_t bit) override
    {
        auto scrambled = lfsr(bit);

        input_buffer_ <<= 1;
        input_buffer_ |= scrambled;

        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
            input_index_ = 1;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_DisableLSCO();
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::IDLE, osWaitForever);
            fill_empty(buffer_.data());
            fill_empty(buffer_.data() + TRANSFER_LEN);
            ptt_->on();
            start_conversion();
            state = State::STARTING;
            break;
        case State::STARTING:
            input_index_ += 1;
            if (input_index_ == BLOCKSIZE) {
                osMessagePut(dacOutputQueueHandle_, input_buffer_, osWaitForever);
                state = State::RUNNING;
                input_index_ = 0;
            }
            break;
       case State::RUNNING:
           input_index_ += 1;
            if (input_index_ == BLOCKSIZE) {
                osMessagePut(dacOutputQueueHandle_, input_buffer_, osWaitForever);
                input_index_ = 0;
            }
            break;
        }
    }

    void tone(uint16_t freq) override {}

    // DAC DMA interrupt functions.

    void fill_first(uint8_t bits) override
    {
        fill(buffer_.data(), bits);
    }

    void fill_last(uint8_t bits) override
    {
        fill(buffer_.data() + TRANSFER_LEN, bits);
    }

    void empty_first() override
    {
        empty(buffer_.data());
    }

    void empty_last() override
    {
        empty(buffer_.data() + TRANSFER_LEN);
    }

    void empty(uint16_t* buffer)
    {
        HAL_IWDG_Refresh(&hiwdg);
        switch (state)
        {
        case State::STARTING:
            fill_empty(buffer);
            break;
        case State::RUNNING:
            // Flush the input buffer. Because of bit-stuffing, the transmit
            // data is not byte-aligned.
            input_buffer_ <<= (8 - input_index_);

            for (uint8_t i = 0; i != BLOCKSIZE; ++i) {
                symbols[i] = i < input_index_ ? (input_buffer_ & 0x80 ? UPSAMPLE : -UPSAMPLE) : 0.0;
                input_buffer_ <<= 1;
            }

            arm_fir_interpolate_f32(&fir_interpolator, symbols.data(), tmp, BLOCKSIZE);

            for (uint8_t i = 0; i != TRANSFER_LEN; ++i)
            {
                buffer[i] = adjust_level(tmp[i]);
            }

            input_index_ = 0;
            stop_count_ = 0;

            state = State::STOPPING;
            break;
        case State::STOPPING:
            // Flush the FIR filter.
            symbols.fill(0.0f);

            arm_fir_interpolate_f32(&fir_interpolator, symbols.data(), tmp, BLOCKSIZE);

            for (uint8_t i = 0; i != TRANSFER_LEN; ++i)
            {
                buffer[i] = adjust_level(tmp[i]);
            }

            if (++stop_count_ == 5)
                state = State::STOPPED;
            break;
        case State::STOPPED:
            stop_conversion();
            ptt_->off();
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
            osMessagePut(audioInputQueueHandle, tnc::audio::DEMODULATOR, osWaitForever);

            break;
        }
    }

    void abort() override
    {
        state = State::STOPPED;
        stop_conversion();
        ptt_->off();
        level = Level::HIGH;
#if defined(KISS_LOGGING) && defined(HAVE_LSCO)
            HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
        INFO("Fsk9600Modulator::abort");
        // Drain the queue.
        while (osMessageGet(dacOutputQueueHandle_, 0).status == osEventMessage);
    }

    float bits_per_ms() const override
    {
        return 9.6f;
    }

private:

    /**
     * Configure the DAC for timer-based DMA conversion, start the timer,
     * and start DMA to DAC.
     */
    void start_conversion()
    {
        DAC_ChannelConfTypeDef sConfig;

        sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
        sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
        sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
        if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
        {
          CxxErrorHandler();
        }

        HAL_TIM_Base_Start(&htim7);
        HAL_DAC_Start_DMA(
            &hdac1, DAC_CHANNEL_1,
            reinterpret_cast<uint32_t*>(buffer_.data()), buffer_.size(),
            DAC_ALIGN_12B_R);
    }

    uint16_t adjust_level(float sample) const
    {
        sample *= volume_;
        sample /= 10;
        sample += 2048;
        if (sample > 4095) sample = 4095;
        else if (sample < 0) sample = 0;
        return sample;
    }

    void fill(uint16_t* buffer, uint8_t bits);

    [[gnu::noinline]]
    void fill_empty(uint16_t* buffer)
    {
        for (size_t i = 0; i != TRANSFER_LEN; ++i)
        {
            buffer[i] = 2048;
        }
    }

};

}} // mobilinkd::tnc
