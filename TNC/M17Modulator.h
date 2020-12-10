// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "Modulator.hpp"
#include "HdlcFrame.hpp"

#include <array>
#include <algorithm>
#include <cstdint>

namespace mobilinkd { namespace tnc {

struct M17Modulator : Modulator
{
    static constexpr int16_t DAC_BUFFER_LEN = 384;
    static constexpr int16_t TRANSFER_LEN = DAC_BUFFER_LEN / 2;
    static constexpr uint16_t VREF = 4095;

    enum class State { STOPPED, STARTING, RUNNING, STOPPING };

    osMessageQId dacOutputQueueHandle_{0};
    PTT* ptt_{nullptr};
    uint16_t volume_{4096};
    std::array<uint16_t, DAC_BUFFER_LEN> buffer_;
    State state{State::STOPPED};
    bool loopback_ = false;
    std::array<int16_t, TRANSFER_LEN> loopback_data_;

    M17Modulator(osMessageQId queue, PTT* ptt)
    : dacOutputQueueHandle_(queue), ptt_(ptt)
    {}

    ~M17Modulator() override {}

    void start_loopback() override
    {
        loopback_ = true;
        buffer_.fill(0);
#ifdef KISS_LOGGING
        HAL_RCCEx_DisableLSCO();
#endif
        start_conversion();
    }

    void stop_loopback() override
    {
        loopback_ = false;
        stop_conversion();
#ifdef KISS_LOGGING
        HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif
    }

    void loopback(const void* input) override
    {
        auto data = static_cast<const std::array<int16_t, TRANSFER_LEN>*>(input);
        loopback_ = true;
        std::copy(data->begin(), data->end(), loopback_data_.begin());
        for (auto& x : loopback_data_) x = (x >> 2) + 2048;
        osMessagePut(dacOutputQueueHandle_, 1, osWaitForever);
    }

    void init(const kiss::Hardware& hw) override;

    void deinit() override
    {
        state = State::STOPPED;
        HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim7);
//        ptt_->off();
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
//        ptt_->off();
    }

    void send(bool bit) override
    {
        switch (state)
        {
        case State::STOPPING:
        case State::STOPPED:
//            ptt_->on();
            #ifdef KISS_LOGGING
                HAL_RCCEx_DisableLSCO();
            #endif

//            fill_first(bit);
            state = State::STARTING;
            break;
        case State::STARTING:
//            fill_last(bit);
            state = State::RUNNING;
            start_conversion();
            break;
        case State::RUNNING:
            osMessagePut(dacOutputQueueHandle_, bit, osWaitForever);
            break;
        }
    }

    // DAC DMA interrupt functions.

    void fill_first(bool bit) override
    {
        if (!loopback_) fill(buffer_.data(), bit);
        else
        {
            std::copy(loopback_data_.begin(), loopback_data_.end(), buffer_.begin());
        }
    }

    void fill_last(bool bit) override
    {
        if (!loopback_) fill(buffer_.data() + TRANSFER_LEN, bit);
        else
        {
            std::copy(loopback_data_.begin(), loopback_data_.end(), buffer_.begin() + TRANSFER_LEN);
        }
    }

    void empty() override
    {
        switch (state)
        {
        case State::STARTING:
            // fall-through
        case State::RUNNING:
            state = State::STOPPING;
            break;
        case State::STOPPING:
            state = State::STOPPED;
            stop_conversion();
//            ptt_->off();
            #ifdef KISS_LOGGING
                HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
            #endif
            break;
        case State::STOPPED:
            break;
        }
    }

    void abort() override
    {
        state = State::STOPPED;
        stop_conversion();
//        ptt_->off();
        #ifdef KISS_LOGGING
            HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
        #endif
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

    uint16_t adjust_level(int32_t sample) const
    {
        sample *= volume_;
        sample >>= 12;
        sample += 2048;
        return sample;
    }

    void fill(uint16_t*, bool)
    {
         CxxErrorHandler();
    }
};

}} // mobilinkd::tnc
