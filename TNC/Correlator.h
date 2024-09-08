// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "IirFilter.hpp"

#include "stm32l4xx_hal.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <tuple>

namespace mobilinkd {

template <typename FloatType, size_t Symbols, size_t SamplesPerSymbol>
struct Correlator
{
    static constexpr size_t SYMBOLS = Symbols;
    static constexpr size_t SAMPLES_PER_SYMBOL = SamplesPerSymbol;
    using value_type = FloatType;
    using buffer_t = std::array<FloatType, SYMBOLS * SAMPLES_PER_SYMBOL>;
    using sync_t = std::array<int8_t, SYMBOLS>;
    using sample_filter_t = tnc::IirFilter<3>;

    buffer_t buffer_;

    float limit_ = 0.;
    uint8_t symbol_pos_ = 0;
    uint8_t buffer_pos_ = 0;
    uint8_t prev_buffer_pos_ = 0;
    int code = -1;

    // IIR with Nyquist of 1/240.  This is used to determine the baseline
    // signal level, which is then used to scale the correlation value.
    // This makes the detector self-calibrating.
    static constexpr std::array<float,3> b = {4.24433681e-05, 8.48867363e-05, 4.24433681e-05};
    static constexpr std::array<float,3> a = {1.0, -1.98148851,  0.98165828};
    sample_filter_t sample_filter{b, a};
    std::array<int, SYMBOLS> tmp;

    void sample(float value)
    {
        limit_ = sample_filter(std::abs(value));
        buffer_[buffer_pos_] = value;
        prev_buffer_pos_ = buffer_pos_;
        if (++buffer_pos_ == buffer_.size()) buffer_pos_ = 0;
    }

    float correlate(sync_t sync)
    {
        float result = 0.f;
        size_t pos = prev_buffer_pos_ + SAMPLES_PER_SYMBOL;

        for (size_t i = 0; i != sync.size(); ++i)
        {
            if (pos >= buffer_.size()) pos -= buffer_.size(); // wrapped
            result += sync[i] * buffer_[pos];
            pos += SAMPLES_PER_SYMBOL;
        }
        return result;
    }


    float limit() const {return limit_;}
    uint8_t index() const {return prev_buffer_pos_ % SAMPLES_PER_SYMBOL;}

    /**
     * Get the average outer symbol levels at a given index.  This makes trhee
     * assumptions.
     *
     *  1. The max symbol value is above 0 and the min symbol value is below 0.
     *  2. The samples at the given index only contain outer symbols.
     *  3. The index is a peak correlation index.
     *
     *  The first should hold true except for extreme frequency errors.  The
     *  second holds true for the sync words used for M17.
     */
    std::tuple<float, float> outer_symbol_levels(uint8_t sample_index)
    {
        float min_sum = 0;
        float max_sum = 0;
        uint8_t min_count = 0;
        uint8_t max_count = 0;
        uint8_t index = 0;
        for (size_t i = sample_index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
        {
            tmp[index++] = buffer_[i] * 1000.f;
            max_sum += buffer_[i] * (buffer_[i] > 0.f);
            min_sum += buffer_[i] * (buffer_[i] < 0.f);
            max_count += (buffer_[i] > 0.f);
            min_count += (buffer_[i] < 0.f);
        }
        INFO("osl: %d, %d, %d, %d,%d, %d, %d, %d",
            tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7]);
        return std::make_tuple(min_sum / min_count, max_sum / max_count);
    }

    template <typename F>
    void apply(F func, uint8_t index)
    {
        for (size_t i = index; i < buffer_.size(); i += SAMPLES_PER_SYMBOL)
        {
            func(buffer_[i]);
        }
    }
};

struct Indicator
{
    GPIO_TypeDef* gpio;
    uint16_t pin;

    void on()
    {
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
    }

    void off()
    {
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
    }

    void toggle()
    {
        HAL_GPIO_TogglePin(gpio, pin);
    }
};

template <typename Correlator>
struct SyncWord
{
    static constexpr size_t SYMBOLS = Correlator::SYMBOLS;
    static constexpr size_t SAMPLES_PER_SYMBOL = Correlator::SAMPLES_PER_SYMBOL;
    using value_type = typename Correlator::value_type;

    using buffer_t = std::array<int8_t, SYMBOLS>;
    using sample_buffer_t = std::array<value_type, SAMPLES_PER_SYMBOL>;

    buffer_t sync_word_;
    sample_buffer_t samples_;
    uint8_t pos_ = 0;
    uint8_t timing_index_ = 0;
    bool triggered_ = false;
    int8_t updated_ = 0;
    float magnitude_1_ = 1.f;
    float magnitude_2_ = -1.f;

    SyncWord(buffer_t&& sync_word, float magnitude_1, float magnitude_2 = std::numeric_limits<float>::lowest())
    : sync_word_(std::move(sync_word)), magnitude_1_(magnitude_1), magnitude_2_(magnitude_2)
    {}

    float triggered(Correlator& correlator)
    {
        float limit_1 = correlator.limit() * magnitude_1_;
        float limit_2 = correlator.limit() * magnitude_2_;
        auto value = correlator.correlate(sync_word_);

        return (value > limit_1 || value < limit_2) ? value : 0.0;
    }

    uint8_t operator()(Correlator& correlator)
    {
        auto value = triggered(correlator);

        value_type peak_value = 0;

        if (std::abs(value) > 0.0)
        {
            if (!triggered_)
            {
                samples_.fill(0);
                triggered_ = true;
                INFO("trigger = %d, limit = %d", int(value), int(correlator.limit()));
            }
            samples_[correlator.index()] = value;
        }
        else
        {
            if (triggered_)
            {
                // Calculate the timing index on the falling edge.
                triggered_ = false;
                timing_index_ = 0;
                peak_value = value;
                uint8_t index = 0;
                for (auto f : samples_)
                {
                    if (abs(f) > abs(peak_value))
                    {
                        peak_value = f;
                        timing_index_ = index;
                    }
                    index += 1;
                }
                updated_ = peak_value > 0 ? 1 : -1;
            }
        }
        return timing_index_;
    }

    /**
     * Returns 0 if the sync word was not detected, otherwise it returns
     * either -1 or 1, depending on whether the inverse or normal sync
     * was detected.
     */
    int8_t updated()
    {
        auto result = updated_;
        updated_ = 0;
        return result;
    }
};

namespace m17 {

using Correlator = ::mobilinkd::Correlator<float, 8, 10>;

}} // mobilinkd::m17
