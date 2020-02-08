// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "HdlcFrame.hpp"
#include "FirFilter.hpp"
#include "AudioInput.hpp"

#include <functional>

#include <arm_math.h>

namespace mobilinkd { namespace tnc {

constexpr size_t FILTER_TAP_NUM = 132;

using demod_filter_t = std::function<q15_t*(q15_t*, size_t)>;
using demodulator_t = std::function<hdlc::IoFrame*(q15_t*)>;

struct IDemodulator
{
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual hdlc::IoFrame* operator()(const q15_t* samples) = 0;
    virtual float readTwist() = 0;

    virtual bool locked() const = 0;
    virtual size_t size() const = 0;

    virtual ~IDemodulator() {}

    static void startADC(uint32_t period, uint32_t block_size);

    static void stopADC() {
        if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
            CxxErrorHandler();
        if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
            CxxErrorHandler();
    }
};

}} // mobilinkd::tnc
