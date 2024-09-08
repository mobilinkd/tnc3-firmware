// Copyright 2021-2022 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "KalmanFilter.h"
#include "StandardDeviation.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace mobilinkd { namespace m17 {
template <typename FloatType, size_t SYNC_WORD_LEN = 8>
class FreqDevEstimator
{
    static constexpr FloatType DEVIATION = 1.;

    SymbolKalmanFilter<FloatType> minFilter_;
    SymbolKalmanFilter<FloatType> maxFilter_;
    RunningStandardDeviation<FloatType, 184> stddev_;
    FloatType idev_ = 1.;
    FloatType offset_ = 0.;
    uint8_t count_ = 0;
    FloatType min_ = 0.;
    FloatType max_ = 0.;
    uint8_t minCount_ = 0;
    uint8_t maxCount_ = 0;
    size_t updateCount_ = 0;
    std::array<FloatType, SYNC_WORD_LEN> accum_;
    bool reset_ = true;

public:

    void reset()
    {
        idev_ = 1.;
        offset_ = 0.;
        count_ = 0;
        min_ = 0.;
        max_ = 0.;
        minCount_ = 0;
        maxCount_ = 0;
        updateCount_ = 0;
        stddev_.reset();
        accum_.fill(0.);
        reset_ = true;
    }

    /**
     * This function takes the index samples from the correlator to build
     * the outer symbol samples for the frequency offset (signal DC offset)
     * and the deviation (signal magnitude). It expects bursts of 8 samples,
     * one for each symbol in a sync word.
     *
     * @param sample
     */
    void sample(FloatType sample)
    {
        count_ += 1;

        if (count_ < SYNC_WORD_LEN) {
            accum_[count_ - 1] = sample;
            return;
        } else if (count_ == SYNC_WORD_LEN) {
            accum_[count_ - 1] = sample;
            FloatType min_value = accum_[0];
            FloatType max_value = accum_[0];
            for (size_t i = 1; i != SYNC_WORD_LEN; ++i) {
                min_value = std::min(min_value, accum_[i]);
                max_value = std::max(max_value, accum_[i]);
            }
            
            FloatType avg = (min_value + max_value) / 2.;

            for (size_t i = 0; i != SYNC_WORD_LEN; ++i)
            {
                if (accum_[i] < avg)
                {
                    minCount_ += 1;
                    min_ += accum_[i];
                }
                else
                {
                    maxCount_ += 1;
                    max_ += accum_[i];
                }
            }

            auto minAvg = minCount_ > 0 ? min_ / minCount_ : min_value;
            auto maxAvg = maxCount_ > 0 ? max_ / maxCount_ : max_value;
            if (reset_)
            {
                minFilter_.reset(minAvg);
                maxFilter_.reset(maxAvg);
                idev_ = 6.0 / (maxAvg - minAvg);
                offset_ = (maxAvg + minAvg) / 2;
                reset_ = false;
            }
            else
            {
                auto minFiltered = minFilter_.update(minAvg, count_ + updateCount_);
                auto maxFiltered = maxFilter_.update(maxAvg, count_ + updateCount_);
                idev_ = 6.0 / (maxFiltered[0] - minFiltered[0]);
                offset_ = (maxFiltered[0] + minFiltered[0]) / 2;
            }

            count_ = 0;
            updateCount_ = 0;
            min_ = 0.;
            max_ = 0.;
            minCount_ = 0;
            maxCount_ = 0;
        }
    }

    FloatType normalize(FloatType sample) const
    {
        return (sample - offset_) * idev_;
    }

    FloatType evm() const { return stddev_.stdev(); }

    void update() const {}

    /**
     * Capture EVM of a symbol.
     *
     * @param sample is a normalized sample captured at the best sample point.
     */
    void update(FloatType sample)
    {
        if (sample > 2)
        {
            stddev_.capture(sample - 3);
        }
        else if (sample > 0)
        {
            stddev_.capture(sample - 1);
        }
        else if (sample > -2)
        {
            stddev_.capture(sample + 1);
        }
        else
        {
            stddev_.capture(sample + 3);
        }

        updateCount_ += 1;
    }

    FloatType idev() const { return idev_; }
    FloatType offset() const { return offset_; }
    FloatType deviation() const { return DEVIATION / idev_; }
    FloatType error() const { return evm(); }
};


}} // mobilinkd::m17
