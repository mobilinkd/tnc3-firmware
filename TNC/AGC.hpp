// Copyright 2018-2019 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <cmath>
#include <array>
#include <numeric>
#include <algorithm>

namespace mobilinkd { namespace tnc {

template <typename T>
struct BaseAGC {

	typedef T float_type;
	float_type attack_;
	float_type decay_;
	float_type reference_;
	float_type max_gain_;
	bool has_max_gain_;
	
	float_type gain_;
	
	BaseAGC(float_type attack, float_type decay, float_type reference = 1.0)
	: attack_(attack), decay_(decay), reference_(reference)
	, max_gain_(0.0), has_max_gain_(false), gain_(1.0)
	{}

	BaseAGC(float_type decay, float_type attack, float_type reference, float_type max_gain)
	: attack_(attack), decay_(decay), reference_(reference)
	, max_gain_(max_gain), has_max_gain_(true), gain_(1.0)
	{}
	
	float_type operator()(float_type value) {
		
		float_type output = value * gain_;
		float_type tmp = fabs(output) - reference_;
		float_type rate = decay_;
		
		if (fabs(tmp) > gain_) {
			rate = attack_;
		}

		gain_ -= tmp * rate;

		if (gain_ < 0.0f) {
			gain_ = .000001f;
		}

		if (has_max_gain_ and (gain_ > max_gain_)) {
			gain_ = max_gain_;
		}

		return output;
	}
};

typedef BaseAGC<double> AGC;
typedef BaseAGC<float> FastAGC;

inline uint32_t log_2(const uint32_t x)
{
    if (x == 0) return 0;
    return (31 - __builtin_clz (x));
}

/**
 * Feed-forward AGC implementation.  It assumes symbol width is an integer
 * multiple of samples and that the audio type is a int16_t.  It
 * assumes that it will operate on blocks of data that are integer multiples
 * of the symbol block_size.
 *
 * The delay is two blocks of data.
 *
 * @tparam SymbolSize is the symbol width in samples.
 * @tparam BlockSize is the block_size of the block in symbols.
 * @tparam Reference is the target reference value.
 */
template <size_t SymbolSize, size_t BlockSize, int16_t Reference = 8192>
class FeedForwardAGC
{
    using buffer_type = std::array<int16_t, SymbolSize * BlockSize * 2>;  // delay line.
    using block_type = std::array<int16_t, SymbolSize * BlockSize>;   // I/O block block_size.

    // 256 -> 512
    static constexpr std::array<int16_t, SymbolSize> init_increment()
    {
        std::array<int16_t, SymbolSize> result{};
        for (size_t i = 0; i != SymbolSize; ++i)
        {
            result[i] = 256  + ((256 * (i + 1)) / SymbolSize);
        }
        return result;
    }

    // 256 -> 128
    static constexpr std::array<int16_t, SymbolSize> init_decrement()
    {
        std::array<int16_t, SymbolSize> result{};
        for (size_t i = SymbolSize; i != 0; ++i)
        {
            result[i] = 128  + ((128 * (i - 1)) / SymbolSize);
        }
        return result;
    }

    static constexpr std::array<int16_t, SymbolSize> increment{init_increment()};
    static constexpr std::array<int16_t, SymbolSize> decrement{init_decrement()};

    buffer_type buffer{0};
    block_type block{0};
    block_type ibuffer{0};
    size_t idx{0};
    size_t index{0};
    bool full{false};
    int16_t gain{1};
    const size_t buffer_size{SymbolSize * BlockSize * 2};
    const size_t block_size{SymbolSize * BlockSize};


    int16_t amplitude(size_t idx) const
    {
        int16_t result = 0;
        for (size_t i = 0; i != SymbolSize; ++i)
        {
            int16_t y = std::abs(buffer[idx + i]);
            result = std::max(y, result);
        }
        return result;
    }

    /**
     * Compute max_amplitude for each symbol width window.  Use average of
     * top 4 to set gain target.
     *
     * @return
     */
    int16_t compute_target_gain() const
    {
        std::array<int16_t, 4> top = {0,0,0,0};
        for (size_t i = 0; i != buffer_size; i += SymbolSize)
        {
            auto amp = amplitude(i);
            if (amp > top[0]) {
                top[3] = top[2];
                top[2] = top[1];
                top[1] = top[0];
                top[0] = amp;
            } else if (amp > top[1]) {
                top[3] = top[2];
                top[2] = top[1];
                top[1] = amp;
            } else if (amp > top[2]) {
                top[3] = top[2];
                top[2] = amp;
            } else if (amp > top[3]) {
                top[3] = amp;
            }
        }
        int32_t sum = top[0] + top[1] + top[2] + top[3];
        int32_t avg = sum >> 2;
        return std::max(int16_t(Reference >> log_2(avg)), int16_t(1));
    }

    /**
     * Apply gain to next part of buffer. @p index points to the next
     * block to be returned.  Gain is changed gradually, by 6dB per symbol.
     *
     * @param target
     */
    void apply_gain(int16_t target)
    {
        auto current_gain = gain;
        for (size_t sym = 0; sym < block_size; sym += SymbolSize)
        {
            for (size_t i = 0; i != SymbolSize; ++i)
            {
                int32_t g = 0;
                if (current_gain < target) {
                    g = current_gain * increment[i];
                } else if (current_gain > target) {
                    g = current_gain * increment[i];
                } else {
                    g = current_gain << 8;
                }
                int32_t x = buffer[index + sym + i];
                x = (x * g) >> 8;
                if (x > 32767) x = 32767;
                else if (x < -32768) x = -32768;
                buffer[index + sym + i] = x;
            }
            if (current_gain < target) {
                current_gain *= 2;
            } else if (current_gain > target) {
                current_gain /= 2;
            }
        }
        gain = current_gain;
    }

public:

    int16_t* operator()(int16_t* input)
    {
        for (size_t i = 0; i != block_size; ++i) {
            block[i] = buffer[index];
            buffer[index++] = input[i];
            if (index == buffer_size) index = 0;
        }

        const auto target_gain = compute_target_gain();

        apply_gain(target_gain);

        return block.begin();
    }

    int16_t operator()(int16_t input)
    {
        int16_t result = block[idx];
        ibuffer[idx++] = input;
        if (idx == block_size) {
            (*this)(ibuffer.begin());
            idx = 0;
        }
        return result;
    }
};

}} // mobilinkd::tnc


