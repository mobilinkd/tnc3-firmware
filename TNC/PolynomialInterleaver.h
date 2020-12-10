// Copyright 2020 Mobilinkd LLC.

#pragma once

#include <algorithm>
#include <array>

namespace mobilinkd
{

template <size_t F1= 45, size_t F2 = 92, size_t K= 368>
struct PolynomialInterleaver
{
    using buffer_t = std::array<int8_t, K>;

    alignas(16) buffer_t buffer_;

    size_t index(size_t i)
    {
        return ((F1 * i) + (F2 * i * i)) % K;
    }
    
    void interleave(buffer_t& data)
    {
        buffer_.fill(0);

        for (size_t i = 0; i != K; ++i)
            buffer_[index(i)] = data[i];
        
        std::copy(std::begin(buffer_), std::end(buffer_), std::begin(data));
    }

    void deinterleave(buffer_t& frame)
    {
        buffer_.fill(0);

        for (size_t i = 0; i != K; ++i)
        {
            auto idx = index(i);
            buffer_[i] = frame[idx];
        }
        
        std::copy(buffer_.begin(), buffer_.end(), frame.begin());
    }
};

} // mobilinkd
