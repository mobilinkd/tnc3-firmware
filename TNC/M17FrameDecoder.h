// Copyright 2020 Mobilinkd LLC.

#pragma once

#include "M17Randomizer.h"
#include "PolynomialInterleaver.h"
#include "Trellis.h"
#include "Viterbi.h"
#include "CRC16.h"
#include "LinkSetupFrame.h"
#include "HdlcFrame.hpp"
#include "Golay24.h"

#include <algorithm>
#include <array>
#include <functional>

namespace mobilinkd
{

namespace detail
{

template <typename T, size_t N>
tnc::hdlc::IoFrame* to_frame(uint8_t frame_type, std::array<T, N> in)
{
    auto frame = tnc::hdlc::acquire_wait();
    frame->push_back(frame_type);

    uint8_t out = 0;
    size_t b = 0;

    for (auto c : in)
    {
        out |= (c << (7 - b));
        if (++b == 8)
        {
            frame->push_back(out);
            out = 0;
            b = 0;
        }
    }
    return frame;
}

} // detail

struct M17FrameDecoder
{
    M17Randomizer<368> derandomize_;
    PolynomialInterleaver<45, 92, 368> interleaver_;
    Trellis<4,2> trellis_{makeTrellis<4, 2>({031,027})};
    Viterbi<decltype(trellis_), 4> viterbi_{trellis_};
    CRC16<0x5935, 0xFFFF> crc_;

    enum class State {LS_FRAME, LS_LICH, AUDIO};

    State state_ = State::LS_FRAME;

    using buffer_t = std::array<int8_t, 368>;

    using lsf_conv_buffer_t = std::array<uint8_t, 46>;
    using lsf_buffer_t = std::array<uint8_t, 30>;

    using audio_conv_buffer_t = std::array<uint8_t, 34>;
    using audio_buffer_t = std::array<uint8_t, 20>;

    using link_setup_callback_t = std::function<void(audio_buffer_t)>;
    using audio_callback_t = std::function<void(audio_buffer_t)>;
    
    link_setup_callback_t link_setup_callback_;
    audio_callback_t audio_callback_;
    std::array<uint8_t, 240> lsf_output;
    std::array<uint8_t, 160> audio_output;
    std::array<int8_t, 272> tmp_buffer;
    uint8_t lich_segments{0};       ///< one bit per received LICH fragment.


    M17FrameDecoder(
        link_setup_callback_t link_setup_callback = link_setup_callback_t(),
        audio_callback_t audio_callback = audio_callback_t()
    )
    : link_setup_callback_(link_setup_callback)
    , audio_callback_(audio_callback)
    {}

    ~M17FrameDecoder()
    {}

    void reset() { state_ = State::LS_FRAME; }

    bool decode_lsf(buffer_t& buffer, tnc::hdlc::IoFrame*& lsf, int& ber)
    {
        auto dp = depunctured<488>(P1, buffer);
        ber = viterbi_.decode(dp, lsf_output);
        ber = ber > 60 ? ber - 60 : 0;
        lsf = detail::to_frame(0, lsf_output);
        crc_.reset();
        auto it = lsf->begin();
        ++it;
        for (; it != lsf->end(); ++it) crc_(*it);
        auto checksum = crc_.get();
        INFO("crc = %04x", checksum);
        if (checksum != 0)
        {
            state_ = State::LS_LICH;
            lich_segments = 0;
            lsf_output.fill(0);
            return false;
        }

        state_ = State::AUDIO;
        lsf->push_back(0);  // last two bytes are skipped by hdlc frame.
        lsf->push_back(0);
        return true;
    }

    // Unpack  & decode LICH fragments into tmp_buffer.
    bool unpack_lich(buffer_t& buffer)
    {
        size_t index = 0;
        // Read the 4 24-bit codewords from LICH
        for (size_t i = 0; i != 4; ++i) // for each codeword
        {
            uint32_t codeword = 0;
            for (size_t j = 0; j != 24; ++j) // for each bit in codeword
            {
                codeword <<= 1;
                codeword |= (buffer[i * 24 + j] > 0);
            }
            uint32_t decoded = 0;
            if (!Golay24::decode(codeword, decoded))
            {
                INFO("Golay decode failed for %08lx (%du)", codeword, i);
                return false;
            }
            decoded >>= 12; // Remove check bits and parity.
            INFO("Golay decode good for %08lx (%du)", decoded, i);
            // append codeword.
            if (i & 1)
            {
                tmp_buffer[index++] |= (decoded >> 8);     // upper 4 bits
                tmp_buffer[index++] = (decoded & 0xFF);    // lower 8 bits
            }
            else
            {
                tmp_buffer[index++] |= (decoded >> 4);     // upper 8 bits
                tmp_buffer[index] = (decoded & 0x0F) << 4; // lower 4 bits
            }
        }
        return true;
    }

    bool decode_lich(buffer_t& buffer, tnc::hdlc::IoFrame*& lsf, int& ber)
    {
        tmp_buffer.fill(0);
        // Read the 4 12-bit codewords from LICH into tmp_buffer.
        if (!unpack_lich(buffer)) return false;

        uint8_t fragment_number = tmp_buffer[5];   // Get fragment number.
        fragment_number >>= 5;

        // Copy decoded LICH to superframe buffer.
        std::copy(tmp_buffer.begin(), tmp_buffer.begin() + 5,
            lsf_output.begin() + (fragment_number * 5));

        lich_segments |= (1 << fragment_number);        // Indicate segment received.
        INFO("got segment %d, have %02x", int(fragment_number), int(lich_segments));
        if (lich_segments != 0x3F) return false;        // More to go...

        crc_.reset();
        auto it = lsf_output.begin();
        ++it;
        for (auto it = lsf_output.begin(); it != lsf_output.end(); ++it)
            crc_(*it);
        auto checksum = crc_.get();
        INFO("LICH crc = %04x", checksum);
        if (checksum == 0)
        {
            state_ = State::AUDIO;
            lsf = tnc::hdlc::acquire_wait();
            lsf->push_back(0);
            for (auto it = lsf_output.begin(); it != lsf_output.end(); ++it)
                lsf->push_back(*it);
            return true;
        }
        // Failed CRC... try again.
        lich_segments = 0;
        lsf_output.fill(0);
        return false;
    }

    bool decode_audio(buffer_t& buffer, tnc::hdlc::IoFrame*& audio, int& ber)
    {
        std::copy(buffer.begin() + 96, buffer.end(), tmp_buffer.begin());
        auto dp = depunctured<328>(P2, tmp_buffer);
        ber = viterbi_.decode(dp, audio_output);
        ber = ber > 28 ? ber - 28 : 0;
        audio = detail::to_frame(1, audio_output);
        crc_.reset();
        auto it = audio->begin();
        ++it;
        for (; it != audio->end(); ++it) crc_(*it);
        auto checksum = crc_.get();
        INFO("crc = %04x", checksum);
        audio->push_back(0);
        audio->push_back(0);
        return true;
    }

    bool operator()(buffer_t& buffer, tnc::hdlc::IoFrame*& result, int& ber)
    {
        derandomize_(buffer);
        interleaver_.deinterleave(buffer);

        switch(state_)
        {
        case State::LS_FRAME:
            return decode_lsf(buffer, result, ber);
        case State::LS_LICH:
            return decode_lich(buffer, result, ber);
        case State::AUDIO:
            return decode_audio(buffer, result, ber);
        }
        return false;
    }
};

} // mobilinkd
