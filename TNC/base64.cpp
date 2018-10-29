// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "base64.h"
#include "assert.h"

namespace {
const char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

uint32_t base64encode(const uint8_t* src, uint32_t src_len, char* dest,
    uint32_t* dest_len)
{
    uint32_t expected = ((src_len * 4) + 2) / 3;
    uint32_t dpos = 0;
    uint8_t b1, b2, b3, b4;
    for (uint32_t i = 0; i < src_len; i += 3)
    {
        if (dpos == *dest_len) return expected;
        b1 = src[i] >> 2;
        b2 = ((src[i] & 0x3) << 4);
        dest[dpos++] = alphabet[b1];

        if (dpos == *dest_len) return expected;
        if (i + 1 == src_len)
        {
            dest[dpos++] = alphabet[b2];
            break;
        }
        else
        {
            b2 |=  (src[i + 1] >> 4);
            dest[dpos++] = alphabet[b2];
            b3 = (src[i + 1] & 0x0F) << 2;
        }

        if (dpos == *dest_len) return expected;
        if (i + 2 == src_len)
        {
            dest[dpos++] = alphabet[b3];
            break;
        }
        else
        {
            b3 |= src[i + 2] >> 6;
            dest[dpos++] = alphabet[b3];
            b4 = src[i + 2] & 0x3F;
        }

        if (dpos == *dest_len) return expected;
        dest[dpos++] = alphabet[b4];
    }
    *dest_len = dpos;
    assert(*dest_len == expected);
    return expected;
}
