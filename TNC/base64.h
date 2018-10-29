// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__BASE64_H_
#define MOBILINKD__TNC__BASE64_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Base64 encode binary data from src, storing the result in dest.  The
 * input value of dest_len limits the length of the string which will be
 * written to dest.  If this value is not >= 4/3 * src_len, the output
 * string will be truncated.  The amount written will be returned in
 * dest_len (in this case in==out) and the amount that would have been
 * written is returned.
 *
 * If the return value and the value returned in dest_len are different,
 * dest did not contain enough space to fully encode the result.
 *
 * @param src is the binary data to be base64 encoded.
 * @param src_len is the length of the binary data to encode.
 * @param dest is the destination buffer to contain the encoded string.
 * @param[in,out] dest_len on input is the size of the dest buffer and on
 *  output is the number of characters written to dest.
 * @return the length of src fully base64 encoded.
 */
uint32_t base64encode(const uint8_t* src, uint32_t src_len, char* dest,
    uint32_t* dest_len);

#ifdef __cplusplus
}
#endif

#endif // MOBILINKD__TNC__BASE64_H_
