// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.


#ifndef MOBILINKD_TNC_USBPORT_H_
#define MOBILINKD_TNC_USBPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

void cdc_receive(const uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // TNC_USBPORT_H_
