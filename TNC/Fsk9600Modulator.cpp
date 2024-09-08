// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Fsk9600Modulator.hpp"

namespace mobilinkd { namespace tnc {

void Fsk9600Modulator::init(const kiss::Hardware& hw)
{
    for (auto& x : buffer_) x = 2048;

    UNUSED(hw);

    state = State::STOPPED;

    // Configure 72MHz clock for 96ksps.
    
    __HAL_TIM_SET_AUTORELOAD(&htim7, 749);
    __HAL_TIM_SET_PRESCALER(&htim7, 0);

    DAC_ChannelConfTypeDef sConfig;

    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
      CxxErrorHandler();
    }

    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048) != HAL_OK) CxxErrorHandler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) CxxErrorHandler();
    INFO("Fsk9600Modulator::init");
}

void Fsk9600Modulator::fill(uint16_t* buffer, uint8_t bits)
{
    HAL_IWDG_Refresh(&hiwdg);

    for (uint8_t i = 0; i != BLOCKSIZE; ++i) {
        symbols[i] = (bits & 0x80 ? UPSAMPLE : -UPSAMPLE);
        bits <<= 1;
    }

    arm_fir_interpolate_f32(&fir_interpolator, symbols.data(), tmp, BLOCKSIZE);

    for (uint8_t i = 0; i != TRANSFER_LEN; ++i)
    {
        buffer[i] = adjust_level(tmp[i]);
    }
}

}} // mobilinkd::tnc
