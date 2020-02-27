// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Fsk9600Modulator.hpp"

namespace mobilinkd { namespace tnc {

/*
const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
    2047,  2020,  1937,  1801,  1616,  1387,  1120,   822,   502,   169,
    -169,  -502,  -822, -1120, -1387, -1616, -1801, -1937, -2020, -2048
};
*/

const Fsk9600Modulator::cos_table_type Fsk9600Modulator::cos_table = {
     2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,  2047,
    -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048, -2048
};

void Fsk9600Modulator::init(const kiss::Hardware& hw)
{
    for (auto& x : buffer_) x = 2048;

    (void) hw; // unused

    state = State::STOPPED;
    level = Level::HIGH;

    if (HAL_RCC_GetHCLKFreq() != 80000000)
    {
        ERROR("Clock is not 80MHz");
        CxxErrorHandler();
    }

    // Configure 80MHz clock for 192ksps.
    htim7.Init.Period = 416;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        ERROR("htim7 init failed");
        CxxErrorHandler();
    }

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
}


}} // mobilinkd::tnc
