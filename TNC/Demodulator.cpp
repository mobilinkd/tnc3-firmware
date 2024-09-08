// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Demodulator.hpp"
#include "power.h"

namespace mobilinkd { namespace tnc {

/**
 * Start the ADC DMA transfer.  The block size is equal to the number of
 * 32-bit elements in the buffer.  This is also equal to the number of
 * 16-bit elements in each DMA "half complete" transfer.  Each DMA transfer
 * results in block_size * 2 bytes being transferred.
 *
 * We must bear in mind that the DMA buffer size is expressed in DWORDs,
 * the DMA transfer size is expressed in WORDs, and the memory copy operation
 * in BYTEs.
 *
 * @param period
 * @param block_size
 */
void IDemodulator::startADC(uint32_t period, uint32_t block_size)
{
    HAL_StatusTypeDef status;

    audio::set_adc_block_size(block_size);

#ifndef TNC_HAS_ADC2
    stop_power_monitor();
#endif

    __HAL_TIM_SET_PRESCALER(&htim6, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim6, period);

    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_ADC_Start_DMA(&DEMODULATOR_ADC_HANDLE, audio::adc_buffer, audio::dma_transfer_size);
    if (status != HAL_OK) CxxErrorHandler2(status);
}

void IDemodulator::stopADC()
{
    HAL_StatusTypeDef status;

    status = HAL_ADC_Stop_DMA(&DEMODULATOR_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_TIM_Base_Stop(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

#ifndef TNC_HAS_ADC2
    start_power_monitor();
#endif
}


}} // mobilinkd::tnc
