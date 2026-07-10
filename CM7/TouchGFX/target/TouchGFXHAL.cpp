/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : TouchGFXHAL.cpp
  ******************************************************************************
  * This file was created by TouchGFX Generator 4.26.1. This file is only
  * generated once! Delete this file from your project and re-generate code
  * using STM32CubeMX or change this file manually to update it.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <TouchGFXHAL.hpp>
#include <touchgfx/hal/OSWrappers.hpp>

extern "C" {
#include "main.h"
#include "cmsis_os2.h"
}

/**
 * Specifies the width of the frame buffer stride.
 * This number should be the result of your display width rounded-up to the closest multiple of 64.
 */
static const uint16_t framebufferStride = 480;

/* USER CODE BEGIN TouchGFXHAL.cpp */

using namespace touchgfx;

extern "C" void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc_local)
{
    /* HAL_LTDC_IRQHandler clears LTDC_IT_LI after every line interrupt, so
     * without re-arming here the LIE bit stays low after the first frame
     * and we never get another VSYNC. The result is exactly one rendered
     * frame at boot and a frozen TouchGFX tick chain (Model::tick and
     * Screen::handleTickEvent stop firing), which means no input is ever
     * applied to the menu. Re-enable LIE on every interrupt to keep the
     * VSYNC stream alive. */
    __HAL_LTDC_ENABLE_IT(hltdc_local, LTDC_IT_LI);

    /* The CMSIS-RTOS2 OSWrappers signal VSYNC through a FreeRTOS queue.
     * Calling a *FromISR API before osKernelStart() trips the port's
     * configASSERT in vPortValidateInterruptPriority() (ulMaxPRIGROUPValue
     * is only computed inside xPortStartScheduler), which hangs the CPU
     * inside this ISR during boot. VSYNC is meaningless until the GUI task
     * runs, so drop it while the kernel is not yet running. */
    if (osKernelGetState() == osKernelRunning)
    {
        OSWrappers::signalVSync();
    }
}

void TouchGFXHAL::initialize()
{
    TouchGFXGeneratedHAL::initialize();
}

uint16_t* TouchGFXHAL::getTFTFrameBuffer() const
{
    return TouchGFXGeneratedHAL::getTFTFrameBuffer();
}

void TouchGFXHAL::setTFTFrameBuffer(uint16_t* address)
{
    TouchGFXGeneratedHAL::setTFTFrameBuffer(address);
}

void TouchGFXHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    TouchGFXGeneratedHAL::flushFrameBuffer(rect);
}

bool TouchGFXHAL::blockCopy(void* RESTRICT dest, const void* RESTRICT src, uint32_t numBytes)
{
    return TouchGFXGeneratedHAL::blockCopy(dest, src, numBytes);
}

void TouchGFXHAL::configureInterrupts()
{
    TouchGFXGeneratedHAL::configureInterrupts();
}

void TouchGFXHAL::enableInterrupts()
{
    TouchGFXGeneratedHAL::enableInterrupts();
}

void TouchGFXHAL::disableInterrupts()
{
    TouchGFXGeneratedHAL::disableInterrupts();
}

void TouchGFXHAL::enableLCDControllerInterrupt()
{
    TouchGFXGeneratedHAL::enableLCDControllerInterrupt();
}

bool TouchGFXHAL::beginFrame()
{
    return TouchGFXGeneratedHAL::beginFrame();
}

void TouchGFXHAL::endFrame()
{
    TouchGFXGeneratedHAL::endFrame();
}

/* USER CODE END TouchGFXHAL.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
