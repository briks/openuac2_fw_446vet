/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"
#include "log.h"

USBD_HandleTypeDef hUsbDeviceHS;

void MX_USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK)
  {
    LOG_ERR("USBD_Init failed");
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_AUDIO) != USBD_OK)
  {
    LOG_ERR("USBD_RegisterClass failed");
    Error_Handler();
  }
  if (USBD_AUDIO_RegisterInterface(&hUsbDeviceHS, &USBD_AUDIO_fops) != USBD_OK)
  {
    LOG_ERR("USBD_AUDIO_RegisterInterface failed");
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    LOG_ERR("USBD_Start failed");
    Error_Handler();
  }
}

