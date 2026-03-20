/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_device.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_device_descriptors.h"
#include "ux_device_class_storage.h"
#include "ux_dcd_stm32.h"
#include "usb_otg.h"
#include "ux_device_msc.h"
#include "lx_nor_w25q128_driver.h"
#include "sdio.h"
#include "app_filex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Storage class parameter: defines both LUNs */
static UX_SLAVE_CLASS_STORAGE_PARAMETER storage_parameter;
/* USB hardware init thread – runs after the scheduler starts so that
   HAL_Delay() (needed by USB_SetCurrentMode) works correctly.  The
   tx_initialize_low_level.s disables interrupts (CPSID i) for the
   entire tx_application_define() call, so TIM4 can never tick there. */
static TX_THREAD  usb_hw_init_thread;
static UCHAR      usb_hw_init_thread_stack[2048];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void usb_hw_init_thread_entry(ULONG arg);
/* USER CODE END PFP */
/**
  * @brief  Application USBX Device Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Device_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Device_Init */
  UCHAR *pointer;
  ULONG  descriptors_len;
  ULONG  string_framework_len;
  ULONG  lang_id_framework_len;

  /* Allocate memory for the USBX system from the byte pool.
     Leave ~4 KB headroom for pool overhead.                     */
#define USBX_MEMORY_SIZE  (60U * 1024U)

  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       USBX_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    return TX_POOL_ERROR;
  /* USER CODE END MX_USBX_Device_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Device_Init */

  /* ---- 1. Initialize the USBX system ---------------------------------- */
  ret = ux_system_initialize(pointer, USBX_MEMORY_SIZE, UX_NULL, 0);
  if (ret != UX_SUCCESS)
    return ret;

  /* ---- 2. Build device framework descriptors --------------------------
     Each builder MUST be called exactly once before ux_device_stack_initialize
     because the builders share static state (USBD_Device_FS.classId etc.).
     Calling the same builder twice in a single argument list has undefined
     evaluation order in C and corrupts the descriptor or passes length=0. */
  uint8_t *fs_desc  = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED, &descriptors_len);
  ULONG    fs_len   = descriptors_len;

  uint8_t *str_fw   = USBD_Get_String_Framework(&string_framework_len);
  ULONG    str_len  = string_framework_len;

  uint8_t *lang_fw  = USBD_Get_Language_Id_Framework(&lang_id_framework_len);
  ULONG    lang_len = lang_id_framework_len;

  /* Initialize the USBX device stack.
     This is an FS-only device; pass the same FS descriptor for the HS slot. */
  ret = ux_device_stack_initialize(
          fs_desc, fs_len,   /* HS slot  – reuse FS descriptor */
          fs_desc, fs_len,   /* FS slot                        */
          str_fw,  str_len,
          lang_fw, lang_len,
          UX_NULL);
  if (ret != UX_SUCCESS)
    return ret;

  /* ---- 3. Retrieve SD card geometry once ------------------------------ */
  HAL_SD_CardInfoTypeDef card_info = {0};
  HAL_SD_GetCardInfo(&hsd, &card_info);

  /* ---- 4. Configure the MSC storage class with 2 LUNs ---------------- */
  storage_parameter.ux_slave_class_storage_parameter_number_lun = 2;

  /* --- LUN 0: W25Q128 NOR Flash via LevelX ----------------------------- */
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_last_lba        = W25Q128_LX_USABLE_SECTORS - 1;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_block_length    = 512;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_type            = UX_SLAVE_CLASS_STORAGE_MEDIA_FAT_DISK;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_removable_flag  = 0x80;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_read_only_flag  = 0;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_read            = ux_device_msc_media_read_lun0;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_write           = ux_device_msc_media_write_lun0;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_flush           = ux_device_msc_media_flush_lun0;
  storage_parameter.ux_slave_class_storage_parameter_lun[0]
      .ux_slave_class_storage_media_status          = ux_device_msc_media_status_lun0;

  /* --- LUN 1: SD Card via SDIO HAL ------------------------------------- */
  /* card_info.LogBlockNbr = logical 512-byte blocks accessible on the card */
  ULONG sd_last_lba = (card_info.LogBlockNbr > 0) ? (card_info.LogBlockNbr - 1) : 0;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_last_lba        = sd_last_lba;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_block_length    = 512;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_type            = UX_SLAVE_CLASS_STORAGE_MEDIA_FAT_DISK;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_removable_flag  = 0x80;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_read_only_flag  = 0;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_read            = ux_device_msc_media_read_lun1;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_write           = ux_device_msc_media_write_lun1;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_flush           = ux_device_msc_media_flush_lun1;
  storage_parameter.ux_slave_class_storage_parameter_lun[1]
      .ux_slave_class_storage_media_status          = ux_device_msc_media_status_lun1;

  /* ---- 5. Register the MSC storage class with the device stack -------- */
  ret = ux_device_stack_class_register(
          _ux_system_slave_class_storage_name,
          ux_device_class_storage_entry,
          1, 0,                               /* config index, interface num */
          (VOID *)&storage_parameter);
  if (ret != UX_SUCCESS)
    return ret;

  /* ---- 6. Start USB hardware from a dedicated thread -----------------
     MX_USB_OTG_FS_PCD_Init() calls USB_SetCurrentMode() which calls
     HAL_Delay().  HAL_Delay() requires TIM4 to tick.  However,
     tx_initialize_low_level.s does CPSID i (global interrupt disable)
     before calling tx_application_define(), so TIM4 can never fire
     here and HAL_Delay() hangs forever.  Deferring PCD init to a real
     ThreadX thread solves this: the first context switch re-enables
     interrupts via CPSIE i in the thread restore sequence.            */
  ret = (UINT)tx_thread_create(&usb_hw_init_thread,
                               "USB HW Init",
                               usb_hw_init_thread_entry,
                               0,
                               usb_hw_init_thread_stack,
                               sizeof(usb_hw_init_thread_stack),
                               5, 5,
                               TX_NO_TIME_SLICE,
                               TX_AUTO_START);
  if (ret != TX_SUCCESS)
    return ret;

  /* USER CODE END MX_USBX_Device_Init */

  return UX_SUCCESS;
}

/* USER CODE BEGIN 1 */
static void usb_hw_init_thread_entry(ULONG arg)
{
    (VOID)arg;
    /* Interrupts are enabled now (re-enabled by the first context switch).
       MX_USB_OTG_FS_PCD_Init() -> USB_SetCurrentMode() -> HAL_Delay() works.
       ux_system_initialize() and ux_device_stack_initialize() were already
       called in tx_application_define(), so _ux_system_slave is valid.    */

    /* 1. Init USB OTG FS hardware (GPIO, clock, FIFO, NVIC, PCD). */
    MX_USB_OTG_FS_PCD_Init();

    /* 2. Link USBX to the HAL PCD handle. */
    _ux_dcd_stm32_initialize(0, (ULONG)&hpcd_USB_OTG_FS);

    /* 3. Wait for FileX to finish formatting/verifying all filesystems
       so that Windows sees valid FAT volumes from the first read.   */
    while (!fx_filesystems_ready)
        tx_thread_sleep(10);

    /* 4. Start USB enumeration – media is now ready on all LUNs. */
    HAL_PCD_Start(&hpcd_USB_OTG_FS);

    /* Thread has completed its one-shot init; suspend forever            */
    tx_thread_suspend(tx_thread_identify());
}
/* USER CODE END 1 */
