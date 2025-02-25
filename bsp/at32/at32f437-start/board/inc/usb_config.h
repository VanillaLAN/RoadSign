/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_CONFIG_H
#define USB_CONFIG_H

/* ================ USB common Configuration ================ */

#include "rtthread.h"

#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* data align size when use dma */
#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 in and out transfer buffer */
#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

/* Setup packet log for debug */
// #define CONFIG_USBDEV_SETUP_LOG_PRINT

/* Send ep0 in data from user buffer instead of copying into ep0 reqdata
 * Please note that user buffer must be aligned with CONFIG_USB_ALIGN_SIZE
*/
// #define CONFIG_USBDEV_EP0_INDATA_NO_COPY

/* Check if the input descriptor is correct */
// #define CONFIG_USBDEV_DESC_CHECK

/* Enable test mode */
// #define CONFIG_USBDEV_TEST_MODE

#ifndef CONFIG_USBDEV_MSC_MAX_LUN
#define CONFIG_USBDEV_MSC_MAX_LUN 1
#endif

#ifndef CONFIG_USBDEV_MSC_MAX_BUFSIZE
#define CONFIG_USBDEV_MSC_MAX_BUFSIZE 512
#endif

#ifndef CONFIG_USBDEV_MSC_MANUFACTURER_STRING
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_PRODUCT_STRING
#define CONFIG_USBDEV_MSC_PRODUCT_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_VERSION_STRING
#define CONFIG_USBDEV_MSC_VERSION_STRING "0.01"
#endif

/* move msc read & write from isr to while(1), you should call usbd_msc_polling in while(1) */
// #define CONFIG_USBDEV_MSC_POLLING

/* move msc read & write from isr to thread */
// #define CONFIG_USBDEV_MSC_THREAD

#ifndef CONFIG_USBDEV_MSC_PRIO
#define CONFIG_USBDEV_MSC_PRIO 4
#endif

#ifndef CONFIG_USBDEV_MSC_STACKSIZE
#define CONFIG_USBDEV_MSC_STACKSIZE 2048
#endif

#ifndef CONFIG_USBDEV_RNDIS_RESP_BUFFER_SIZE
#define CONFIG_USBDEV_RNDIS_RESP_BUFFER_SIZE 156
#endif

/* rndis transfer buffer size, must be a multiple of (1536 + 44)*/
#ifndef CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE
#define CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE 1580
#endif

#ifndef CONFIG_USBDEV_RNDIS_VENDOR_ID
#define CONFIG_USBDEV_RNDIS_VENDOR_ID 0x0000ffff
#endif

#ifndef CONFIG_USBDEV_RNDIS_VENDOR_DESC
#define CONFIG_USBDEV_RNDIS_VENDOR_DESC "CherryUSB"
#endif

#define CONFIG_USBDEV_RNDIS_USING_LWIP

/* ================ USB HOST Stack Configuration ================== */

#define CONFIG_USBHOST_MAX_RHPORTS          1
#define CONFIG_USBHOST_MAX_EXTHUBS          1
#define CONFIG_USBHOST_MAX_EHPORTS          4
#define CONFIG_USBHOST_MAX_INTERFACES       8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 8
#define CONFIG_USBHOST_MAX_ENDPOINTS        4

#define CONFIG_USBHOST_MAX_CDC_ACM_CLASS 4
#define CONFIG_USBHOST_MAX_HID_CLASS     4
#define CONFIG_USBHOST_MAX_MSC_CLASS     2
#define CONFIG_USBHOST_MAX_AUDIO_CLASS   1
#define CONFIG_USBHOST_MAX_VIDEO_CLASS   1

#define CONFIG_USBHOST_DEV_NAMELEN 16

#ifndef CONFIG_USBHOST_PSC_PRIO
#define CONFIG_USBHOST_PSC_PRIO 0
#endif
#ifndef CONFIG_USBHOST_PSC_STACKSIZE
#define CONFIG_USBHOST_PSC_STACKSIZE 2048
#endif

//#define CONFIG_USBHOST_GET_STRING_DESC

// #define CONFIG_USBHOST_MSOS_ENABLE
#ifndef CONFIG_USBHOST_MSOS_VENDOR_CODE
#define CONFIG_USBHOST_MSOS_VENDOR_CODE 0x00
#endif

/* Ep0 max transfer buffer */
#ifndef CONFIG_USBHOST_REQUEST_BUFFER_LEN
#define CONFIG_USBHOST_REQUEST_BUFFER_LEN 512
#endif

#ifndef CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT
#define CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT 5000//500
#endif

#ifndef CONFIG_USBHOST_MSC_TIMEOUT
#define CONFIG_USBHOST_MSC_TIMEOUT 5000
#endif

/* This parameter affects usb performance, and depends on (TCP_WND)tcp eceive windows size,
 * you can change to 2K ~ 16K and must be larger than TCP RX windows size in order to avoid being overflow.
 */
#ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE
#define CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE (2048)
#endif

/* Because lwip do not support multi pbuf at a time, so increasing this variable has no performance improvement */
#ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE
#define CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE (2048)
#endif

/* This parameter affects usb performance, and depends on (TCP_WND)tcp eceive windows size,
 * you can change to 2K ~ 16K and must be larger than TCP RX windows size in order to avoid being overflow.
 */
#ifndef CONFIG_USBHOST_CDC_NCM_ETH_MAX_RX_SIZE
#define CONFIG_USBHOST_CDC_NCM_ETH_MAX_RX_SIZE (2048)
#endif
/* Because lwip do not support multi pbuf at a time, so increasing this variable has no performance improvement */
#ifndef CONFIG_USBHOST_CDC_NCM_ETH_MAX_TX_SIZE
#define CONFIG_USBHOST_CDC_NCM_ETH_MAX_TX_SIZE (2048)
#endif

/* This parameter affects usb performance, and depends on (TCP_WND)tcp eceive windows size,
 * you can change to 2K ~ 16K and must be larger than TCP RX windows size in order to avoid being overflow.
 */
#ifndef CONFIG_USBHOST_ASIX_ETH_MAX_RX_SIZE
#define CONFIG_USBHOST_ASIX_ETH_MAX_RX_SIZE (2048)
#endif
/* Because lwip do not support multi pbuf at a time, so increasing this variable has no performance improvement */
#ifndef CONFIG_USBHOST_ASIX_ETH_MAX_TX_SIZE
#define CONFIG_USBHOST_ASIX_ETH_MAX_TX_SIZE (2048)
#endif

/* This parameter affects usb performance, and depends on (TCP_WND)tcp eceive windows size,
 * you can change to 2K ~ 16K and must be larger than TCP RX windows size in order to avoid being overflow.
 */
#ifndef CONFIG_USBHOST_RTL8152_ETH_MAX_RX_SIZE
#define CONFIG_USBHOST_RTL8152_ETH_MAX_RX_SIZE (2048)
#endif
/* Because lwip do not support multi pbuf at a time, so increasing this variable has no performance improvement */
#ifndef CONFIG_USBHOST_RTL8152_ETH_MAX_TX_SIZE
#define CONFIG_USBHOST_RTL8152_ETH_MAX_TX_SIZE (2048)
#endif

#define CONFIG_USBHOST_BLUETOOTH_HCI_H4
// #define CONFIG_USBHOST_BLUETOOTH_HCI_LOG

#ifndef CONFIG_USBHOST_BLUETOOTH_TX_SIZE
#define CONFIG_USBHOST_BLUETOOTH_TX_SIZE 2048
#endif
#ifndef CONFIG_USBHOST_BLUETOOTH_RX_SIZE
#define CONFIG_USBHOST_BLUETOOTH_RX_SIZE 2048
#endif

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1 // for now, bus num must be 1 except hpm ip
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 8
#endif

/* When your chip hardware supports high-speed and wants to initialize it in high-speed mode, the relevant IP will configure the internal or external high-speed PHY according to CONFIG_USB_HS. */
// #define CONFIG_USB_HS

/* ---------------- FSDEV Configuration ---------------- */
//#define CONFIG_USBDEV_FSDEV_PMA_ACCESS 2 // maybe 1 or 2, many chips may have a difference

/* ---------------- DWC2 Configuration ---------------- */
/* (5 * number of control endpoints + 8) + ((largest USB packet used / 4) + 1 for
 * status information) + (2 * number of OUT endpoints) + 1 for Global NAK
 */
 #define CONFIG_USB_DWC2_RXALL_FIFO_SIZE (512 / 4)
/* IN Endpoints Max packet Size / 4 */
 #define CONFIG_USB_DWC2_TX0_FIFO_SIZE (96 / 4)
 #define CONFIG_USB_DWC2_TX1_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX2_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX3_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX4_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX5_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX6_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX7_FIFO_SIZE (80 / 4)
 #define CONFIG_USB_DWC2_TX8_FIFO_SIZE (0 / 4)

// #define CONFIG_USB_DWC2_DMA_ENABLE

/* ---------------- MUSB Configuration ---------------- */
// #define CONFIG_USB_MUSB_SUNXI

/* ================ USB Host Port Configuration ==================*/
#ifndef CONFIG_USBHOST_MAX_BUS
#define CONFIG_USBHOST_MAX_BUS 1
#endif

#ifndef CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USBHOST_PIPE_NUM 16
#endif

/* ---------------- EHCI Configuration ---------------- */

//#define CONFIG_USB_EHCI_HCCR_OFFSET     (0x0)
//#define CONFIG_USB_EHCI_FRAME_LIST_SIZE 1024
//#define CONFIG_USB_EHCI_QH_NUM          CONFIG_USBHOST_PIPE_NUM
//#define CONFIG_USB_EHCI_QTD_NUM         3
//#define CONFIG_USB_EHCI_ITD_NUM         20
// #define CONFIG_USB_EHCI_HCOR_RESERVED_DISABLE
// #define CONFIG_USB_EHCI_CONFIGFLAG
// #define CONFIG_USB_EHCI_ISO
// #define CONFIG_USB_EHCI_WITH_OHCI

/* ---------------- OHCI Configuration ---------------- */
//#define CONFIG_USB_OHCI_HCOR_OFFSET (0x0)

/* ---------------- XHCI Configuration ---------------- */
//#define CONFIG_USB_XHCI_HCCR_OFFSET (0x0)

/* ---------------- DWC2 Configuration ---------------- */
/* largest non-periodic USB packet used / 4 */
 #define CONFIG_USB_DWC2_NPTX_FIFO_SIZE (384 / 4)
/* largest periodic USB packet used / 4 */
 #define CONFIG_USB_DWC2_PTX_FIFO_SIZE (384 / 4)
/*
 * (largest USB packet used / 4) + 1 for status information + 1 transfer complete +
 * 1 location each for Bulk/Control endpoint for handling NAK/NYET scenario
 */
 #define CONFIG_USB_DWC2_RX_FIFO_SIZE ((320 - CONFIG_USB_DWC2_NPTX_FIFO_SIZE - CONFIG_USB_DWC2_PTX_FIFO_SIZE))

/* ---------------- MUSB Configuration ---------------- */
// #define CONFIG_USB_MUSB_SUNXI


/* ================ USB BSP Configuration ================ */
/**
  * @brief enable usb device mode
  */
/* #define USE_OTG_DEVICE_MODE */

/**
  * @brief enable usb host mode
  */
#define USE_OTG_HOST_MODE

/**
  * @brief select otgfs1 or otgfs2 define
  */

/* use otgfs1 */
#define OTG_USB_ID                           1

/* use otgfs2 */
/* #define OTG_USB_ID                         2 */

#ifdef RT_CHERRYUSB_HOST
#define USB_ID                           0
#define USB_OTG_FS_PERIPH_BASE           OTGFS1_BASE
#define OTG_CLOCK                        CRM_OTGFS1_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS1_IRQn
#define OTG_IRQ_HANDLER                  OTGFS1_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS1_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS1_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_18

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK

#define OTG_PIN_DP                       GPIO_PINS_12
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE12

#define OTG_PIN_DM                       GPIO_PINS_11
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE11

#define OTG_PIN_VBUS                     GPIO_PINS_9
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE9

#define OTG_PIN_ID                       GPIO_PINS_10
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE10

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_8
#define OTG_PIN_SOF_SOURCE               GPIO_PINS_SOURCE8

#define OTG_PIN_MUX                      GPIO_MUX_10

#define OTG_PIN_POWER_SWITCH_GPIO        GPIOH
#define OTG_PIN_POWER_SWITCH_CLOCK       CRM_GPIOH_PERIPH_CLOCK
#define OTG_PIN_POWER_SWITCH             GPIO_PINS_3
#endif


#ifdef RT_CHERRYUSB_DEVICE
#define USB_ID                           1
#define USB_OTG_FS_PERIPH_BASE           OTGFS2_BASE
#define OTG_CLOCK                        CRM_OTGFS2_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS2_IRQn
#define OTG_IRQ_HANDLER                  OTGFS2_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS2_WKUP_IRQn
#define OTG_WKUP_HANDLER                 OTGFS2_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_20

#define OTG_PIN_GPIO                     GPIOB
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOB_PERIPH_CLOCK

#define OTG_PIN_DP                       GPIO_PINS_15
#define OTG_PIN_DP_SOURCE                GPIO_PINS_SOURCE15

#define OTG_PIN_DM                       GPIO_PINS_14
#define OTG_PIN_DM_SOURCE                GPIO_PINS_SOURCE14

#define OTG_PIN_VBUS                     GPIO_PINS_13
#define OTG_PIN_VBUS_SOURCE              GPIO_PINS_SOURCE13

#define OTG_PIN_ID                       GPIO_PINS_12
#define OTG_PIN_ID_SOURCE                GPIO_PINS_SOURCE12

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_4
#define OTG_PIN_SOF_SOURCE               GPIO_PINS_SOURCE4

#define OTG_PIN_MUX                      GPIO_MUX_12

#define OTG_PIN_POWER_SWITCH_GPIO        GPIOB
#define OTG_PIN_POWER_SWITCH_CLOCK       CRM_GPIOB_PERIPH_CLOCK
#define OTG_PIN_POWER_SWITCH             GPIO_PINS_10
#endif



#ifdef USE_OTG_HOST_MODE
/* usb host vbus power switch */
//#define USBH_5V_POWER_SWITCH
#endif

/**
  * @brief usb sof output enable
  */
/* #define USB_SOF_OUTPUT_ENABLE */

/**
  * @brief usb vbus ignore
  */
#define USB_VBUS_IGNORE

void OTG_HS_IRQHandler(void);

#endif
