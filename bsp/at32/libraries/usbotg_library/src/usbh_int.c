/**
  **************************************************************************
  * @file     usbh_int.c
  * @brief    usb host interrupt request
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
#include "usbh_int.h"

#ifdef USE_OTG_HOST_MODE

/** @defgroup USBH_drivers_interrupt
  * @brief usb host interrupt
  * @{
  */

/** @defgroup USBH_int_private_functions
  * @{
  */

/**
  * @brief  usb host interrupt handler
  * @param  otgdev: to the structure of otg_core_type
  * @retval none
  */
void usbh_irq_handler(otg_core_type *otgdev)
{
  otg_global_type *usbx = otgdev->usb_reg;
  usbh_core_type *uhost = &otgdev->host;
  uint32_t intsts = usb_global_get_all_interrupt(usbx);

  if(usbx->gintsts_bit.curmode == 1)
  {
    if(intsts & USB_OTG_HCH_FLAG)
    {
      usbh_hch_handler(uhost);
      usb_global_clear_interrupt(usbx, USB_OTG_HCH_FLAG);
    }
    if(intsts & USB_OTG_SOF_FLAG)
    {
      usbh_sof_handler(uhost);
      usb_global_clear_interrupt(usbx, USB_OTG_SOF_FLAG);
    }
    if(intsts & USB_OTG_MODEMIS_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_MODEMIS_FLAG);
    }
    if(intsts & USB_OTG_WKUP_FLAG)
    {
      usbh_wakeup_handler(uhost);
      usb_global_clear_interrupt(usbx, USB_OTG_WKUP_FLAG);
    }
    while(usbx->gintsts & USB_OTG_RXFLVL_FLAG)
    {
      usbh_rx_qlvl_handler(uhost);
      usb_global_clear_interrupt(usbx, USB_OTG_RXFLVL_FLAG);
    }
    if(intsts & USB_OTG_DISCON_FLAG)
    {
      usbh_disconnect_handler(uhost);
      usb_global_clear_interrupt(usbx, USB_OTG_DISCON_FLAG);
    }
    if(intsts & USB_OTG_PRT_FLAG)
    {
      usbh_port_handler(uhost);
    }
    if(intsts & USB_OTG_INCOMPIP_INCOMPISOOUT_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_INCOMPIP_INCOMPISOOUT_FLAG);
    }
    if(intsts & USB_OTG_INCOMISOIN_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_INCOMISOIN_FLAG);
    }
    if(intsts & USB_OTG_PTXFEMP_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_PTXFEMP_FLAG);
    }
    if(intsts & USB_OTG_ISOOUTDROP_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_ISOOUTDROP_FLAG);
    }
    if(intsts & USB_OTG_NPTXFEMP_FLAG)
    {
      /* Nonperiodic TxFIFO empty interrupt */
      usb_global_clear_interrupt(usbx, USB_OTG_NPTXFEMP_FLAG);
    }

  }
}

/**
  * @brief  usb host wakeup handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_wakeup_handler(usbh_core_type *uhost)
{
  uhost->global_state = USBH_WAKEUP;
}

/**
  * @brief  usb host sof handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_sof_handler(usbh_core_type *uhost)
{
  uhost->timer ++;
}

/**
  * @brief  usb host disconnect handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_disconnect_handler(usbh_core_type *uhost)
{
  otg_global_type *usbx = uhost->usb_reg;

  uint8_t i_index;

  usb_host_disable(usbx);

  uhost->conn_sts = 0;

  uhost->global_state = USBH_DISCONNECT;

  for(i_index = 0; i_index < USB_HOST_CHANNEL_NUM; i_index ++)
  {
    usbh_free_channel(uhost, i_index);
  }
  usbh_fsls_clksel(usbx, USB_HCFG_CLK_48M);

  usbh_disconnect_callback(uhost);
}

/**
  * @brief  usb host in transfer request handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  chn: channel number
  * @retval none
  */
void usbh_hch_in_handler(usbh_core_type *uhost, uint8_t chn)
{
  otg_global_type *usbx = uhost->usb_reg;
  otg_hchannel_type *usb_chh = USB_CHL(usbx, chn);
  uint32_t hcint_value = usb_chh->hcint & usb_chh->hcintmsk;

  if( hcint_value & USB_OTG_HC_ACK_FLAG)
  {
    usb_chh->hcint = USB_OTG_HC_ACK_FLAG;
    rt_kprintf("IN %d ACK:%X\n", chn, usb_chh->hctsiz);
  }
  else if(hcint_value & USB_OTG_HC_STALL_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_chh->hcint = USB_OTG_HC_NAK_FLAG | USB_OTG_HC_STALL_FLAG;
    uhost->hch[chn].state = HCH_STALL;
    usb_hch_halt(usbx, chn);
  }
  else if(hcint_value & USB_OTG_HC_DTGLERR_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_hch_halt(usbx, chn);
    usb_chh->hcint = USB_OTG_HC_DTGLERR_FLAG | USB_OTG_HC_NAK_FLAG;
    uhost->hch[chn].state = HCH_DATATGLERR;
  }

  else if(hcint_value & USB_OTG_HC_FRMOVRRUN_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_hch_halt(usbx, chn);
    usb_chh->hcint = USB_OTG_HC_FRMOVRRUN_FLAG;
  }
  else if(hcint_value & USB_OTG_HC_XFERC_FLAG)
  {
    uhost->hch[chn].state = HCH_XFRC;
    uhost->err_cnt[chn] = 0;
    usb_chh->hcint = USB_OTG_HC_XFERC_FLAG;

    if(usb_chh->hcchar_bit.eptype == EPT_BULK_TYPE || usb_chh->hcchar_bit.eptype == EPT_CONTROL_TYPE)
    {
      usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
      usb_hch_halt(usbx, chn);
      usb_chh->hcint = USB_OTG_HC_NAK_FLAG;
    }
    else if(usb_chh->hcchar_bit.eptype == EPT_INT_TYPE)
    {
      uhost->urb_state[chn] = URB_DONE;
      if(uhost->dma_en == FALSE)
      {
        usb_chh->hcchar_bit.oddfrm = TRUE;
        uhost->hch[chn].toggle_in ^= 1U;
      }
      else
      {
        uint32_t num_packets = (uint16_t)((uhost->hch[chn].trans_len + uhost->hch[chn].maxpacket - 1U) / uhost->hch[chn].maxpacket);
        uint32_t has_sent_packets = num_packets - usb_chh->hctsiz_bit.pktcnt;
        uhost->hch[chn].trans_count += uhost->hch[chn].trans_len - usb_chh->hctsiz_bit.xfersize;
        if(has_sent_packets % 2 == 1)
        {
          usb_chh->hcchar_bit.oddfrm = TRUE;
          uhost->hch[chn].toggle_in ^= 1U; 
        }
      }  
      rt_kprintf("IN %d URB_DONE recv:%d ,tgl_in:%d at interrupt!\n", chn, uhost->hch[chn].trans_count, uhost->hch[chn].toggle_in);
			
      usbd_notify_urbchange_callback(uhost, chn, uhost->urb_state[chn]);
    }
    else if(usb_chh->hcchar_bit.eptype == EPT_ISO_TYPE)
    {
      uhost->urb_state[chn] = URB_DONE;
      usbd_notify_urbchange_callback(uhost, chn, uhost->urb_state[chn]);
    }

    rt_kprintf("IN %d XFRC recv:%d\n", chn, uhost->hch[chn].trans_count);
  }
  else if(hcint_value & USB_OTG_HC_CHHLTD_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = FALSE;
    if(uhost->hch[chn].state == HCH_XFRC )
    {
      if(uhost->dma_en == TRUE)
      {
        /*use the dma, change the toggle in*/
        uint32_t num_packets = (uint16_t)(((uhost->hch[chn].trans_len == 0 ? 1: uhost->hch[chn].trans_len) + 
                                              uhost->hch[chn].maxpacket - 1U) / uhost->hch[chn].maxpacket);
        uint32_t has_sent_packets = num_packets - usb_chh->hctsiz_bit.pktcnt;
        uhost->hch[chn].trans_count += uhost->hch[chn].trans_len - usb_chh->hctsiz_bit.xfersize;
        if(has_sent_packets % 2 == 1)
        {
          uhost->hch[chn].toggle_in ^= 1U; 
        }
      }
			uhost->urb_state[chn]  = URB_DONE;
			rt_kprintf("IN %d URB_DONE recv:%d, tgl_in:%d\n", chn, uhost->hch[chn].trans_count,uhost->hch[chn].toggle_in);
    }
    else if(uhost->hch[chn].state == HCH_STALL)
    {
      uhost->urb_state[chn]  = URB_STALL;
    }
    else if(uhost->hch[chn].state == HCH_XACTERR ||
            uhost->hch[chn].state == HCH_DATATGLERR)
    {
      uhost->err_cnt[chn] ++;
      if(uhost->err_cnt[chn] > 3)
      {
        uhost->urb_state[chn] = URB_ERROR;
        uhost->err_cnt[chn] = 0;
      }
      else
      {
        uhost->urb_state[chn] = URB_NOTREADY;
      }
			
			/* re-activate the channel  */
      usb_chh->hcchar_bit.chdis = FALSE;
      usb_chh->hcchar_bit.chena = TRUE;
    }
    else if(uhost->hch[chn].state == HCH_NAK)
    {
      /* re-activate the channel  */
      usb_chh->hcchar_bit.chdis = FALSE;
      usb_chh->hcchar_bit.chena = TRUE;
      uhost->urb_state[chn] = URB_NOTREADY;
      //rt_kprintf("IN %d  Nak re acktive\n", chn);
    }
    else
    {
        /* this usb urb_state is idle, re-activate the channel. 
            use the L501C 4D0103 test enter there, L501C 2B0402 no enter zhaoshimin 20211120*/
        usb_chh->hcchar_bit.chdis = FALSE;
        usb_chh->hcchar_bit.chena = TRUE;
    }
    usb_chh->hcint = USB_OTG_HC_CHHLTD_FLAG;
    usbd_notify_urbchange_callback(uhost, chn, uhost->urb_state[chn]);
  }
  else if(hcint_value & USB_OTG_HC_XACTERR_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    uhost->hch[chn].state = HCH_XACTERR;
    usb_hch_halt(usbx, chn);
    uhost->err_cnt[chn] ++;
    usb_chh->hcint = USB_OTG_HC_XACTERR_FLAG;
  }
  else if(hcint_value & USB_OTG_HC_NAK_FLAG)
  {
    if(usb_chh->hcchar_bit.eptype == EPT_INT_TYPE)
    {
      uhost->err_cnt[chn] = 0;
      if(uhost->dma_en == FALSE)
      {
        usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
        usb_hch_halt(usbx, chn);
      }
    }
    else if(usb_chh->hcchar_bit.eptype == EPT_BULK_TYPE ||
      usb_chh->hcchar_bit.eptype == EPT_CONTROL_TYPE)
    {
      uhost->err_cnt[chn] = 0;
      if(uhost->dma_en == FALSE)
      {
        usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
        usb_hch_halt(usbx, chn);
      }
    }
    uhost->hch[chn].state = HCH_NAK;
    usb_chh->hcint = USB_OTG_HC_NAK_FLAG;
  }
  else if(hcint_value & USB_OTG_HC_BBLERR_FLAG)
  {
    usb_chh->hcint = USB_OTG_HC_BBLERR_FLAG;
  }
}

/**
  * @brief  usb host out transfer request handler
  * @param  uhost: to the structure of usbh_core_type
  * @param  chn: channel number
  * @retval none
  */
void usbh_hch_out_handler(usbh_core_type *uhost, uint8_t chn)
{
  otg_global_type *usbx = uhost->usb_reg;
  otg_hchannel_type *usb_chh = USB_CHL(usbx, chn);
  uint32_t hcint_value = usb_chh->hcint & usb_chh->hcintmsk;

  if( hcint_value & USB_OTG_HC_ACK_FLAG)
  {
    usb_chh->hcint = USB_OTG_HC_ACK_FLAG;
#if defined (SOC_SERIES_AT32F402) || defined (SOC_SERIES_AT32F405)
    if(uhost->hch[chn].do_ping == TRUE)
    {
      uhost->urb_state[chn] = URB_NOTREADY;
      uhost->hch[chn].do_ping = FALSE;
      usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
      usb_hch_halt(usbx, chn);
    }
#endif
	rt_kprintf("OUT %d ACK\n", chn);
  }
  else if( hcint_value & USB_OTG_HC_FRMOVRRUN_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_hch_halt(usbx, chn);
    usb_chh->hcint = USB_OTG_HC_FRMOVRRUN_FLAG;
  }
  else if( hcint_value & USB_OTG_HC_XFERC_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    uhost->err_cnt[chn] = 0;

    if(usb_chh->hcint & USB_OTG_HC_NYET_FLAG)
    {
#if defined (SOC_SERIES_AT32F402) || defined (SOC_SERIES_AT32F405)
      uhost->hch[chn].do_ping = TRUE;
#endif
      usb_chh->hcint = USB_OTG_HC_NYET_FLAG;
    }
    usb_hch_halt(usbx, chn);
    uhost->hch[chn].state = HCH_XFRC;
    usb_chh->hcint = USB_OTG_HC_XFERC_FLAG;
		if(uhost->dma_en == FALSE)
    {
      uhost->hch[chn].trans_count = uhost->hch[chn].trans_len;
    }
    rt_kprintf("OUT %d XFRC\n", chn);
  }
  else if( hcint_value & USB_OTG_HC_STALL_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_chh->hcint =  USB_OTG_HC_STALL_FLAG;
    uhost->hch[chn].state = HCH_STALL;
    usb_hch_halt(usbx, chn);
  }
  else if(hcint_value & USB_OTG_HC_NYET_FLAG)
  {
#if defined (SOC_SERIES_AT32F402) || defined (SOC_SERIES_AT32F405)
    uhost->hch[chn].do_ping = TRUE;
#endif
    uhost->err_cnt[chn] = 0;
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_hch_halt(usbx, chn);
    uhost->hch[chn].state = HCH_NYET;
    usb_chh->hcint = USB_OTG_HC_NYET_FLAG;
  }
  else if( hcint_value & USB_OTG_HC_DTGLERR_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    usb_hch_halt(usbx, chn);
    usb_chh->hcint = USB_OTG_HC_DTGLERR_FLAG | USB_OTG_HC_NAK_FLAG;
    uhost->hch[chn].state = HCH_DATATGLERR;
		rt_kprintf("OUT %d DTRERR\n", chn);
  }
  else if( hcint_value & USB_OTG_HC_CHHLTD_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = FALSE;
		usbx->gintmsk_bit.nptxfempmsk = FALSE;
		usbx->gintmsk_bit.ptxfempmsk = FALSE;
		
    if(uhost->hch[chn].state == HCH_XFRC)
    {
      uhost->urb_state[chn] = URB_DONE;
      if(uhost->hch[chn].ept_type == EPT_BULK_TYPE ||
        uhost->hch[chn].ept_type == EPT_INT_TYPE ||
        uhost->hch[chn].ept_type == EPT_CONTROL_TYPE)
      {
				if(uhost->dma_en)
        { 
          /*DMA enable */
          uint32_t num_packets = (uint16_t)(((uhost->hch[chn].trans_len == 0 ? 1: uhost->hch[chn].trans_len) + 
                                              uhost->hch[chn].maxpacket - 1U) / uhost->hch[chn].maxpacket);

          uint32_t count = usb_chh->hctsiz_bit.xfersize; /* last send size */

          if (count == uhost->hch[chn].maxpacket) {
              uhost->hch[chn].trans_count += num_packets * uhost->hch[chn].maxpacket;
          } else {
              uhost->hch[chn].trans_count += (num_packets - 1) * uhost->hch[chn].maxpacket + count;
          }

          if(uhost->hch[chn].ept_type != EPT_CONTROL_TYPE)
          {
            if(num_packets % 2 == 1)
            {
              uhost->hch[chn].toggle_out ^= 1U;   
            }
          }
           
        }
        else
        {       
          if(uhost->hch[chn].ept_type != EPT_CONTROL_TYPE)
          { 
            uhost->hch[chn].toggle_out ^= 1U;
          }  
        }
      }
      //rt_kprintf("OUT %d xfc:%d, tgl_out:%d\n", chn, uhost->hch[chn].trans_count, uhost->hch[chn].toggle_out);
    }
    else if(uhost->hch[chn].state == HCH_NAK || uhost->hch[chn].state == HCH_NYET)
    {
      uhost->urb_state[chn] = URB_NOTREADY;
    }
    else if(uhost->hch[chn].state == HCH_STALL)
    {
      uhost->hch[chn].urb_sts = URB_STALL;
    }
    else if(uhost->hch[chn].state == HCH_XACTERR ||
            uhost->hch[chn].state == HCH_DATATGLERR)
    {
      uhost->err_cnt[chn] ++;
      if(uhost->err_cnt[chn] > 3)
      {
        uhost->urb_state[chn] = URB_ERROR;
        uhost->err_cnt[chn] = 0;usbd_notify_urbchange_callback(uhost, chn, uhost->urb_state[chn]);
      }
      else
      {
        uhost->urb_state[chn] = URB_NOTREADY;
      }

      /* re-activate the channel  */
			usb_chh->hcchar_bit.chdis = FALSE;
      usb_chh->hcchar_bit.chena = TRUE;
    }
    usb_chh->hcint = USB_OTG_HC_CHHLTD_FLAG;
    usbd_notify_urbchange_callback(uhost, chn, uhost->urb_state[chn]);
  }
  else if( hcint_value & USB_OTG_HC_XACTERR_FLAG)
  {
    if(uhost->dma_en == TRUE)
    {
      uhost->err_cnt[chn] ++;
      if(uhost->err_cnt[chn] > 2)
      {
        uhost->urb_state[chn] = URB_ERROR;
        uhost->err_cnt[chn] = 0;
      }
      else
      {
        uhost->urb_state[chn] = URB_NOTREADY;
      }
    }
    else
    {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    uhost->err_cnt[chn] ++;
    uhost->hch[chn].state = HCH_XACTERR;
    usb_hch_halt(usbx, chn);
    }
    usb_chh->hcint = USB_OTG_HC_XACTERR_FLAG;
  }
  else if( hcint_value & USB_OTG_HC_NAK_FLAG)
  {
    usb_chh->hcintmsk_bit.chhltdmsk = TRUE;
    uhost->err_cnt[chn] = 0;
    usb_hch_halt(usbx, chn);
    uhost->hch[chn].state = HCH_NAK;
    usb_chh->hcint = USB_OTG_HC_NAK_FLAG;
#if defined (SOC_SERIES_AT32F402) || defined (SOC_SERIES_AT32F405)
    if(uhost->hch[chn].do_ping == 0)
    {
      if(uhost->hch[chn].speed == USB_PRTSPD_HIGH_SPEED)
      {
        uhost->hch[chn].do_ping = 1;
      }
    }
#endif
		rt_kprintf("OUT %d NAK\n", chn);
  }
}

/**
  * @brief  usb host channel request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_hch_handler(usbh_core_type *uhost)
{
  otg_global_type *usbx = uhost->usb_reg;
  otg_host_type *usb_host = OTG_HOST(usbx);
  uint32_t intsts, i_index;

  intsts = usb_host->haint & 0xFFFF;
  for(i_index = 0; i_index < 16; i_index ++)
  {
    if(intsts & (1 << i_index))
    {
      if(USB_CHL(usbx, i_index)->hcchar_bit.eptdir)
      {
        //hc in
        usbh_hch_in_handler(uhost, i_index);
      }
      else
      {
        //hc out
        usbh_hch_out_handler(uhost, i_index);
      }
    }
  }
}

/**
  * @brief  usb host rx buffer not empty request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_rx_qlvl_handler(usbh_core_type *uhost)
{
  uint8_t chn;
  uint32_t pktsts;
  uint32_t pktcnt;
  uint32_t tmp;
  otg_hchannel_type *ch;
  otg_global_type *usbx = uhost->usb_reg;

  usbx->gintmsk_bit.rxflvlmsk = 0;

  tmp = usbx->grxstsp;
  chn = tmp & 0xF;
  pktsts = (tmp >> 17) & 0xF;
  pktcnt = (tmp >> 4) & 0x7FF;
  ch = USB_CHL(usbx, chn);
  switch(pktsts)
  {
    case PKTSTS_IN_DATA_PACKET_RECV:
      if(pktcnt > 0 && (uhost->hch[chn].trans_buf) != 0)
      {
        usb_read_packet(usbx, uhost->hch[chn].trans_buf, chn, pktcnt);
        uhost->hch[chn].trans_buf += pktcnt;
        uhost->hch[chn].trans_count += pktcnt;

        if(ch->hctsiz_bit.pktcnt > 0)
        {
          ch->hcchar_bit.chdis = FALSE;
          ch->hcchar_bit.chena = TRUE;
          uhost->hch[chn].toggle_in ^= 1;
        }
				rt_kprintf("IN %d RXQLVL:%d, PID:%d\n", chn, pktcnt, (USB_OTG_GRXSTSP_DPID & tmp) >> 15);
      }
			else
      {
         /*����������յİ���Ϊ�����������������豸���ᷢ��һ��0�������������ݱ�ʾ���ݷ�����ɣ�����Ҫ�����ݰ�״̬���б仯�����Ҳ���ʹ�ܽ���ͨ��*/
         uhost->hch[chn].toggle_in ^= 1U;
         rt_kprintf("ZIN %d RXQLVL:%d, PID:%d\n", chn, pktcnt, (USB_OTG_GRXSTSP_DPID & tmp) >> 15);     
      }
      break;
    case PKTSTS_IN_TRANSFER_COMPLETE:
      break;
    case PKTSTS_DATA_BIT_ERROR:
      break;
    case PKTSTS_CHANNEL_STOP:
      break;
    default:
      break;

  }
  usbx->gintmsk_bit.rxflvlmsk = 1;
}

/**
  * @brief  usb host port request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval none
  */
void usbh_port_handler(usbh_core_type *uhost)
{
  otg_global_type *usbx = uhost->usb_reg;
  otg_host_type *usb_host = OTG_HOST(usbx);

  uint32_t prt = 0, prt_0;

  prt = usb_host->hprt;
  prt_0 = prt;

  prt_0 &= ~(USB_OTG_HPRT_PRTENA | USB_OTG_HPRT_PRTENCHNG |
               USB_OTG_HPRT_PRTOVRCACT | USB_OTG_HPRT_PRTCONDET);
  rt_kprintf("HPRT0:%x\n", prt_0);
  if(prt & USB_OTG_HPRT_PRTCONDET)
  {
    if(prt & USB_OTG_HPRT_PRTCONSTS)
    {
      /* connect callback */
      uhost->conn_sts = 1;
    }
		
		rt_kprintf("connet PCDET\n");
		usbh_connect_callback(uhost);
    prt_0 |= USB_OTG_HPRT_PRTCONDET;
  }

  if(prt & USB_OTG_HPRT_PRTENCHNG)
  {
    prt_0 |= USB_OTG_HPRT_PRTENCHNG;

    if(prt & USB_OTG_HPRT_PRTENA)
    {
      if((prt & USB_OTG_HPRT_PRTSPD) == (USB_PRTSPD_LOW_SPEED << 17))
      {
        usbh_fsls_clksel(usbx, USB_HCFG_CLK_6M);
      }
      else if((prt & USB_OTG_HPRT_PRTSPD) == (USB_PRTSPD_FULL_SPEED << 17))
      {
        usbh_fsls_clksel(usbx, USB_HCFG_CLK_48M);
      }
      else
      {
        usbh_fsls_clksel(usbx, USB_HCFG_CLK_60M);
      }
      /* connect callback */
      uhost->port_enable = 1;
      rt_kprintf("Port enable\n");    
    }
    else
    {
      /* clean up hprt */
      uhost->port_enable = 0;
      rt_kprintf("Port Disable\n");
    }
  }

  if(prt & USB_OTG_HPRT_PRTOVRCACT)
  {
    prt_0 |= USB_OTG_HPRT_PRTOVRCACT;
  }

  usb_host->hprt = prt_0;
}

rt_weak void usbh_connect_callback(usbh_core_type *uhost)
{
}

rt_weak void usbh_disconnect_callback(usbh_core_type *uhost)
{
}

rt_weak void usbd_notify_urbchange_callback(usbh_core_type *uhost, uint8_t chnum, urb_sts_type sts)
{
}

/**
  * @}
  */

/**
  * @}
  */

#endif
