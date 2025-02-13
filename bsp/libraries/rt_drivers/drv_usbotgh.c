/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-02-28     leo          first version
 * 2024-02-26     shelton      update drv_pipe_xfer
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

#if defined(BSP_USING_HOST_USBOTG1) || defined(BSP_USING_HOST_USBOTG2)
#include "usbh_int.h"
#include "drv_usbotg.h"
#include "drv_config.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.usb.fsh"
#include <drv_log.h>

static struct rt_completion urb_completion;
static volatile rt_bool_t connect_status = RT_FALSE;
static struct at32_usbotg *p_usbotg_instance = RT_NULL;
static rt_sem_t   usb_urb_sem[USB_HOST_CHANNEL_NUM];

enum
{
#ifdef BSP_USING_HOST_USBOTG1
    USBOTG1_INDEX,
#endif
#ifdef BSP_USING_HOST_USBOTG2
    USBOTG2_INDEX,
#endif
};

static struct at32_usbotg usbotgh_config[] = {
#ifdef BSP_USING_HOST_USBOTG1
    USBOTG1_CONFIG,
#endif
#ifdef BSP_USING_HOST_USBOTG2
    USBOTG2_CONFIG,
#endif
};

#ifdef BSP_USING_HOST_USBOTG1
void OTGFS1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    usbh_irq_handler(p_usbotg_instance->p_otg_core);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef BSP_USING_HOST_USBOTG2
void OTGFS2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    usbh_irq_handler(p_usbotg_instance->p_otg_core);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

void usbh_connect_callback(usbh_core_type *uhost)
{
    uhcd_t hcd = (uhcd_t)uhost->pdata;
		rt_kprintf("time:%dms, connect %d\n", rt_tick_get(), connect_status);
    if (!connect_status)
    {
        connect_status = RT_TRUE;
        LOG_D("usb connected");
        rt_usbh_root_hub_connect_handler(hcd, 1, RT_FALSE);
    }
}

void usbh_disconnect_callback(usbh_core_type *uhost)
{
    uhcd_t hcd = (uhcd_t)uhost->pdata;
		rt_kprintf("time:%dms disconnect %d\n", rt_tick_get(),connect_status);
    if (connect_status)
    {
        connect_status = RT_FALSE;
        LOG_D("usb disconnnect");
        rt_usbh_root_hub_disconnect_handler(hcd, 1);
    }
}

void usbd_notify_urbchange_callback(usbh_core_type *uhost, uint8_t chnum, urb_sts_type sts)
{
//    rt_completion_done(&urb_completion);
		/*  when use the L501 4D0103 usb urb_state is idle, L501 2B0402 no enter zhaoshimin 20211120*/
    if((sts != URB_NOTREADY) && (sts != URB_IDLE))
    {
        rt_sem_release(usb_urb_sem[chnum]);
    }
}

static rt_err_t drv_reset_port(rt_uint8_t port)
{
    LOG_D("reset port");
    usbh_reset_port(&p_usbotg_instance->p_otg_core->host);
    return RT_EOK;
}


static int drv_pipe_xfer(upipe_t pipe, rt_uint8_t token, void *buffer, int nbytes, int timeouts)
{
    volatile int retry = 0;
		rt_int32_t tick = 0;
		void *pusb_buffer = RT_NULL;  // 临时缓冲区指针
    int len = 0;                  // 缓冲区长度计算
    int ret = 0;                  // 返回值变量
		usbh_core_type *uhost = &(p_usbotg_instance->p_otg_core->host);

		if(pipe->ep.bEndpointAddress & 0x80)
    {
        /*IN ep*/
        
        /* Because the USB bottom layer will set the receiving buffer to an integer multiple of the maximum packet length, 
           so the length of the receiving buffer should be  converted to an integer multiple of the maximum packet length zhaoshimin 20211119 */
        if((nbytes) && ((nbytes % pipe->ep.wMaxPacketSize) == 0))
        {
            pusb_buffer = buffer;
        }
        else
        {
            if(nbytes == 0)
            {
                len = pipe->ep.wMaxPacketSize;
            }
            else
            {
                len = (nbytes + pipe->ep.wMaxPacketSize - 1) / pipe->ep.wMaxPacketSize;
                len = len * pipe->ep.wMaxPacketSize;
            }

            pusb_buffer = rt_malloc(len);
            if(pusb_buffer == RT_NULL)
            {
                LOG_D("drv_pipe_xfer malloc: %d faile\n", len);
                return -1;
            }
        }
    }
    else
    {
        pusb_buffer = buffer;
    }    

		// 重置信号量
		rt_sem_control(usb_urb_sem[pipe->pipe_index], RT_IPC_CMD_RESET, RT_NULL);
		
		p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].dir = (pipe->ep.bEndpointAddress & 0x80) >> 7;

    if(token == 0U)
    {
        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_SETUP;
    }
    else
    {
        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
    }

    /* endpoint type */
    switch(pipe->ep.bmAttributes)
    {
        /* endpoint is control type */
        case EPT_CONTROL_TYPE:
            if((token == 1U) && (((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U))
            {
                if(nbytes == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out = 1U;
                }
                if((&p_usbotg_instance->p_otg_core->host)->hch[pipe->pipe_index].toggle_out == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
                }
                else
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
                }
            }
            break;
        /* endpoint is bulk type */
        case EPT_BULK_TYPE:
            if(((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U)
            {
                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
                }
                else
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
                }
            }
            else
            {
                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_in == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
                }
                else
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
                }
            }
            break;
        /* endpoint is int type */
        case  EPT_INT_TYPE:
            if(((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U)
            {
                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
                }
                else
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
                }
            }
            else
            {
                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_in == 0U)
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
                }
                else
                {
                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
                }
            }
            break;
        /* endpoint is isoc type */
        case EPT_ISO_TYPE:
            p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
            break;

        default:
            break;
    }

    /* set transfer buffer */
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_buf = pusb_buffer;
    /* set transfer len*/
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_len = nbytes;
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].urb_sts = URB_IDLE;
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].ch_num = pipe->pipe_index;
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_count = 0;
    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].state = HCH_IDLE;

__resend:
    /* data in/out for host */
    usbh_in_out_request((&p_usbotg_instance->p_otg_core->host), pipe->pipe_index);

		if(timeouts == 0)
		{
				tick = RT_WAITING_FOREVER;
		}
		else
		{
				tick = rt_tick_from_millisecond(timeouts);
		}
	 
		if(-RT_ETIMEOUT == rt_sem_take(usb_urb_sem[pipe->pipe_index], tick))
		{
				rt_kprintf("sem %d timeout!\n", pipe->pipe_index);
				usb_hch_halt(uhost->usb_reg, pipe->pipe_index);
				ret =  -RT_ETIMEOUT;
				goto __exit;
		}
	
		if(!connect_status)
		{
				ret =  -RT_ERROR;
				goto __exit;
		}
		
		if(usbh_get_urb_status((&p_usbotg_instance->p_otg_core->host), pipe->pipe_index) == URB_NOTREADY)
		{
				LOG_D("nak\n");
				if (pipe->ep.bmAttributes == USB_EP_ATTR_INT)
        {
            rt_thread_delay((pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) > 0 ? (pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) : 1);
        }
				if((pipe->ep.bEndpointAddress & 0x80) == 0)
				{
						goto __resend;
				}
		}
		else if (usbh_get_urb_status(&p_usbotg_instance->p_otg_core->host, pipe->pipe_index) == URB_STALL)
		{
				LOG_D("stall");
				pipe->status = UPIPE_STATUS_STALL;
				if (pipe->callback != RT_NULL)
				{
						pipe->callback(pipe);
				}
				ret = -1;
				goto __exit;
		}
		else if (usbh_get_urb_status(&p_usbotg_instance->p_otg_core->host, pipe->pipe_index) == URB_ERROR)
		{
				LOG_D("error");
				pipe->status = UPIPE_STATUS_ERROR;
				if (pipe->callback != RT_NULL)
				{
						pipe->callback(pipe);
				}
				ret = -1;
				goto __exit;
		}
		else if (usbh_get_urb_status(&p_usbotg_instance->p_otg_core->host, pipe->pipe_index) == URB_DONE)
		{
				LOG_D("ok");
				pipe->status = UPIPE_STATUS_OK;
				if (pipe->callback != RT_NULL)
				{
						pipe->callback(pipe);
				}
				rt_size_t size = (&p_usbotg_instance->p_otg_core->host)->hch[pipe->pipe_index].trans_count;
				
				if ((size > 0) && (pusb_buffer != buffer) && (pusb_buffer) && (buffer))
				{
						rt_memcpy(buffer, pusb_buffer, nbytes);
				}
				
				if (pipe->ep.bEndpointAddress & 0x80)
				{
						ret =  size;
				}
				else
				{
						ret =  nbytes;
				}
				goto __exit;
		}
		else
		{
				LOG_D("usb status:%d\n", usb_state);
				
				usb_hch_halt(uhost->usb_reg, pipe->pipe_index);
				ret = 0;
		}
		
__exit:	//统一释放临时缓冲区，避免内存泄漏
    if((pusb_buffer != buffer) && (pusb_buffer))
    {
        rt_free(pusb_buffer);
        pusb_buffer = RT_NULL;
    }    
    return ret;
}

//static int drv_pipe_xfer(upipe_t pipe, rt_uint8_t token, void *buffer, int nbytes, int timeouts)
//{
//    rt_int32_t tick = 0;
//    void *pusb_buffer = RT_NULL;
//    int   len = 0;
//    int   ret = 0;
//    urb_sts_type usb_state;
//		usbh_core_type *uhost = &(p_usbotg_instance->p_otg_core->host);

//    if(pipe->ep.bEndpointAddress & 0x80)
//    {
//        /*IN ep*/
//        
//        /* Because the USB bottom layer will set the receiving buffer to an integer multiple of the maximum packet length, 
//           so the length of the receiving buffer should be  converted to an integer multiple of the maximum packet length zhaoshimin 20211119 */
//        if((nbytes) && ((nbytes % pipe->ep.wMaxPacketSize) == 0))
//        {
//            pusb_buffer = buffer;
//        }
//        else
//        {
//            if(nbytes == 0)
//            {
//                len = pipe->ep.wMaxPacketSize;
//            }
//            else
//            {
//                len = (nbytes + pipe->ep.wMaxPacketSize - 1) / pipe->ep.wMaxPacketSize;
//                len = len * pipe->ep.wMaxPacketSize;
//            }

//            pusb_buffer = rt_malloc(len);
//            if(pusb_buffer == RT_NULL)
//            {
//                LOG_D(RT_DEBUG_USB,
//                             ("drv_pipe_xfer malloc: %d faile\n", len));
//                return -1;
//            }
//        }
//    }
//    else
//    {
//        pusb_buffer = buffer;
//    }    

//    rt_sem_control(usb_urb_sem[pipe->pipe_index], RT_IPC_CMD_RESET, RT_NULL);     
//    
//		p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].dir = (pipe->ep.bEndpointAddress & 0x80) >> 7;

//    if(token == 0U)
//    {
//        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_SETUP;
//    }
//    else
//    {
//        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//    }

//		/* endpoint type */
//    switch(pipe->ep.bmAttributes)
//    {
//        /* endpoint is control type */
//        case EPT_CONTROL_TYPE:
//            if((token == 1U) && (((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U))
//            {
//                if(nbytes == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out = 1U;
//                }
//                if((&p_usbotg_instance->p_otg_core->host)->hch[pipe->pipe_index].toggle_out == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//                }
//                else
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//                }
//            }
//            break;
//        /* endpoint is bulk type */
//        case EPT_BULK_TYPE:
//            if(((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U)
//            {
//                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//                }
//                else
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//                }
//            }
//            else
//            {
//                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_in == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//                }
//                else
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//                }
//            }
//            break;
//        /* endpoint is int type */
//        case  EPT_INT_TYPE:
//            if(((pipe->ep.bEndpointAddress & 0x80) >> 7) == 0U)
//            {
//                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//                }
//                else
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//                }
//            }
//            else
//            {
//                if( p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_in == 0U)
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//                }
//                else
//                {
//                    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA1;
//                }
//            }
//            break;
//        /* endpoint is isoc type */
//        case EPT_ISO_TYPE:
//            p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].data_pid = HCH_PID_DATA0;
//            break;

//        default:
//            break;
//    }

//    /* set transfer buffer */
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_buf = pusb_buffer;
//    /* set transfer len*/
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_len = nbytes;
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].urb_sts = URB_IDLE;
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].ch_num = pipe->pipe_index;
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].trans_count = 0;
//    p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].state = HCH_IDLE;
//		
//		/* data in/out for host */
//    usbh_in_out_request((&p_usbotg_instance->p_otg_core->host), pipe->pipe_index);

//		
//    if(timeouts == 0)
//    {
//        tick = RT_WAITING_FOREVER;
//    }
//    else
//    {
//        tick = rt_tick_from_millisecond(timeouts);
//    }
//   
//    if(-RT_ETIMEOUT == rt_sem_take(usb_urb_sem[pipe->pipe_index], tick))
//    {
//        rt_kprintf("sem %d timeout!\n", pipe->pipe_index);
//        usb_hch_halt(uhost->usb_reg, pipe->pipe_index);
//        ret =  -RT_ETIMEOUT;
//        goto __exit;
//    }
//#ifndef DRV_USBH_USE_HS    
//    if((pipe->ep.bEndpointAddress & 0x80) == 0)
//    {
//        /*OUT EP */
//        /*between twice usb out transfer, should add the less 1ms delay. 
//         if not next out transfer return NAK zhaoshimin 2021050 */
//        rt_thread_mdelay(1);
//    }
//#endif    
//   
//    if (!connect_status)
//    {
//        ret =  -RT_ERROR;
//        goto __exit;
//    }
//    
//    usb_state = usbh_get_urb_status(uhost, pipe->pipe_index);
//    if (usb_state == URB_NOTREADY)
//    {
//        LOG_D(RT_DEBUG_USB, ("nak\n"));
//        if (pipe->ep.bmAttributes == USB_EP_ATTR_INT)
//        {
//            rt_thread_delay((pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) > 0 ? (pipe->ep.bInterval * RT_TICK_PER_SECOND / 1000) : 1);
//        }                 
//        
//    }
//    else if(usb_state == URB_STALL)
//    {
//        LOG_D(RT_DEBUG_USB, ("stall\n"));
//        pipe->status = UPIPE_STATUS_STALL;
//        if (pipe->callback != RT_NULL)
//        {
//            pipe->callback(pipe);
//        }
//        ret =  -RT_ERROR;
//    }
//    else if(usb_state == URB_ERROR)
//    {
//        LOG_D(RT_DEBUG_USB, ("error\n"));
//        pipe->status = UPIPE_STATUS_ERROR;
//        if (pipe->callback != RT_NULL)
//        {
//            pipe->callback(pipe);
//        }
//        ret =  -RT_ERROR;
//    }
//    else if(URB_DONE == usb_state)
//    {
//        LOG_D(RT_DEBUG_USB, ("ok\n"));
//        pipe->status = UPIPE_STATUS_OK;
//        if (pipe->callback != RT_NULL)
//        {
//            pipe->callback(pipe);
//        }
//        ret = uhost->hch[pipe->pipe_index].trans_count;

//        if((ret > 0) && (pusb_buffer != buffer) && (pusb_buffer) && (buffer))
//        {
//            rt_memcpy(buffer, pusb_buffer, nbytes);
//            
//        }
//        
//    }
//    else
//    {
//        LOG_D(RT_DEBUG_USB, ("usb status:%d\n", usb_state));
//        
//        usb_hch_halt(uhost->usb_reg, pipe->pipe_index);
//        ret =  -RT_ERROR;
//    }

//__exit:
//    if((pusb_buffer != buffer) && (pusb_buffer))
//    {
//        rt_free(pusb_buffer);
//        pusb_buffer = RT_NULL;
//    }    
//    return ret;
//}

static rt_uint16_t pipe_index = 0;
static rt_uint8_t  drv_get_free_pipe_index(void)
{
    rt_uint8_t idx = 0;
    for (idx = 0; idx < USB_HOST_CHANNEL_NUM; idx++)
    {
        if (!(pipe_index & (0x01 << idx)))
        {
            pipe_index |= (0x01 << idx);
            return idx;
        }
    }
    return 0xff;
}

static void drv_free_pipe_index(rt_uint8_t index)
{
    pipe_index &= ~(0x01 << index);
}

static rt_err_t drv_open_pipe(upipe_t pipe)
{
    pipe->pipe_index = drv_get_free_pipe_index();
		if(pipe->pipe_index > USB_HOST_CHANNEL_NUM)
    {
        return -RT_ERROR;
    }
    usbh_hc_open(&p_usbotg_instance->p_otg_core->host,
                 pipe->pipe_index,
                 pipe->ep.bEndpointAddress,
                 pipe->inst->address,
                 pipe->ep.bmAttributes,
                 pipe->ep.wMaxPacketSize,
                 USB_PRTSPD_FULL_SPEED);
    /* set data0 pid token*/
    if (p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].dir)
    {
        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_in = 0;
    }
    else
    {
        p_usbotg_instance->p_otg_core->host.hch[pipe->pipe_index].toggle_out = 0;
    }
    return RT_EOK;
}

static rt_err_t drv_close_pipe(upipe_t pipe)
{
    if(pipe->pipe_index < USB_HOST_CHANNEL_NUM)
    {
			usb_hch_halt((&p_usbotg_instance->p_otg_core->host)->usb_reg, pipe->pipe_index);
			drv_free_pipe_index(pipe->pipe_index);
		}
    return RT_EOK;
}

static struct uhcd_ops _uhcd_ops =
{
    drv_reset_port,
    drv_pipe_xfer,
    drv_open_pipe,
    drv_close_pipe,
};

static rt_err_t at32_hcd_init(rt_device_t device)
{
    rt_uint32_t i = 0;
    char sem_name[10] = {0};		
		
		for(i = 0; i < USB_HOST_CHANNEL_NUM; i++)
    {
        rt_sprintf(sem_name, "urb_sem%d", i);
        usb_urb_sem[i] = rt_sem_create(sem_name, 0, RT_IPC_FLAG_FIFO);
        if(usb_urb_sem[i] == RT_NULL)
        {
            return -RT_ERROR;
        }
    }
		
		/* usb gpio config */
    at32_msp_usb_init(device);

    /* enable otgfs irq */
    nvic_irq_enable(p_usbotg_instance->irqn, 2, 0);

    /* init usb */
    usbh_init(p_usbotg_instance->p_otg_core,
              p_usbotg_instance->dev_spd,
              p_usbotg_instance->id);
    return RT_EOK;
}

int at32_usbh_register(void)
{
    rt_size_t obj_num;
    rt_err_t result = 0;
    int index;

    obj_num = sizeof(usbotgh_config) / sizeof(struct at32_usbotg);

    for (index = 0; index < obj_num; index++) {
        uhcd_t uhcd = (uhcd_t)rt_malloc(sizeof(struct uhcd));
        if (uhcd == RT_NULL)
        {
            rt_kprintf("uhcd malloc failed\r\n");
            return -RT_ERROR;
        }
        rt_memset((void *)uhcd, 0, sizeof(struct uhcd));

        otg_core_type *p_otg_core = (otg_core_type *)rt_malloc(sizeof(otg_core_type));
        if (p_otg_core == RT_NULL)
        {
            rt_kprintf("otg_core malloc failed\r\n");
            return -RT_ERROR;
        }
        rt_memset((void *)p_otg_core, 0, sizeof(otg_core_type));

        uhcd->parent.type = RT_Device_Class_USBHost;
        uhcd->parent.init = at32_hcd_init;
        uhcd->parent.user_data = &(p_otg_core->host);

        uhcd->ops = &_uhcd_ops;
        uhcd->num_ports = 1;
        p_otg_core->host.pdata = uhcd;
        usbotgh_config[index].p_otg_core = p_otg_core;

        result = rt_device_register(&uhcd->parent, usbotgh_config[index].name, RT_DEVICE_FLAG_DEACTIVATE);
        RT_ASSERT(result == RT_EOK);

        p_usbotg_instance = &usbotgh_config[index];

        result = rt_usb_host_init(usbotgh_config[index].name);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

INIT_DEVICE_EXPORT(at32_usbh_register);

#endif
