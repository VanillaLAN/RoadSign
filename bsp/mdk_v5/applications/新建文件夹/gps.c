#include "at32f435_437.h"
#include <stdbool.h>
#include <string.h>

#include <rtthread.h>
#include <rtconfig.h>

#define BUFFER_SIZE 200
#define GPS_LOG rt_kprintf

typedef struct
{
	char GPS_Buffer[200];//数据接收
	bool isGetData;//接收数据是否完成
	char UTCTime[10];//时间戳 hhmmss.ss
	char latitude[12];//纬度 ddmm.mmmmmm
	char N_S[2];//南北
	char longitude[13];//经度 dddmm.mmmmmm
	char E_W[2];//东西
	bool isParseData;//是否解析完成
	bool isUsefull;//是否为有效数据位
}gps_data_type;


uint8_t gps_rx_buffer[BUFFER_SIZE];
gps_data_type gps_data;
static rt_device_t _gps_device = RT_NULL;

static void parseGpsBuffer(void);
static void printfGpsBuffer(void);

rt_device_t rt_gps_set_device(const char *name)
{
    rt_device_t new_device, old_device;

    /* save old device */
    old_device = _gps_device;

    /* find new console device */
    new_device = rt_device_find(name);
    
    /* check whether it's a same device */
    if (new_device == old_device) return RT_NULL;
    
    if (new_device != RT_NULL)
    {
        if (_gps_device != RT_NULL)
        {
            /* close old console device */
            rt_device_close(_gps_device);
        }

        /* set new console device */
        rt_device_open(new_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);
        _gps_device = new_device;
    }

    return old_device;
}

void gps_get_data(void)
{
	int i,j,len=0;
	int start_flag = 0;
	
	len = rt_device_read(_gps_device, 0, gps_rx_buffer, BUFFER_SIZE);
	GPS_LOG("gps = %s\n", gps_rx_buffer);
	if(len>0)
  {
		
		memset(gps_data.GPS_Buffer, 0, sizeof(gps_data.GPS_Buffer));      //清空
		for(i=0;i<BUFFER_SIZE;i++)
		{
			if(gps_rx_buffer[i]=='$')
			{
				start_flag=1;
				j=0;
			}
			if(start_flag)
			{
				if (gps_rx_buffer[i] == '\r')
				{
					gps_data.GPS_Buffer[j] = '\0';   // 字符串的结尾补0
//					uart_log_printf("GPS_Buffer = %s\r\n",gps_data.GPS_Buffer);
					if(gps_data.GPS_Buffer[0] == '$' && gps_data.GPS_Buffer[4] == 'M' && gps_data.GPS_Buffer[5] == 'C')
					{
						gps_data.isGetData = true;
						parseGpsBuffer();
						printfGpsBuffer();
					}
					memset(gps_data.GPS_Buffer, 0, sizeof(gps_data.GPS_Buffer));      //清空
				}
				
				gps_data.GPS_Buffer[j] = gps_rx_buffer[i];
				j++;
			}
		}
	}
}

/**
  * @brief  解析GPS数据
  * @param  none
  * @retval none
  */
static void parseGpsBuffer(void)
{
	char *subString;
	char *subStringNext;
	int subStringLength = 0;
	int i = 0;
	if(gps_data.isGetData)
	{
		gps_data.isGetData = false;
		GPS_LOG("%s\r\n", gps_data.GPS_Buffer);
		for(i = 0; i <= 6; i++)
		{
			if(i == 0)
			{
				if((subString = strstr(gps_data.GPS_Buffer, ",")) == NULL)	GPS_LOG("解析错误");
			}
			else
			{
				subString++;//到达逗号的下一位
				if((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2];
					//利用subStringNext和subString的首地址相减来确定指针大小
					subStringLength = subStringNext - subString;
					switch(i){
						case 1: 
							memset(gps_data.UTCTime, 0, sizeof(gps_data.UTCTime)); 
							memcpy(gps_data.UTCTime, subString, subStringLength); 
							gps_data.UTCTime[subStringLength]='\0';
							break;
						case 2: 
							memcpy(usefullBuffer, subString, subStringLength); 
							break;
						case 3: 
							memset(gps_data.latitude, 0, sizeof(gps_data.latitude)); 
							memcpy(gps_data.latitude, subString, subStringLength); 
							gps_data.latitude[subStringLength]='\0';
							break;
						case 4: 
							memset(gps_data.N_S, 0, sizeof(gps_data.N_S));
							memcpy(gps_data.N_S, subString, subStringLength); 
							gps_data.N_S[subStringLength]='\0';
							break;
						case 5: 
							memset(gps_data.longitude, 0, sizeof(gps_data.longitude)); 
							memcpy(gps_data.longitude, subString, subStringLength); 
							gps_data.longitude[subStringLength]='\0';
							break;
						case 6: 
							memset(gps_data.E_W, 0, sizeof(gps_data.E_W)); 
							memcpy(gps_data.E_W, subString, subStringLength); 
							gps_data.E_W[subStringLength]='\0';
							break;
						default: break;
					}
					subString = subStringNext;
					gps_data.isParseData = true;
					if(usefullBuffer[0] == 'A')				gps_data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')	gps_data.isUsefull = false;
				}
				else	GPS_LOG("解析错误2");
			}
		}
		//memcpy(&DATA_BUFFER+DATA_BUFFER_GPS_IDX,gps_data.latitude,29);
	}

}

/**
  * @brief  打印GPS数据
  * @param  none
  * @retval none
  */
static void printfGpsBuffer(void)
{
	if(gps_data.isParseData){
		gps_data.isParseData = false;
		GPS_LOG("gps_data.UTCTime = %s\r\n", gps_data.UTCTime);
		
		if(gps_data.isUsefull){
			gps_data.isUsefull = false;
			GPS_LOG("gps_data.latitude = %s\r\n", gps_data.latitude);
			GPS_LOG("gps_data.N_S = %s\r\n", gps_data.N_S);
			GPS_LOG("gps_data.longitude = %s\r\n", gps_data.longitude);
			GPS_LOG("gps_data.E_W = %s\r\n", gps_data.E_W);
		}
		else
		{
			GPS_LOG("GPS DATA is not usefull!\r\n");
		}
	}
}





