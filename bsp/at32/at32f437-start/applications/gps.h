#ifndef __GPS_H_
#define __GPS_H_

//rt_device_t rt_gps_set_device(const char *name);
//void gps_get_data(void);

int gps_rmc_sample_entry_init(void);
void gps_rmc_sample_entry(void *p);

#endif

