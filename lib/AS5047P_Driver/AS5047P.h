/*
 * AS5047P.h
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#ifndef AS5047P_H_
#define AS5047P_H_

#include <stdint.h>
#include "AS5047P_Config.h"
#include "pll_filter.h"

typedef enum {
    SENSOR_DIR_NORMAL,
    SENSOR_DIR_REVERSE
}sensor_dir_t;

typedef struct {
    uint8_t spi_rx_buffer[2];
    
    int32_t count_raw;
    int32_t last_count_raw;
    int32_t countinuous_count;
    pll_filter_t count_filt;
    float continuous_deg;

    sensor_dir_t dir;
    float count_to_deg_scale;

    _Bool spi_transfer_flag;
    _Bool spi_transfer_done_flag;

    int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t);
    void (*spi_cs)(_Bool);
}AS5047P_t;

void AS5047P_spi_config(AS5047P_t *encd, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t), void (*spi_cs)(_Bool));
void AS5047P_init(AS5047P_t *encd, sensor_dir_t dir, float scale);
void AS5047P_set_pll_filter(AS5047P_t *encd, float bw_hz, float damping_ratio, float Ts);
void AS5047P_pll_filter_update(AS5047P_t *encd);
void AS5047P_update(AS5047P_t *encd);
void AS5047P_set_spi_transfer_done(AS5047P_t *encd);
float AS5047P_get_degree(AS5047P_t *encd);
float AS5047P_get_rpm(AS5047P_t *encd);
float AS5047P_get_continuous_degree(AS5047P_t *encd);

#endif /* AS5047P_H_ */
