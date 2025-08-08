/*
 * tm1637.h
 *
 *  Created on: Jul 23, 2025
 *      Author: God
 */

#ifndef INC_TM1637_H_
#define INC_TM1637_H_

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

/*************************************************************************************************/
/** Macros and Definitions **/
/*************************************************************************************************/

typedef enum
{
  TM1637_ERR_NONE                   = 0,  /* No error */
  TM1637_ERR_ERROR,                       /* Acknowledge error */

} tm1637_err_t;

/*************************************************************************************************/
/** Strucs and Enums **/
/*************************************************************************************************/

typedef struct
{
  GPIO_TypeDef        *gpio_clk;
  GPIO_TypeDef        *gpio_dat;
  uint16_t            pin_clk;
  uint16_t            pin_dat;
  uint8_t             seg_cnt;

} tm1637_t;

/*************************************************************************************************/
/** Function Declarations **/
/*************************************************************************************************/

tm1637_err_t  tm1637_init(tm1637_t *handle);
tm1637_err_t  tm1637_brightness(tm1637_t *handle, uint8_t brightness_0_8);
void          tm1637_seg(tm1637_t *handle, uint8_t seg_1_6);
tm1637_err_t  tm1637_disp_raw(tm1637_t *handle, const uint8_t *data);
tm1637_err_t  tm1637_disp_str(tm1637_t *handle, const char *str);
tm1637_err_t  tm1637_disp_printf(tm1637_t *handle, const char *format, ...);
tm1637_err_t  tm1637_disp_clear(tm1637_t *handle);

#endif /* INC_TM1637_H_ */
