/*
 * svpwm.h
 *
 *  Created on: 14.01.2016
 *      Author: Michael Meiler
 */

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_

#include <arm_math.h>
#include "stm32f4xx_hal.h"

/***************************************************************
 *
 *					Defines
 *
 ***************************************************************/
/* Maximum Voltage applying */
#define _U_DC	560
/* Square Root of 3 */
#define _SQRT3	1.73205081
/* Pi divided by 3 */
#define _PIdiv3	1.04719755



/***************************************************************
 *
 *					Variables
 *
 ***************************************************************/
extern float32_t	U_alpha, U_beta, Theta;
extern float32_t	T_halfsample;
extern float32_t	counterfrequency;
extern float32_t	U_max;

extern uint16_t		*switchtime[3];


/*********************************************************************
*
*                   Functions
*
*********************************************************************/
//void svpwm(void);
uint8_t svpwm(float u_alpha, float u_beta, float t_s, float u_dc, uint16_t *tcm);

#endif /* INC_SVPWM_H_ */
