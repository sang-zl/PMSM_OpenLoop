/*
 *  svpwm.c
 *
 *  Created on: 09.01.2016
 *      Author: Michael Meiler
 */



#include <math.h>
#include <arm_math.h>
#include <stdint.h>
#include <svpwm.h>
#include <stdio.h>



/*-----------------------------------Space Vector Modulation -----
|  Function:	svpwm()
|
|  Purpose:	This subroutine determines the sector (1 out of 6 possible)
|		which the voltage vector should be modulated,
|		then calculates the on/off times, the counter values for switching respectively.
|		Addresses of calculated values are saved in pointer array "switchtime".
|
|
|
|  Description:	_______________________________________________________
|			   |         __|	     __|	     __|
|			   |	   |/  |	   |/  | 	   |/  |
|			   |    0__|  /_\       1__|  /_\       2__|  /_\
|			   |	   |\__| 	   |\__| 	   |\__|
|	 		 * | *	       |	       |	       |
|     		      *    |    *      +---------------|---------------|-------u
|                    *     |     *     |	       +---------------|-------v
|                    *     |     *     |	       |	       +-------w
|                     *    |    *      |	       |	       |
|                        * | *	     __|	     __|	     __|
|			   |  	   |/  |	   |/  |	   |/  |
|			   |    3__|  /_\       4__|  /_\       5__|  /_\
|			   |	   |\__|	   |\__|	   |\__|
|			   |___________|_______________|_______________|
|
|
|  Parameters:	[in]	None.
|			Uses global variables (Theta, U_alpha, U_beta), without changing them.
|
|		[out]	uint16_t *switchtime[3]
|
|  Returns:  	No return function used, but updates counter values in pointer array "switchtime".
|
*-------------------------------------------------------------------*/


//void svpwm(void)	{
//
//
//	uint8_t	sector = Theta/_PIdiv3;
//	float32_t	U_ref = hypotf(U_alpha,U_beta);
//	if (U_ref > U_max) {
//		U_ref = U_max;
//	}
//	float32_t	angle = Theta - (sector*_PIdiv3);
//	float32_t	U_ref_percent = (_SQRT3)*(U_ref/_U_DC); // previous: (2/_SQRT3)
//	float32_t	t_1 = U_ref_percent*arm_sin_f32(_PIdiv3-angle)*T_halfsample;
//	float32_t	t_2 = U_ref_percent*arm_sin_f32(angle)*T_halfsample;
//	float32_t	t_0 = T_halfsample - t_1 - t_2;
//	float32_t	t_0_half = t_0/2;
//
//
//	/* Switching counter values for Timer Interrupts */
//
//	/* Upper switches */
//	uint16_t	ontime_t_0_half = (t_0_half) * counterfrequency;
//	uint16_t	ontime_value_1 = (t_0_half + t_1) * counterfrequency;
//	uint16_t	ontime_value_2 = (t_0_half + t_2) * counterfrequency;
//	uint16_t	ontime_value_3 = (t_0_half + t_1 + t_2) * counterfrequency;
//
//	switch (sector)	{
//
//		/*					Upper switches			*/
//
//		/* Sector 1 */
//		case 0:		switchtime[0] = &ontime_t_0_half;
//					switchtime[1] = &ontime_value_1;
//					switchtime[2] = &ontime_value_3;
//				break;
//
//		/* Sector 2 */
//		case 1:		switchtime[0] = &ontime_value_2;
//					switchtime[1] = &ontime_t_0_half;
//					switchtime[2] = &ontime_value_3;
//				break;
//
//		/* Sector 3 */
//		case 2:		switchtime[0] = &ontime_value_3;
//					switchtime[1] = &ontime_t_0_half;
//					switchtime[2] = &ontime_value_1;
//				break;
//
//		/* Sector 4 */
//		case 3:		switchtime[0] = &ontime_value_3;
//					switchtime[1] = &ontime_value_2;
//					switchtime[2] = &ontime_t_0_half;
//				break;
//
//		/* Sector 5 */
//		case 4:		switchtime[0] = &ontime_value_1;
//					switchtime[1] = &ontime_value_3;
//					switchtime[2] = &ontime_t_0_half;
//				break;
//
//		/* Sector 6 */
//		case 5:		switchtime[0] = &ontime_t_0_half;
//					switchtime[1] = &ontime_value_3;
//					switchtime[2] = &ontime_value_2;
//				break;
//	}
//}

uint8_t svpwm(float u_alpha, float u_beta, float t_s, float u_dc, uint16_t *tcm)
{
    float u_ref1,u_ref2, u_ref3;
    float x, y, z;
    float t1, t2;
    float ta, tb, tc;
    uint8_t sector= 0;

    u_ref1 = u_beta;
    u_ref2 = (_SQRT3 * u_alpha - u_beta) / 2.0f;
    u_ref3 = (-_SQRT3 * u_alpha - u_beta) / 2.0f;

    //扇区判断
    if (u_ref1 > 0.0f)
        sector += 1;
    if (u_ref2 > 0.0f)
        sector += 2;
    if (u_ref3 > 0.0f)
        sector += 4;

    //计算XYZ
    x = _SQRT3 * u_beta * t_s / u_dc;
    y = t_s / u_dc * (1.5f * u_alpha + u_beta * _SQRT3 / 2.0f);
    z = t_s / u_dc * (-1.5f * u_alpha + u_beta * _SQRT3 / 2.0f);

    switch (sector) {
        case 1:
            t1 = z;
            t2 = y;
            break;
        case 2:
            t1 = y;
            t2 = -x;
            break;
        case 3:
            t1 = -z;
            t2 = x;
            break;
        case 4:
            t1 = -x;
            t2 = z;
            break;
        case 5:
            t1 = x;
            t2 = -y;
            break;
        case 6:
            t1 = -y;
            t2 = -z;
    }

    if(t_s < t1 + t2)
    {
        float t1_tmp;
        t1_tmp = t1;
        t1 = t1 * t_s / (t1 + t2);
        t2 = t2 * t_s / (t1_tmp + t2);
    }
//    printf("%.10f,%.10f,%.10f\r\n",t1, t2, t_s);
    ta = (uint16_t)((t_s - t1 - t2) / 4.0f);
    tb = (uint16_t)((t_s + t1 - t2) / 4.0f);
    tc = (uint16_t)((t_s + t1 + t2) / 4.0f);
    switch (sector) {
        case 1:
            tcm[0] = tb;
            tcm[1] = ta;
            tcm[2] = tc;
            break;
        case 2:
            tcm[0] = ta;
            tcm[1] = tc;
            tcm[2] = tb;
            break;
        case 3:
            tcm[0] = ta;
            tcm[1] = tb;
            tcm[2] = tc;
            break;
        case 4:
            tcm[0] = tc;
            tcm[1] = tb;
            tcm[2] = ta;
            break;
        case 5:
            tcm[0] = tc;
            tcm[1] = ta;
            tcm[2] = tb;
            break;
        case 6:
            tcm[0] = tb;
            tcm[1] = tc;
            tcm[2] = ta;
    }
    return sector;
}