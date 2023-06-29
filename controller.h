/**
 * @file controller.c
 * @author Watanabe Shohei
 * @brief velocity format PID controller module
 * @version 0.2
 * @date 2021-09-20
 * 
 */

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#define __MYABS(x) ((x) > 0 ? (x) : (-x))
#define __MYLIMIT(x, lim) ((x) > (lim) ? (lim) : ((x) < -(lim) ? -(lim) : (x)))
#define __MYLIMIT2(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))
#define LIN_MAP(x, in_min, in_max, out_min, out_max) ((out_min) + ((in_val) - (in_min))/((in_max) - (in_min))*((out_max) - (out_min)))
#define __MYPI    (3.141592653589793)
#define __MYTWOPI (6.283185307179586)

typedef struct __PID_Controller
{
    float KP;
    float KI;
    float KD;
    float ref_lim;
    float Tc;
    float Ts;
    float __coeff_a;// Tc/(Tc+Ts)
    float __input[2];
    float __Iout;
    float __Pout;
    float __Dout[2];
    float __output_pre;
    uint8_t __index_count;
    

} PID_Controller;


/**
 * @brief initialize a PID controller
 * 
 * @param controller pointer to a controller
 * @param KP Propotial gain
 * @param KI Integral gain
 * @param KD Defferential gain
 * @param ref_lim limit value of output
 * @param f_cutoff cut-off freq.[Hz] of each LPF
 * @param fs sampling(+calclation) freq. [Hz]
 */
void PID_Controller_Init(PID_Controller* controller, float KP, float KI, float KD, float ref_lim, float f_cutoff, float fs);


/**
 * @brief initialize a PID controller(standard form)
 * 
 * @param controller pointer to a controller
 * @param KP Propotial gain
 * @param TI Integral time
 * @param TD Defferential time
 * @param ref_lim limit value of output
 * @param f_cutoff cut-off freq.[Hz] of each LPF
 * @param fs sampling(+calclation) freq. [Hz]
 */
void PID_Controller_Init_std(PID_Controller* controller, float KP, float TI, float TD, float ref_lim, float f_cutoff, float fs);

/**
 * @brief calculate manipulating value
 * 
 * @param controller pointer to the controller
 * @param val_ref reference value
 * @param val feedback value
 * @return float  manipulating value
 */
float PID_Controller_Calculate(PID_Controller* controller, float err_input);


void PI_Controller_Init(PID_Controller* controller, float KP, float KI, float ref_lim, float f_cut, float fs);
void PI_Controller_Init_std(PID_Controller* controller, float KP, float TI, float ref_lim, float f_cut, float fs);
float PI_Controller_Calculate(PID_Controller* controller, float err_input);
float PI_Controller_Calculate_nonfilter(PID_Controller* controller, float err_input);


#endif