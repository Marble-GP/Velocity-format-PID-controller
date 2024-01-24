/**
 * @file controller.c
 * @author S.Watanabe
 * @brief velocity format PID controller module
 * @version 0.3.3
 * @date 2024-01-24
 * 
 */


#include "controller.h"


void PID_Controller_Init(PID_Controller_t* controller, float KP, float KI, float KD, float ref_lim, float f_cut, float fs)
{
    controller->KP = KP;
    controller->KI = KI;
    controller->KD = KD;
    controller->ref_lim = __MYABS(ref_lim);
    controller->Ts = __MYABS(1.0f/fs);
    controller->Tc = 1.0f/((float)__MYTWOPI*f_cut);

    controller->__coeff_a = controller->Tc/(controller->Tc+controller->Ts);

    controller->__index_count = 0;

    controller->__Pout = 0.0f;
    controller->__Iout = 0.0f;
    controller->__Dout[0] = 0.0f;
    controller->__Dout[1] = 0.0f;
    controller->__input[0] = 0.0f;
    controller->__input[1] = 0.0f;
    controller->__output_pre = 0.0f;
}

void PID_Controller_Init_std(PID_Controller_t* controller, float KP, float TI, float TD, float ref_lim, float f_cut, float fs)
{
    controller->KP = KP;
    controller->KI = KP/TI;
    controller->KD = TD*KP;
    controller->ref_lim = __MYABS(ref_lim);
    controller->Tc = 1.0f / ((float)__MYTWOPI * f_cut);
    controller->Ts = __MYABS(1.0f/fs);


    controller->__coeff_a = controller->Tc / (controller->Tc + controller->Ts);

    controller->__index_count = 0;

    controller->__Pout = 0.0f;
    controller->__Iout = 0.0f;
    controller->__Dout[0] = 0.0f;
    controller->__Dout[1] = 0.0f;
    controller->__input[0] = 0.0f;
    controller->__input[1] = 0.0f;
    controller->__output_pre = 0.0f;
}

void PI_Controller_Init(PID_Controller_t* controller, float KP, float KI, float ref_lim, float f_cut, float fs)
{
    controller->KP = KP;
    controller->KI = KI;
    controller->KD = 0.0f;
    controller->ref_lim = __MYABS(ref_lim);
    controller->Tc = 1.0f / ((float)__MYTWOPI * f_cut);
    controller->Ts = __MYABS(1.0f / fs);


    controller->__coeff_a = controller->Tc / (controller->Tc + controller->Ts);

    controller->__index_count = 0;

    controller->__Pout = 0.0f;
    controller->__Iout = 0.0f;
    controller->__Dout[0] = 0.0f;
    controller->__Dout[1] = 0.0f;
    controller->__input[0] = 0.0f;
    controller->__input[1] = 0.0f;
    controller->__output_pre = 0.0f;
}

void PI_Controller_Init_std(PID_Controller_t* controller, float KP, float TI, float ref_lim, float f_cut, float fs)
{
    controller->KP = KP;
    controller->KI = KP/TI;
    controller->KD = 0.0f;
    controller->ref_lim = __MYABS(ref_lim);
    controller->Tc = 1.0f / ((float)__MYTWOPI * f_cut);
    controller->Ts = __MYABS(1.0f/fs);


    controller->__coeff_a = controller->Tc / (controller->Tc + controller->Ts);

    controller->__index_count = 0;

    controller->__Pout = 0.0f;
    controller->__Iout = 0.0f;
    controller->__Dout[0] = 0.0f;
    controller->__Dout[1] = 0.0f;
    controller->__input[0] = 0.0f;
    controller->__input[1] = 0.0f;
    controller->__output_pre = 0.0f;
}


float PID_Operate(PID_Controller_t* controller, float err_input)
{
    controller->__Pout = 
    controller->KP*(err_input - controller->__input[controller->__index_count^1])/(controller->Ts+controller->Tc)
     + controller->__coeff_a*controller->__Pout;//update Pout

    controller->__Iout = controller->KI*err_input;//update Iout

    controller->__Dout[controller->__index_count] = controller->KD*(err_input - 2.0f*controller->__input[controller->__index_count^1] + controller->__input[controller->__index_count]) / (controller->Ts + controller->Tc) / (controller->Ts + controller->Tc)
        + 2.0f*controller->__coeff_a*controller->__Dout[controller->__index_count^1] - controller->__Dout[controller->__index_count]*controller->__coeff_a * controller->__coeff_a;
    //update Dout[index]

    controller->__input[controller->__index_count] = err_input;//update previous input

    controller->__index_count ^= 1;// update 0/1 counter
    //return d(P+I+D)+output_pre
    return controller->__output_pre = __MYLIMIT((controller->__Pout + controller->__Iout + controller->__Dout[controller->__index_count^1])*controller->Ts + controller->__output_pre, controller->ref_lim);
}

float PI_Operate(PID_Controller_t* controller, float err_input)
{
    controller->__Pout =
        controller->KP * (err_input - controller->__input[0]) / (controller->Ts + controller->Tc)
        + controller->__coeff_a * controller->__Pout;//update Pout

    controller->__Iout = controller->KI * err_input;//update Iout

    controller->__input[0] = err_input;//update previous input
    //return d(P+I)+output_pre
    return controller->__output_pre = __MYLIMIT((controller->__Pout + controller->__Iout)*controller->Ts + controller->__output_pre, controller->ref_lim);
}

float PI_Operate_nonfiltering(PID_Controller_t* controller, float err_input)
{
        controller->__Pout =
        controller->KP * (err_input - controller->__input[0]) /controller->Ts;
        controller->__Iout = controller->KI * err_input;
        
        controller->__input[0] = err_input;//update previous input
    return controller->__output_pre = __MYLIMIT((controller->__Pout + controller->__Iout)*controller->Ts + controller->__output_pre, controller->ref_lim);
}

void PID_Reset(PID_Controller_t* controller)
{
    controller->__Pout = 0.0f;
    controller->__Iout = 0.0f;
    controller->__Dout[0] = 0.0f;
    controller->__Dout[1] = 0.0f;
    controller->__input[0] = 0.0f;
    controller->__input[1] = 0.0f;
    controller->__output_pre = 0.0f;
}