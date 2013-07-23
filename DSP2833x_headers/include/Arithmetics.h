#ifndef Arithmetics_H
#define Arithmetics_H


#include "DSP2833x_Device.h"


//定义浮点位置式PID算法结构体
typedef struct PosPID {

        float32  SetPoint;           //  设定目标 Desired Value 参考值 

        float32  Proportion;         //  比例常数 Proportional Const
        float32  Integral;           //  积分常数 Integral Const
        float32  Derivative;         //  微分常数 Derivative Const

        float32  LastError;          //  Error[-1]
        float32  PrevError;          //  Error[-2]
        float32  SumError;           //  Sums of Errors

} PosPID;


//定义定点增量式PID算法结构体
typedef struct IncPID {

    	
    	
//其范围需要和ADC采样进来的值一致，12位ADC，所以int16足够  ADC定标 	
    	int16 RefPoint;           //  设定目标 Desired Value 参考值 
                                           //pid控制的参考值，参考输入                                           
//采用Q4定标，最大可达2047，最小精确到小数点后0.0625		
        int16  Proportion;         //  比例常数 Proportional Const      Kp
        int16  Integral;           //  积分常数 Integral Const            ki=Kp*(T/Ti)
        int16  Derivative;         //  微分常数 Derivative Const      Kd=Kp*(Td/T)
//同样是Q4定标，int32是因为害怕溢出
	 	int32 a0,a1,a2;   //增量式PID算法中引入的中间变量，由比例积分微分常数得到
		
//由ADC输入和参考决定，所以是ADC定标	
		int16  PrevPrevError;          //  Error[k-2]   
 		int16  PrevError;          //  Error[k-1]

//输出，定标由ADC和Q4标共同决定		
	  	int32  PrevPIDOut;    //前一次的输出值

} IncPID;




/*
//定义定点增量式PID算法结构体
typedef struct IncPID {

    	
    	
//其范围需要和ADC采样进来的值一致，12位ADC，所以int16足够  ADC定标 	
    	int16 RefPoint;           //  设定目标 Desired Value 参考值 
                                           //pid控制的参考值，参考输入                                           
//采用Q4定标，最大可达2047，最小精确到小数点后0.0625		
        int16  Proportion;         //  比例常数 Proportional Const      Kp
        int16  Integral;           //  积分常数 Integral Const            ki=Kp*(T/Ti)
        int16  Derivative;         //  微分常数 Derivative Const      Kd=Kp*(Td/T)
//同样是Q4定标，int32是因为害怕溢出
	 	int16 a0,a1,a2;   //增量式PID算法中引入的中间变量，由比例积分微分常数得到
		
//由ADC输入和参考决定，所以是ADC定标	
		int16  PrevPrevError;          //  Error[k-2]   
 		int16  PrevError;          //  Error[k-1]

//输出，定标由ADC和Q4标共同决定		
	  	int16  PrevPIDOut;    //前一次的输出值

} IncPID;

*/
#endif    //end of Arithmetics.h definition


