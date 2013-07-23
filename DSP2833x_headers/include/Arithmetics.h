#ifndef Arithmetics_H
#define Arithmetics_H


#include "DSP2833x_Device.h"


//���帡��λ��ʽPID�㷨�ṹ��
typedef struct PosPID {

        float32  SetPoint;           //  �趨Ŀ�� Desired Value �ο�ֵ 

        float32  Proportion;         //  �������� Proportional Const
        float32  Integral;           //  ���ֳ��� Integral Const
        float32  Derivative;         //  ΢�ֳ��� Derivative Const

        float32  LastError;          //  Error[-1]
        float32  PrevError;          //  Error[-2]
        float32  SumError;           //  Sums of Errors

} PosPID;


//���嶨������ʽPID�㷨�ṹ��
typedef struct IncPID {

    	
    	
//�䷶Χ��Ҫ��ADC����������ֵһ�£�12λADC������int16�㹻  ADC���� 	
    	int16 RefPoint;           //  �趨Ŀ�� Desired Value �ο�ֵ 
                                           //pid���ƵĲο�ֵ���ο�����                                           
//����Q4���꣬���ɴ�2047����С��ȷ��С�����0.0625		
        int16  Proportion;         //  �������� Proportional Const      Kp
        int16  Integral;           //  ���ֳ��� Integral Const            ki=Kp*(T/Ti)
        int16  Derivative;         //  ΢�ֳ��� Derivative Const      Kd=Kp*(Td/T)
//ͬ����Q4���꣬int32����Ϊ�������
	 	int32 a0,a1,a2;   //����ʽPID�㷨��������м�������ɱ�������΢�ֳ����õ�
		
//��ADC����Ͳο�������������ADC����	
		int16  PrevPrevError;          //  Error[k-2]   
 		int16  PrevError;          //  Error[k-1]

//�����������ADC��Q4�깲ͬ����		
	  	int32  PrevPIDOut;    //ǰһ�ε����ֵ

} IncPID;




/*
//���嶨������ʽPID�㷨�ṹ��
typedef struct IncPID {

    	
    	
//�䷶Χ��Ҫ��ADC����������ֵһ�£�12λADC������int16�㹻  ADC���� 	
    	int16 RefPoint;           //  �趨Ŀ�� Desired Value �ο�ֵ 
                                           //pid���ƵĲο�ֵ���ο�����                                           
//����Q4���꣬���ɴ�2047����С��ȷ��С�����0.0625		
        int16  Proportion;         //  �������� Proportional Const      Kp
        int16  Integral;           //  ���ֳ��� Integral Const            ki=Kp*(T/Ti)
        int16  Derivative;         //  ΢�ֳ��� Derivative Const      Kd=Kp*(Td/T)
//ͬ����Q4���꣬int32����Ϊ�������
	 	int16 a0,a1,a2;   //����ʽPID�㷨��������м�������ɱ�������΢�ֳ����õ�
		
//��ADC����Ͳο�������������ADC����	
		int16  PrevPrevError;          //  Error[k-2]   
 		int16  PrevError;          //  Error[k-1]

//�����������ADC��Q4�깲ͬ����		
	  	int16  PrevPIDOut;    //ǰһ�ε����ֵ

} IncPID;

*/
#endif    //end of Arithmetics.h definition


