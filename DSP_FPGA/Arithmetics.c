#include <string.h>
#include <stdio.h>

#include "Arithmetics.h"
#include "qmath.h"

#include "CommonDefine.h"

///////// LockAZero(int16 Voltage)/////////
int16 APlusPeriod=0;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
int16 APlusNum=0;
int16 AMinusNum=0; 

///////// LockBZero(int16 Voltage)/////////
int16 BPlusPeriod=0;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
int16 BPlusNum=0;
int16 BMinusNum=0; 

///////// LockCZero(int16 Voltage)/////////
int16 CPlusPeriod=0;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
int16 CPlusNum=0;
int16 CMinusNum=0; 

int16 SinAcount=0;
int16 SinAcountAdd=0;
int16 SinBcount=0;
int16 SinBcountAdd=0;
int16 SinCcount=0;
int16 SinCcountAdd=0;

/*====================================================================================================
    PID Function
    
    The PID (���������֡�΢��) function is used in mainly
    control applications. PIDCalc performs one iteration of the PID
    algorithm.

=====================================================================================================*/

/*====================================================================================================
һ������λ��ʽPID 
//ȱ���ǣ�ÿ��PID�Ľ������Ҫ�ۼ�����ÿ�ν��������ǰ�����йأ��ۼӽ���������
=====================================================================================================*/

/*=====================================================================================================
  PosPID Calc
=====================================================================================================*/

double PosPIDCalc( PosPID *pp, double NextPoint )   //��ǰ������
{
    double  dError, Error;

        Error = pp->SetPoint -  NextPoint;          // ��ǰƫ�� 
        pp->SumError += Error;                      // ��ǰ����
              
        pp->PrevError = pp->LastError;
        pp->LastError = Error;
        dError = pp->LastError - pp->PrevError;     //��ǰ΢��
        
        
        return (pp->Proportion * Error              // ������
            +   pp->Integral * pp->SumError         // ������
            +   pp->Derivative * dError             // ΢����
        );
}

/*=====================================================================================================
  Initialize PosPID Structure
=====================================================================================================*/

void PosPIDInit (PosPID *pp)
{
    memset ( pp,0,sizeof(PosPID));   //��pp�ڲ������г�Ա��ʼ��Ϊ0  be included <string.h>
}



/*====================================================================================================
��������ʽ����PID
=====================================================================================================*/
/*
int32 IncPIDCalc( IncPID *pp, int16 NowPoint )   //��ǰ������ //����ֵΪADC����
{
//////       int16   Error;
       int32 Error;//////
           
       int32   PIDOut;       
       int32   FactPIDOut;   

	int32  PIDTemp;

//ע��:ʵ����PID�����ֵ��point���������point�����ֵ�Ļ�׼���Բ���ֵ��׼�ģ�
             //����׼��3v -> fff0��������ʵ��Ҫ�ĵõ��Ŀ���pwm
             //��ֵ����7.5v��Ӧ���ǲ���ֵ�õ��ģ���7.5->1875,��3v->750,���Ե�Ч�任�Ļ�����Ҫ
             //��point��ֵ����fff0/750��87.36, Ϊ�˸��򵥣����ǳ���64��������6λ������е�Ӱ��
             //ϵͳ�ȶ��ԣ��迼�ǣ��൱����PID���ں��ּ���һ��p���ڣ����С��87.36/64
             //���Ӱ�����ֱ�������PID����ʱ����


//����ADC�Ķ���	  	
        Error = pp->RefPoint-NowPoint;          // ��ǰƫ�� e[k]
//		Error = pp->RefPoint+NowPoint;  //������������
//���е�ERROR����ADC����
//a0,a1,a2����Q11����
//�������out������ADC�����Q11���깲ͬ����  
//???????????ע�⣺������int16��int32��ͬ���������Ӧ����int32��

//	PIDTemp=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp=PIDTemp>>11; //ADC����
//ADC����	
//	PIDOut=pp->PrevPIDOut+PIDTemp;
		//����ʽPID�㷨��u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2�ɻ��ֳ�����

//yybb

//////	PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
	PIDOut=Error*3;////////



//��ֹ���,PIDOut��int32��������11λ��ת��ΪADC���꣬Ȼ���ֹ����������int16Ϊ��Χ����ӦADC����ʵ���Ͼ���3*8v����
//////	if((PIDOut>>11)>2048)
//////	   	PIDOut=4194304;     
//////	else if((PIDOut>>11)<-2048)
//////		PIDOut=-4194304;
		
		
//			if(PIDOut>2048)//////
//	   	PIDOut=2048;     //////
//	else if(PIDOut<-2048)//////
//		PIDOut=-2048;//////

		

//yybb
//	if(PIDOut>2048)
//	   	PIDOut=2048;     
//	else if(PIDOut<-2048)
//		PIDOut=-2048;

//	ControlCValue=PIDOut;  //DA

	//����
//////	pp->PrevPIDOut=PIDOut;
//////	pp->PrevPrevError=pp->PrevError;	
//////	pp->PrevError=Error;

//����ֵ����,����ͳһΪADC����
//yybb
//////	FactPIDOut=PIDOut>>11;      //int32�ı���������֤����ֵ��32767��-32768֮�䣬������ADC���꣬�൱����24v��ֵ
//	FactPIDOut=PIDOut;
//////      return FactPIDOut;
return PIDOut;//////
}

*/


int32 CurIncPIDCalc( IncPID *pp, int16 NowPoint )   //��ǰ������ //����ֵΪADC����
{
      int16   Error;
//       int32 Error;//////
           
       int32   PIDOut;       
       int32   FactPIDOut;   

//ע��:ʵ����PID�����ֵ��point���������point�����ֵ�Ļ�׼���Բ���ֵ��׼�ģ�
             //����׼��3v -> fff0��������ʵ��Ҫ�ĵõ��Ŀ���pwm
             //��ֵ����7.5v��Ӧ���ǲ���ֵ�õ��ģ���7.5->1875,��3v->750,���Ե�Ч�任�Ļ�����Ҫ
             //��point��ֵ����fff0/750��87.36, Ϊ�˸��򵥣����ǳ���64��������6λ������е�Ӱ��
             //ϵͳ�ȶ��ԣ��迼�ǣ��൱����PID���ں��ּ���һ��p���ڣ����С��87.36/64
             //���Ӱ�����ֱ�������PID����ʱ����


//����ADC�Ķ���	  	
        Error = pp->RefPoint-NowPoint;          // ��ǰƫ�� e[k]
//		Error = pp->RefPoint+NowPoint;  //������������
//���е�ERROR����ADC����
//a0,a1,a2����Q11����
//�������out������ADC�����Q11���깲ͬ����  
//???????????ע�⣺������int16��int32��ͬ���������Ӧ����int32��

//	PIDTemp=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp=PIDTemp>>11; //ADC����
//ADC����	
//	PIDOut=pp->PrevPIDOut+PIDTemp;
		//����ʽPID�㷨��u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2�ɻ��ֳ�����

//yybb

	PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
//	PIDOut=Error*2;//*KKKK*4;//////



//��ֹ���,PIDOut��int32��������11λ��ת��ΪADC���꣬Ȼ���ֹ����������int16Ϊ��Χ����ӦADC����ʵ���Ͼ���3*8v����
	if((PIDOut>>11)>2048)
	   	PIDOut=4194304;     
	else if((PIDOut>>11)<-2048)
		PIDOut=-4194304;
		
//����by lw 
	if(LEFlag==1)
		{
		 	PIDOut=0;
		}

	//����
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;

//����ֵ����,����ͳһΪADC����
//yybb
	FactPIDOut=PIDOut>>11;      //int32�ı���������֤����ֵ��32767��-32768֮�䣬������ADC���꣬�൱����24v��ֵ
//	FactPIDOut=PIDOut;
      return FactPIDOut;
//return PIDOut;//////
}




int32 VolIncPIDCalc( IncPID *pp, int16 NowPoint )   //������ѹ��PID,��ռ�ձ����ƣ���3750-11250
{
       int16   Error;
         
       int32   PIDOut;       
       int32   FactPIDOut;   



//����ADC�Ķ���	  	
        Error =NowPoint-pp->RefPoint; //NowPoint-pp->RefPoint ;          // ��ǰƫ�� e[k]

//���е�ERROR����ADC����
//a0,a1,a2����Q11����
//�������out������ADC�����Q11���깲ͬ����  
//???????????ע�⣺������int16��int32��ͬ���������Ӧ����int32��
//PIDTemp1=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp1=PIDTemp1>>11; //ADC����
//PIDOut=pp->PrevPIDOut+PIDTemp1; //ADC����


//adc Q11

//PIDTemp1=pp->a0*Error;
//PIDTemp2=pp->a1*pp->PrevError;
//PIDOut=pp->PrevPIDOut+PIDTemp1-PIDTemp2;
//PIDTemp3=4106*512000;
//PIDTemp3=PIDTemp3>>11;
PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
		//����ʽPID�㷨��u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2�ɻ��ֳ�����

//��ֹ���
//yybb

//��ֹ���


	if((PIDOut>>11)>11250)
	   	PIDOut=23040000;     
	
	else if((PIDOut>>11)<3750)
		PIDOut=7680000;



	//����
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;
//yybb
//����ֵ����,����ͳһΪADC����  int32���ͣ���ֵ�޶���int16��Χ
     FactPIDOut=PIDOut>>11;  
//	FactPIDOut=PIDOut;
      return FactPIDOut;
}


int32 HarVolIncPIDCalc( IncPID *pp, int16 NowPoint )     //г����ѹ��PID,��ռ�ձ����ƣ���1-14999
{
       int16   Error;
         
       int32   PIDOut;       
       int32   FactPIDOut;   



//����ADC�Ķ���	  	
        Error =NowPoint-pp->RefPoint; //NowPoint-pp->RefPoint ;          // ��ǰƫ�� e[k]

//���е�ERROR����ADC����
//a0,a1,a2����Q11����
//�������out������ADC�����Q11���깲ͬ����  
//???????????ע�⣺������int16��int32��ͬ���������Ӧ����int32��
//PIDTemp1=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp1=PIDTemp1>>11; //ADC����
//PIDOut=pp->PrevPIDOut+PIDTemp1; //ADC����


//adc Q11

//PIDTemp1=pp->a0*Error;
//PIDTemp2=pp->a1*pp->PrevError;
//PIDOut=pp->PrevPIDOut+PIDTemp1-PIDTemp2;
//PIDTemp3=4106*512000;
//PIDTemp3=PIDTemp3>>11;
PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
		//����ʽPID�㷨��u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2�ɻ��ֳ�����

//��ֹ���
//yybb

//��ֹ���

	if((PIDOut>>11)>5303)		//7500/(2^0.5)
	   	PIDOut=10861160;     
	
	else if((PIDOut>>11)<-5303)
		PIDOut=-10861160;


	//����
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;
//yybb
//����ֵ����,����ͳһΪADC����  int32���ͣ���ֵ�޶���int16��Χ
     FactPIDOut=PIDOut>>11;  
//	FactPIDOut=PIDOut;
      return FactPIDOut;
}


//�ɱ�������΢�ֳ�����������ʽPID�е���ʱ����
void IncPIDTempVar( IncPID *pp)
{
pp->a0=(int32)pp->Proportion+pp->Integral+pp->Derivative;
pp->a1=(int32)pp->Proportion+((int32)pp->Derivative<<1);
pp->a2=(int32)pp->Derivative;
}


//PID�������㶨��   �˴�����Q11���꣬�������ɴ�16����С��ȷ��С�����0.00048828125
void PIDParaF2F(IncPID *pp,float32 Prop, float32 Integ,float32 Deriv)  //q15��ʽ
{	pp->Proportion=(int16)(Prop*2048);              // <<11;
	pp->Integral=(int16)(Integ*2048);              // <<11;
	pp->Derivative=(int16)(Deriv *2048);           // <<11;

}


void IncPIDInit (IncPID *pp)
{
    memset ( pp,0,sizeof(IncPID));   //��pp�ڲ������г�Ա��ʼ��Ϊ0  be included <string.h>
}



//����PID �㷨
//����:1.����PID  �����ṹ��2.��λ��
//���: �ο�������ʼ��λ

int16 LockPhaseINcPIDCal(IncPID *pp,int16 LockPhase_error)
{

   int16 Error;
   int32   PIDOut;       
   int16   FactPIDOut;   

   Error =LockPhase_error;
   PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);

   //�޶�PID ���
   if((PIDOut>>11)> 5 )
	   	PIDOut=10240;     
	else if((PIDOut>>11)< 0 )
		PIDOut= 0;



   //���²���
   pp->PrevPIDOut=PIDOut;
   pp->PrevPrevError=pp->PrevError;	
   pp->PrevError=Error;

   FactPIDOut=PIDOut>>11;
   return FactPIDOut;
   
}









/*====================================================================================================
    ������Ƶ�е�Ƶ�ʺ���λ Function

����������40-60Hz ,�ز�Ϊ20Khz, ����ӦΪ500-334����
=====================================================================================================*/

/*====================================================================================================
һ������������ѹ�������(��������)  
=====================================================================================================*/
//Ҫ��:ÿ���ز����ڶ�Ҫ����һ�Σ���ⷽ���������ҵ������ܣ�Ȼ�������������ܵ������ܵĹ����
//����:�����Ĳ�����ѹ
//���:�Ƿ����������1Ϊ�������

//A��
int16 LockAZero(int16 Voltage)
{	
	//�ҵ�������
	if(APlusPeriod==0)   
		{
		if(Voltage>=0)   
			APlusNum++;
		else
			if(APlusNum>0)
				APlusNum--;
			else
				APlusNum=0;

		if(APlusNum==2)  //һ������������
			{
			APlusPeriod=1;
			APlusNum=0;

			}
		}
	else 
		{
		
		if(APlusPeriod==1) //����������
			{//����Ƶ�ʿ�ʼ
			 //�����Ѿ�ȷ����������,ֻҪ�ҵ�һЩ����,����Ϊ���㣬�˴��Ĺ���������㷨������Ӧ�ÿ�����1%
			if(Voltage<=0)				
				AMinusNum++;				
			else			
				if(AMinusNum>0)
				AMinusNum--;
				else
				AMinusNum=0;
			
			if(AMinusNum==2)  //����ҵ�
				{
				AMinusNum=0;
				APlusPeriod=0;
				return 1;
				}

			else 
				return 0;
			}

		else 
			return 0;

		}

	return 0;
			
}



//B��
int16 LockBZero(int16 Voltage)
{	

	//�ҵ�������
	if(BPlusPeriod==0)   
		{
		if(Voltage>=0)  
			BPlusNum++;
		else
			if(BMinusNum>0)
				BMinusNum--;
			else
				BMinusNum=0;

		if(BPlusNum==2)  //һ������������
			{
			BPlusPeriod=1;
			BPlusNum=0;
			}
		}
	else 
		{
		
		if(BPlusPeriod==1) //����������
			{//����Ƶ�ʿ�ʼ
			
			if(Voltage<=0)				
				BMinusNum++;				
			else			
				if(BMinusNum>0)
				BMinusNum--;
				else
				BMinusNum=0;
			
			if(BMinusNum==2)  //����ҵ�
				{
				BMinusNum=0;
				BPlusPeriod=0;

				return 1;
				}

			else 
				return 0;
			}

		else 
			return 0;

		}

	return 0;
			
}

//C��
int16 LockCZero(int16 Voltage)
{	

	//�ҵ�������
	if(CPlusPeriod==0)   
		{
		if(Voltage>=0)   
			CPlusNum++;
		else
			if(CMinusNum>0)
				CMinusNum--;
			else
				CMinusNum=0;

		if(CPlusNum==2)  //һ������������
			{
			CPlusPeriod=1;
			CPlusNum=0;
			}
		}
	else 
		{
		
		if(CPlusPeriod==1) //����������
			{//����Ƶ�ʿ�ʼ

			if(Voltage<=0)				
				CMinusNum++;				
			else			
				if(CMinusNum>0)
				CMinusNum--;
				else
				CMinusNum=0;
			
			if(CMinusNum==2)  //����ҵ�
				{
				CMinusNum=0;
				CPlusPeriod=0;

				return 1;
				}

			else 
				return 0;
			}

		else 
			return 0;

		}

	return 0;
			
}


/*====================================================================================================
���������ο��������(��������)  
=====================================================================================================*/

//����:1. �ο��������ڵ���,2��ǰ��������
//���:�Ƿ����������1Ϊ�������

int16 LockRefZero(int16 Period, int16 NowPoint)
{
	if(NowPoint<Period)
		return 0;
	else
		return 1;
}




/*====================================================================================================
�ġ���������ѹ: �Ƿ���ͣ�������Ϊ���ඪʧ������pwm
//PS: �˴����ѹ
=====================================================================================================*/
//��ⷽ�����Ǽ��ÿ����Ƶ���ڲ�����ѹ��ֵ�����ߺ͹��͵Ĵ���̫�඼������Ϊ�ǹ���ѹ��Ƿ��ѹ

void LockVolSine(int16 AVol, int16 BVol, int16 CVol)
{

	if ((AVol < SineLowP) && (AVol > SineLowN))
		SinAcount++;
	else if ((AVol> SineHighP) || (AVol < SineHighN))
		SinAcountAdd++;

	if ((BVol < SineLowP) && (BVol > SineLowN))
		SinBcount++;
	else if ((BVol> SineHighP) || (AVol < SineHighN))
		SinBcountAdd++;
	
	if ((CVol < SineLowP) && (CVol > SineLowN))
		SinCcount++;
	else if ((CVol> SineHighP) || (CVol < SineHighN))
		SinCcountAdd++;

	if (Lock_Grid_Zero_flag_A== 1)
		{
		if ((SinAcount > LowDefine) || (SinBcount > LowDefine)||(SinCcount > LowDefine)) //Low Voltage Protection
			grid_vol_state= -1;
		else if ((SinAcountAdd > HighDefine) || (SinBcountAdd > HighDefine)|| (SinCcountAdd > HighDefine))//High Voltage Protection
			grid_vol_state = -2;
		else
			grid_vol_state = 1;


		//reset related Variables
		SinAcount = 0;
		SinBcount = 0;
		SinCcount = 0;
		SinAcountAdd = 0;
		SinBcountAdd = 0;
		SinCcountAdd = 0;

		}
/*	else if ((SinAcount > (LowDefine << 1)) ||( SinBcount > (LowDefine << 1))||( SinCcount > (LowDefine << 1)))
		{
		grid_vol_state = -1;
		}*/


}



/*====================================================================================================
   ���Ҳ�������
=====================================================================================================
//������-3.14��3.14�����Ҳ�
//����:1. һ�����ڵ��ܵ���,2.��ǰ����ĵ�
//����:��ǰ���Ӧ�����Ҳ�ֵQ15��ʽ
int16 SinProduce(int16 PeriodNum, int16 Point)
{

	int16 PointValue; //��ǰ���Ӧ�Ļ���

	if(Point>PeriodNum) // �Գ���һ������,�����»ص�������
		{
		Point=Point-PeriodNum;
		}

	if(Point<0)  //����sineƫ�õĴ��ڣ����ܻ���<0�����
		{
		Point=Point+PeriodNum;    
		}
	
	PointValue=-32768+lock_angle_inter*Point;   //��-1��ʼ����  Q15
		
	return qsin(PointValue);     //Q15��ʽ
}

*/



