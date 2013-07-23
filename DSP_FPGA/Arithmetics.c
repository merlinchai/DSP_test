#include <string.h>
#include <stdio.h>

#include "Arithmetics.h"
#include "qmath.h"

#include "CommonDefine.h"

///////// LockAZero(int16 Voltage)/////////
int16 APlusPeriod=0;  //正半周标志位0为不处于,1为处于正半周
int16 APlusNum=0;
int16 AMinusNum=0; 

///////// LockBZero(int16 Voltage)/////////
int16 BPlusPeriod=0;  //正半周标志位0为不处于,1为处于正半周
int16 BPlusNum=0;
int16 BMinusNum=0; 

///////// LockCZero(int16 Voltage)/////////
int16 CPlusPeriod=0;  //正半周标志位0为不处于,1为处于正半周
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
    
    The PID (比例、积分、微分) function is used in mainly
    control applications. PIDCalc performs one iteration of the PID
    algorithm.

=====================================================================================================*/

/*====================================================================================================
一、浮点位置式PID 
//缺点是：每次PID的结果都需要累加误差，即每次结果都和以前数据有关，累加结果容易溢出
=====================================================================================================*/

/*=====================================================================================================
  PosPID Calc
=====================================================================================================*/

double PosPIDCalc( PosPID *pp, double NextPoint )   //当前采样点
{
    double  dError, Error;

        Error = pp->SetPoint -  NextPoint;          // 当前偏差 
        pp->SumError += Error;                      // 当前积分
              
        pp->PrevError = pp->LastError;
        pp->LastError = Error;
        dError = pp->LastError - pp->PrevError;     //当前微分
        
        
        return (pp->Proportion * Error              // 比例项
            +   pp->Integral * pp->SumError         // 积分项
            +   pp->Derivative * dError             // 微分项
        );
}

/*=====================================================================================================
  Initialize PosPID Structure
=====================================================================================================*/

void PosPIDInit (PosPID *pp)
{
    memset ( pp,0,sizeof(PosPID));   //将pp内部的所有成员初始化为0  be included <string.h>
}



/*====================================================================================================
二、增量式定点PID
=====================================================================================================*/
/*
int32 IncPIDCalc( IncPID *pp, int16 NowPoint )   //当前采样点 //返回值为ADC定标
{
//////       int16   Error;
       int32 Error;//////
           
       int32   PIDOut;       
       int32   FactPIDOut;   

	int32  PIDTemp;

//注意:实际上PID输出的值是point；但是这个point算出来值的基准是以采样值基准的，
             //即基准是3v -> fff0，而我们实际要的得到的控制pwm
             //的值是以7.5v对应三角波峰值得到的，即7.5->1875,即3v->750,所以等效变换的话，需要
             //将point的值除以fff0/750＝87.36, 为了更简单，我们除以64，即右移6位。这个有点影响
             //系统稳定性，需考虑，相当于在PID环节后，又加了一个p环节，其大小是87.36/64
             //这个影响可以直接在设计PID参数时考虑


//采用ADC的定标	  	
        Error = pp->RefPoint-NowPoint;          // 当前偏差 e[k]
//		Error = pp->RefPoint+NowPoint;  //采样电流反向
//所有的ERROR采用ADC定标
//a0,a1,a2采用Q11定标
//所有输出out定标由ADC定标和Q11定标共同决定  
//???????????注意：计算中int16和int32共同，则计算结果应该是int32吧

//	PIDTemp=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp=PIDTemp>>11; //ADC定标
//ADC定标	
//	PIDOut=pp->PrevPIDOut+PIDTemp;
		//增量式PID算法：u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2由积分常数算

//yybb

//////	PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
	PIDOut=Error*3;////////



//防止溢出,PIDOut是int32，先右移11位，转化为ADC定标，然后防止溢出。如果以int16为范围，对应ADC定标实际上就是3*8v饱和
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

	//更新
//////	pp->PrevPIDOut=PIDOut;
//////	pp->PrevPrevError=pp->PrevError;	
//////	pp->PrevError=Error;

//返回值计算,重新统一为ADC定标
//yybb
//////	FactPIDOut=PIDOut>>11;      //int32的变量，但保证了其值在32767－-32768之间，由于是ADC定标，相当于是24v的值
//	FactPIDOut=PIDOut;
//////      return FactPIDOut;
return PIDOut;//////
}

*/


int32 CurIncPIDCalc( IncPID *pp, int16 NowPoint )   //当前采样点 //返回值为ADC定标
{
      int16   Error;
//       int32 Error;//////
           
       int32   PIDOut;       
       int32   FactPIDOut;   

//注意:实际上PID输出的值是point；但是这个point算出来值的基准是以采样值基准的，
             //即基准是3v -> fff0，而我们实际要的得到的控制pwm
             //的值是以7.5v对应三角波峰值得到的，即7.5->1875,即3v->750,所以等效变换的话，需要
             //将point的值除以fff0/750＝87.36, 为了更简单，我们除以64，即右移6位。这个有点影响
             //系统稳定性，需考虑，相当于在PID环节后，又加了一个p环节，其大小是87.36/64
             //这个影响可以直接在设计PID参数时考虑


//采用ADC的定标	  	
        Error = pp->RefPoint-NowPoint;          // 当前偏差 e[k]
//		Error = pp->RefPoint+NowPoint;  //采样电流反向
//所有的ERROR采用ADC定标
//a0,a1,a2采用Q11定标
//所有输出out定标由ADC定标和Q11定标共同决定  
//???????????注意：计算中int16和int32共同，则计算结果应该是int32吧

//	PIDTemp=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp=PIDTemp>>11; //ADC定标
//ADC定标	
//	PIDOut=pp->PrevPIDOut+PIDTemp;
		//增量式PID算法：u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2由积分常数算

//yybb

	PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
//	PIDOut=Error*2;//*KKKK*4;//////



//防止溢出,PIDOut是int32，先右移11位，转化为ADC定标，然后防止溢出。如果以int16为范围，对应ADC定标实际上就是3*8v饱和
	if((PIDOut>>11)>2048)
	   	PIDOut=4194304;     
	else if((PIDOut>>11)<-2048)
		PIDOut=-4194304;
		
//调试by lw 
	if(LEFlag==1)
		{
		 	PIDOut=0;
		}

	//更新
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;

//返回值计算,重新统一为ADC定标
//yybb
	FactPIDOut=PIDOut>>11;      //int32的变量，但保证了其值在32767－-32768之间，由于是ADC定标，相当于是24v的值
//	FactPIDOut=PIDOut;
      return FactPIDOut;
//return PIDOut;//////
}




int32 VolIncPIDCalc( IncPID *pp, int16 NowPoint )   //基波电压环PID,有占空比限制，从3750-11250
{
       int16   Error;
         
       int32   PIDOut;       
       int32   FactPIDOut;   



//采用ADC的定标	  	
        Error =NowPoint-pp->RefPoint; //NowPoint-pp->RefPoint ;          // 当前偏差 e[k]

//所有的ERROR采用ADC定标
//a0,a1,a2采用Q11定标
//所有输出out定标由ADC定标和Q11定标共同决定  
//???????????注意：计算中int16和int32共同，则计算结果应该是int32吧
//PIDTemp1=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp1=PIDTemp1>>11; //ADC定标
//PIDOut=pp->PrevPIDOut+PIDTemp1; //ADC定标


//adc Q11

//PIDTemp1=pp->a0*Error;
//PIDTemp2=pp->a1*pp->PrevError;
//PIDOut=pp->PrevPIDOut+PIDTemp1-PIDTemp2;
//PIDTemp3=4106*512000;
//PIDTemp3=PIDTemp3>>11;
PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
		//增量式PID算法：u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2由积分常数算

//防止溢出
//yybb

//防止溢出


	if((PIDOut>>11)>11250)
	   	PIDOut=23040000;     
	
	else if((PIDOut>>11)<3750)
		PIDOut=7680000;



	//更新
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;
//yybb
//返回值计算,重新统一为ADC定标  int32类型，但值限定在int16范围
     FactPIDOut=PIDOut>>11;  
//	FactPIDOut=PIDOut;
      return FactPIDOut;
}


int32 HarVolIncPIDCalc( IncPID *pp, int16 NowPoint )     //谐波电压环PID,无占空比限制，从1-14999
{
       int16   Error;
         
       int32   PIDOut;       
       int32   FactPIDOut;   



//采用ADC的定标	  	
        Error =NowPoint-pp->RefPoint; //NowPoint-pp->RefPoint ;          // 当前偏差 e[k]

//所有的ERROR采用ADC定标
//a0,a1,a2采用Q11定标
//所有输出out定标由ADC定标和Q11定标共同决定  
//???????????注意：计算中int16和int32共同，则计算结果应该是int32吧
//PIDTemp1=pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError;
//	PIDTemp1=PIDTemp1>>11; //ADC定标
//PIDOut=pp->PrevPIDOut+PIDTemp1; //ADC定标


//adc Q11

//PIDTemp1=pp->a0*Error;
//PIDTemp2=pp->a1*pp->PrevError;
//PIDOut=pp->PrevPIDOut+PIDTemp1-PIDTemp2;
//PIDTemp3=4106*512000;
//PIDTemp3=PIDTemp3>>11;
PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);
		//增量式PID算法：u(k)=u(k-1)+a0*e(k)-a1*e(k-1)+a2*e(k-2)
       //    a0,a1,a2由积分常数算

//防止溢出
//yybb

//防止溢出

	if((PIDOut>>11)>5303)		//7500/(2^0.5)
	   	PIDOut=10861160;     
	
	else if((PIDOut>>11)<-5303)
		PIDOut=-10861160;


	//更新
	pp->PrevPIDOut=PIDOut;
	pp->PrevPrevError=pp->PrevError;	
	pp->PrevError=Error;
//yybb
//返回值计算,重新统一为ADC定标  int32类型，但值限定在int16范围
     FactPIDOut=PIDOut>>11;  
//	FactPIDOut=PIDOut;
      return FactPIDOut;
}


//由比例积分微分常数计算增量式PID中的临时变量
void IncPIDTempVar( IncPID *pp)
{
pp->a0=(int32)pp->Proportion+pp->Integral+pp->Derivative;
pp->a1=(int32)pp->Proportion+((int32)pp->Derivative<<1);
pp->a2=(int32)pp->Derivative;
}


//PID参数浮点定标   此处采用Q11定标，参数最大可达16，最小精确到小数点后0.00048828125
void PIDParaF2F(IncPID *pp,float32 Prop, float32 Integ,float32 Deriv)  //q15格式
{	pp->Proportion=(int16)(Prop*2048);              // <<11;
	pp->Integral=(int16)(Integ*2048);              // <<11;
	pp->Derivative=(int16)(Deriv *2048);           // <<11;

}


void IncPIDInit (IncPID *pp)
{
    memset ( pp,0,sizeof(IncPID));   //将pp内部的所有成员初始化为0  be included <string.h>
}



//锁相PID 算法
//输入:1.锁相PID  参数结构体2.相位差
//输出: 参考量的起始相位

int16 LockPhaseINcPIDCal(IncPID *pp,int16 LockPhase_error)
{

   int16 Error;
   int32   PIDOut;       
   int16   FactPIDOut;   

   Error =LockPhase_error;
   PIDOut=pp->PrevPIDOut+(pp->a0*Error-pp->a1*pp->PrevError+pp->a2*pp->PrevPrevError);

   //限定PID 输出
   if((PIDOut>>11)> 5 )
	   	PIDOut=10240;     
	else if((PIDOut>>11)< 0 )
		PIDOut= 0;



   //更新参数
   pp->PrevPIDOut=PIDOut;
   pp->PrevPrevError=pp->PrevError;	
   pp->PrevError=Error;

   FactPIDOut=PIDOut>>11;
   return FactPIDOut;
   
}









/*====================================================================================================
    锁定工频市电频率和相位 Function

适用于锁定40-60Hz ,载波为20Khz, 即对应为500-334个点
=====================================================================================================*/

/*====================================================================================================
一、锁定采样电压正弦零点(从正到负)  
=====================================================================================================*/
//要求:每个载波周期都要调用一次，检测方法就是先找到正半周，然后锁定从正半周到负半周的过零点
//输入:单相电的采样电压
//输出:是否锁定到零点1为锁定零点

//A相
int16 LockAZero(int16 Voltage)
{	
	//找到正半周
	if(APlusPeriod==0)   
		{
		if(Voltage>=0)   
			APlusNum++;
		else
			if(APlusNum>0)
				APlusNum--;
			else
				APlusNum=0;

		if(APlusNum==2)  //一定处于正半周
			{
			APlusPeriod=1;
			APlusNum=0;

			}
		}
	else 
		{
		
		if(APlusPeriod==1) //处于正半周
			{//锁定频率开始
			 //由于已经确认了正半周,只要找到一些负点,即认为过零，此处的过零误差由算法决定，应该控制在1%
			if(Voltage<=0)				
				AMinusNum++;				
			else			
				if(AMinusNum>0)
				AMinusNum--;
				else
				AMinusNum=0;
			
			if(AMinusNum==2)  //零点找到
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



//B相
int16 LockBZero(int16 Voltage)
{	

	//找到正半周
	if(BPlusPeriod==0)   
		{
		if(Voltage>=0)  
			BPlusNum++;
		else
			if(BMinusNum>0)
				BMinusNum--;
			else
				BMinusNum=0;

		if(BPlusNum==2)  //一定处于正半周
			{
			BPlusPeriod=1;
			BPlusNum=0;
			}
		}
	else 
		{
		
		if(BPlusPeriod==1) //处于正半周
			{//锁定频率开始
			
			if(Voltage<=0)				
				BMinusNum++;				
			else			
				if(BMinusNum>0)
				BMinusNum--;
				else
				BMinusNum=0;
			
			if(BMinusNum==2)  //零点找到
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

//C相
int16 LockCZero(int16 Voltage)
{	

	//找到正半周
	if(CPlusPeriod==0)   
		{
		if(Voltage>=0)   
			CPlusNum++;
		else
			if(CMinusNum>0)
				CMinusNum--;
			else
				CMinusNum=0;

		if(CPlusNum==2)  //一定处于正半周
			{
			CPlusPeriod=1;
			CPlusNum=0;
			}
		}
	else 
		{
		
		if(CPlusPeriod==1) //处于正半周
			{//锁定频率开始

			if(Voltage<=0)				
				CMinusNum++;				
			else			
				if(CMinusNum>0)
				CMinusNum--;
				else
				CMinusNum=0;
			
			if(CMinusNum==2)  //零点找到
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
二、锁定参考正弦零点(从正到负)  
=====================================================================================================*/

//输入:1. 参考正弦周期点数,2当前计数点数
//输出:是否锁定到零点1为锁定零点

int16 LockRefZero(int16 Period, int16 NowPoint)
{
	if(NowPoint<Period)
		return 0;
	else
		return 1;
}




/*====================================================================================================
四、检测三相电压: 是否过低，过低认为三相丢失，封锁pwm
//PS: 此处相电压
=====================================================================================================*/
//检测方法就是检测每个工频周期采样电压的值，过高和过低的次数太多都可以认为是过电压或欠电压

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
   正弦波发生器
=====================================================================================================
//产生从-3.14到3.14的正弦波
//输入:1. 一个周期的总点数,2.当前计算的点
//返回:当前点对应的正弦波值Q15格式
int16 SinProduce(int16 PeriodNum, int16 Point)
{

	int16 PointValue; //当前点对应的弧度

	if(Point>PeriodNum) // 以超过一个周期,则重新回到负半周
		{
		Point=Point-PeriodNum;
		}

	if(Point<0)  //由于sine偏置的存在，可能会有<0的情况
		{
		Point=Point+PeriodNum;    
		}
	
	PointValue=-32768+lock_angle_inter*Point;   //从-1开始递增  Q15
		
	return qsin(PointValue);     //Q15格式
}

*/



