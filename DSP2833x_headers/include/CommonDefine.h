
//切记切记：全局变量定义不要在.h文件中定义，会导致变量重复定义
//.h文件中放define（没有重复定义问题），变量形式的定义（比如结构体的内部定义，不是变量定义），
//变量声明，以及其他的.h文件。
//全局变量的定义要放到.c文件中



#ifndef CommonDefine_H
#define CommonDefine_H

//-------------------------------------------------------------------------

//头文件

#include "Arithmetics.h"   


//-------------------------------------------------------------------------
//宏定义
#define VersionN 9999 //version,9999=debug version

//#define EnergyPtDegree 3600000  

//grid low and high voltage
#define SineLowP 984
#define SineLowN -984
#define SineHighP 1640 
#define SineHighN -1640

//int16 IsVolNormal=22;   //
#define LowDefine 1000//195   //正弦低压判定  10K=200
#define  HighDefine 1000//5   //正弦高压判定 10K=5

//grid frequency
#define GridFreq 50
#define SPeriodNor 400

//sample coefficent
#define VolACSampCoef 2555    //0.2555 
#define CurACSampCoef 144    //0.0144
#define VolDCSampCoef 147    //0.0147 
#define CurDCSampCoef 103     //0.0103  

//adc define
#define ADC_CKPS   0x1   // ADC module clock内部分频 = HSPCLK/2*ADC_CKPS
#define ADC_CPS 0x0   // ADC module clock额外的一次内部二分频 
#define ADC_SHCLK  0xf   // S/H width in ADC module periods = 16 ADC clocks

//ADC define
#define AdcVOffset 2048        
//#define AdcCOffset 2034
#define AdcBusOffset  0  //8           

//DC define
#define DCAVOffset 0;        
#define DCACOffset 0;//2048
#define DCBVOffset 0;        
#define DCBCOffset 0;//2048


//sin的偏置
#define SineErrOffset 2//由于电网电压过零点检测至少滞后2个点，所以补偿要提前2个点
#define SinePhaOffset 0 //33 30度 //相电压和线电压的相位差
//----------------------------------------------------------------------------

//epwm define DCAC
#define EPWM_PERIOD 15000   //150M  100us 10k 单边斜坡
#define EPWMU_MIN_DB 300   //2us
#define EPWMV_MIN_DB 300   //2us
#define EPWMW_MIN_DB 300   //2us


#define EPWMDC_PERIOD 2500
#define DCADuty 1
#define DCBDuty 1
#define EPWMGD1_MIN_DB 1
#define EPWMGD2_MIN_DB 1
#define EPWMGD3_MIN_DB 1
#define EPWMGD4_MIN_DB 1


#define LD_IM 3 //直接装载
#define CSF_DIS 0	//软件强制输出不是能
#define CSF_HIGH 2	//软件强制置高
#define CSF_LOW 1	//软件强制置低



//逆变器相关参数
#define AD_Bus_rec  1135        //1135       //不控整流母线电压，用于开启逆变
#define AD_Bus_ref  1513              //1475 母线电压参考，该参考为稳定母线电压的参考

#define currref_lw  100             //电流环限流值

//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define Set4094Clk	GpioDataRegs.GPASET.bit.GPIO12=1   //4094 clk  
#define Set4094Dat	GpioDataRegs.GPASET.bit.GPIO13=1 //4094 data 
#define Set4094Str	GpioDataRegs.GPASET.bit.GPIO14=1 //4094 str  

#define Clr4094Clk	GpioDataRegs.GPACLEAR.bit.GPIO12=1   //4094 clk  
#define Clr4094Dat	GpioDataRegs.GPACLEAR.bit.GPIO13=1 //4094 data 
#define Clr4094Str	GpioDataRegs.GPACLEAR.bit.GPIO14=1 //4094 str  


#define SET_DequiverTicksShort 4   //  key Dequiver time of short key 18ms*4

#define LED_a 0x0088
#define LED_b 0x0083
#define LED_c 0x00c6
#define LED_d 0x00a1
#define LED_e 0x0086
#define LED_f 0x008e
#define LED_g 0x00c2
#define LED_h 0x0089
#define LED_i 0x00cf
#define LED_j 0x00e1
#define LED_k 0x0085
#define LED_l 0x00c7
#define LED_m 0x00aa
#define LED_N 0x00c8
#define LED_n 0x00ab
#define LED_o 0x00a3
#define LED_p 0x008c
#define LED_q 0x0098
#define LED_r 0x00af
#define LED_s 0x0092
#define LED_t 0x0087
#define LED_T 0x00ce
#define LED_U 0x00c1
#define LED_u 0x00e3
#define LED_v 0x0081
#define LED_w 0x0095
#define LED_x 0x00c9
#define LED_y 0x0091
#define LED_z 0x00b6

#define LED_1 0x00f9
#define LED_2 0x00a4
#define LED_3 0x00b0
#define LED_4 0x0099
#define LED_5 0x0092
#define LED_6 0x0082
#define LED_7 0x00f8
#define LED_8 0x0080
#define LED_9 0x0090
#define LED_0 0x00c0
#define LED_DP 0x0080

#define Led_Bar	0x00bf
#define LED_Null 0x00ff


#define MenuFirstMax 10 //9
#define MenuFnnMax 3
#define MenuTwoMax 19
///////////////////////////////////////////////////////////////////
//sample coefficent
#define VolSampCoef 28 //0.356 
#define CursampCoef 333 //407 //一拖二333 

//0.1 degree=360000w*s   but magnify 10 
#define EnergyPtDegree 1781156250//1457459578//一拖二1781156250


//vol curr loop p i 
#define Voltage_base_Ref 1312////对应输出基波有效值为220 //600 对应调压器电压150V左右
#define Duty_DC_up_limit 11250 //基波占空比比较器值上限为11250,为150%
#define Duty_DC_low_limit 3750 //基波占空比比较器值下限为3750,为50%
#define Duty_up_limit 14999 //整理占空比限制为1到14999
#define Duty_low_limit 1

#define MaxVolLpP 50  //  /10
#define MinVolLpP  10
#define MaxVolIFre 20 //*10
#define MinVolIFre  2  //*10
#define MaxCurrLpP 100   // /10
#define MinCurrLpP  10
#define MaxCurrIFre 250  //*10
#define MinCurrIFre 50 //*10
#define StepVolLpP 1
#define StepVolIFre 1  //*10
#define StepCurrLpP 1
#define StepCurrIFre 5 //*10
//switch freq
#define MaxSwFreq 200
#define MinSwFreq  10
#define StepSwFreq 1
//dc voltage
#define MaxDcVol 400
#define MinDcVol 200
#define StepDcVol 5
//dc real voltage: dc ref vol
#define MaxRVol 800
#define MinRVol 500
#define StepRVol 100
#define SmallStepRvol 5
#define MaxRVoffset 100
#define MinRVoffset -100
#define StepRVoffset 5
#define MinPaVol 100
#define MaxPaVol 250
//default s-code (backdoor)
#define BackdoorCode 432231

#define EEPErrNumer 16 //记录错误个数


//////////////////////////////////////////////////////////
///触摸屏程序宏定义
/////////////////////////////////////////////////////////

/* These macro for Mudbus function code */
#define ReadMulCoilSta 0x02 //read multi coil function
#define ForceSingleCoil 0x05 // 0X force single coil function
#define ReadHoldReg 0x03 // 4X read input register
#define PresetMulReg 0x10 // 4X write multi register

/* These macro describe index of communiction message from start to end */
#define CommIndexSlaveID 0 // macro for slave ID
#define CommIndexFunction 1 // macro for function code
#define CommIndexStartAdrHi 2 // macro for data hight adr
#define CommIndexStartAdrLo 3 // macro for data low adr
#define CommIndexNoPointHi 4 // macro for hight quantity datas
#define CommIndexNoPointLo 5 // macro for low quantity datas

/* These macro describe recieve message lenght of different function code */
/* But not include data lenght ,the data is master write to slave */
/* because we don't know the data lenght prior, it depend on ComBuf[ComIndexNoPoint] value */
#define ReadCoilStaIndexEnd 7 // 0~7,0x02,0X
#define ForceSingleCoilIndexEnd 7 // 0~7,0x05,0X
#define ReadHoldRegIndexEnd 7 // 0~7,0x03,4X
#define PresetMulRegIndexEnd 7 // 0~x,0x10,4X

/* these macro describe send or recieve allowed */
#define SEND 1 // send allowed
#define RECIEVE 0 // recieve allowed
#define SlaveID 0x01 // the slave's ID
#define SendRecieveDelay 50 // Send turn to Recieve or Recieve turn to Send delay time vlaue
#define MaxDataLen 300 // preset CommBuf array Max lenght
#define MaxRegLen 300 // preset SlaveOutputBuf Max lenght
#define MaxCoilsNum 32 //preset Max Num of Coils(switch)
#define MaxRegsNum 256  //preset Max Num of Regs (data)
#define MaxWaveformLen 2000//preset the max datalength for waveform display




//全局变量声明
////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
//0.1 degree=360000w*s   but magnify 10 

enum DeviceState {Init1,PhaseLocking, PhaseLocked, Open_Loop,Run,StatusIsError};
extern enum DeviceState DS;//device state machine

extern unsigned int EPwmU_Compare;
extern unsigned int EPwmV_Compare;
extern unsigned int EPwmW_Compare;

//////////////////////////////
extern int16 SinAcount;
extern int16 SinAcountAdd;
extern int16 SinBcount;
extern int16 SinBcountAdd;
extern int16 SinCcount;
extern int16 SinCcountAdd;

extern int32 CurrAInitSum;
extern int32 CurrInitCount;
extern int16 AdcACOffset;

extern float32 A1_sinwt;		//电压幅值计算

extern int16 AdcCOffset;


//AD sample
extern int16 A_voltage_IN;
extern int16 B_voltage_IN;
extern int16 C_voltage_IN;
extern int16 A_voltage_OUT;
extern int16 B_voltage_OUT;
extern int16 C_voltage_OUT;  
extern int16 A_current_1;
extern int16 B_current_1;
extern int16 C_current_1;   
extern int16 A_current_2;
extern int16 B_current_2;
extern int16 C_current_2;  

extern int16 ADCIni_A_voltageIN_offset;	//输入电压偏移量
extern int16 ADCIni_B_voltageIN_offset;
extern int16 ADCIni_C_voltageIN_offset;

extern int16 ADCIni_A_voltageOUT_offset;	//输出电压偏移量
extern int16 ADCIni_B_voltageOUT_offset;
extern int16 ADCIni_C_voltageOUT_offset;

extern int16 ADCIni_A_current1_offset;   //电感1电流偏移量
extern int16 ADCIni_B_current1_offset;
extern int16 ADCIni_C_current1_offset;

extern int16 ADCIni_A_current2_offset;   //电感2电流偏移量
extern int16 ADCIni_B_current2_offset;
extern int16 ADCIni_C_current2_offset;

extern Uint16 ADSampIniFlag;                        //ADC初始化标志位1: 正在进行初始化2:初始化完毕
extern Uint16 ADCIni_periodflag;                       //ADC初始化周期计数，每3ms增1

extern int32 ADCIni_A_volIN_Count;	//用于纠正电压采样偏移量
extern int32 ADCIni_B_volIN_Count;
extern int32 ADCIni_C_volIN_Count;

extern int32 ADCIni_A_volIN_total;	
extern int32 ADCIni_B_volIN_total;
extern int32 ADCIni_C_volIN_total;

extern int16 ADCIni_A_volIN_offset_err;	//输入电压偏移量调整用
extern int16 ADCIni_B_volIN_offset_err;
extern int16 ADCIni_C_volIN_offset_err;

extern int16 ADC_RegoffsetStartFlag;              //开始调整AD 偏置标志�
extern int16 ADC_RegoffsetEndFlag;

extern int32 ADC_A_volIN_Count;                      //主循环程序调整AD 偏移量
extern int32 ADC_B_volIN_Count;
extern int32 ADC_C_volIN_Count;

extern int32 ADC_A_volIN_total;
extern int32 ADC_B_volIN_total;
extern int32 ADC_C_volIN_total;

extern int16 ADC_GridPeriod_Count_A;             //工频周期计数
extern int16 ADC_GridPeriod_Count_B;  
extern int16 ADC_GridPeriod_Count_C;  

extern int32 ADCIni_Volout_Cur_Count_A;
extern int32 ADCIni_Volout_Cur_Count_B;
extern int32 ADCIni_Volout_Cur_Count_C;
extern int32 ADCIni_A_volOUT_total;	
extern int32 ADCIni_B_volOUT_total;
extern int32 ADCIni_C_volOUT_total;
extern int32 ADCIni_A_Current1_total;	
extern int32 ADCIni_B_Current1_total;
extern int32 ADCIni_C_Current1_total;
extern int32 ADCIni_A_Current2_total;	
extern int32 ADCIni_B_Current2_total;
extern int32 ADCIni_C_Current2_total;

//lock phase
extern int16 Lock_Grid_Zero_flag_A;
extern int16 Lock_Grid_Zero_flag_B;
extern int16 Lock_Grid_Zero_flag_C;

///////// LockAZero(int16 Voltage)/////////
extern int16 APlusPeriod;  //正半周标志位0为不处于,1为处于正半周
extern int16 APlusNum;
extern int16 AMinusNum; 

///////// LockBZero(int16 Voltage)/////////
extern int16 BPlusPeriod;  //正半周标志位0为不处于,1为处于正半周
extern int16 BPlusNum;
extern int16 BMinusNum; 

///////// LockCZero(int16 Voltage)/////////
extern int16 CPlusPeriod;  //正半周标志位0为不处于,1为处于正半周
extern int16 CPlusNum;
extern int16 CMinusNum; 

extern Uint16 Lock_Ref_gene_flag;                  //正弦参考产生标志位(0:未产生正弦参考1:已产生正弦参考)

extern Uint16 Lock_Grid_positive_flag;             //电网电压在正半周标志位(0:未在正半周1:在正半周)
extern int16 Lock_Grid_posnum;                          //电网运行在正半周计数，用于检测是否运行在正半周
extern int16 Lock_Grid_negnum;                         //电网运行在负半周计数
extern Uint16 Lock_Grid_Zero1st_flag;             //电网电压第一次过零点标志(由正变负过零)
extern Uint16 Lock_Grid_Zero2nd_flag;            //电网电压第二次过零点标志
extern Uint16 Lock_Grid_Zero_flag;                 //电网电压过零标志(1表示过零)

extern Uint16 Lock_Ref_Zero_flag;                      //参考值过零标志(1表示过零)
extern Uint16 Lock_Ref_lead_flag;                       //参考值相位超前电网电压标志
extern Uint16 Lock_Ref_lag_flag;                         //参考值相位滞后电网电压标志

extern int16 Lock_Ref_count;                             //参考值计数
extern int16 Lock_Ref_period;                            //参考值周期
extern int16 Lock_Grid_count;                            //电网电压计数
extern int16 Lock_Grid_peiod;                            //电网电压周期

extern IncPID LockPhasePID;                                 //锁相PI 调节
extern float32 LockPhase_P;                              //锁相 PI 参数Q11定标
extern float32 LockPhase_I;
extern float32 LockPhase_D;
extern int16 PhaseError;                                      //参考值与电网之间的相位差
extern int16 LockPhasePID_Out;                         //锁相PID  输出

extern Uint16 lock_angle_inter;	                                                 //正弦参考

//判断电网电压是否正常变量
extern int16 gridcount;                                        //用于电网电压计数
extern int16 grid_low;                                      //220V  电压正常的最低值(45度)

//状态标志位
extern int16 grid_vol_state;                             //电网幅值状态1为正常，0为异常
extern int16 grid_fre_state;                             //电网频率状态1为正常，0为异常
extern int16 gridstate;                                      //电网状态标志位 1为正常，0为异常
extern int16 Busstate;                                      //直流母线状态标志位1为正常，0为异常
                                                                  //当直流母线电压高于整流值时为正常
extern Uint16 relayflag;                                      //继电器状态标志位0为断开1为闭合
extern Uint16 relay_delayflag;                          //继电器闭合延迟标志位0未延迟1已延迟																  
extern Uint16 relay_delay_period;                    //继电器闭合延迟周期计数

extern Uint16 Control_En_flag;					//闭环控制使能标志
extern Uint16 Control_Arith_wait_flag;			//闭环控制使能等待标志
extern Uint16 Control_Arith_wait_count;		//闭环控制使能等待计数


extern int16 LEFlag;                                             //LE 标志位，0为LE 低电平，使能PWM，1为LE 高电平，禁止PWM
                                                                          //用于软启动


//用于电压环
extern int32 CurrRefConst;                                 //电流参考值作为电流内环 
extern int16 CurrRefFlag;                                   //电流参考改变标志位，工频周期改变

//电压环和电流环PID
extern IncPID VolALoopPID;
extern IncPID CurrALoopPID;
//电压环PID参数 0.00048828125-16  q11定标
//在s域中，VolLoop_P对应Kp，VolLoop_I对应kp*(T/Ti),VolLoop_D对应Kp*(Td/T)
extern float32 VolALoop_P; //Set PID Coefficients
extern float32 VolALoop_I;  //20hz
extern float32 VolALoop_D;
//A相电流环PID参数
extern float32 CurrALoop_P;//2.5;//1.5;//4              //  Set PID Coefficients
extern float32 CurrALoop_I;//0.25;//0.1;//0.2;
extern float32 CurrALoop_D;


#endif    // end of Commondefine.h definition




 



