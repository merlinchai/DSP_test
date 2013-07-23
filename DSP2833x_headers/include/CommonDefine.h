
//�м��мǣ�ȫ�ֱ������岻Ҫ��.h�ļ��ж��壬�ᵼ�±����ظ�����
//.h�ļ��з�define��û���ظ��������⣩��������ʽ�Ķ��壨����ṹ����ڲ����壬���Ǳ������壩��
//�����������Լ�������.h�ļ���
//ȫ�ֱ����Ķ���Ҫ�ŵ�.c�ļ���



#ifndef CommonDefine_H
#define CommonDefine_H

//-------------------------------------------------------------------------

//ͷ�ļ�

#include "Arithmetics.h"   


//-------------------------------------------------------------------------
//�궨��
#define VersionN 9999 //version,9999=debug version

//#define EnergyPtDegree 3600000  

//grid low and high voltage
#define SineLowP 984
#define SineLowN -984
#define SineHighP 1640 
#define SineHighN -1640

//int16 IsVolNormal=22;   //
#define LowDefine 1000//195   //���ҵ�ѹ�ж�  10K=200
#define  HighDefine 1000//5   //���Ҹ�ѹ�ж� 10K=5

//grid frequency
#define GridFreq 50
#define SPeriodNor 400

//sample coefficent
#define VolACSampCoef 2555    //0.2555 
#define CurACSampCoef 144    //0.0144
#define VolDCSampCoef 147    //0.0147 
#define CurDCSampCoef 103     //0.0103  

//adc define
#define ADC_CKPS   0x1   // ADC module clock�ڲ���Ƶ = HSPCLK/2*ADC_CKPS
#define ADC_CPS 0x0   // ADC module clock�����һ���ڲ�����Ƶ 
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


//sin��ƫ��
#define SineErrOffset 2//���ڵ�����ѹ������������ͺ�2���㣬���Բ���Ҫ��ǰ2����
#define SinePhaOffset 0 //33 30�� //���ѹ���ߵ�ѹ����λ��
//----------------------------------------------------------------------------

//epwm define DCAC
#define EPWM_PERIOD 15000   //150M  100us 10k ����б��
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


#define LD_IM 3 //ֱ��װ��
#define CSF_DIS 0	//���ǿ�����������
#define CSF_HIGH 2	//���ǿ���ø�
#define CSF_LOW 1	//���ǿ���õ�



//�������ز���
#define AD_Bus_rec  1135        //1135       //��������ĸ�ߵ�ѹ�����ڿ������
#define AD_Bus_ref  1513              //1475 ĸ�ߵ�ѹ�ο����òο�Ϊ�ȶ�ĸ�ߵ�ѹ�Ĳο�

#define currref_lw  100             //����������ֵ

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
#define CursampCoef 333 //407 //һ�϶�333 

//0.1 degree=360000w*s   but magnify 10 
#define EnergyPtDegree 1781156250//1457459578//һ�϶�1781156250


//vol curr loop p i 
#define Voltage_base_Ref 1312////��Ӧ���������ЧֵΪ220 //600 ��Ӧ��ѹ����ѹ150V����
#define Duty_DC_up_limit 11250 //����ռ�ձȱȽ���ֵ����Ϊ11250,Ϊ150%
#define Duty_DC_low_limit 3750 //����ռ�ձȱȽ���ֵ����Ϊ3750,Ϊ50%
#define Duty_up_limit 14999 //����ռ�ձ�����Ϊ1��14999
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

#define EEPErrNumer 16 //��¼�������


//////////////////////////////////////////////////////////
///����������궨��
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




//ȫ�ֱ�������
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

extern float32 A1_sinwt;		//��ѹ��ֵ����

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

extern int16 ADCIni_A_voltageIN_offset;	//�����ѹƫ����
extern int16 ADCIni_B_voltageIN_offset;
extern int16 ADCIni_C_voltageIN_offset;

extern int16 ADCIni_A_voltageOUT_offset;	//�����ѹƫ����
extern int16 ADCIni_B_voltageOUT_offset;
extern int16 ADCIni_C_voltageOUT_offset;

extern int16 ADCIni_A_current1_offset;   //���1����ƫ����
extern int16 ADCIni_B_current1_offset;
extern int16 ADCIni_C_current1_offset;

extern int16 ADCIni_A_current2_offset;   //���2����ƫ����
extern int16 ADCIni_B_current2_offset;
extern int16 ADCIni_C_current2_offset;

extern Uint16 ADSampIniFlag;                        //ADC��ʼ����־λ1: ���ڽ��г�ʼ��2:��ʼ�����
extern Uint16 ADCIni_periodflag;                       //ADC��ʼ�����ڼ�����ÿ3ms��1

extern int32 ADCIni_A_volIN_Count;	//���ھ�����ѹ����ƫ����
extern int32 ADCIni_B_volIN_Count;
extern int32 ADCIni_C_volIN_Count;

extern int32 ADCIni_A_volIN_total;	
extern int32 ADCIni_B_volIN_total;
extern int32 ADCIni_C_volIN_total;

extern int16 ADCIni_A_volIN_offset_err;	//�����ѹƫ����������
extern int16 ADCIni_B_volIN_offset_err;
extern int16 ADCIni_C_volIN_offset_err;

extern int16 ADC_RegoffsetStartFlag;              //��ʼ����AD ƫ�ñ�־�
extern int16 ADC_RegoffsetEndFlag;

extern int32 ADC_A_volIN_Count;                      //��ѭ���������AD ƫ����
extern int32 ADC_B_volIN_Count;
extern int32 ADC_C_volIN_Count;

extern int32 ADC_A_volIN_total;
extern int32 ADC_B_volIN_total;
extern int32 ADC_C_volIN_total;

extern int16 ADC_GridPeriod_Count_A;             //��Ƶ���ڼ���
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
extern int16 APlusPeriod;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
extern int16 APlusNum;
extern int16 AMinusNum; 

///////// LockBZero(int16 Voltage)/////////
extern int16 BPlusPeriod;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
extern int16 BPlusNum;
extern int16 BMinusNum; 

///////// LockCZero(int16 Voltage)/////////
extern int16 CPlusPeriod;  //�����ܱ�־λ0Ϊ������,1Ϊ����������
extern int16 CPlusNum;
extern int16 CMinusNum; 

extern Uint16 Lock_Ref_gene_flag;                  //���Ҳο�������־λ(0:δ�������Ҳο�1:�Ѳ������Ҳο�)

extern Uint16 Lock_Grid_positive_flag;             //������ѹ�������ܱ�־λ(0:δ��������1:��������)
extern int16 Lock_Grid_posnum;                          //���������������ܼ��������ڼ���Ƿ�������������
extern int16 Lock_Grid_negnum;                         //���������ڸ����ܼ���
extern Uint16 Lock_Grid_Zero1st_flag;             //������ѹ��һ�ι�����־(�����为����)
extern Uint16 Lock_Grid_Zero2nd_flag;            //������ѹ�ڶ��ι�����־
extern Uint16 Lock_Grid_Zero_flag;                 //������ѹ�����־(1��ʾ����)

extern Uint16 Lock_Ref_Zero_flag;                      //�ο�ֵ�����־(1��ʾ����)
extern Uint16 Lock_Ref_lead_flag;                       //�ο�ֵ��λ��ǰ������ѹ��־
extern Uint16 Lock_Ref_lag_flag;                         //�ο�ֵ��λ�ͺ������ѹ��־

extern int16 Lock_Ref_count;                             //�ο�ֵ����
extern int16 Lock_Ref_period;                            //�ο�ֵ����
extern int16 Lock_Grid_count;                            //������ѹ����
extern int16 Lock_Grid_peiod;                            //������ѹ����

extern IncPID LockPhasePID;                                 //����PI ����
extern float32 LockPhase_P;                              //���� PI ����Q11����
extern float32 LockPhase_I;
extern float32 LockPhase_D;
extern int16 PhaseError;                                      //�ο�ֵ�����֮�����λ��
extern int16 LockPhasePID_Out;                         //����PID  ���

extern Uint16 lock_angle_inter;	                                                 //���Ҳο�

//�жϵ�����ѹ�Ƿ���������
extern int16 gridcount;                                        //���ڵ�����ѹ����
extern int16 grid_low;                                      //220V  ��ѹ���������ֵ(45��)

//״̬��־λ
extern int16 grid_vol_state;                             //������ֵ״̬1Ϊ������0Ϊ�쳣
extern int16 grid_fre_state;                             //����Ƶ��״̬1Ϊ������0Ϊ�쳣
extern int16 gridstate;                                      //����״̬��־λ 1Ϊ������0Ϊ�쳣
extern int16 Busstate;                                      //ֱ��ĸ��״̬��־λ1Ϊ������0Ϊ�쳣
                                                                  //��ֱ��ĸ�ߵ�ѹ��������ֵʱΪ����
extern Uint16 relayflag;                                      //�̵���״̬��־λ0Ϊ�Ͽ�1Ϊ�պ�
extern Uint16 relay_delayflag;                          //�̵����պ��ӳٱ�־λ0δ�ӳ�1���ӳ�																  
extern Uint16 relay_delay_period;                    //�̵����պ��ӳ����ڼ���

extern Uint16 Control_En_flag;					//�ջ�����ʹ�ܱ�־
extern Uint16 Control_Arith_wait_flag;			//�ջ�����ʹ�ܵȴ���־
extern Uint16 Control_Arith_wait_count;		//�ջ�����ʹ�ܵȴ�����


extern int16 LEFlag;                                             //LE ��־λ��0ΪLE �͵�ƽ��ʹ��PWM��1ΪLE �ߵ�ƽ����ֹPWM
                                                                          //����������


//���ڵ�ѹ��
extern int32 CurrRefConst;                                 //�����ο�ֵ��Ϊ�����ڻ� 
extern int16 CurrRefFlag;                                   //�����ο��ı��־λ����Ƶ���ڸı�

//��ѹ���͵�����PID
extern IncPID VolALoopPID;
extern IncPID CurrALoopPID;
//��ѹ��PID���� 0.00048828125-16  q11����
//��s���У�VolLoop_P��ӦKp��VolLoop_I��Ӧkp*(T/Ti),VolLoop_D��ӦKp*(Td/T)
extern float32 VolALoop_P; //Set PID Coefficients
extern float32 VolALoop_I;  //20hz
extern float32 VolALoop_D;
//A�������PID����
extern float32 CurrALoop_P;//2.5;//1.5;//4              //  Set PID Coefficients
extern float32 CurrALoop_I;//0.25;//0.1;//0.2;
extern float32 CurrALoop_D;


#endif    // end of Commondefine.h definition




 



