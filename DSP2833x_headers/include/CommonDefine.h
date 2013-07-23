
//ÇĞ¼ÇÇĞ¼Ç£ºÈ«¾Ö±äÁ¿¶¨Òå²»ÒªÔÚ.hÎÄ¼şÖĞ¶¨Òå£¬»áµ¼ÖÂ±äÁ¿ÖØ¸´¶¨Òå
//.hÎÄ¼şÖĞ·Ådefine£¨Ã»ÓĞÖØ¸´¶¨ÒåÎÊÌâ£©£¬±äÁ¿ĞÎÊ½µÄ¶¨Òå£¨±ÈÈç½á¹¹ÌåµÄÄÚ²¿¶¨Òå£¬²»ÊÇ±äÁ¿¶¨Òå£©£¬
//±äÁ¿ÉùÃ÷£¬ÒÔ¼°ÆäËûµÄ.hÎÄ¼ş¡£
//È«¾Ö±äÁ¿µÄ¶¨ÒåÒª·Åµ½.cÎÄ¼şÖĞ



#ifndef CommonDefine_H
#define CommonDefine_H

//-------------------------------------------------------------------------

//Í·ÎÄ¼ş

#include "Arithmetics.h"   


//-------------------------------------------------------------------------
//ºê¶¨Òå
#define VersionN 9999 //version,9999=debug version

//#define EnergyPtDegree 3600000  

//grid low and high voltage
#define SineLowP 984
#define SineLowN -984
#define SineHighP 1640 
#define SineHighN -1640

//int16 IsVolNormal=22;   //
#define LowDefine 1000//195   //ÕıÏÒµÍÑ¹ÅĞ¶¨  10K=200
#define  HighDefine 1000//5   //ÕıÏÒ¸ßÑ¹ÅĞ¶¨ 10K=5

//grid frequency
#define GridFreq 50
#define SPeriodNor 400

//sample coefficent
#define VolACSampCoef 2555    //0.2555 
#define CurACSampCoef 144    //0.0144
#define VolDCSampCoef 147    //0.0147 
#define CurDCSampCoef 103     //0.0103  

//adc define
#define ADC_CKPS   0x1   // ADC module clockÄÚ²¿·ÖÆµ = HSPCLK/2*ADC_CKPS
#define ADC_CPS 0x0   // ADC module clock¶îÍâµÄÒ»´ÎÄÚ²¿¶ş·ÖÆµ 
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


//sinµÄÆ«ÖÃ
#define SineErrOffset 2//ÓÉÓÚµçÍøµçÑ¹¹ıÁãµã¼ì²âÖÁÉÙÖÍºó2¸öµã£¬ËùÒÔ²¹³¥ÒªÌáÇ°2¸öµã
#define SinePhaOffset 0 //33 30¶È //ÏàµçÑ¹ºÍÏßµçÑ¹µÄÏàÎ»²î
//----------------------------------------------------------------------------

//epwm define DCAC
#define EPWM_PERIOD 15000   //150M  100us 10k µ¥±ßĞ±ÆÂ
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


#define LD_IM 3 //Ö±½Ó×°ÔØ
#define CSF_DIS 0	//Èí¼şÇ¿ÖÆÊä³ö²»ÊÇÄÜ
#define CSF_HIGH 2	//Èí¼şÇ¿ÖÆÖÃ¸ß
#define CSF_LOW 1	//Èí¼şÇ¿ÖÆÖÃµÍ



//Äæ±äÆ÷Ïà¹Ø²ÎÊı
#define AD_Bus_rec  1135        //1135       //²»¿ØÕûÁ÷Ä¸ÏßµçÑ¹£¬ÓÃÓÚ¿ªÆôÄæ±ä
#define AD_Bus_ref  1513              //1475 Ä¸ÏßµçÑ¹²Î¿¼£¬¸Ã²Î¿¼ÎªÎÈ¶¨Ä¸ÏßµçÑ¹µÄ²Î¿¼

#define currref_lw  100             //µçÁ÷»·ÏŞÁ÷Öµ

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
#define CursampCoef 333 //407 //Ò»ÍÏ¶ş333 

//0.1 degree=360000w*s   but magnify 10 
#define EnergyPtDegree 1781156250//1457459578//Ò»ÍÏ¶ş1781156250


//vol curr loop p i 
#define Voltage_base_Ref 1312////¶ÔÓ¦Êä³ö»ù²¨ÓĞĞ§ÖµÎª220 //600 ¶ÔÓ¦µ÷Ñ¹Æ÷µçÑ¹150V×óÓÒ
#define Duty_DC_up_limit 11250 //»ù²¨Õ¼¿Õ±È±È½ÏÆ÷ÖµÉÏÏŞÎª11250,Îª150%
#define Duty_DC_low_limit 3750 //»ù²¨Õ¼¿Õ±È±È½ÏÆ÷ÖµÏÂÏŞÎª3750,Îª50%
#define Duty_up_limit 14999 //ÕûÀíÕ¼¿Õ±ÈÏŞÖÆÎª1µ½14999
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

#define EEPErrNumer 16 //¼ÇÂ¼´íÎó¸öÊı


//////////////////////////////////////////////////////////
///´¥ÃşÆÁ³ÌĞòºê¶¨Òå
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




//È«¾Ö±äÁ¿ÉùÃ÷
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

extern float32 A1_sinwt;		//µçÑ¹·ùÖµ¼ÆËã

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

extern int16 ADCIni_A_voltageIN_offset;	//ÊäÈëµçÑ¹Æ«ÒÆÁ¿
extern int16 ADCIni_B_voltageIN_offset;
extern int16 ADCIni_C_voltageIN_offset;

extern int16 ADCIni_A_voltageOUT_offset;	//Êä³öµçÑ¹Æ«ÒÆÁ¿
extern int16 ADCIni_B_voltageOUT_offset;
extern int16 ADCIni_C_voltageOUT_offset;

extern int16 ADCIni_A_current1_offset;   //µç¸Ğ1µçÁ÷Æ«ÒÆÁ¿
extern int16 ADCIni_B_current1_offset;
extern int16 ADCIni_C_current1_offset;

extern int16 ADCIni_A_current2_offset;   //µç¸Ğ2µçÁ÷Æ«ÒÆÁ¿
extern int16 ADCIni_B_current2_offset;
extern int16 ADCIni_C_current2_offset;

extern Uint16 ADSampIniFlag;                        //ADC³õÊ¼»¯±êÖ¾Î»1: ÕıÔÚ½øĞĞ³õÊ¼»¯2:³õÊ¼»¯Íê±Ï
extern Uint16 ADCIni_periodflag;                       //ADC³õÊ¼»¯ÖÜÆÚ¼ÆÊı£¬Ã¿3msÔö1

extern int32 ADCIni_A_volIN_Count;	//ÓÃÓÚ¾ÀÕıµçÑ¹²ÉÑùÆ«ÒÆÁ¿
extern int32 ADCIni_B_volIN_Count;
extern int32 ADCIni_C_volIN_Count;

extern int32 ADCIni_A_volIN_total;	
extern int32 ADCIni_B_volIN_total;
extern int32 ADCIni_C_volIN_total;

extern int16 ADCIni_A_volIN_offset_err;	//ÊäÈëµçÑ¹Æ«ÒÆÁ¿µ÷ÕûÓÃ
extern int16 ADCIni_B_volIN_offset_err;
extern int16 ADCIni_C_volIN_offset_err;

extern int16 ADC_RegoffsetStartFlag;              //¿ªÊ¼µ÷ÕûAD Æ«ÖÃ±êÖ¾»
extern int16 ADC_RegoffsetEndFlag;

extern int32 ADC_A_volIN_Count;                      //Ö÷Ñ­»·³ÌĞòµ÷ÕûAD Æ«ÒÆÁ¿
extern int32 ADC_B_volIN_Count;
extern int32 ADC_C_volIN_Count;

extern int32 ADC_A_volIN_total;
extern int32 ADC_B_volIN_total;
extern int32 ADC_C_volIN_total;

extern int16 ADC_GridPeriod_Count_A;             //¹¤ÆµÖÜÆÚ¼ÆÊı
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
extern int16 APlusPeriod;  //Õı°ëÖÜ±êÖ¾Î»0Îª²»´¦ÓÚ,1Îª´¦ÓÚÕı°ëÖÜ
extern int16 APlusNum;
extern int16 AMinusNum; 

///////// LockBZero(int16 Voltage)/////////
extern int16 BPlusPeriod;  //Õı°ëÖÜ±êÖ¾Î»0Îª²»´¦ÓÚ,1Îª´¦ÓÚÕı°ëÖÜ
extern int16 BPlusNum;
extern int16 BMinusNum; 

///////// LockCZero(int16 Voltage)/////////
extern int16 CPlusPeriod;  //Õı°ëÖÜ±êÖ¾Î»0Îª²»´¦ÓÚ,1Îª´¦ÓÚÕı°ëÖÜ
extern int16 CPlusNum;
extern int16 CMinusNum; 

extern Uint16 Lock_Ref_gene_flag;                  //ÕıÏÒ²Î¿¼²úÉú±êÖ¾Î»(0:Î´²úÉúÕıÏÒ²Î¿¼1:ÒÑ²úÉúÕıÏÒ²Î¿¼)

extern Uint16 Lock_Grid_positive_flag;             //µçÍøµçÑ¹ÔÚÕı°ëÖÜ±êÖ¾Î»(0:Î´ÔÚÕı°ëÖÜ1:ÔÚÕı°ëÖÜ)
extern int16 Lock_Grid_posnum;                          //µçÍøÔËĞĞÔÚÕı°ëÖÜ¼ÆÊı£¬ÓÃÓÚ¼ì²âÊÇ·ñÔËĞĞÔÚÕı°ëÖÜ
extern int16 Lock_Grid_negnum;                         //µçÍøÔËĞĞÔÚ¸º°ëÖÜ¼ÆÊı
extern Uint16 Lock_Grid_Zero1st_flag;             //µçÍøµçÑ¹µÚÒ»´Î¹ıÁãµã±êÖ¾(ÓÉÕı±ä¸º¹ıÁã)
extern Uint16 Lock_Grid_Zero2nd_flag;            //µçÍøµçÑ¹µÚ¶ş´Î¹ıÁãµã±êÖ¾
extern Uint16 Lock_Grid_Zero_flag;                 //µçÍøµçÑ¹¹ıÁã±êÖ¾(1±íÊ¾¹ıÁã)

extern Uint16 Lock_Ref_Zero_flag;                      //²Î¿¼Öµ¹ıÁã±êÖ¾(1±íÊ¾¹ıÁã)
extern Uint16 Lock_Ref_lead_flag;                       //²Î¿¼ÖµÏàÎ»³¬Ç°µçÍøµçÑ¹±êÖ¾
extern Uint16 Lock_Ref_lag_flag;                         //²Î¿¼ÖµÏàÎ»ÖÍºóµçÍøµçÑ¹±êÖ¾

extern int16 Lock_Ref_count;                             //²Î¿¼Öµ¼ÆÊı
extern int16 Lock_Ref_period;                            //²Î¿¼ÖµÖÜÆÚ
extern int16 Lock_Grid_count;                            //µçÍøµçÑ¹¼ÆÊı
extern int16 Lock_Grid_peiod;                            //µçÍøµçÑ¹ÖÜÆÚ

extern IncPID LockPhasePID;                                 //ËøÏàPI µ÷½Ú
extern float32 LockPhase_P;                              //ËøÏà PI ²ÎÊıQ11¶¨±ê
extern float32 LockPhase_I;
extern float32 LockPhase_D;
extern int16 PhaseError;                                      //²Î¿¼ÖµÓëµçÍøÖ®¼äµÄÏàÎ»²î
extern int16 LockPhasePID_Out;                         //ËøÏàPID  Êä³ö

extern Uint16 lock_angle_inter;	                                                 //ÕıÏÒ²Î¿¼

//ÅĞ¶ÏµçÍøµçÑ¹ÊÇ·ñÕı³£±äÁ¿
extern int16 gridcount;                                        //ÓÃÓÚµçÍøµçÑ¹¼ÆÊı
extern int16 grid_low;                                      //220V  µçÑ¹Õı³£µÄ×îµÍÖµ(45¶È)

//×´Ì¬±êÖ¾Î»
extern int16 grid_vol_state;                             //µçÍø·ùÖµ×´Ì¬1ÎªÕı³££¬0ÎªÒì³£
extern int16 grid_fre_state;                             //µçÍøÆµÂÊ×´Ì¬1ÎªÕı³££¬0ÎªÒì³£
extern int16 gridstate;                                      //µçÍø×´Ì¬±êÖ¾Î» 1ÎªÕı³££¬0ÎªÒì³£
extern int16 Busstate;                                      //Ö±Á÷Ä¸Ïß×´Ì¬±êÖ¾Î»1ÎªÕı³££¬0ÎªÒì³£
                                                                  //µ±Ö±Á÷Ä¸ÏßµçÑ¹¸ßÓÚÕûÁ÷ÖµÊ±ÎªÕı³£
extern Uint16 relayflag;                                      //¼ÌµçÆ÷×´Ì¬±êÖ¾Î»0Îª¶Ï¿ª1Îª±ÕºÏ
extern Uint16 relay_delayflag;                          //¼ÌµçÆ÷±ÕºÏÑÓ³Ù±êÖ¾Î»0Î´ÑÓ³Ù1ÒÑÑÓ³Ù																  
extern Uint16 relay_delay_period;                    //¼ÌµçÆ÷±ÕºÏÑÓ³ÙÖÜÆÚ¼ÆÊı

extern Uint16 Control_En_flag;					//±Õ»·¿ØÖÆÊ¹ÄÜ±êÖ¾
extern Uint16 Control_Arith_wait_flag;			//±Õ»·¿ØÖÆÊ¹ÄÜµÈ´ı±êÖ¾
extern Uint16 Control_Arith_wait_count;		//±Õ»·¿ØÖÆÊ¹ÄÜµÈ´ı¼ÆÊı


extern int16 LEFlag;                                             //LE ±êÖ¾Î»£¬0ÎªLE µÍµçÆ½£¬Ê¹ÄÜPWM£¬1ÎªLE ¸ßµçÆ½£¬½ûÖ¹PWM
                                                                          //ÓÃÓÚÈíÆô¶¯


//ÓÃÓÚµçÑ¹»·
extern int32 CurrRefConst;                                 //µçÁ÷²Î¿¼Öµ×÷ÎªµçÁ÷ÄÚ»· 
extern int16 CurrRefFlag;                                   //µçÁ÷²Î¿¼¸Ä±ä±êÖ¾Î»£¬¹¤ÆµÖÜÆÚ¸Ä±ä

//µçÑ¹»·ºÍµçÁ÷»·PID
extern IncPID VolALoopPID;
extern IncPID CurrALoopPID;
//µçÑ¹»·PID²ÎÊı 0.00048828125-16  q11¶¨±ê
//ÔÚsÓòÖĞ£¬VolLoop_P¶ÔÓ¦Kp£¬VolLoop_I¶ÔÓ¦kp*(T/Ti),VolLoop_D¶ÔÓ¦Kp*(Td/T)
extern float32 VolALoop_P; //Set PID Coefficients
extern float32 VolALoop_I;  //20hz
extern float32 VolALoop_D;
//AÏàµçÁ÷»·PID²ÎÊı
extern float32 CurrALoop_P;//2.5;//1.5;//4              //  Set PID Coefficients
extern float32 CurrALoop_I;//0.25;//0.1;//0.2;
extern float32 CurrALoop_D;


#endif    // end of Commondefine.h definition




 



