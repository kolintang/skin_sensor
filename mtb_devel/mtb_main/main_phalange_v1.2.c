// Based on IIT's code
// Updated 2017-10-31 by Tito
// uSkin phalange 1.2 (Has chip order 00 01 10 11)

#include<p30f4011.h>
#include"can_interface.h"
#include "AD7147RegMap.h"
#include "ADC.h"
#include "I2C.h"
#include "timers.h"
#include "LED.h"
#include "eeprom.h"
#include "options.h"
#include <timer.h>
#include <libpic30.h>
#include <adc10.h>
#include "l3g4200d.h"
#include "lis331dlh.h"
#include "math.h"
//#include "MLX90393.h"
//#include "MLX90393_MEM_CMD.h"


#define CAN_MSG_CLASS_ACC_GYRO 0x500
#define MSG_TYPE_GYRO 0x000
#define MSG_TYPE_ACC 0x001
#define MSG_TYPE_ANALOG_ACC 0x002

//#define DEBUG
//#define DEBUG_EMS //send a message in the triangle 0 pad 0 that goes from 0 to 255



// inizializzazione bit di configurazione (p30f4013.h)
_FOSC(CSW_FSCM_OFF & EC_PLL8); 
  // Clock switching disabled Fail safe Clock Monitor disabled
  // External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FWDT(WDT_OFF);      // WD disabled
//
 
 _FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV27);  // BOR 2.7V POR 64msec
_FGS(CODE_PROT_OFF); // Code protection disabled


enum Errors
    {
      error_ok,
      error_noack,
      error_notconnected
    };

// static unsigned int value=0;
//------------------------------------------------------------------------
//								Function prototypes
//------------------------------------------------------------------------
static void ServiceAD7147Isr(unsigned char Channel);
void ServiceAD7147Isr_three(unsigned char Channel);
static void ServiceAD7147Isr_fingertip(unsigned char Channel);
void ServiceAD7147Isr_all(unsigned char Channel);
void Wait(unsigned int value);
static void FillCanMessages8bit(unsigned char Channel,unsigned char triangleN);
void FillCanMessages8bit_three(unsigned char Channel,unsigned char triangleN);
void FillCanMessages8bit_all(unsigned char Channel,unsigned char triangleN);

static void FillCanMessages8bit_test(unsigned char Channel,unsigned char triangleN);

// Custom code for Melexis
static void FillCanMessages8bit_hall(unsigned char Channel,unsigned char sda_no);
static void read_register_test(unsigned char address, unsigned char *read_buffer_sda1, unsigned char *read_buffer_sda2, unsigned char *read_buffer_sda3, unsigned char *read_buffer_sda4);
void MLX90393_init(unsigned char i2c_address);
void MLX90393_calibration(unsigned char i2c_address);


//
void TrianglesInit(unsigned char Channel);
void TrianglesInit_all(unsigned char Channel);
       
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void);  


//------------------------------------------------------------------------
//								Global variables
//------------------------------------------------------------------------
struct s_eeprom _EEDATA(1) ee_data = 
{
  0x0,           // EE_B_EEErased             :1
  0x0,           // EE_B_EnableWD             :1
  0x1,           // EE_B_EnableBOR            :1
  0x05,          // EE_CAN_BoardAddress;      :8
  0x01,          // EE_CAN_MessageDataRate    :8
  0x04,          // EE_CAN_Speed;             :8
  {'u','S','k','i','n',' '},
  0x0000 // Checksum
};

// Board Configuration image from EEPROM
struct s_eeprom BoardConfig = {0}; 

typedef struct error_cap
{
    unsigned int error_outofrange;
    unsigned int error;
} error_cap;

static struct error_cap err[16];



triangle_cfg_t triangle_cfg_list[16];

volatile char flag;
volatile char flag2;
unsigned int AD7147Registers[16][12];  //Element[23] = 0x17 = ID register @ 0x17
unsigned int MLX90393Buffer_1[8];
unsigned int MLX90393Buffer_2[8];
unsigned int MLX90393Buffer_3[8];
unsigned int MLX90393Buffer_4[8];
int write_buffer[4];
unsigned int RM[1] = {0x4E};
unsigned int SB[1] = {0x1F};
unsigned int EX[1] = {0x80};
int test=0;
const	unsigned char AD7147_ADD[4]={0x2C,0x2D,0x2E,0x2F};
const	unsigned char MLX90393_ADD[4]={0x0C,0x0D,0x0E,0x0F}; //Melexis Address
unsigned int chip_id;

typedef unsigned const int __prog__ * FlashAddress; //flsh address 
unsigned  int __attribute__ ((space(prog), aligned(_FLASH_PAGE*2)))   CapOffset[16][12]={0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0,
                                                                                        0,0,0,0,0,0,0,0,0,0,0
                                                                                       };     //Offset of the capacitance
const FlashAddress _pCapOffset[16]={&CapOffset[0],&CapOffset[1],&CapOffset[2],&CapOffset[3],&CapOffset[4],&CapOffset[5],&CapOffset[6],&CapOffset[7],
							     &CapOffset[8],&CapOffset[9],&CapOffset[10],&CapOffset[11],&CapOffset[12],&CapOffset[13],&CapOffset[14],&CapOffset[15]
							      }; 
unsigned int __attribute__((space(prog), aligned(_FLASH_PAGE*2) ))   CapOffset_all[16][3]={      
												                                         0,0,0,
																				         0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																				         0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0,
																					     0,0,0
																					    };     //Offset of the capacitance 
const FlashAddress _pCapOffset_all[]={&CapOffset_all[0],&CapOffset_all[1],&CapOffset_all[2],&CapOffset_all[3],&CapOffset_all[4],&CapOffset_all[5],&CapOffset_all[6],&CapOffset_all[7],
							     &CapOffset_all[8],&CapOffset_all[9],&CapOffset_all[10],&CapOffset_all[11],&CapOffset_all[12],&CapOffset_all[13],&CapOffset_all[14],&CapOffset_all[15]								
							     }; 	
unsigned int BitToSend;    // number of bit to be send
//unsigned int stagecomplete;  
unsigned int _board_ID=2;
unsigned char board_MODE=EIGHT_BITS;
unsigned char new_board_MODE=EIGHT_BITS;
char _additional_info [32]={'T','a','c','t','i','l','e',' ','S','e','n','s','o','r'};
unsigned int PW_CONTROL= 0x0B0; // 0x1B0 for 128 decim  
unsigned int TIMER_VALUE= TIMER_10ms; // Timer duration 0x3000=> 40ms  //For controlling the loop timer as defined in options.h; currently the delay is 10ms
unsigned int TIMER_VALUE2= 0xC00; //0x132; -> 50ms? //0xc00;//0x99;//1ms 0xc00;//0xC00; // Timer duration 0xC00=> 10ms
unsigned char SHIFT=2; //shift of the CDC value for removing the noise
unsigned char SHIFT_THREE=3;// shift of the CDC value for removing the noise
unsigned char SHIFT_ALL=4; //shift of the CDC value for removing the noise
unsigned char NOLOAD=245;
unsigned char ANALOG_ACC=0; //analog accelerometer, if one 1 messsage is sent every 10ms
unsigned int ANALOG_ID=0x552; // default value
unsigned char DIG_EXT_GYRO=0; //gyro of the MMSP (PALM)
unsigned char DIG_EXT_ACC=0; //accelerometer of the MMSP (PALM) 
unsigned char DIG_ACC=0; // internal accelerometer 
unsigned char TEMP_COMPENSATION=1; //if 1 means internal temperature drift compensation 
int Tpad_base; //initial value of the Tpad
int Tpad;      //current value of the Tpad
int Tpad_palm_base; //initial value of the Tpad in the palm
int Tpad_palm;      //current value of the Tpad in the palm
int drift;      //current drift
uint8_t can_transmission_enabled = 0;
uint8_t transmission_was_enabled = 0;
uint8_t new_configuration = 0; /*if mtb receives ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG or ICUBCANPROTO_POL_SK_CMD__SET_TRIANG_CFG 
                                 then it uses class 4 to send periodic messages*/

enum skin_type TYPE=new_skin; //SKIN_2; //if =0 means new skin with drift compensation and 10 pads
unsigned int TRIANGLE_MASK=0xFFFF; //all the triangles are enabled for default
unsigned char CONFIG_TYPE=CONFIG_SINGLE;
unsigned char ERROR_COUNTER=0; //it counts the errors in reading the triangles.
unsigned char counter=0;
unsigned int ConValue[2]={0x2200, 0x2200}; //offset of the CDC reading 
volatile unsigned int PMsgID; //pressure measurement ID 
unsigned char acc[]={0,0,0,0,0,0,0,0}; //value of the three accelerometers
unsigned char gyro[]={0,0,0,0,0,0,0,0}; //value of the three gyro

volatile  int AN2 = 0; //Analog Accelerometer X axes
volatile  int AN3 = 0; //Analog Accelerometer Y axes
volatile  int AN4 = 0; //Analog Accelerometer Z axes
	tL3GI2COps l3g;
	tLISI2COps l3a;
    
uint32_t period_timer=0; ///for debug  purpose
 




//
//------------------------------------------------------------------------------ 
//								External functions
//------------------------------------------------------------------------------

extern void ConfigAD7147(unsigned char Channel, unsigned int i,unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_THREE(unsigned char Channel, unsigned int i,unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_FINGERTIP(unsigned char Channel, unsigned int i,unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_ALL(unsigned char Channel, unsigned int i,unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_onSdaX(unsigned char Channel, unsigned char setNum, unsigned char indexInSet, unsigned int pw_control_val, uint16_t cdcoffset/*unsigned int * cdcOffset*/);
//
//------------------------------------------------------------------------------ 
//								Static functions
//------------------------------------------------------------------------------

void SetCDCoffsetOnAllTriangles(uint16_t cdcOffset);
void SetCDCoffsetOnSingleTriangle(uint16_t cdcOffset, unsigned char triangleN);
//------------------------------------------------------------------------------
// 								Interrrupt routines
//------------------------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    flag=1;
     _T1IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
	flag2=1;
	if (ANALOG_ACC)
{
	acc[0]=((AN2 &0xFF00) >>0x8); // axis X
        acc[1]=(AN2 & 0xFF);
        acc[2]=((AN3 &0xFF00) >>0x8); // axis Y
        acc[3]=(AN3 & 0xFF);
        acc[4]=((AN4 &0xFF00) >>0x8); // axis Z
        acc[5]=(AN4 & 0xFF);

        while (!CAN1IsTXReady(1));    
                	CAN1SendMessage( (CAN_TX_SID(ANALOG_ID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ,
                        (CAN_TX_EID(0)) & CAN_NOR_TX_REQ, acc, 6,1);
        //executing the sampling of the
        ADCON1bits.SAMP = 1;    
} 
	
    _T2IF = 0;
}
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{
	//ready to send a CAN message
    if (C1INTFbits.TX0IF || C1INTFbits.TX1IF || C1INTFbits.TX2IF)
    {
        CAN1_interruptTx();
    }
	
	//receive a CAN message
    if (C1INTFbits.RX0IF || C1INTFbits.RX1IF )
    {
        CAN1_interruptRx();   
    }
    IFS1bits.C1IF =0;
}
//------------------------------------------------------------------------
//									MAIN
//------------------------------------------------------------------------
int main(void)
{
    //------------------------------------------------------------------------
    //									Variables   
    //------------------------------------------------------------------------
    char init;
    unsigned char i,l,j;
    unsigned int counter;
    unsigned int led_counter=0;
   	int gx=0;
	int gy=0;
	int gz=0;
	int ax=0;
	int ay=0;
	int az=0;
	int calib_timeout=0;
   	//
    // EEPROM Data Recovery
    // 
    // Initialize BoardConfig variable in RAM with the Data EEPROM stored values 
    RecoverConfigurationFromEEprom();
    _board_ID=BoardConfig.EE_CAN_BoardAddress;
    //------------------------------------------------------------------------
    //								Peripheral init
    //------------------------------------------------------------------------
    T1_Init(TIMER_VALUE);
    
    I2C_Init(CH0); 
    LED_Init();
    CAN_Init();
    /*
    l3g.i2c_write=WriteByteViaI2C;
	l3g.i2c_read=ReadByteViaI2C;
	l3g.i2c_burst=ReadBurstViaI2C;
	L3GInit(l3g);
    l3a.i2c_write=WriteByteViaI2C;
	l3a.i2c_read=ReadByteViaI2C;
	l3a.i2c_burst=ReadBurstViaI2C;
	LISInit(l3a,1,0);
    */
            
if (ANALOG_ACC)
{
    T2_Init(TIMER_VALUE2);
    ANALOG_ID=0x500;   
    ANALOG_ID |= (BoardConfig.EE_CAN_BoardAddress<<4); 
    ADC_Init();             //Initialize the A/D converter
}


if (DIG_EXT_GYRO || DIG_ACC || DIG_EXT_ACC)
{
    T2_Init(TIMER_VALUE2);

	
  
}    
    
    //Read Silicon versions to check communication, It should read 0x147X where X is the revision code

    for (i=0;i<4;i++)
    {
    //    ReadViaI2C(CH0,AD7147_ADD[i],DEVID, 1, AD7147Registers[0],AD7147Registers[4],AD7147Registers[8],AD7147Registers[12], i);
    } 
    
       //............................Configure AD7147
  
  switch (CONFIG_TYPE)
	{
		case CONFIG_SINGLE:
		{
           
		}
		break;	
		case CONFIG_THREE:
		{
	
		}
		break;
		case CONFIG_FINGERTIP:
		{
			 
		}
		break;
		case CONFIG_ALL:
		{
		   
		}
		break;
	}
    l=0;
    i=0;
    counter=0; 
    
    
    //init triangle_cfg_list
    for(j=0;j<triangles_max_num; j++)
    {
        triangle_cfg_list[j].shift = SHIFT;
        triangle_cfg_list[j].CDCoffset = ConValue[0];
        triangle_cfg_list[j].setNum = j/4;
        triangle_cfg_list[j].indexInSet =  j%4;
    }
    
    //SendViaI2C_MLX_sdaX(CH0, 0x0C, 1, SB, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    //FillCanMessages8bit_hall(CH0, i);
    
    for (i = 0; i < 4; i ++){
    MLX90393_init(MLX90393_ADD[i]);
    MLX90393_calibration(MLX90393_ADD[i]);
    Wait(WAIT_value);
    }
    //T2_Init(TIMER_VALUE2);
    //SendViaI2C_MLX(CH0, 0x0C, RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 7);
    //FillCanMessages8bit_hall(CH0, i);
        
//______________________________________  MAIN LOOP ____________________________________________
//
//
//
    
	led_counter=0;
	led0=1;
        EnableIntCAN1;
        DisableIntT1;
        DisableIntT2;

        flag=0;
        flag2+0;
        
    for (;;)
    {
        
        if ((DIG_EXT_GYRO || DIG_ACC || DIG_EXT_ACC) && (flag2))
        {
            flag2=0;
            if (DIG_EXT_GYRO)
                {
               
            }
            if (DIG_ACC || DIG_EXT_ACC)
            {

            }  
        }
        if (flag==1)
        {
			calib_timeout=0;
            flag=0;
            i = 0;
            if (led_counter==20)
            {
                if (led0==1) led0=0;
                else led0=1;

                    led_counter=0;
            }
            led_counter++;
            switch (board_MODE)
            {
            case  (EIGHT_BITS):
            {
                switch(CONFIG_TYPE)
                {
                case CONFIG_SINGLE :
                {
                    // Service routine for the triangles
                    //ServiceAD7147Isr(CH0);
                    
                   // Read the measurement
                    
                    for (i = 0; i < 4; i++){ //read 4 chips
                    
                    chip_id = i;
                    SendViaI2C_MLX(CH0, MLX90393_ADD[i], RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 7);
                    
                    // fill the CAN with measurement data
                    for(j = 0; j < 4; j++){
                    FillCanMessages8bit_hall(CH0, j); // Custom code, comment this again!
                    }
                    
                    } // end for 
                    
					if(can_transmission_enabled)
					{
                    	for (i=0;i<16;i++)
                    	{
                        	if (TRIANGLE_MASK & (0x1<<i))
                        	{
                           	 	//FillCanMessages8bit(CH0,i);
                        	}
                       		//FillCanMessages8bit_test(CH0, i); 
                           
                            //MagneticReading(CH0, MLX90393_ADD[1], 1, MLX90393Buffer[0],MLX90393Buffer[1],MLX90393Buffer[2],MLX90393Buffer[3], 0);
                            
                    	}
					}	
                }
                break;
                case CONFIG_THREE:
                {
                   // ServiceAD7147Isr_three(CH0);
                    for (i=0;i<16;i++)
                    {
                       // FillCanMessages8bit_three(CH0,i);
                    }
                }
                break;
                case CONFIG_ALL:
                {
                   // ServiceAD7147Isr_all(CH0);
                    for (i=0;i<16;i++)
                    {
                      //  FillCanMessages8bit_all(CH0,i);
                    }
                }
                break;
                }
            }
            break;
            case  (CALIB):
            {	
                board_MODE=EIGHT_BITS;
                switch (CONFIG_TYPE)
                {
                case CONFIG_SINGLE:
                {
                    for(i=0; i<triangles_max_num;i++)
                    {
                        if(triangle_cfg_list[i].isToUpdate)
                        {
                            SetCDCoffsetOnSingleTriangle(triangle_cfg_list[i].CDCoffset, i);
                            triangle_cfg_list[i].isToUpdate = 0;
                        }
                    }
                    flag=0;
                    init=0;
                    WriteTimer1(0);
                    counter=0;
					calib_timeout==0;
                    while (flag==0)					
					{	
					}			
                    // Calibration
                   // ServiceAD7147Isr(CH0);
                    flag=0;					
                    WriteTimer1(0);
                    while (flag==0)					
					{
					}
                   // TrianglesInit(CH0); 
                    if(!transmission_was_enabled)
                        can_enaDisa_transmission_messages(1);
                }
                break;
                case CONFIG_THREE:
                {
                    for (i=0;i<4;i++)
                    {
                    //    ConfigAD7147_THREE(CH0,i,PW_CONTROL,ConValue);
                    }
                    flag=0;
                    init=0;
                    WriteTimer1(0);
                    counter=0;
                    while (flag==0);
                    // Calibration
                   // ServiceAD7147Isr_three(CH0);
                    flag=0;
                    WriteTimer1(0);
                    while (flag==0);
                   // TrianglesInit_all(CH0);
                }
                break;
                case CONFIG_FINGERTIP:
                {
                    for (i=0;i<4;i++)
                    {
                     //   ConfigAD7147_FINGERTIP(CH0,i,PW_CONTROL,ConValue);
                    }
                    flag=0;
                    init=0;
                    WriteTimer1(0);
                    counter=0;
                    while (flag==0);
                    // Calibration
                   // ServiceAD7147Isr_fingertip(CH0);
                    flag=0;
                    WriteTimer1(0);
                    while (flag==0);
                   // TrianglesInit_all(CH0);
                }
                break;
                case CONFIG_ALL:
                {
                    for (i=0;i<4;i++)
                    {
                     //   ConfigAD7147_ALL(CH0,i,PW_CONTROL,ConValue);
                    }
                    flag=0;
                    init=0;
                    WriteTimer1(0);
                    counter=0;
                    while (flag==0);
                    // Calibration
                    //ServiceAD7147Isr_all(CH0);
                    flag=0;
                    WriteTimer1(0);
                    while (flag==0);
                    //TrianglesInit_all(CH0);
                }
                break;
                }
            }
            break;
            default: break;
            } //switch
        }//if (flag==1)
     CAN1_handleRx(_board_ID);
    	  
    }//for(;;)
    
}//main



static void ServiceAD7147Isr(unsigned char Channel)
{
    unsigned int i=0;
   unsigned int ConfigBuffer[0];
   unsigned int nets=4;
    
    		//Calibration configuration
	ConfigBuffer[0]=0x8000;//0x8000;//0x220;
  
	    for (i=0;i<nets;i++)
	    {		
	    // Added 0x0B because of register remapping
	       ReadViaI2C(CH0,AD7147_ADD[i],(ADCRESULT_S0+0x0B), 12, AD7147Registers[i],AD7147Registers[i+4],AD7147Registers[i+8],AD7147Registers[i+12], ADCRESULT_S0);
	    }
   		
	    for (i=0;i<nets;i++)
	    {
		 //  ReadViaI2C(CH0,AD7147_ADD[i],STAGE_COMPLETE_LIMIT_INT, 1, stagecomplete0[0],stagecomplete1[0],stagecomplete2[0],stagecomplete3[0], 0);		
//		     WriteViaI2C(CH0,AD7147_ADD[i],AMB_COMP_CTRL0,1, ConfigBuffer, 0);
//
	///DEBUG	      
	  //        ConfigAD7147(CH0,i,PW_CONTROL,ConValue); //0 is the number of the device		 
	    } 
}
void ServiceAD7147Isr_all(unsigned char Channel)
{
    int i=0;

    unsigned int ConfigBuffer[12];
    unsigned int ntriangles=4;
    
    //Calibration configuration
	ConfigBuffer[AMB_COMP_CTRL0]=0x8000;//0x220;

		//Read ADC Values
//	    for (i=0;i<4;i++)
//	    {
//		    ReadViaI2C(CH0,AD7147_ADD[i],STAGE_COMPLETE_LIMIT_INT, 1, stagecomplete0[0],stagecomplete1[0],stagecomplete2[0],stagecomplete3[0], 0);		
//	//	    WriteViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0,1, ConfigBuffer, AMB_COMP_CTRL0);
//	    }
	   	for (i=0;i<ntriangles;i++)
	    {		
	    // Added 0x0B because of register remapping
	       ReadViaI2C(CH0,AD7147_ADD[i],(ADCRESULT_S0+0x0B), 1, AD7147Registers[i],AD7147Registers[i+4],AD7147Registers[i+8],AD7147Registers[i+12], ADCRESULT_S0);
	    }

}
void ServiceAD7147Isr_three(unsigned char Channel)
{
    int i=0;
     unsigned int ntriangles=4;
    		//Calibration configuration
		//Read ADC Values
	    for (i=0;i<ntriangles;i++)
	    {		
	    // Added 0x0B because of register remapping
	       ReadViaI2C(CH0,AD7147_ADD[i],(ADCRESULT_S0+0x0B), 3, AD7147Registers[i],AD7147Registers[i+4],AD7147Registers[i+8],AD7147Registers[i+12], ADCRESULT_S0);
	    }
}

static void ServiceAD7147Isr_fingertip(unsigned char Channel)
{
    unsigned int i=0;
   unsigned int ConfigBuffer[0];
   unsigned int nets=4;
    
    		//Calibration configuration
	ConfigBuffer[0]=0x8000;//0x8000;//0x220;
  
	    for (i=0;i<nets;i++)
	    {		
	    // Added 0x0B because of register remapping
	       ReadViaI2C(CH0,AD7147_ADD[i],(ADCRESULT_S0+0x0B), 2, AD7147Registers[i],AD7147Registers[i+4],AD7147Registers[i+8],AD7147Registers[i+12], ADCRESULT_S0);
	    }
}
//------------------------------------------------------------------------
//					This is just a simple delay routine.
//------------------------------------------------------------------------
void Wait(unsigned int value)    
{
   while (value>0)
{
   value--;
}
        
}//Wait();

void TrianglesInit(unsigned char Channel)
{
    int i,j,k;
	_prog_addressT p;
    int unconnect=0;
    int  source[_FLASH_ROW];

		_init_prog_address(p, CapOffset);  /* get address in program space */	
		// delete all the space for the _pCapOffset[32][12]  the ROW is 32 then 12 is the number of ROW to be deleted 		
		for(i=0;i<6;i++)
		{
			_erase_flash(p);
			p += (_FLASH_ROW * 2);	
		} 
		j=0; 
		_init_prog_address(p, CapOffset);  /* get address in program space */		
    	for (i=0;i<16;i++)
    	{
            err[i].error=error_ok;
            for (k=0;k<12;k++)
            {
                source[j]=AD7147Registers[i][ADCRESULT_S0+k];
                j++;
                if (j==_FLASH_ROW)
                {
                    _write_flash16(p,source);
                    p += (_FLASH_ROW * 2);
                    j=0;
                }
                if (AD7147Registers[i][ADCRESULT_S0+k]==0xFFFF)
                {
                    unconnect +=1;
                }
            }
            if (unconnect==12) // no answer from the chip at startup
            {
                err[i].error |=error_notconnected;
               
            }
            unconnect=0;
    	}
}
void TrianglesInit_all(unsigned char Channel)
{
    int i,j,k;
	_prog_addressT p;
	 int  source[_FLASH_ROW];

		_init_prog_address(p, CapOffset_all);  /* get address in program space */
		
		// delete all the space for the _pCapOffset[32][12]  the ROW is 32 then 12 is the number of ROW to be deleted 		
		for(i=0;i<3;i++)
		{
			_erase_flash(p);
			p += (_FLASH_ROW * 2);	
		} 
		j=0; 
		_init_prog_address(p, CapOffset_all);  /* get address in program space */
		
    	for (i=0;i<16;i++)
    	{
        	for (k=0;k<3;k++)
        	{
				source[j]=AD7147Registers[i][ADCRESULT_S0+k];
				j++;
				if (j==_FLASH_ROW)
				{
					_write_flash16(p,source);
					p += (_FLASH_ROW * 2);	
					j=0;	
				}		
        	
		}
    	}
    	_write_flash16(p,source);
	p += (_FLASH_ROW * 2);	
	
}

static void FillCanMessages8bit(unsigned char Channel,unsigned char triangleN)
{
    unsigned char data[8];
    unsigned int i,j;
    int value; //difference of the current measurement and the initial value (_pCapOffset)
    unsigned int txdata[12];
    int unconnected=0;
    unsigned int GAIN[12]={70,96,83,38,38,70, 0,45,77,164,0,77}; //this gains are moltiplied by 128 with respect to matlab
    unsigned int GAIN_PALM[12]={0,0,0,0,0,0,0,0,0,0,0,0}; //this gains are moltiplied by 128 with respect to matlab
    int UP_LIMIT, BOT_LIMIT;

    // UP_LIMIT=((MAXVAL-NOLOAD)<<SHIFT);
    // BOT_LIMIT=(NOLOAD)<<SHIFT;
    UP_LIMIT=((MAXVAL-NOLOAD)<<triangle_cfg_list[triangleN].shift);
    BOT_LIMIT=(NOLOAD)<<triangle_cfg_list[triangleN].shift;

    Tpad_base=_pCapOffset[triangleN][ADCRESULT_S6];
    Tpad=AD7147Registers[triangleN][ADCRESULT_S6];

    Tpad_palm_base=_pCapOffset[triangleN][ADCRESULT_S11];
    Tpad_palm=AD7147Registers[triangleN][ADCRESULT_S11];
    err[triangleN].error=error_ok;
    err[triangleN].error_outofrange=0;
	unconnected=0;
    for (i=0;i<12;i++)
    {
        if (((_pCapOffset[triangleN][i]!=0) && ((AD7147Registers[triangleN][ADCRESULT_S0+i]==0)))) //reading error
        {
            err[triangleN].error |=error_noack;
			    
        }
        if (TEMP_COMPENSATION==1)
        {
            switch (TYPE)
            {
            case new_skin:
            {
                    if (Tpad>Tpad_base)
                    {
                    drift=(((Tpad-Tpad_base)>>2)*GAIN[i])>>5;
                    }
                    else
                    {
                    drift=-((((Tpad_base-Tpad)>>2)*GAIN[i])>>5);
                    }
                    test=drift;
            }
            break;

            case palm_fingertips:
            {
                    if (Tpad_palm>Tpad_palm_base)
                    {
                    drift=(((Tpad_palm-Tpad_palm_base)>>2)*GAIN_PALM[i])>>5;
                    }
                    else
                    {
                    drift=-((((Tpad_palm_base-Tpad_palm)>>2)*GAIN_PALM[i])>>5);
                    }
                    test=drift;
            }
            break;
            default:
            {
                    drift=0;
            }
            break;
            }
        }
        else drift=0;

        value=(AD7147Registers[triangleN][ADCRESULT_S0+i]-_pCapOffset[triangleN][i])-drift;

        if (value<=-UP_LIMIT)
        {
            txdata[i]=MAXVAL; // out of range, pressure too low
        }
        if (value>=BOT_LIMIT)
        {
            txdata[i]=MINVAL; // out of range, pressure too high
        }
        if ((value>-UP_LIMIT) && (value<BOT_LIMIT))
        {
                txdata[i]=NOLOAD-(value>>triangle_cfg_list[triangleN].shift);
        }
//check if the sensor is far from the limits -> taxel could be broken;
        if ((value<=-(UP_LIMIT<<1)) || (value>=(BOT_LIMIT<<1)))
        {
            err[triangleN].error_outofrange |= 1<<i;
        }
        if (AD7147Registers[triangleN][ADCRESULT_S0+i]==0xffff)
        {
            unconnected +=1; //if all the taxels has a 0xFFFF the triangle is most probably unconnected
        }
    }

    if (unconnected==12) //if all the taxels has a 0xFFFF the triangle is most probably unconnected
    {
        err[triangleN].error=error_notconnected;
    }

    if(new_configuration)
    {
        PMsgID = ICUBCANPROTO_CLASS_PERIODIC_SKIN << 8;
    }
    else
    {
        PMsgID=ICUBCANPROTO_CLASS_PERIODIC_ANALOGSENSOR << 8;
    }
    
    PMsgID |= ((triangleN) | BoardConfig.EE_CAN_BoardAddress<<4);
    //First message
    data[0]=0x40;
        for (i=1;i<8;i++)
        {
            data[i]    = (unsigned char)   (txdata[i-1] & 0xFF); //the last 6 bits
        }
    CAN1_send(PMsgID,1,8,data);
    //Second message
    data[0]=0xC0;
    for (i=1;i<6;i++)
    {
        data[i]    = (unsigned char)   (txdata[i+6] & 0xFF); //the last 6 bits
    }
    data[6]=(unsigned char) ((err[triangleN].error_outofrange &0x0ff0)>>4);
    data[7]=(unsigned char) ((err[triangleN].error_outofrange &0xf)<<4) | err[triangleN].error;
    CAN1_send(PMsgID,1,8,data);

    if ((err[triangleN].error!=error_ok)  && (err[triangleN].error != error_notconnected))//do again the configuration since an error has occured
    {
        j=(triangleN/4);
   //     SetCDCoffsetOnAllTriangles(ConValue[0]);
    //    ConfigAD7147(CH0,j,PW_CONTROL,ConValue); //0 is the number of the device
	//	board_MODE=CALIB;
		err[triangleN].error=error_ok;
		ERROR_COUNTER++;
        
    }
	if (ERROR_COUNTER==5)  //after x errors I do the CALIB of the board
	{
		triangle_cfg_list[triangleN].isToUpdate=1;
		board_MODE=CALIB;
		ERROR_COUNTER=0; 
		return;
	}
}






void SetCDCoffsetOnSingleTriangle(uint16_t cdcOffset, unsigned char triangleN)
{
//    uint16_t cdcOffset_aux = cdcOffset;
    ConfigAD7147_onSdaX(CH0, triangle_cfg_list[triangleN].setNum, triangle_cfg_list[triangleN].indexInSet,  PW_CONTROL, cdcOffset/*&cdcOffset_aux*/);
    
    //ConfigAD7147(CH0,triangle_cfg_list[triangleN].indexInSet,PW_CONTROL, &cdcOffset_aux); 
}

void SetCDCoffsetOnAllTriangles(uint16_t cdcOffset)
{
    uint8_t i;
    uint16_t cdcOffset_aux = cdcOffset; 
    for (i=0;i<4;i++)
    {
        ConfigAD7147(CH0,i,PW_CONTROL, &cdcOffset_aux); //0 is the number of the device
    }
}
static void FillCanMessages8bit_test(unsigned char Channel,unsigned char triangleN)
{
    unsigned char data[8];
    PMsgID=0x700;
    PMsgID |= ((triangleN) | BoardConfig.EE_CAN_BoardAddress<<4);
    
    /*
    shift;
    uint8_t                    indexInSet; //triangle index in set
    uint8_t                    setNum; //equal to sda num
    uint8_t                    isToUpdate; //if =1 triangle needs to be recalibrated 
    uint16_t                   CDCoffset;
    */
    
    
    //First message
    data[0] = 0x40+ triangle_cfg_list[triangleN].isToUpdate;
    data[1] = triangle_cfg_list[triangleN].setNum;
    data[2] = triangle_cfg_list[triangleN].indexInSet;
    data[3] = (triangle_cfg_list[triangleN].CDCoffset & 0x00ff );
    data[4] = (triangle_cfg_list[triangleN].CDCoffset & 0xff00)>>8;
    data[5] = triangle_cfg_list[triangleN].shift;
    data[6] = TRIANGLE_MASK & 0x00ff;
    data[7] = (TRIANGLE_MASK & 0xff00)>>8;
    
    CAN1_send(PMsgID,1,8,data);
    //Second message
    data[0]=0xC0;
    /*
    data[1] = (period_timer & 0x000000ff);
    data[2] = (period_timer &0x0000ff00) >> 8;
    data[3] = (period_timer &0x00ff0000) >> 16;
    data[4] = (period_timer &0xff000000) >> 24;
    data[5] = TIMER_VALUE & 0x00ff;
    data[6] = (TIMER_VALUE & 0xff00)>>8;
    data[7] = 0;
    */
    data[1] = 0xA0;
    data[2] = 0xB0;
    data[3] = 0xC5;
    data[4] = 0xC2;
    data[5] = 0xFF;
    data[6] = 0xED;
    data[7] = 0;
    
    CAN1_send(PMsgID,1,8,data);
}


void FillCanMessages8bit_all(unsigned char Channel,unsigned char triangleN)
{
    unsigned char data[8];
    unsigned int i,val;
    unsigned int txdata[12];

			i=0;
	        if (_pCapOffset_all[triangleN][i]>=AD7147Registers[triangleN][ADCRESULT_S0+i])
	        {
	            val=(_pCapOffset_all[triangleN][i]-AD7147Registers[triangleN][ADCRESULT_S0+i])>>SHIFT_ALL;
	            if (val>=10) txdata[i]=255;
	            else
	                txdata[i]=val+244;
	        } else
	        {
	            val=(AD7147Registers[triangleN][ADCRESULT_S0+i]-_pCapOffset_all[triangleN][i])>>SHIFT_ALL;
	            if (val>=243)   txdata[i]=1;
	            else
	                txdata[i]=244-val;
	        }
	    
	    PMsgID=0x300;   
	    PMsgID |= ((triangleN) | BoardConfig.EE_CAN_BoardAddress<<4);
	    //First message	
	    data[0]=0x40;       
        data[1]= (unsigned char)   (txdata[0] & 0xFF); 
	    CAN1_send(PMsgID,1,2,data); 
}
void FillCanMessages8bit_three(unsigned char Channel,unsigned char triangleN)
{
    unsigned char data[8];
    unsigned int i,val;
    unsigned int txdata[12];
 
	    for(i=0; i<3; i++)
	    {
	        if (_pCapOffset_all[triangleN][i]>=AD7147Registers[triangleN][ADCRESULT_S0+i])
	        {
	            val=(_pCapOffset_all[triangleN][i]-AD7147Registers[triangleN][ADCRESULT_S0+i])>>SHIFT_THREE;
	            if (val>=10) txdata[i]=255;
	            else
	                txdata[i]=val+244;
	        } else
	        {
	            val=(AD7147Registers[triangleN][ADCRESULT_S0+i]-_pCapOffset_all[triangleN][i])>>SHIFT_THREE;
	            if (val>=243)   txdata[i]=1;
	            else
	                txdata[i]=244-val;
	        }
	    }
	    PMsgID=0x300;   
	    PMsgID |= ((triangleN) | BoardConfig.EE_CAN_BoardAddress<<4);
	    //First message	
	    data[0]=0x40;       
		for (i=1;i<4;i++)
		{
		    data[i]    = (unsigned char)   (txdata[i-1] & 0xFF); 
	 	}  	
	    CAN1_send(PMsgID,1,4,data); 
}

///////////////////////////////////////////////////
// Melexis code  starts from here
// Tito

//to fill the can messages with Melexis' reading
static void FillCanMessages8bit_hall(unsigned char Channel,unsigned char sda_no)
{
    unsigned char data[8];
    unsigned int i,val;
    
    PMsgID=0x700;
    PMsgID |= ( chip_id | sda_no << 2 | BoardConfig.EE_CAN_BoardAddress<<4); //configure Message ID
    // | 11 | 10 | 9 | 8 |  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
    // |  Header ID      |  |   Board ID    |  SDA  | Chip  |
    if(sda_no == 0){
        for (i = 0; i <7; i++){
            data[i] = MLX90393Buffer_1[i]; //store status byte, x, y, z data
        }
        data[7] = sda_no + 1; //can be used for storing ID or temperature
    }
    
    else if(sda_no == 1){
        for (i = 0; i <7; i++){
            data[i] = MLX90393Buffer_2[i]; //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }
    
    else if(sda_no == 2){
        for (i = 0; i <7; i++){
            data[i] = MLX90393Buffer_3[i]; //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }
    
    else if(sda_no == 3){
        for (i = 0; i <7; i++){
            data[i] = MLX90393Buffer_4[i]; //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }
        CAN1_send(PMsgID,1,8,data);
}

/////////// Function for MLX90393 ///////////////////
void MLX90393_init(unsigned char i2c_address){
    //Write TCMP
    unsigned int i;
    unsigned int data[7];
    // Config address 0
    unsigned int Z_SERIES = 0x00 << 7; //0: Default and recommended. 1: Enable all plates for Z-measurement
    unsigned int GAIN_SEL = 0x07 << 4; // 0x07 is the highest response for the new skin // 0x00 is the lowest response// for old skin: 
    unsigned int HALLCONF = 0x0C; //Default configuration but does not allow the OSR & DIG_FILT to be 00, 01, or 11 (faster rate). We will use DIG_FILT 5 anyway
    // Config address 1
    unsigned int TRIG_INT =  0x0 << 15; //Puts TRIG_INT pin in TRIG mode when cleared
    unsigned int COMM_MODE = 0x3 << 13; //10b : only SPI. 11b : only I2C
    unsigned int WOC_DIFF = 0x0 << 12;  //Setup wake up on change, 0: off
    unsigned int EXT_TRIG = 0x0 << 11; // External trigger, 0: not allowed
    unsigned int TCMP_EN = 0x1 << 10; // Drift compensation, 0: off 1: on
    unsigned int BURST_SEL = 0xF << 6; //Select xyzt 
    unsigned int BURST_DATA_RATE = 0x00; //Defines T_INTERVAL = BURST_DATA_RATE * 20ms
    
    // Config address 2
    unsigned int OSR2 = 0x00 << 11;
    unsigned int RES_XYZ = 0x01 << 5; // If TCMP_EN is on, max resolution is 1. X = 1, Y = 1, Z = 1  //0x01 means x = 0 , y = 0, z = 1 //0x06 means x = 1, y = 1, z = 0; z with lowest resolution, x,y are about the same as z 
    unsigned int DIG_FILT = 0x05 << 2; //if measuring xyzt, we still can get around 138Hz of sampling rate
    unsigned int OSR = 0x00;

    //Write configuration
    data[0] = Z_SERIES | GAIN_SEL | HALLCONF;
    data[1] = TRIG_INT | COMM_MODE | WOC_DIFF | EXT_TRIG | TCMP_EN | BURST_SEL | BURST_DATA_RATE;
    data[2] = OSR2 | RES_XYZ | DIG_FILT | OSR;
    
    for (i = 0; i < 3; i++){ //configure register address 0 - 2
    write_buffer[0] = 0x60;
    write_buffer[1] = (data[i]&0xFF00)>>8;
    write_buffer[2] = data[i]&0x00FF;
    write_buffer[3] = i << 2;
    
    SendCommandI2C_MLX(CH0, i2c_address, write_buffer, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);
    }
      
    //Reset offset
    for (i = 4; i < 7; i++){ //configure register address 4 - 6
    write_buffer[0] = 0x60;
    write_buffer[1] = 0x00;
    write_buffer[2] = 0x00;
    write_buffer[3] = i << 2;
    SendCommandI2C_MLX(CH0, i2c_address, write_buffer, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);
    }

    //Start Burst
    SendViaI2C_MLX(CH0, i2c_address, SB, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);    
       
}

void MLX90393_calibration(unsigned char i2c_address) {
    unsigned int i;
    unsigned int offset_data[7];  //if we use unsigned int, it cannot recognize negative value
    char data[8];
    unsigned int DataToWrite;

    unsigned int offset_x;
    unsigned int offset_y;
    unsigned int offset_z;
    
    for(i = 0; i < 100; i++){
        // Do reading 100 times
    SendViaI2C_MLX(CH0, 0x0C, RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 7);
    FillCanMessages8bit_hall(CH0, 0);
    
    Wait(WAIT_value);
    }
    
    //Simply take the last reading as an offset. (Usually the very first readings are 0xFFFF, messing up with the average)
    offset_x = (MLX90393Buffer_1[1] << 8 | MLX90393Buffer_1[2]);
    offset_y = (MLX90393Buffer_1[3] << 8 | MLX90393Buffer_1[4]);
    offset_z = (MLX90393Buffer_1[5] << 8 | MLX90393Buffer_1[6]);
    
    SendViaI2C_MLX(CH0, i2c_address, EX, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);
    Wait(WAIT_value);
    
    //Write offset
    
    offset_data[4] = 0x8000;
    offset_data[5] = 0x8000;
    offset_data[6] = 0x5000; //make the z axis offset lower so the range will be higher [default]
    //offset_data[6] = 0x8000; //for old uSkin
    
    for (i = 4; i < 7; i++){ //configure register address 4 - 6
    DataToWrite = offset_data[i];
        
    write_buffer[0] = 0x60;
    write_buffer[1] = (DataToWrite & 0xFF00) >> 8;
    write_buffer[2] = DataToWrite & 0x00FF;
    write_buffer[3] = i << 2;
    
    SendCommandI2C_MLX(CH0, i2c_address, write_buffer, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);
    //to check whether the data is correct or not
    data[0] = MLX90393Buffer_1[0]; //Status byte
    data[1] = write_buffer[1]; //MSB
    data[2] = write_buffer[2]; //LSB
    data[3] = 0; 
    data[4] = 0; 
    data[5] = 0;
    data[6] = 0; 
    data[7] = i; //register address
    CAN1_send(0x700,1,8,data);
    }
    
    SendViaI2C_MLX(CH0, i2c_address, SB, 1, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
    FillCanMessages8bit_hall(CH0, 0);
}