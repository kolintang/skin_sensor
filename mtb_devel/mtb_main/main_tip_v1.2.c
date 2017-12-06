// Based on IIT's code
// Updated 2017-10-31 by Tito
// Updated 2017-11-13 by Shu
// uSkin phalange 1.8

#include <timer.h>
#include <adc10.h>
#include <libpic30.h>
#include <p30f4011.h>

#include "LED.h"
#include "ADC.h"
#include "I2C.h"
#include "math.h"
#include "timers.h"
#include "eeprom.h"
#include "options.h"
#include "l3g4200d.h"
#include "lis331dlh.h"
#include "AD7147RegMap.h"
#include "can_interface.h"

#define CAN_MSG_CLASS_ACC_GYRO 0x500
#define MSG_TYPE_GYRO 0x000
#define MSG_TYPE_ACC 0x001
#define MSG_TYPE_ANALOG_ACC 0x002

// Inizializzazione bit di configurazione (p30f4013.h)
_FOSC(CSW_FSCM_OFF & EC_PLL8);                      // Clock switching disabled Fail safe Clock Monitor disabled
// External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FWDT(WDT_OFF);                                     // WD disabled
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV27);     // BOR 2.7V POR 64msec
_FGS(CODE_PROT_OFF);                                // Code protection disabled

enum Errors
{
    error_ok,
    error_noack,
    error_notconnected
};

/////////////////////////
// Function prototypes //
/////////////////////////

void Wait(unsigned int value);
unsigned int max(unsigned int a, unsigned int b);
unsigned int min(unsigned int a, unsigned int b);
void FillCanMessages8bit_hall(unsigned char Channel, unsigned char sda_no);
void FillCanMessages8bit_hall_limited(unsigned char Channel, unsigned int i2c_no, unsigned char sda_no);
void MLX90393_init(unsigned char i2c_address);
void MLX90393_calibration(unsigned char i2c_address);
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void);

////////////////////////
//  Global variables  //
////////////////////////
//
struct s_eeprom _EEDATA(1) ee_data =
{
    0x0,          // EE_B_EEErased             :1
    0x0,          // EE_B_EnableWD             :1
    0x1,          // EE_B_EnableBOR            :1
    0x05,         // EE_CAN_BoardAddress;      :8
    0x01,         // EE_CAN_MessageDataRate    :8
    0x04,         // EE_CAN_Speed;             :8
    {'u','S','k','i','n',' '},
    0x0000        // Checksum
};

// Board Configuration image from EEPROM
struct s_eeprom BoardConfig = { 0 };

typedef struct error_cap
{
    unsigned int error_outofrange;
    unsigned int error;
} error_cap;

triangle_cfg_t triangle_cfg_list[16];
volatile char flag;
volatile char flag2;

unsigned int AD7147Registers[16][12];
unsigned int MLX90393Buffer[4][4][8];
unsigned int MLX90393Buffer_0[8];
unsigned int MLX90393Buffer_1[8];
unsigned int MLX90393Buffer_2[8];
unsigned int MLX90393Buffer_3[8];
unsigned int MLX90393Buffer_4[8];
//unsigned int MLX90393Buffer_previous[4][4][8];
unsigned int MLX90393Buffer_1_previous[4][8];
unsigned int MLX90393Buffer_2_previous[4][8];
unsigned int MLX90393Buffer_3_previous[4][8];
unsigned int MLX90393Buffer_4_previous[4][8];

int track[4][4][3];
int difference[3];
int write_buffer[4];
int write_buffer_single[4];

int test = 0;
unsigned int RM[1] = { 0x4E };
unsigned int SB[1] = { 0x1F };
unsigned int EX[1] = { 0x80 };
const unsigned char AD7147_ADD[4] = { 0x2C,0x2D,0x2E,0x2F };
const unsigned char MLX90393_ADD[4] = { 0x0C,0x0D,0x0E,0x0F }; //Melexis Address
unsigned int chip_id;


unsigned int BitToSend;                         // Number of bit to be send
unsigned int _board_ID = 2;
unsigned char board_MODE = EIGHT_BITS;
unsigned char new_board_MODE = EIGHT_BITS;
char _additional_info[32] = { 'T','a','c','t','i','l','e',' ','S','e','n','s','o','r' };
unsigned int PW_CONTROL = 0x0B0;                // 0x1B0 for 128 decim  
unsigned int TIMER_VALUE = TIMER_10ms;          // Timer duration 0x3000=> 40ms  //For controlling the loop timer as defined in options.h; currently the delay is 10ms
//unsigned int TIMER_VALUE = TIMER_8ms;          // Timer duration 0x3000=> 40ms  //For controlling the loop timer as defined in options.h; currently the delay is 10ms
unsigned int TIMER_VALUE2 = 0xC00;              // 0x132; -> 50ms? //0xc00;//0x99;//1ms 0xc00;//0xC00; // Timer duration 0xC00=> 10ms
unsigned char SHIFT = 2;                        // Shift of the CDC value for removing the noise
unsigned char SHIFT_THREE = 3;                  // Shift of the CDC value for removing the noise
unsigned char SHIFT_ALL = 4;                    // Shift of the CDC value for removing the noise
unsigned char NOLOAD = 245;
unsigned char ANALOG_ACC = 0;                   // Snalog accelerometer, if one 1 messsage is sent every 10ms
unsigned int ANALOG_ID = 0x552;                 // Default value
unsigned char DIG_EXT_GYRO = 0;                 // Gyro of the MMSP (PALM)
unsigned char DIG_EXT_ACC = 0;                  // Accelerometer of the MMSP (PALM) 
unsigned char DIG_ACC = 0;                      // Internal accelerometer 
unsigned char TEMP_COMPENSATION = 1;            // If 1 means internal temperature drift compensation 
int Tpad_base;                                  // Initial value of the Tpad
int Tpad;                                       // Current value of the Tpad
int Tpad_palm_base;                             // Initial value of the Tpad in the palm
int Tpad_palm;                                  // Current value of the Tpad in the palm
int drift;                                      // Current drift
uint8_t can_transmission_enabled = 0;
uint8_t transmission_was_enabled = 0;
uint8_t new_configuration = 0;                  // If mtb receives ICUBCANPROTO_POL_SK_CMD__SET_BRD_CFG or ICUBCANPROTO_POL_SK_CMD__SET_TRIANG_CFG
// Then it uses class 4 to send periodic messages

enum skin_type TYPE = new_skin;                 // SKIN_2; if =0 means new skin with drift compensation and 10 pads
unsigned int TRIANGLE_MASK = 0xFFFF;            // All the triangles are enabled for default
unsigned char CONFIG_TYPE = CONFIG_SINGLE;
unsigned char ERROR_COUNTER = 0;                // It counts the errors in reading the triangles.
unsigned int ConValue[2] = { 0x2200, 0x2200 };  // Offset of the CDC reading 
volatile unsigned int PMsgID;                   // Pressure measurement ID 
unsigned char acc[] = { 0,0,0,0,0,0,0,0 };      // Value of the three accelerometers
unsigned char gyro[] = { 0,0,0,0,0,0,0,0 };     // Value of the three gyro
volatile  int AN2 = 0;                          // Analog Accelerometer X axes
volatile  int AN3 = 0;                          // Analog Accelerometer Y axes
volatile  int AN4 = 0;                          // Analog Accelerometer Z axes
tL3GI2COps l3g;
tLISI2COps l3a;
uint32_t period_timer = 0;                      // For debug purpose



//////////////////////////
//  External Functions  //
//////////////////////////

extern void ConfigAD7147(unsigned char Channel, unsigned int i, unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_THREE(unsigned char Channel, unsigned int i, unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_FINGERTIP(unsigned char Channel, unsigned int i, unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_ALL(unsigned char Channel, unsigned int i, unsigned int pw_control_val, unsigned int * convalue);
extern void ConfigAD7147_onSdaX(unsigned char Channel, unsigned char setNum, unsigned char indexInSet, unsigned int pw_control_val, uint16_t cdcoffset/*unsigned int * cdcOffset*/);



///////////////////////////
//  Interrrupt Routines  //
///////////////////////////

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    flag = 1;
    _T1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    flag2 = 1;
    if (ANALOG_ACC)
    {
        acc[0] = ((AN2 & 0xFF00) >> 0x8);   // axis X
        acc[1] = (AN2 & 0xFF);
        acc[2] = ((AN3 & 0xFF00) >> 0x8);   // axis Y
        acc[3] = (AN3 & 0xFF);
        acc[4] = ((AN4 & 0xFF00) >> 0x8);   // axis Z
        acc[5] = (AN4 & 0xFF);

        while (!CAN1IsTXReady(1));
        CAN1SendMessage((CAN_TX_SID(ANALOG_ID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ,
                (CAN_TX_EID(0)) & CAN_NOR_TX_REQ, acc, 6, 1);
        // Executing the sampling of the
        ADCON1bits.SAMP = 1;
    }
    _T2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{
    // Ready to send a CAN message
    if (C1INTFbits.TX0IF || C1INTFbits.TX1IF || C1INTFbits.TX2IF) {
        CAN1_interruptTx();
    }

    // Receive a CAN message
    if (C1INTFbits.RX0IF || C1INTFbits.RX1IF) {
        CAN1_interruptRx();
    }
    IFS1bits.C1IF = 0;
}



/////////////////////
//  Main Function  //
/////////////////////

int main(void)
{
    unsigned int  i, j, k;
    unsigned int led_counter = 0;
    int calib_timeout = 0;

    // EEPROM Data Recovery
    // Initialize BoardConfig variable in RAM with the Data EEPROM stored values
    RecoverConfigurationFromEEprom();
    _board_ID = BoardConfig.EE_CAN_BoardAddress;

    ///////////////////////
    //  Peripheral init  //
    ///////////////////////

    T1_Init(TIMER_VALUE);
    I2C_Init(CH0);
    LED_Init();
    CAN_Init();

    for (i = 0; i < 4; i++) {
        MLX90393_init(MLX90393_ADD[i]);
        MLX90393_calibration(MLX90393_ADD[i]);
        Wait(WAIT_value);
    }

    // Initialize tracking and difference buffer
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 3; k++) {
                track[i][j][k] = 0;
                difference[k] = 0;
            }
        }
    }

    // Feed initial reading into previous buffer
    for (i = 0; i < 4; i++) {
        chip_id = i;
        SendViaI2C_MLX_finger(CH0, MLX90393_ADD[i], RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, 7);

        // Save previous time-step buffer
        for (k = 0; k < 8; k++) {
            MLX90393Buffer_1_previous[i][k] = MLX90393Buffer_1[k];
            MLX90393Buffer_2_previous[i][k] = MLX90393Buffer_2[k];
        }
    }

    for (i = 0; i < 1; i++) {
        chip_id = i;
        SendViaI2C_MLX_finger(CH0, MLX90393_ADD[i], RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, 7);

        // Save previous time-step buffer
        for (k = 0; k < 8; k++) {
            MLX90393Buffer_1_previous[i][k] = MLX90393Buffer_1[k];
            MLX90393Buffer_2_previous[i][k] = MLX90393Buffer_2[k];
        }
    }



    /////////////////
    //  Main Loop  //
    /////////////////

    led_counter = 0;
    led0 = 1;
    EnableIntCAN1;
    DisableIntT1;
    DisableIntT2;

    flag = 0;
    flag2 + 0;

    for (;;)
    {

        if ((DIG_EXT_GYRO || DIG_ACC || DIG_EXT_ACC) && (flag2)) {
            flag2 = 0;
        }

        if (flag == 1)
        {
            calib_timeout = 0;
            flag = 0;
            i = 0;

            if (led_counter == 20) {
                if (led0 == 1) led0 = 0;
                else led0 = 1;

                led_counter = 0;
            }
            led_counter++;

            // Read 4 chips for the measurement
            for (i = 0; i < 4; i++) {
                chip_id = i;

                // Save previous time-step buffer
                for (k = 0; k < 8; k++) {
                    MLX90393Buffer_1_previous[i][k] = MLX90393Buffer_1[k];
                    MLX90393Buffer_2_previous[i][k] = MLX90393Buffer_2[k];
                }

                SendViaI2C_MLX_finger(CH0, MLX90393_ADD[i], RM, 1, MLX90393Buffer_1, MLX90393Buffer_2, 7);
                //SendViaI2C_MLX_onSdaX(CH0, MLX90393_ADD[0], 0, RM, 1, MLX90393Buffer_1, 7);

                // Fill the CAN with measurement data
                for (j = 0; j < 2; j++) {
                    FillCanMessages8bit_hall(CH0, j);
                    //FillCanMessages8bit_hall_limited(CH0, i, j);
                }
            }

        } // if (flag==1)
        CAN1_handleRx(_board_ID);
    } // for(;;)
} // main



//////////////////////////
// Function Definitions //
//////////////////////////


void Wait(unsigned int value)
{
    while (value > 0) {
        value--;
    }
}


unsigned int max(unsigned int a, unsigned int b) {
    return (a > b) ? a : b;
}


unsigned int min(unsigned int a, unsigned int b) {
    return (a < b) ? a : b;
}


void FillCanMessages8bit_hall(unsigned char Channel, unsigned char sda_no)
{
    unsigned char data[8];
    unsigned int i;

    PMsgID = 0x700;
    PMsgID |= (chip_id | sda_no << 2 | BoardConfig.EE_CAN_BoardAddress << 4); //configure Message ID

    // | 11 | 10 | 9 | 8 |  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
    // |  Header ID      |  |   Board ID    |  SDA  | Chip  |

    if (sda_no == 0) {
        for (i = 0; i < 7; i++) {
            data[i] = MLX90393Buffer_1[i];  //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;               //can be used for storing ID or temperature
    }

    else if (sda_no == 1) {
        for (i = 0; i < 7; i++) {
            data[i] = MLX90393Buffer_2[i];  //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }

    else if (sda_no == 2) {
        for (i = 0; i < 7; i++) {
            data[i] = MLX90393Buffer_3[i];  //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }

    else if (sda_no == 3) {
        for (i = 0; i < 7; i++) {
            data[i] = MLX90393Buffer_4[i];  //store status byte, x, y, z data
        }
        data[7] = sda_no + 1;
    }
    CAN1_send(PMsgID, 1, 8, data);
}


void FillCanMessages8bit_hall_limited(unsigned char Channel, unsigned int i2c_no, unsigned char sda_no)
{
    // Z-axis has a range of 45000 = AF
    int limit_threshold = 0xB0;
    int limit_upper = 0xFA;
    int limit_lower = 0x0A;
    unsigned char data[8] = {0,0,0,0,0,0,0,0};
    unsigned int ii;

    PMsgID = 0x700;
    PMsgID |= (chip_id | sda_no << 2 | BoardConfig.EE_CAN_BoardAddress << 4);

    if (sda_no == 0) {

        // Transfer MLX buffer to CAN buffer
        for (ii = 0; ii < 8; ii++) {
            data[ii] = MLX90393Buffer_1[ii];
        }

        // Take difference
        for (ii = 0; ii < 3; ii++) {
            difference[ii] = (max(MLX90393Buffer_1_previous[i2c_no][ii], MLX90393Buffer_1[ii*2+1]) - min(MLX90393Buffer_1_previous[i2c_no][ii], MLX90393Buffer_1[ii*2+1]));

            // Track handler
            if (difference[ii] >= limit_threshold) {
                if (MLX90393Buffer_1_previous[i2c_no][ii] > MLX90393Buffer_1[ii*2+1]) {
                    track[i2c_no][sda_no][ii]++;
                }

                else if (MLX90393Buffer_1_previous[i2c_no][ii] < MLX90393Buffer_1[ii*2+1]) {
                    track[i2c_no][sda_no][ii] = track[i2c_no][sda_no][ii] - 1;
                }
            }

            // Limit CAN buffer according to tracking
            //track[i2c_no][sda_no][0] = -1;
            //track[i2c_no][sda_no][1] = -1;

            if (track[i2c_no][sda_no][ii] > 0) {
                data[ii*2+1] = limit_upper;
            }
            else if (track[i2c_no][sda_no][ii] < 0) {
                data[ii*2+1] = limit_lower;
            }
        }
    }

    else if (sda_no == 1) {

        // Transfer MLX buffer to CAN buffer
        for (ii = 0; ii < 8; ii++) {
            data[ii] = MLX90393Buffer_2[ii];
        }

        // Take difference
        for (ii = 0; ii < 3; ii++) {
            difference[ii] = (max(MLX90393Buffer_2_previous[i2c_no][ii], MLX90393Buffer_2[ii*2+1]) - min(MLX90393Buffer_2_previous[i2c_no][ii], MLX90393Buffer_2[ii*2+1]));

            // Track handler
            if (difference[ii] >= limit_threshold) {
                if (MLX90393Buffer_2_previous[i2c_no][ii] > MLX90393Buffer_2[ii*2+1]) {
                    track[i2c_no][sda_no][ii]++;
                }

                else if (MLX90393Buffer_2_previous[i2c_no][ii] < MLX90393Buffer_2[ii*2+1]) {
                    track[i2c_no][sda_no][ii] = track[i2c_no][sda_no][ii] - 1;
                }
            }

            // Limit CAN buffer according to tracking
            //track[i2c_no][sda_no][0] = -1;
            //track[i2c_no][sda_no][1] = -1;

            if (track[i2c_no][sda_no][ii] > 0) {
                data[ii * 2 + 1] = limit_upper;
            }
            else if (track[i2c_no][sda_no][ii] < 0) {
                data[ii * 2 + 1] = limit_lower;
            }
        }
    }

    else if (sda_no == 2) {

        // Transfer MLX buffer to CAN buffer
        for (ii = 0; ii < 8; ii++) {
            data[ii] = MLX90393Buffer_3[ii];
        }

        // Take difference
        for (ii = 0; ii < 3; ii++) {
            difference[ii] = (max(MLX90393Buffer_3_previous[i2c_no][ii], MLX90393Buffer_3[ii*2+1]) - min(MLX90393Buffer_3_previous[i2c_no][ii], MLX90393Buffer_3[ii*2+1]));

            // Track handler
            if (difference[ii] >= limit_threshold) {
                if (MLX90393Buffer_3_previous[i2c_no][ii] > MLX90393Buffer_3[ii*2+1]) {
                    track[i2c_no][sda_no][ii]++;
                }

                else if (MLX90393Buffer_3_previous[i2c_no][ii] < MLX90393Buffer_3[ii*2+1]) {
                    track[i2c_no][sda_no][ii] = track[i2c_no][sda_no][ii] - 1;
                }
            }

            // Limit CAN buffer according to tracking
            //track[i2c_no][sda_no][0] = -1;
            //track[i2c_no][sda_no][1] = -1;

            if (track[i2c_no][sda_no][ii] > 0) {
                data[ii * 2 + 1] = limit_upper;
            }
            else if (track[i2c_no][sda_no][ii] < 0) {
                data[ii * 2 + 1] = limit_lower;
            }
        }
    }

    else if (sda_no == 3) {

        // Transfer MLX buffer to CAN buffer
        for (ii = 0; ii < 7; ii++) {
            data[ii] = MLX90393Buffer_4[ii];
        }

        // Take difference
        for (ii = 0; ii < 3; ii++) {
            difference[ii] = (max(MLX90393Buffer_4_previous[i2c_no][ii], MLX90393Buffer_4[ii*2+1]) - min(MLX90393Buffer_4_previous[i2c_no][ii], MLX90393Buffer_4[ii*2+1]));

            // Track handler
            if (difference[ii] >= limit_threshold) {
                if (MLX90393Buffer_4_previous[i2c_no][ii] > MLX90393Buffer_4[ii*2+1]) {
                    track[i2c_no][sda_no][ii]++;
                }

                else if (MLX90393Buffer_4_previous[i2c_no][ii] < MLX90393Buffer_4[ii*2+1]) {
                    track[i2c_no][sda_no][ii] = track[i2c_no][sda_no][ii] - 1;
                }
            }

            // Limit CAN buffer according to tracking
            //track[i2c_no][sda_no][0] = -1;
            //track[i2c_no][sda_no][1] = -1;

            if (track[i2c_no][sda_no][ii] > 0) {
                data[ii * 2 + 1] = limit_upper;
            }
            else if (track[i2c_no][sda_no][ii] < 0) {
                data[ii * 2 + 1] = limit_lower;
            }
        }
    }

    CAN1_send(PMsgID, 1, 8, data);
}


void MLX90393_init(unsigned char i2c_address)
{
    // Write TCMP
    unsigned int i;
    unsigned int j;
    unsigned int data[7];

    // Configure address 0
    unsigned int Z_SERIES = 0x00 << 7;      // 0: Default and recommended. 1: Enable all plates for Z-measurement
    unsigned int GAIN_SEL = 0x07 << 4;      // 0x07 is the highest response for the new skin // 0x00 is the lowest response for old skin:
    unsigned int GAIN_SEL_ARRAY[8] = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15
    unsigned int HALLCONF = 0x0C;           // Default configuration but does not allow the OSR & DIG_FILT to be 00, 01, or 11 (faster rate). We will use DIG_FILT 5 anyway

    // Configure address 1
    unsigned int TRIG_INT = 0x0 << 15;      // Puts TRIG_INT pin in TRIG mode when cleared
    unsigned int COMM_MODE = 0x3 << 13;     // 10b : only SPI. 11b : only I2C
    unsigned int WOC_DIFF = 0x0 << 12;      // Setup wake up on change, 0: off
    unsigned int EXT_TRIG = 0x0 << 11;      // External trigger, 0: not allowed
    unsigned int TCMP_EN = 0x1 << 10;       // Drift compensation, 0: off 1: on
    unsigned int BURST_SEL = 0xF << 6;      // Select xyzt 
    unsigned int BURST_DATA_RATE = 0x00;    // Defines T_INTERVAL = BURST_DATA_RATE * 20ms

    // Configure address 2
    unsigned int OSR2 = 0x00 << 11;
    unsigned int RES_XYZ = 0x01 << 5;       // If TCMP_EN is on, max resolution is 1. X = 1, Y = 1, Z = 1  //0x01 means x = 0 , y = 0, z = 1 //0x06 means x = 1, y = 1, z = 0; z with lowest resolution, x,y are about the same as z
    unsigned int RES_XYZ_ARRAY[8] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
    unsigned int DIG_FILT = 0x05 << 2;      // If measuring xyzt, we still can get around 138Hz of sampling rate
    unsigned int OSR = 0x00;

    // Write configuration individually
    for (i = 0; i < 8; i++) {

        GAIN_SEL_ARRAY[i] = GAIN_SEL_ARRAY[i] << 4;
        RES_XYZ_ARRAY[i] = RES_XYZ_ARRAY[i] << 5;

        data[0] = Z_SERIES | GAIN_SEL_ARRAY[i] | HALLCONF;
        data[1] = TRIG_INT | COMM_MODE | WOC_DIFF | EXT_TRIG | TCMP_EN | BURST_SEL | BURST_DATA_RATE;
        data[2] = OSR2 | RES_XYZ_ARRAY[i] | DIG_FILT | OSR;

        unsigned int i2c_address_buffer = MLX90393_ADD[i/2];
        unsigned int sda_num_buffer = i%2;

        for (j = 0; j < 3; j++) {
            write_buffer[0] = 0x60;
            write_buffer[1] = (data[j] & 0xFF00) >> 8;
            write_buffer[2] = data[j] & 0x00FF;
            write_buffer[3] = j << 2;
            SendCommandI2C_MLX_sdax(CH0, i2c_address_buffer, sda_num_buffer, write_buffer, MLX90393Buffer_0, 1);
        }
    }

    // Reset offset by configuring register address 4 - 6
    for (i = 4; i < 7; i++) {
        write_buffer[0] = 0x60;
        write_buffer[1] = 0x00;
        write_buffer[2] = 0x00;
        write_buffer[3] = i << 2;
        SendCommandI2C_MLX(CH0, i2c_address, write_buffer, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
        //FillCanMessages8bit_hall(CH0, 0);
    }

    // Start Burst
    SendViaI2C_MLX_finger(CH0, i2c_address, SB, 1, MLX90393Buffer_1, MLX90393Buffer_2, 1);
    //SendViaI2C_MLX_onSdaX(CH0, MLX90393_ADD[0], 0, SB, 1, MLX90393Buffer_1, 7);
    FillCanMessages8bit_hall(CH0, 0);

}


void MLX90393_calibration(unsigned char i2c_address) {

    unsigned int i, j;
    unsigned int offset_data[3];
    unsigned int target_value[3] = { 0x80, 0x80, 0x0C };
    unsigned int DataToWrite;
    unsigned int DataToWrite_array;

    // One Taxel Settings = {{X}, {Y}, {Z}}
    //unsigned int offset_data_array[3][16] = {
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90} };
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    // MTB2 Taxel Settings = {{X}, {Y}, {Z}}
    //unsigned int offset_data_array[3][16] = {
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0xA0, 0x90, 0x98, 0x98, 0xC8, 0xA0, 0x68, 0xB8, 0xD0, 0xA8, 0xA8, 0x98, 0xA8, 0xA8, 0x60, 0xA8} };
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    // MTB3 Taxel Settings = {{X}, {Y}, {Z}}
    unsigned int offset_data_array[3][16] = {
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
        {0xA0, 0x90, 0x98, 0x98, 0xC8, 0xA0, 0x68, 0xB8, 0xD0, 0xA8, 0xA8, 0x98, 0xA8, 0xA8, 0x60, 0xA8} };
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    // Fabric Settings = {{X}, {Y}, {Z}}
    //unsigned int offset_data_array[3][16] = {
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60},
    //    {0x70, 0x70, 0x70, 0x70, 0x70, 0x50, 0x50, 0x70, 0x70, 0x50, 0x70, 0x70, 0x80, 0x70, 0x70, 0x80}};
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    // Foil Settings = {{X}, {Y}, {Z}}
    //unsigned int offset_data_array[3][16] = {
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
    //    {0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0}};
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    // 0 as Initial = {{X}, {Y}, {Z}}
    //unsigned int offset_data_array[3][16] = {
    //    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    //    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    //    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
    //     0     4     8     12    1     5     9     13    2     6     10    14    3     7     11    15

    SendViaI2C_MLX_finger(CH0, i2c_address, EX, 1, MLX90393Buffer_1, MLX90393Buffer_2, 1);
    FillCanMessages8bit_hall(CH0, 0);
    Wait(WAIT_value);

    // Write offset
    offset_data[0] = 0x8000;	 //X
    offset_data[1] = 0x8000;	 //Y
    offset_data[2] = 0x5000;	 //Z

    // Configure register address 4 - 6
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 8; j++) {

            //DataToWrite = offset_data[i];
            DataToWrite = offset_data_array[i][j];
            DataToWrite = DataToWrite << 8 | 0x00;

            unsigned int i2c_address_buffer = MLX90393_ADD[j/2];
            unsigned int sda_num_buffer = j%2;

            write_buffer[0] = 0x60;
            write_buffer[1] = (DataToWrite & 0xFF00) >> 8;
            write_buffer[2] = DataToWrite & 0x00FF;
            write_buffer[3] = i+4 << 2;

            //SendCommandI2C_MLX(CH0, i2c_address, write_buffer, MLX90393Buffer_1, MLX90393Buffer_2, MLX90393Buffer_3, MLX90393Buffer_4, 1);
            SendCommandI2C_MLX_sdax(CH0, i2c_address_buffer, sda_num_buffer, write_buffer, MLX90393Buffer_0, 1);
            //FillCanMessages8bit_hall(CH0, 0);

        }
    }

    SendViaI2C_MLX_finger(CH0, i2c_address, SB, 1, MLX90393Buffer_1, MLX90393Buffer_2, 1);
    //SendViaI2C_MLX_onSdaX(CH0, MLX90393_ADD[0], 0, SB, 1, MLX90393Buffer_1, 7);
    //FillCanMessages8bit_hall(CH0, 0);
}

