// ***********************************************************
// ***          OpenBee Configuration file               **
// **       This Source code licensed under GPL             **
// ***********************************************************
// Version Number     : 1.01
// Latest Code Update : 2012-10-05
// Supported Hardware : OpenBee boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/
// Google Code Page   : http://code.google.com/p/openbee/
// **********************************************************


//####### BOARD TYPE #######
// 0 = Original OpenBee 100mW Board
#define BOARD_TYPE 0

//####### BOARD Mode #######
//0 = SimpleUnidirectional / 1 = Master / 2 = Slave
// Master and Slave modes under development
#define BOARD_Mode 0

//####### PC BASED CONFIGURATOR #######
#define PC_CONFIGURATION_ENABLED 1 // 1 = Enabled  0 = Disabled

//######### TRANSMISSION VARIABLES ##########
//!!! These values configurable over PC with OpenLRS Configurator Software
//Open the configurator software when FTDı cable connected. 
//And press Set Dafaults button for configuring your device EEPROM

unsigned long CARRIER_FREQUENCY = 435000;  // 435Mhz startup frequency

unsigned char HOPPING_STEP_SIZE = 6;// 60kHz hopping steps

#define FREQUENCY_HOPPING 0 // 1 = Enabled  0 = Disabled (It's disabled in this version)

//###### HOPPING CHANNELS #######
//Select the hopping channels between 0-255
// Default values are 13,54 and 23 for all transmitters and receivers, you should change it before your first flight for safety.
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static unsigned char hop_list[3] = {13,54,23};



//###### RF DEVICE ID HEADERS #######
// Change this 4 byte values for isolating your transmission, RF module accepts only data with same header
static unsigned char RF_Header[4] = {'O','L','R','S'};  

//###### SERIAL PORT SPEED #######
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed


//###### RANGE ALERTS #######
// Alerts only works when the telemetry system active
// If you using the 7W booster, disable the alert modes and telemetry because our 7W booster is unidirectional
// (Code from OpenLRS project, functions currently disabled)
//#define Rx_RSSI_Alert_Level 0  // 40 is the package lost limit, 0 for disabling
//#define Tx_RSSI_Alert_Level 0  // 40 is the package lost limit, 0 for disabling
//#define Lost_Package_Alert 0 // 0 = Disabled, 1=Alert with each lost pack, 2=Alert with 2 or more lost package(suggested value) 


//############ VARIABLES ########################

#define RF_PACK_SIZE 40

unsigned char RF_Rx_Buffer[RF_PACK_SIZE];
unsigned char RF_Tx_Buffer[RF_PACK_SIZE]; 

unsigned char Telemetry_Buffer[RF_PACK_SIZE];

volatile unsigned int transmitted=1;

static unsigned char hopping_channel = 1;
unsigned long time,old_time; //system timer

unsigned char Rx_RSSI = 110;
unsigned char Tx_RSSI = 110;


volatile unsigned char RF_Mode = 0;
#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

//####### Board Pinouts #########
#if (BOARD_TYPE == 0)
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= 0x04 //C2
      #define  SCK_off PORTC &= 0xFB //C2
      
      #define  SDI_on PORTC |= 0x02 //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0
      
      //#### Other interface pinouts ###
      #define BLUE_LED_pin 13
      #define RED_LED_pin A3
    
      #define Red_LED_ON  PORTC |= _BV(3);
      #define Red_LED_OFF  PORTC &= ~_BV(3);
      
      #define Blue_LED_ON  PORTB |= _BV(5);
      #define Blue_LED_OFF  PORTB &= ~_BV(5);
      
      
#endif  




