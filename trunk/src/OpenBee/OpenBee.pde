// **********************************************************
// ******************   OpenBee Code      *******************
// ***  OpenBee Designed by Melih Karakelle on 2012       ***
// **              an Arudino based RF system              **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.01
// Latest Code Update : 2012-10-05
// Supported Hardware : OpenBee boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/
// Google Code Page   : http://code.google.com/p/openbee/
// **********************************************************

// ******************** OpenBee DEVELOPERS ****************** 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)

#include "config.h"

#include <EEPROM.h>


void setup() {   
       
        //LED and other interfaces
        pinMode(RED_LED_pin, OUTPUT); //RED LED
        pinMode(BLUE_LED_pin, OUTPUT); //GREEN LED
       
        //EEPROM check and update
        #if (PC_CONFIGURATION_ENABLED==1)
        eeprom_check(); 
        #endif
        
        //RF module pins
        pinMode(SDO_pin, INPUT); //SDO
        pinMode(SDI_pin, OUTPUT); //SDI        
	pinMode(SCLK_pin, OUTPUT); //SCLK
        pinMode(IRQ_pin, INPUT); //IRQ
        pinMode(nSel_pin, OUTPUT); //nSEL
        
        
        pinMode(0, INPUT); // Serial Rx
        pinMode(1, OUTPUT);// Serial Tx
        
           
        Serial.begin(SERIAL_BAUD_RATE);
        
     
      
         TCCR1B   =   0x00;   //stop timer
         TCNT1H   =   0x00;   //setup
         TCNT1L   =   0x00;
         ICR1     =   60005;   // used for TOP, makes for 50 hz
         TCCR1A   =   0x02;   
         TCCR1B   =   0x1A; //start timer with 1/8 prescaler for measuring 0.5us PPM resolution
         
         attachInterrupt(IRQ_interrupt,RFM22B_Int,FALLING);
}


//############ MAIN LOOP ##############
void loop() {
  
unsigned char i;

//wdt_enable(WDTO_1S);

RF22B_init_parameter(); 
frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RF module for this frequency, frequency hopping starts from this frequency.

Power_Set(7); // 100mW maximum RF output power

Red_LED_ON ;
delay(100);	

Red_LED_OFF;

transmitted = 0;
 to_rx_mode(); 
 sei();
 RF_Mode = Receive;
	
time = millis();
old_time = time;

ClearBuffers();

while(1)

              {    /* MAIN LOOP */	              
              time = millis();  
              if (_spi_read(0x0C)==0) // detect the locked module and reboot
                 {
                 Red_LED_ON;  
                 RF22B_init_parameter();
                 frequency_configurator(CARRIER_FREQUENCY);
                 rx_reset;
                 Red_LED_OFF;
                 }
              
               // telemetry receive procedure begin
               if (RF_Mode == Received)
                 {
                  Blue_LED_ON;  
                  send_read_address(0x7f); // Send the package read command
		  for(i = 0; i<RF_PACK_SIZE; i++) //read all buffer 
			{ 
			 RF_Rx_Buffer[i] = read_8bit_data(); 
			}  
		  rx_reset(); 
                  
                                    
                  if (RF_Rx_Buffer[0]=='B') // Brige values
                           {
                             for(i = 2; i<RF_Rx_Buffer[1]+2; i++) //write serial
                                 Serial.print(RF_Rx_Buffer[i]);
                              
                             RF_Rx_Buffer[0]=0; 
                           }  
                  /*
                  #if (BOARD_Mode==1) //for Master Only
                       Red_LED_ON ;
                       Telemetry_Bridge_Write(); 
                       Red_LED_OFF;
                  #endif  
                
                   rx_reset(); 
                   */   
                  RF_Mode = Receive;
                   
                  Blue_LED_OFF;
                 }	    
	      // telemetry receive procedure end

           // #if (BOARD_Mode==0) //for Master Only
           //  if (time> old_time+20) // Automatic 50hz transmit code for Master OpenBee 
             #if (BOARD_Mode==0) //for SimpleUnidirectional
              if (Serial.available() > 0) 
                     {
                      RF_Mode = Transmit; 
                       
                      old_time = time; 
		      //Green LED will be on during transmission  
	              Red_LED_ON ;

                      Telemetry_Bridge_Write();

                      //Blue LED will be OFF
                      Red_LED_OFF; 
  
                      //Hop to the next frequency
                      #if (FREQUENCY_HOPPING==1)
                         Hopping();
                      #endif   
                      
                      rx_reset(); 
                      RF_Mode = Receive;
                      			  
	              }

                  #endif  
		}
		 
}


