// **********************************************************
// ******************   OpenLRS Rx Code   *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.10
// Latest Code Update : 2011-10-04
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
// # PROJECT DEVELOPERS # 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
// thUndead (forum nick name: thUndead) 



#include "config.h"

#include <EEPROM.h>

#include <Wire.h>

#if defined(BMP085)
    #include <APM_BMP085.h> // ArduPilot Mega BMP085 Library
#endif  




String GPS_Latitude = "";
String GPS_Longitude = "";
String GPS_Altitude = "";
String GPS_Speed = "";
String GPS_Heading = "";
String GPS_Time = "";
String GPS_Date = "";
String GPS_Status = "";
String GPS_Mag_Variation = "";

//#include <string.h>
//#include <ctype.h>



void setup() {   
        //LEDs
        pinMode(GREEN_LED_pin, OUTPUT);  
        pinMode(RED_LED_pin, OUTPUT);
        
        //RF module pins
        pinMode(SDO_pin, INPUT); //SDO
        pinMode(SDI_pin, OUTPUT); //SDI        
	pinMode(SCLK_pin, OUTPUT); //SCLK
        pinMode(IRQ_pin, INPUT); //IRQ
        pinMode(nSel_pin, OUTPUT); //nSEL
        
        
        pinMode(0, INPUT); // Serial Rx
        pinMode(1, OUTPUT);// Serial Tx
        
        
        pinMode(RSSI_OUT, OUTPUT); //RSSI pinout
        
        pinMode(Servo1_OUT, OUTPUT); //Servo1
        pinMode(Servo2_OUT, OUTPUT); //Servo2
        pinMode(Servo3_OUT, OUTPUT); //Servo3
        pinMode(Servo4_OUT, OUTPUT); //Servo4
        pinMode(Servo5_OUT, OUTPUT); //Servo5
        pinMode(Servo6_OUT, OUTPUT); //Servo6
        pinMode(Servo7_OUT, OUTPUT); //Servo7
        pinMode(Servo8_OUT, OUTPUT); //Servo8
        
        Serial.begin(SERIAL_BAUD_RATE); //Serial Transmission 
        Wire.begin(); //I2C Transmission
        
        
        #if defined(GPS)
         GPS_Init();
        #endif 

        #if defined(WiiMotionPlus)
          wmp_On(); //turn WM+ on
          wmp_calibrateZeroes(); //calibrate zeroes
        #endif  
        
        #if defined(MMA7455)
          MMA7455_Init(); //Set the MMA7455 Accelerometer Sensitity Value @ 2g
          MMA7455_Calibrate();
        #endif 

        #if defined(HMC5883L)
          initHMC5883L();
        #endif
        
        #if defined(BMP085)
          APM_BMP085.Init();   // BMP085 initialization
        #endif
        
       INIT_SERVO_DRIVER();
       
       attachInterrupt(IRQ_interrupt,RFM22B_Int,FALLING);
        	
}


//############ SERVO INTERRUPT ##############
// We configured the ICR1 value for 40.000 into the "init_servo_driver" function. 
// It's mean this interrupt works when the Timer1 value equal 40.000
// we are configuring it for 40.000 - servo_signal_time for each servo channel, and The interrupt generating perfect servo timings for us.
// Timer1 configured for 1/8 CPU clock. with this configuration, each clock time is equal 0.5us and we are driving the servo with 2048 step resolution.
ISR(TIMER1_OVF_vect)
  {
  unsigned int us; // this value is not real microseconds, we are using 0.5us resolution (2048 step), this is why the all values 2 times more than real microseconds.
  
  Servo_Ports_LOW;

  Servo_Number++; // jump to next servo
  if (Servo_Number>8) // back to the first servo 
    {
    total_ppm_time = 0; // clear the total servo ppm time
    Servo_Number=0;
    }
 

  if (Servo_Number == 8)  // Check the servo number. 
      {
        //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
        us = 40000 - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times
      }
      else
        us = Servo_Position[Servo_Number]; // read the servo timing from buffer
  
  total_ppm_time += us; // calculate total servo signal times.
 
  if (receiver_mode==0) // Parallel PPM
    {  
    switch (Servo_Number) {
      case 0:
        Servo1_OUT_HIGH;
        break;
      case 1:
        Servo2_OUT_HIGH;
        break;
      case 2:
        Servo3_OUT_HIGH;
        break;
      case 3:
        Servo4_OUT_HIGH;
        break;
      case 4:
       Servo5_OUT_HIGH;
        break;
      case 5:
        Servo6_OUT_HIGH;
        break;
      case 6:
        Servo7_OUT_HIGH;
        break;
      case 7:
        Servo8_OUT_HIGH;
        break;
      case 8:
        Servo9_OUT_HIGH;
        break;  
      
        }     
    }
  #if (SERIAL_PPM_TYPE == 0)
  else  // Serial PPM over 8th channel
    {
    Serial_PPM_OUT_HIGH;
    }
  #else
  else //Mixed Serial+Parallel PPM
    {
    Servo4_OUT_HIGH; //serial from 4th channel  
    switch (Servo_Number) { //last 4 channel works normally
      case 4:
        Servo5_OUT_HIGH;
        break;
      case 5:
        Servo6_OUT_HIGH;
        break;
      case 6:
        Servo7_OUT_HIGH;
        break;
      case 7:
        Servo8_OUT_HIGH;
        break;  
      case 8:
        Servo9_OUT_HIGH;
        break;
        }   
    }  
  #endif  
    
  TCNT1 = 40000 - us; // configure the timer interrupt for X micro seconds     
}


//############ MAIN LOOP ##############
void loop() {
  
  unsigned char i,tx_data_length;
  unsigned char first_data = 0;

  
  receiver_mode = check_modes(); // Check the possible jumper positions for changing the receiver mode.
  
  load_failsafe_values();   // Load failsafe values on startup
  
  if (receiver_mode == 1) Red_LED_Blink(3); // 3x Red LED blinks for serial PPM mode.
  

  Red_LED_ON;

  RF22B_init_parameter(); // Configure the RFM22B's registers

  frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.
 
  to_rx_mode(); 

  sei();


  //Hop to first frequency from Carrier
  #if (FREQUENCY_HOPPING==1)
    Hopping();
  #endif   
  
  RF_Mode = Receive;
	
  time = millis();

  last_pack_time = time; // reset the last pack receiving time for first usage
  last_hopping_time = time; // reset hopping timer

  while(1){    /* MAIN LOOP */
  
  
  
  
  // ========================================================================================================                        
// Spectrum-analyzer part
// ========================================================================================================

#if (SPECTRUM_ANALYZER == 1)
Serial.begin(115200);

unsigned char read_serial =0;
char serial_buf =0;
long temp_buf[4] = {0,0,0,0};
unsigned char buf_index =0;

long totalreading;
int rssi_max = 0;
int rssi_min = 255;
long number_average = 1500;

int start_freq = (430-1);
int end_freq = (440);

int step_size = 5;

int frequency = 430;

// When running frequency scanning we want 10 khz between each channel, meaning 200 channels = 2 mhz. 
                        _spi_write(0x7a, 0x01);    // 50khz step size (10khz x value)

			// Run infinite loop..
                        while (1==1) {
                          
                          // Set frequency:
                          frequency++;
                          if (frequency > end_freq-1) {
                          frequency = start_freq+1;
                          }
                           _spi_write(0x75, 0x40+((frequency / 10)-24) );                  
                           _spi_write(0x76, 0x19*(frequency % 10));  
                          delay(1);

                          
			// Scan 100 x 10 khz = 1 mhz. 
                        for (unsigned int i =0; i < 100; i+=step_size ) {  
                           _spi_write(0x79, i);
                           delay(1);                            
                           
                           if (_spi_read(0x0C)==0) {RF22B_init_parameter(); to_rx_mode(); }// detect the locked module and reboot		
                           
                           totalreading=0;
                           rssi_max = 0;
                           rssi_min = 255;
                           
			  // Used to average readings and find max if using frequency-hopping transmitter.
                             for (unsigned int j =0; j < number_average; j++) {

                               Rx_RSSI = _spi_read(0x26); // Read the RSSI value 	 
                                totalreading += Rx_RSSI;
                              
                              if (rssi_max < Rx_RSSI) {
                               rssi_max = Rx_RSSI; 
                              }
                              
                              if (rssi_min > Rx_RSSI) {
                               rssi_min = Rx_RSSI; 
                              }
                              
//                               rx_reset();
//                               delayMicroseconds(250);

                               
                               // Always check for incoming serial-data:
                               if (Serial.available())  {     
                                 serial_buf = Serial.read();
                                 
                                 if (serial_buf == 35) {
                                     read_serial =1;
                                 }
                                 
                                 else if (read_serial == 1) {
                                 
                                     if (serial_buf == 44) {
                                     buf_index++; 
                                     }
                                     
                                     else if (serial_buf > 47 && serial_buf < 58){
                                      temp_buf[buf_index] = temp_buf[buf_index]*10 + (serial_buf-48);
                                     }
                                     
                                     else if(serial_buf == 10) {
                                       j=300001;
                                       i=30001;
                                       
                                       read_serial =0;
                                       start_freq = temp_buf[0]-1;
                                       end_freq =   temp_buf[1];
                                       number_average = temp_buf[2];
                                       step_size = temp_buf[3];
                                       
                                       frequency = start_freq;
                                       
                                       temp_buf[0] =0;
                                       temp_buf[1] =0;
                                       temp_buf[2] =0;
                                       temp_buf[3] =0;
                                       buf_index =0;
                                       
                                       delay(100);

                                     }
                                   }
                                     
                                     
                               }                               
                               
                               
                               
                               
                               
                               
                             }


                          if (i < 30001) {
                             Serial.print(frequency*100+i*1);
                             Serial.print(",");                             

                             totalreading = totalreading / number_average;
                              
                             Serial.print(rssi_max);        
                             Serial.print(",");    
                             
                             Serial.print(totalreading);                                                 
                             Serial.print(",");                             

                             Serial.print(rssi_min);    
                             Serial.println(",");                                    
                          }



                            
                            
                          }
                          
                          }
                
						
#endif
                        
// ========================================================================================================
// Spectrum-analyzer part END
// ========================================================================================================
  
  
  
  
            
			time = millis();

                        #if defined(GPS)
                        if (Serial.available()>0)    // Serial command received from the GPS
                          if (GPS_data_status==1) 
                             GPS_data_parser();                          
                             else
                             GPS_read();//GPS reading code 
                        #endif
                        
                        if (_spi_read(0x0C)==0) {RF22B_init_parameter(); to_rx_mode(); }// detect the locked module and reboot			 
			
			//Detect the broken RF link and switch it to failsafe mode after 1 seconds  
			if ((time-last_pack_time > 1000) && (failsafe_mode == 0))
                                {
				failsafe_mode = 1; // Activate failsafe mode
                                load_failsafe_values(); // Load Failsafe positions from EEPROM
                                Direct_Servo_Drive(); // Set directly the channels form Servo buffer
                                Red_LED_ON;
                                #if defined(Analog_RSSI) 
                                analogWrite(RSSI_OUT,0); 
				#endif
                                }
				
			if ((time-last_hopping_time > 25))//automatic hopping for clear channel when rf link down for 35ms.	
                              {
                               Red_LED_ON;
                               last_hopping_time = time;  
                              
                               #if defined(Analog_RSSI) 
                                analogWrite(RSSI_OUT,0); 
				#endif
                               
                               #if (FREQUENCY_HOPPING==1)
                                 Hopping(); //Hop to the next frequency
                               #endif   
                              }  
                              
                        
			if(RF_Mode == Received)   // RFM22B INT pin Enabled by received Data
				 { 
				 failsafe_mode = 0; // deactivate failsafe mode
                                 last_pack_time = time; // record last package time
				 				 
                                 Red_LED_OFF;
                                 Green_LED_ON;
                                 
                                 
                                 
                                 
				 send_read_address(0x7f); // Send the package read command
				 
				 for(i = 0; i<17; i++) //read all buffer 
						{ 
						 RF_Rx_Buffer[i] = read_8bit_data(); 
						}  
				 rx_reset();
				 
				 if (RF_Rx_Buffer[0] == 'S') // servo control data
						{
                                                 for(i = 0; i<8; i++) //Write into the Servo Buffer
                                                        {                                                          
                                                         temp_int = (256*RF_Rx_Buffer[1+(2*i)]) + RF_Rx_Buffer[2+(2*i)];
                                                         if ((temp_int>1500) && (temp_int<4500)) Servo_Buffer[i] = temp_int; 
                                                                                                                  
                                                        }

                                                  //Serial.println( Servo_Buffer[1]);
						 
						 if (first_data==3) // save failsafe
						        {
							 save_failsafe_values(); 
							 first_data = 1;
						        }
                                                 if (first_data==2) first_data = 3; // wait extra 1 frame for save failsafe
						 
						 if (first_data==0) // when you receive the first package, check the stick postitions 
						        {
							first_data = 1;
							if (Servo_Buffer[0]>3200 && Servo_Buffer[1]>3200) // Right stick on right lower corner
							    {
								 Red_LED_Blink(25); //wait for 5 seconds for failsafe position of sticks.
								 first_data = 2; //go to failsafe saving mode. 
                                                                 rx_reset(); // erase all received data during this 5 seconds period
								}							
							
							}
						}
						 
				 if (RF_Rx_Buffer[0] == 'T') // RS232 Tx data received
						 {
						 tx_data_length = RF_Rx_Buffer[1]; // length of RS232 data
						 for(i = 0; i<tx_data_length; i++)
						    RS232_Tx_Buffer[i+1] = RF_Rx_Buffer[i+2]; // fill the RS232 buffer						 
						 }
						 
				 #if (TELEMETRY_MODE == 0)  // Transparent Bridge Telemetry mode                
                                    if (RF_Rx_Buffer[0]=='B') // Brige values
                                         {
                                           for(i = 2; i<RF_Rx_Buffer[1]+2; i++) //write serial
                                            Serial.print(RF_Rx_Buffer[i]);
                                         }                  
                                 #endif	 
						 
				 Rx_RSSI = _spi_read(0x26); // Read the RSSI value
				 				 
                                 #if defined(Analog_RSSI) 
                                   //***************************************
                                   //*   thUndead's RSSI MOD
                                   //*   info: RSSI voltage according to avg rssi value 
                                    rssicounter++;  //counter which resets after getting to 41
    
                                    if (rssicounter <=40) rssipwm= rssipwm+ Rx_RSSI ; //adds values into temp buffer
                                    else rssicounter =0;
                                  
                                    if (rssicounter ==40) 
                                    {
                                    rssipwm = rssipwm / 40; // averege 40 rssi values to get stable reading
                                    rssibuf = map(rssipwm,40,120,10,250);  //map value for pwm: MAX = 2.6v bad rssi unver 1 v
                                    analogWrite(RSSI_OUT,rssibuf);    //write the RSSI voltage
                                    }
                                    //***************************************
                                 #endif
				 
                                 #if defined(Serial_RSSI)
                                    Serial.print("RSSI:");
                                    Serial.println(Rx_RSSI);
                                 #endif
				 /*
                                 // Low RSSI indicator
                                  if (rssi<40) //low RSSI warning
						{
						Red_LED_ON;
						}

                                 */   
                                 
                                  #if defined(MMA7455)
                                        MMA7455_Read();
                                  #endif
                                  
                                  #if defined(WiiMotionPlus)
                                        wmp_receiveData();                                        
                                  #endif
                                  
                                  #if defined(HMC5883L)                                  
                                        HMC5883L_Read();
                                        Serial.print("X: ");
                                        Serial.print(MagX);
                                        Serial.print(" Y: ");
                                        Serial.print(MagY);
                                        Serial.print(" Z: ");
                                        Serial.println(MagZ);
                                  #endif
                                  
                                  #if defined(BMP085)
                                        APM_BMP085.Read();                                     
                                  #endif
                                  
                                  
                                  #if defined(WiiMotionPlus)
                                     Gyro_Stabilized_Servo_Drive(); // mix stick commands and Gyro values for stabilized flight
                                     //Basic_Quadro_Copter_Servo_Drive(); //######### EXPERIMENTAL CODE PART, DONT USE IT ######## 
                                  #else
                                     Direct_Servo_Drive(); // use stick commands directly for standard rc plane flights
                                  #endif
                                
                                 
                                #if (FREQUENCY_HOPPING==1)
                                 Hopping(); //Hop to the next frequency
                                #endif  
                                
                                delay(1);
                                
                                
                                
                                #if (TELEMETRY_ENABLED==1)
                                     #if (TELEMETRY_MODE==0) 
                                         Telemetry_Bridge_Write(); // Write the serial buffer
                                     #endif  
                                     
                                     #if (TELEMETRY_MODE==1) 
                                         Telemetry_Write(); // Write the telemetry vals
                                     #endif   
                                     
                                #endif
                                
                                RF_Mode = Receive;
                                
                                last_hopping_time = time;    
                                
                                Green_LED_OFF;
                                
			        }


		/* //######### EXPERIMENTAL CODE PART, DONT USE IT ######## 
                // Telemetry data transmitted.
                if (RF_Mode == Transmitted)
                      {
                        to_ready_mode(); 
                       
                        to_rx_mode(); 
                                  
                                Hopping();
                                
                                RF_Mode = Receive;
                                
                                last_hopping_time = time;    
                        
                      }		 
                 */     
		
	}
		 

}


