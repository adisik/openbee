// **********************************************************
// **                OpenLRS EEPROM Functions              **
// **        Developed by Melih Karakelle on 2010-2012     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2012-03-21
// Supported Hardware : OpenLRS Rx-Tx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/

#define eeprom_start_address 0 //first byte of eeprom


unsigned int read_eeprom_uint(int address)
{
 return (EEPROM.read(eeprom_start_address+address) * 256) + EEPROM.read(eeprom_start_address+address+1); 
}

unsigned char read_eeprom_uchar(int address)
{
 return  EEPROM.read(eeprom_start_address+address); 
}


void write_eeprom_uint(int address,unsigned int value)
{
 EEPROM.write(eeprom_start_address+address,value / 256);  
 EEPROM.write(eeprom_start_address+address+1,value % 256);  
}

void write_eeprom_uchar(int address,unsigned char value)
{
 return  EEPROM.write(eeprom_start_address+address,value); 
}

/*
void load_failsafe_values(){
 
  for (int i=0;i<8;i++)
      {
         Servo_Buffer[i] =  (EEPROM.read(eeprom_start_address+100+(2*i)) * 256) + EEPROM.read(eeprom_start_address+101+(2*i));
      }  
  
  #if (DEBUG_MODE == 4)
      Serial.print("1:");
      Serial.println(Servo_Buffer[0]); // value x 0.5uS = PPM time, 3000 x 0.5 = 1500uS 
      Serial.print("2:");
      Serial.println(Servo_Buffer[1]);
      Serial.print("3:");
      Serial.println(Servo_Buffer[2]);
      Serial.print("4:");
      Serial.println(Servo_Buffer[3]);
      Serial.print("5:");
      Serial.println(Servo_Buffer[4]);
      Serial.print("6:");
      Serial.println(Servo_Buffer[5]);
      Serial.print("7:");
      Serial.println(Servo_Buffer[6]);
      Serial.print("8:");
      Serial.println(Servo_Buffer[7]); 
   #endif
     

}


void save_failsafe_values(void){

 for (int i=0;i<8;i++)
    {
     EEPROM.write(eeprom_start_address+100+(2*i),Servo_Buffer[i] / 256); 
     EEPROM.write(eeprom_start_address+101+(2*i),Servo_Buffer[i] % 256);
    } 
}

*/

void read_eeprom(void)
{
   //Frequency
   CARRIER_FREQUENCY = 400000 + read_eeprom_uint(0);
   // hopping step size x10khz
   HOPPING_STEP_SIZE = read_eeprom_uchar(2); 
   // hopping channels
   hop_list[0] = read_eeprom_uchar(3);
   hop_list[1] = read_eeprom_uchar(4);
   hop_list[2] = read_eeprom_uchar(5);
   
   RF_Header[0] = read_eeprom_uchar(6);
   RF_Header[1] = read_eeprom_uchar(7);
   RF_Header[2] = read_eeprom_uchar(8);
   RF_Header[3] = read_eeprom_uchar(9);
} 


void write_eeprom(void)
{
   
  //Frequency
  write_eeprom_uint(0,CARRIER_FREQUENCY - 400000) ;  
   
} 


void eeprom_check(void)
{
 byte temp1,temp2,temp3; 
 Serial.begin(9600);
 Serial.print("W4E"); //Mean, "waiting for eeprom data"
 delay(100);
 if (Serial.available()>2) // computer sending "R4T" mean "READY FOR TRANSMISSION"
    {
     if ((Serial.read()=='R') && (Serial.read()=='4') && (Serial.read()=='T'))
        {
          //Start to configuration
          
          while (1)
          {
            if (Serial.available()>2)
               {
                temp1 = Serial.read();
                if (temp1 == 'W') 
                   {
                     Red_LED_ON
                     temp2 = Serial.read();
                     temp3 = Serial.read();
                     temp1 = 0;
                     while(1) //endless loop writes every data from computer to eeprom
                          {
                          if (Serial.available()>0)   
                            {
                            temp3 = Serial.read();
                            write_eeprom_uchar(temp1++,temp3); // W+address+data
                            
                            }
                          }
                   }  
                if (temp1 == 'R') 
                   {
                      Blue_LED_ON
                      temp2 = Serial.read();
                      temp3 = Serial.read();
                      for (int i=0;i<100;i++)
                          {
                          Serial.write(read_eeprom_uchar(i));
                          }
                      Serial.write('0'); //for 101th byte    
                      Serial.write('0'); 
                      Blue_LED_OFF
                   }   
               }
          }
        }
     delay(100);
    } 
 
 
 read_eeprom();
 
 Serial.print(CARRIER_FREQUENCY);
 
 Serial.end(); 
  
}  
