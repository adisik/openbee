// **********************************************************
// **                   OpenLRS Functions                  **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2012-02-14
// Supported Hardware : OpenLRS Tx boards (M1 & M2) (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

void RFM22B_Int()
{
 if (RF_Mode == Transmit) 
    {
     RF_Mode = Receive; 
    } 
 if (RF_Mode == Receive) 
    {
     RF_Mode = Received; 
    }  
}

//############# RED LED BLINK #################
void Red_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Red_LED_ON;
     delay(100);
     Red_LED_OFF;
     }
  }
  
//############# GREEN LED BLINK #################
void Blue_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Blue_LED_ON;
     delay(100);
     Blue_LED_OFF;
     }
  }  

//############# FREQUENCY HOPPING #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void){

hopping_channel++;
if (hopping_channel>2) hopping_channel = 0;

_spi_write(0x79, hop_list[hopping_channel]);

    #if (DEBUG_MODE == 5)
      Serial.println(int(hop_list[hopping_channel]));
    #endif  

}
#endif

//############# RF POWER SETUP #################
void Power_Set(unsigned short level)
{
  //Power Level value between 0-7
  //0 = +1 dBm
  //1 = +2 dBm
  //2 = +5 dBm
  //3 = +8 dBm
  //4 = +11 dBm
  //5 = +14 dBm
  //6 = +17 dBm
  //7 = +20 dB 
  if (level<8) _spi_write(0x6d, level);  
  
}

void ClearBuffers(void)
{
 RF_Tx_Buffer[0] = 0;
 RF_Tx_Buffer[1] = 0;
 RF_Tx_Buffer[2] = 0;
 RF_Tx_Buffer[3] = 0;
 RF_Tx_Buffer[4] = 0;
 RF_Tx_Buffer[5] = 0;
 RF_Tx_Buffer[6] = 0;
 RF_Tx_Buffer[7] = 0;
 RF_Tx_Buffer[8] = 0;
 RF_Tx_Buffer[9] = 0;
 RF_Tx_Buffer[10] = 0;
 RF_Tx_Buffer[11] = 0;
 RF_Tx_Buffer[12] = 0;
 RF_Tx_Buffer[13] = 0;
 RF_Tx_Buffer[14] = 0;
 RF_Tx_Buffer[15] = 0;
 RF_Tx_Buffer[16] = 0;
 
 RF_Rx_Buffer[0] = 0;
 RF_Rx_Buffer[1] = 0;
 RF_Rx_Buffer[2] = 0;
 RF_Rx_Buffer[3] = 0;
 RF_Rx_Buffer[4] = 0;
 RF_Rx_Buffer[5] = 0;
 RF_Rx_Buffer[6] = 0;
 RF_Rx_Buffer[7] = 0;
 RF_Rx_Buffer[8] = 0;
 RF_Rx_Buffer[9] = 0;
 RF_Rx_Buffer[10] = 0;
 RF_Rx_Buffer[11] = 0;
 RF_Rx_Buffer[12] = 0;
 RF_Rx_Buffer[13] = 0;
 RF_Rx_Buffer[14] = 0;
 RF_Rx_Buffer[15] = 0;
 RF_Rx_Buffer[16] = 0;
}

//######## TELEMETRY TRANSPARENT BRIDGE #########
void Telemetry_Bridge_Write(void)
{

RF_Tx_Buffer[0]= 'B'; // Brige command

byte total_rx_byte = Serial.available();  // Read the Serial RX buffer size
if (total_rx_byte>RF_PACK_SIZE-2) total_rx_byte = RF_PACK_SIZE-2;  // Limit the package size as 15 byte

if (total_rx_byte > 0) 
    {
    RF_Tx_Buffer[1]= total_rx_byte;
    for (byte i=0;i<total_rx_byte;i++)
      RF_Tx_Buffer[2+i] = Serial.read();
    }

RF_Mode = Transmit;
to_tx_mode();
rx_reset();     

// Clear buffer (dont use "for loop")
 RF_Tx_Buffer[1] = 0;
 RF_Tx_Buffer[2] = 0;
 RF_Tx_Buffer[3] = 0;
 RF_Tx_Buffer[4] = 0;
 RF_Tx_Buffer[5] = 0;
 RF_Tx_Buffer[6] = 0;
 RF_Tx_Buffer[7] = 0;
 RF_Tx_Buffer[8] = 0;
 RF_Tx_Buffer[9] = 0;
 RF_Tx_Buffer[10] = 0;
 RF_Tx_Buffer[11] = 0;
 RF_Tx_Buffer[12] = 0;
 RF_Tx_Buffer[13] = 0;
 RF_Tx_Buffer[14] = 0;
 RF_Tx_Buffer[15] = 0;
 RF_Tx_Buffer[16] = 0;

}  


