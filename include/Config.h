#include <SPI.h>                                 //the lora device is SPI based so load the SPI library
#include <SX127XLT.h>                            //include the appropriate library   
#include <ProgramLT_Definitions.h>
#include <ChaChaPoly.h>

/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 25/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini, 
//be sure to change the definitions to match your own setup. Some pins such as DIO1,
//DIO2, BUZZER may not be in used by this sketch so they do not need to be
//connected and should be set to -1.

//LoRaHAN Definitions

#define KEY_SIZE 32
#define IV_SIZE 8
#define TAG_SIZE 16
#define ADDRESS_SIZE 5
#define COMMAND_SIZE 5

//Setup Device to work correctly with the LoRa SX1276 Library
#define NSS 15                                   //select pin on LoRa device
#define SCK 14                                  //SCK on SPI3
#define MISO 12                                 //MISO on SPI3 
#define MOSI 13                                 //MOSI on SPI3 

#define NRESET 2                               //reset pin on LoRa device
#define LED1 -1                                  //on board LED, high for on
#define DIO0 4                                 //DIO0 pin on LoRa device, used for RX and TX done 
#define DIO1 -1                                 //DIO1 pin on LoRa device, normally not used so set to -1 
#define DIO2 -1                                 //DIO2 pin on LoRa device, normally not used so set to -1
#define VCCPOWER -1                             //pin controls power to external devices

#define LORA_DEVICE DEVICE_SX1276               //we need to define the device we are using


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
#define Frequency 915000000                     //frequency of transmissions
#define Offset 0                                 //offset frequency for calibration purposes  
#define Bandwidth LORA_BW_125                    //LoRa bandwidth
#define SpreadingFactor LORA_SF7                 //LoRa spreading factor
#define CodeRate LORA_CR_4_5                     //LoRa coding rate
#define Optimisation LDRO_AUTO                   //low data rate optimisation setting, normally set to auto
#define TXpower  10                              //power for transmissions in dBm

 
//Function signatures
void packet_is_OK();
void sendAck(uint32_t);
void packet_is_Error();
void printElapsedTime();
void led_Flash(uint16_t, uint16_t);