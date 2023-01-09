/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 25/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a receiver program that can be used to test the throughput of a LoRa
  transmitter. The matching program '42_LoRa_Data_Throughput_Test_Transmitter' is setup to send packets 
  that require an acknowledgement before sending the next packet. This will slow down the effective
  throughput. Make sure the lora settings in the 'Settings.h' file match those used in the transmitter.
  
  Serial monitor baud rate is set at 115200.
*******************************************************************************************************/

#define Program_Version "V1.0"


#include "Config.h"                            //include the setiings file, frequencies, LoRa settings etc   

SX127XLT LT;                                     //create a library class instance called LT

ChaChaPoly chachapoly;

byte Key[KEY_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                              0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                              0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};

byte IV[IV_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

byte Tag[TAG_SIZE];


uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[255];                           //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
uint8_t TXPacketL;                               //stores length of packet sent
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet

uint8_t PacketType;                              //for packet addressing, identifies packet type
uint32_t packetCheck;

//Function signatures
void packet_is_OK();
void sendAck(uint32_t);
void packet_is_Error();
void printElapsedTime();
void led_Flash(uint16_t, uint16_t);

void processPacket(byte buffer[], byte tag[], byte ciphertext[], size_t ctSize);
void printHex(const char *label, const byte *data, size_t lenOfArray);



void setup()
{
  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start

  Serial.begin(115200);
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.println(F("43_LoRa_Data_Throughput_Acknowledge_Receiver Starting"));
  Serial.println();

  SPI.begin(SCK, MISO, MOSI, NSS);

  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                       //long fast speed LED flash indicates device error
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  Serial.println();
  LT.printModemSettings();                                     //reads and prints the configured LoRa settings, useful check
  Serial.println();
  Serial.println();

  Serial.print(F("Receiver ready"));
  Serial.println();
}


void loop()
{
  RXPacketL = LT.receive(RXBUFFER, 255, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  digitalWrite(LED1, HIGH);                      //something has happened

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  digitalWrite(LED1, LOW);                       //LED off

  Serial.println();
}


void packet_is_OK()
{
  RXpacketCount++;

//Decryption
byte tag[TAG_SIZE];
size_t ctSize = RXPacketL - TAG_SIZE;
byte ciphertext[ctSize];
byte plaintext[ctSize];

processPacket(RXBUFFER, tag, ciphertext, ctSize);
chachapoly.clear();
chachapoly.setKey(Key, KEY_SIZE);
chachapoly.setIV(IV, IV_SIZE);
chachapoly.decrypt(plaintext, ciphertext, ctSize);

bool isTagValid = chachapoly.checkTag(tag, TAG_SIZE);

if(isTagValid){
  Serial.print(plaintext[1]);
  Serial.print(F(" RX"));
  packetCheck = ( (uint32_t) plaintext[4] << 24) + ( (uint32_t) plaintext[3] << 16) + ( (uint32_t) plaintext[2] << 8 ) + (uint32_t) plaintext[1];
  Serial.print(F(",SendACK"));
  sendAck(packetCheck);

  // Serial.println("Tag validated!");
  // printHex("Entire Packet", RXBUFFER, RXPacketL);
  //  printHex("Tag", tag, TAG_SIZE);
  //  printHex("Ciphertext", ciphertext, ctSize);
  //  printHex("Plaintext", plaintext, ctSize);

} else {
  Serial.println("Tag is invalid!!!");
   printHex("Entire Packet", RXBUFFER, RXPacketL);
   printHex("Tag", tag, TAG_SIZE);
   printHex("Ciphertext", ciphertext, ctSize);
   printHex("Plaintext", plaintext, ctSize);
}


}

void processPacket(byte buffer[], byte tag[], byte ciphertext[], size_t ctSize) {
  for (int i = 0; i < ctSize; i++) {
    ciphertext[i] = buffer[i];
  }
  for (int i = ctSize; i < RXPacketL; i ++) {
    tag[i - ctSize] = buffer[i];
  }
}

void sendAck(uint32_t num)
{
  //acknowledge the packet received
  uint8_t len;

  LT.startWriteSXBuffer(0);                   //initialise buffer write at address 0
  LT.writeUint8(ACK);                         //identify type of packet
  LT.writeUint32(num);                        //send the packet check, bytes 1 to 5 of packet
  len = LT.endWriteSXBuffer();                //close buffer write

  digitalWrite(LED1, HIGH);
  TXPacketL = LT.transmitSXBuffer(0, len, 10000, TXpower, WAIT_TX);
  digitalWrite(LED1, LOW);
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();               //read the LoRa device IRQ status register

  if (IRQStatus & IRQ_RX_TIMEOUT)               //check for an RX timeout
  {
    Serial.print(F(",RXTimeout"));
  }
  else
  {
    errors++;
    PacketRSSI = LT.readPacketRSSI();            //read the recived RSSI value
    Serial.print(F("Error"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,Len,"));
    Serial.print(LT.readRXPacketL());            //get the device packet length
  }
}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

//Use this to print any array to the console.
void printHex(const char *label, const byte *data, size_t lenOfArray)
{
  Serial.println(label);
  Serial.print("00: ");
  for (int i = 0; i < lenOfArray; i++)
  {
    Serial.print(data[i], HEX);
    Serial.print(" ");
    if ((i % 16) == 15)
    {
      Serial.println();
      Serial.print(i + 1);
      Serial.print(": ");
    }
  }
  Serial.println();
  Serial.println();
}