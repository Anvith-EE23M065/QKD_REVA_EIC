//Developer : Anvith M
//Code for ESP32 to act as Master to EIC for SPI and to 
//read HDAC current outputs through ADCs

#include <SPI.h> //SPI library

#define HDAC0 15 //ADC pin
#define HDAC1 4 //ADC pin

#define SSN 5 //Chip select pin
#define SCLK 18// S_CLK
#define MISO 19//MISO
#define MOSI 23//MOSI
#define RESETN 16 // Reset pin, asynchronous active low RX2
#define CLOCK_PIN 17  // REFCLK to EIC will be given through this pin TX2

#define CLOCK_FREQ 1000000 //1MHz - may need to lower
#define SPI_FREQ 100000 //100kHz

// const int ADC_RES = 4095; //12bit ADC
// const float ADC_VOLT = 3.3; //0-3.3V
// const float SER_RES = 149.8; //Serial resistance through which HDAC current passes through

//   float h0, h1;

void setup() {
  //For REFCLK
  pinMode(CLOCK_PIN, OUTPUT); //Set GPIO as output
  ledcAttach(CLOCK_PIN, CLOCK_FREQ, 1); //Set clock frequency on pin 17

  //For SPI
  Serial.begin(115200); //baudrate for Serial Monitor
  SPI.begin(SCLK, MISO, MOSI, SSN); // 18, 19, 23, 5 Instatiate SPI with default pin values

  pinMode(SSN, OUTPUT); // Chip select pin as output
  pinMode(RESETN, OUTPUT);
  digitalWrite(SSN, HIGH); //Set it high(spi inactive)
  digitalWrite(RESETN, LOW); // Reset EIC
}

// Function to send Instruction
unsigned long Ix(unsigned long sendData){

  // Serial.println("SPI start");
  
  byte sendbuffer[4]; //Buffer to store the 4 Bytes to send
  byte receivedbuffer[4] = {0}; //Buffer to receive 4 Bytes

  //Split sendData into 4 Bytes
  sendbuffer[0] = (sendData >> 24) & 0xFF; //MSB
  sendbuffer[1] = (sendData >> 16) & 0xFF; 
  sendbuffer[2] = (sendData >> 8) & 0xFF; 
  sendbuffer[3] = sendData & 0xFF; //LSB


  // Serial.println(sendbuffer[0],HEX);
  // Serial.println(sendbuffer[1],HEX);
  // Serial.println(sendbuffer[2],HEX);
  // Serial.println(sendbuffer[3],HEX);

  //Start SPI transaction
  //Activate SPI
  
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0)); // Ready SPI

  for(int i = 0; i < 4; i++){
    receivedbuffer[i] = SPI.transfer(sendbuffer[i]); //SPI transfer  
  }

  SPI.endTransaction();
  
  Serial.println("Received Data");
  Serial.println(receivedbuffer[0],HEX);
  Serial.println(receivedbuffer[1],HEX);
  Serial.println(receivedbuffer[2],HEX);
  Serial.println(receivedbuffer[3],HEX);
  Serial.println("SPI end");

  return (receivedbuffer[0] << 24) | (receivedbuffer[1] << 16) | (receivedbuffer[2] << 8) | (receivedbuffer[3]); //Convert buffer to data

}

void loop() {
  digitalWrite(RESETN, HIGH); // Set reset off
 
  //Serial.println("Begin test");
  ledcWrite(CLOCK_PIN, 1); //clock 50% duty cycle

  uint32_t sendData = 0x40000000; //Write command at register 0x0
  uint32_t receivedData = 0; //Variable to store received data

  //Serial.println("Send first instruction");
  
  digitalWrite(SSN, LOW);   // Activate SPI
  

  receivedData = Ix(sendData); // Call function

  digitalWrite(SSN, HIGH); // Deactivate SPI

  //Serial.println(receivedData, HEX); // Print data
  // h0 = 1000*(ADC_VOLT * analogRead(HDAC0)/ADC_RES)/SER_RES;
  // h1 = 1000*(ADC_VOLT * analogRead(HDAC1)/ADC_RES)/SER_RES;

  // Serial.printf("HDAC0 Current : %f\nHDAC1 Current : %f\n",h0,h1);
  
  delay(1);

  sendData = 0x54540000; //Change register vvalue that controls HDAC 0 and 1 
  //Serial.println("Send second instruction");


  digitalWrite(SSN, LOW);   // Activate SPI

  receivedData = Ix(sendData);

  digitalWrite(SSN, HIGH); //Deactivate SPI

  //Serial.println(receivedData, HEX); //Print data
  // h0 = 1000*(ADC_VOLT * analogRead(HDAC0)/ADC_RES)/SER_RES;
  // h1 = 1000*(ADC_VOLT * analogRead(HDAC1)/ADC_RES)/SER_RES;

  // Serial.printf("HDAC0 Current : %f\nHDAC1 Current : %f\n",h0,h1);
  //Serial.printf("======================================================\n");

  delay(3);

}
