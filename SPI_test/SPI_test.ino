//Developer : Anvith M
//EIC test code, for SPI and register value check

#include <SPI.h> //SPI Library

#define SSN 5 // Chip Select Pin
#define SCLK 18 // S_CLK
#define MISO 19 // MISO
#define MOSI 23 // MOSI
#define RESETN 16 // Reset Pin, asynchronous active low, RX2
#define REF_CLK 17 // Reference clock to EIC, TX2 

#define CLOCK_FREQ 5000000 // 5MHz - tunable
#define SPI_FREQ 1000000 // 1MHz - SPI clock frequency, tunable

// Default register values at EIC reset from SPI datasheet
// Function to send Instruction
unsigned long SPI_transfer(unsigned long sendData){
  byte sendbuffer[4]; // Buffer to store isntruction in 4 bytes(32bits)
  byte receivedbuffer[4] = {0}; // Buffer to store received bytes

  //Split instruction into 4bytes
  sendbuffer[0] = (sendData >> 24) & 0xFF; // MSB(31-24)
  sendbuffer[1] = (sendData >> 16) & 0xFF; // (23-16)
  sendbuffer[2] = (sendData >> 8) & 0xFF; // (15-8)
  sendbuffer[3] = sendData & 0xFF; // LSB(7-0)

  //Start SPI transaction
  digitalWrite(SSN, LOW); // pull chipselect low
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0)); // Ready SPI

  for(int i=0; i<4; i++){
    receivedbuffer[i] = SPI.transfer(sendbuffer[i]); // SPI transfer  
  }

  SPI.endTransaction();
  digitalWrite(SSN, HIGH); // pull chipselct high
  delay(10);

  return (receivedbuffer[0] << 24) | (receivedbuffer[1] << 16) | (receivedbuffer[2] << 8) | (receivedbuffer[3]); // Convert buffers to 32bit data
}

// Function to resest the EIC
void resetEIC(){
  digitalWrite(RESETN, LOW);
  delay(10);
  digitalWrite(RESETN, HIGH);
}

// default register values
const uint32_t default_val[14] = {
  0x00000000, // reg0
  0x00000000, // reg4
  0x00008000, // reg8
  0x41410028, // reg12
  0x41410028, // reg16
  0x00000000, // reg20
  0x000000C0, // reg24
  0xFA000000, // reg28
  0x00000000, // reg32
  0xE562E562, // reg36
  0x00000000, // reg40
  0x00000000, // reg44
  0x00000000, // reg48
  0x00000000  // reg52
};

void setup() {
  
  Serial.begin(115200); // For serial Monitor

  // Attach esp32 clock to pin, and set frequency
  pinMode(REF_CLK, OUTPUT); 
  ledcAttach(REF_CLK, CLOCK_FREQ, 1); 

  pinMode(SSN, OUTPUT); // Chip select pin as output
  pinMode(RESETN, OUTPUT); // Set reset as output pin
  // For SPI, assign pin values
  SPI.begin(SCLK, MISO, MOSI, SSN); 

  // test default registers
  resetEIC(); // Reset the EIC
  int i = 0;
  uint32_t recv_data[15] = {0};
  uint32_t send_data = 0x7FFFFFFC; // RD_DATA_CMD for reg0 is 0x80000000, subtracting 4 out of it for -1th iteration
  for(i; i<=14; i++){
    // Send and recieve data
    if(i!=14){
      send_data += 0x04; // keep incrementing by 4 to iterate between registers
      Serial.printf("Iteration %d, data sent : ", i);
      Serial.println(send_data, HEX);
      recv_data[i] = SPI_transfer(send_data);
    }
    else{
      send_data = 0x00000000;
      Serial.printf("Iteration %d, data sent : ", i);
      Serial.println(send_data, HEX);
      recv_data[i] = SPI_transfer(send_data);
    }

    // Data comparison
    if(i>0){
      Serial.print("Recieved Data : ");
      Serial.println(recv_data[i], BIN);
      Serial.print("Expected Data : ");
      Serial.println(default_val[i-1], BIN);
      if(recv_data[i] == default_val[i-1]){
        Serial.println("TRUE");
      }
      else
        Serial.println("FALSE");
    }
    else
      Serial.println("No data at 0th iteration");
  }

}

void loop() {

  ledcWrite(REF_CLK, 1); // clock 50% duty cycle

}
