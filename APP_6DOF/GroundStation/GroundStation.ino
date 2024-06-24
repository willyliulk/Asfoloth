#include <SPI.h>
#include <LoRa.h>

#define SPI_HAS_NOTUSINGINTERRUPT

//Arduino Pinset
// int ss = 10;        //SPI NSS
// int reset = 9;
// int dio0 = 2;
int pin_RFswitch = 6;  //switch to receive/transmit channel

char incomingByte[10];//serial read message

char read;
char inputstr[2];
long lastSendTime = 0;  // last send time
int interval = 1000;    // interval between sends


int looptime = 0;
// int packetSize = 12;  // test max pack size = 35

//LoRa parameters
int Car_freq = 433E6; //Carrier wave frequency
int SPI_freq = 1E6;   //SPI frequency
int Power = 25;       //TxPower
int SF = 7;           //SpreadingFactor
int BW = 125E3;       //Signal bandwidth
int CR = 5;           //CodingRate
int Preamble = 10;    //Preamble length

//Enable receive and transmit function for convience
bool receiveOn=0;
bool transmitOn=1;

//-------------------------------------------------------
void setup() {
  //Initialize serial port
  Serial.begin(9600);
  while (!Serial);

  //Initialize LoRa
  if (!LoRa.begin(Car_freq)) {
    // Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    // Serial.println("Starting LoRa");
  }

  //Set LoRa RF parameters
  LoRa.setSPIFrequency(SPI_freq);
  LoRa.setTxPower(Power, PA_OUTPUT_RFO_PIN);
  // LoRa.disableInvertIQ();
  // LoRa.enableCrc();
  // LoRa.setGain(6);

  //FEM_CPS, RF switch LF on, set HIGH to switch to receive channel
  // digitalWrite(pin_RFswitch, HIGH);  


  //Register the receive callback, if LoRa read the data and set RxDone to 1, Arduino interrupt and run onReceive
  // LoRa.onReceive(onReceive);
  // put the radio into receive mode
  // LoRa.receive(packetSize);
}

void loop() {
  //Pass through all LoRa register when debug
  // LoRa.dumpRegisters(Serial);
  // if (millis() - lastSendTime > interval) {
  //   digitalWrite(pin_RFswitch, LOW);  // FEM_CPS, RF switch LF on
  //   String outputStr = "GO";
  //   LoRa.beginPacket();  // start packet
  //   for (int i = 0; i < 2; i++) {
  //     LoRa.print(outputStr[i]);
  //     delay(1);
  //   }
  //   LoRa.endPacket();  // finish packet and send it
  //   Serial.println(outputStr);
  //   lastSendTime = millis();  // timestamp the message
  // }

  //Receiving data
  if (receiveOn){
    digitalWrite(pin_RFswitch, HIGH);  // switch to receive channel
    // delay(1);
    // parse for a packet, and call onReceive with the result:
    // LoRa.receive();
    onReceive(LoRa.parsePacket());
  }


  //Transmit LoRa if serial port have data
  if (transmitOn){
    // Serial.println("In transmit");
    if (int n = Serial.available()) {
      LoRa.beginPacket();               //header mode, reset FIFO
      digitalWrite(pin_RFswitch, LOW);  // switch to transmit channel
      // int readfromMatlab[50];
      // int pack_index = 0;
      // unsigned long entertime = micros();
      // int timeout = 1000;
      // while(Serial.read()!=99&&((micros()-entertime)<timeout)){
      //   readfromMatlab[pack_index] = Serial.read();
      //   pack_index++;
      // }
      // for (int i = 0;i<pack_index;i++){
      //   LoRa.print(readfromMatlab[i]);
      //   Serial.println(readfromMatlab[i],DEC);
      // }

      // String readfromMatlab;
      // readfromMatlab = Serial.readString();
      // Serial.println(readfromMatlab);
      char readfromMatlab[20];
      // for (int i = 0;i<n;i++){
      //   // readfromMatlab[i] = Serial.read();
      //   // LoRa.print(send_lora[i]);
        
      //   // Serial.print(Serial.read());
      //   Serial.println(Serial.read());
      // }
      // Serial.print(Serial.read());
      LoRa.print(Serial.read());
      // Serial.print(",");
      // Serial.print(Serial.readStringUntil(','));
      LoRa.endPacket();
    }
  }
}

void onReceive(int packetSize) {
  if (packetSize == 0) {
    // Serial.println("Not received");
    return;//if there's no packet, return to main()
  }
  // received a packet
  int error = 0;
  // for (int i = 0; i < packetSize; i++) {
  //   read = LoRa.read();
  //   if (i != read){
  //     error = error+1;
  //   }
  //   Serial.print(read);
  //   Serial.print(",");
  // }
  while (LoRa.available()) {
    Serial.print(LoRa.read());
    Serial.print(",");
  }

  int RSSI = LoRa.packetRssi();
  float SNR = LoRa.packetSnr();
  Serial.print(RSSI);
  Serial.print(",");
  Serial.print(SNR);
  // Serial.print(",");
  // Serial.print(error);
  Serial.println();
}