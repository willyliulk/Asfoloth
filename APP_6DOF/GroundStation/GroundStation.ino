#include <SPI.h>
#include <LoRa.h>

#define SPI_HAS_NOTUSINGINTERRUPT

int looptime = 0;
int packetSize = 12;  // test max pack size = 35
// int ss = 10;
// int reset = 9;
// int dio0 = 2;

char incomingByte[10];//serial read message

char read;
char inputstr[2];
int pin_RFswitch = 6;
long lastSendTime = 0;  // last send time
int interval = 1000;    // interval between sends


void setup() {
  //Initialize serial port
  Serial.begin(9600);
  while (!Serial);

  //Initialize LoRa
  //Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) {
    // Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    //Serial.println("Starting LoRa");
  }

  //Set LoRa RF parameters
  LoRa.setSPIFrequency(1000000);
  LoRa.setTxPower(25, PA_OUTPUT_RFO_PIN);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(10);
  // LoRa.disableInvertIQ();
  // LoRa.enableCrc();
  // LoRa.setGain(6);

  //FEM_CPS, RF switch LF on, set HIGH to switch to receive channel
  // digitalWrite(pin_RFswitch, HIGH);  


  //Register the receive callback
  // LoRa.onReceive(onReceive);
  // put the radio into receive mode
  // LoRa.receive(packetSize);
}

void loop() {
  //Pass through all register when debug
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
  digitalWrite(pin_RFswitch, HIGH);  // switch to receive channel
  // delay(1);
  // parse for a packet, and call onReceive with the result:
  // LoRa.receive();
  onReceive(LoRa.parsePacket());

  //Transmit LoRa if serial port have data
  if (Serial.available() > 0){
    LoRa.beginPacket();
    digitalWrite(pin_RFswitch, LOW);  // switch to transmit channel
    String tempstr;
    tempstr = Serial.readStringUntil('\n');
    LoRa.print(tempstr);
    LoRa.endPacket();
  }
}

void onReceive(int packetSize) {
  if (packetSize == 0) {
    // Serial.println("Not received");
    return;  // if there's no packet, return
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