// /*
//  SineWavePoints
 
//  Write sine wave points to the serial port, followed by the Carriage Return and LineFeed terminator.
//  */

// int i = 0;

// // The setup routine runs once when you press reset:
// void setup() {
//   // Initialize serial communication at 9600 bits per second:
//   Serial.begin(9600);
// }

// // The loop routine runs over and over again forever:
// void loop() {
//   // Write the sinewave points, followed by the terminator "Carriage Return" and "Linefeed".
//   Serial.print(sin(i*50.0/360.0));
//   // Serial.write(10);// CR
//   // Serial.write(13);// NL
//   Serial.println();
//   i += 1;
// }

#include <LinkedList.h>
LinkedList<int> myList = LinkedList<int>();
// define send data
struct datastruct{
  char UTC_hr;
  char UTC_min;
  char UTC_sec;
  char UTC_subsec;
  char Pack_count;
  char IMUTemp;
  char quat[4];//quaternion
  char Att[3];//euler angle
  char AttRate[3];//euler angle rate
  char Acc[3];//acceleration
  char RealAcc[3];//real acceleration
  char Pos[3];//local position
  char LLH[3];//GPS LLH position
  char Vel[3];//GPS cal velocity
  int RSSI;
  float SNR;
};


int i = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  // Add some stuff to the list
  struct datastruct send_data;
  send_data.UTC_hr = 12, send_data.UTC_min = 34, send_data.UTC_sec = 56, send_data.UTC_subsec = 78,
  send_data.Pack_count = 1, send_data.IMUTemp = 12.3456, 
  send_data.quat[0] = 0.12345, send_data.quat[1] = 0.12345, send_data.quat[2] = 0.12345, send_data.quat[3] = 0.12345, 
  send_data.Att[0] = 123.4567, send_data.Att[1] = 123.4567, send_data.Att[2] = 123.4567, 
  send_data.AttRate[0] = 123.4567, send_data.AttRate[1] = 123.4567, send_data.AttRate[2] = 123.4567,
  send_data.Acc[0] = 123.4567, send_data.Acc[1] = 123.4567, send_data.Acc[2] = 123.4567, 
  send_data.RealAcc[0] = 123.4567, send_data.RealAcc[1] = 123.4567, send_data.RealAcc[2] = 123.4567,
  send_data.Pos[0] = 123.4567, send_data.Pos[1] = 123.4567, send_data.Pos[2] = 123.4567, 
  send_data.LLH[0] = 123.4567, send_data.LLH[1] = 123.4567,  send_data.LLH[2] = 123.4567,
  send_data.Vel[0] = 123.4567, send_data.Vel[1] = 123.4567, send_data.Vel[2] = 123.4567,
  send_data.RSSI = -123, send_data.SNR = 123.4567;

  myList.add(send_data.UTC_hr);
  myList.add(send_data.UTC_min);
  myList.add(send_data.UTC_sec);
  myList.add(send_data.UTC_subsec);
  myList.add(send_data.Pack_count);
  myList.add(send_data.IMUTemp);
  myList.add(send_data.quat[0]);
  myList.add(send_data.quat[1]);
  myList.add(send_data.quat[2]);
  myList.add(send_data.quat[3]);
  myList.add(send_data.Att[0]);
  myList.add(send_data.Att[1]);
  myList.add(send_data.Att[2]);
  myList.add(send_data.AttRate[0]);
  myList.add(send_data.AttRate[1]);
  myList.add(send_data.AttRate[2]);
  myList.add(send_data.Acc[0]);
  myList.add(send_data.Acc[1]);
  myList.add(send_data.Acc[2]);
  myList.add(send_data.RealAcc[0]);
  myList.add(send_data.RealAcc[1]);
  myList.add(send_data.RealAcc[2]);
  myList.add(send_data.Pos[0]);
  myList.add(send_data.Pos[1]);
  myList.add(send_data.Pos[2]);
  myList.add(send_data.LLH[0]);
  myList.add(send_data.LLH[1]);
  myList.add(send_data.LLH[2]);
  myList.add(send_data.Vel[0]);
  myList.add(send_data.Vel[1]);
  myList.add(send_data.Vel[2]);
  myList.add(send_data.RSSI);
  myList.add(send_data.SNR);
  // struct ListNode* newNode = (struct ListNode*)malloc(sizeof(struct ListNode));
  // packetSize = ??
  // struct ListNode head;
  // struct ListNode* current_ptr;
  // for(int i = 0;i<packetSize)
  Serial.flush();
}

void loop() {
  int listSize = myList.size();
  for (int h = 0; h < listSize; h++) {

    // // Get value from list
    int val = myList.get(h);
    Serial.print(val);
    Serial.print(',');
  }
  Serial.println();
}