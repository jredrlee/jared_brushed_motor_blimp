#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>

#define SERVO1 D2
#define SERVO2 D3
#define THRUST1 D0
#define THRUST2 D1



const char * ssid = "AIRLab-BigLab";
const char * password = "Airlabrocks2022";
Servo servo1;
Servo servo2;
Servo thrust1;
Servo thrust2;

AsyncUDP udp;
float joy_data[4] = {0.0, 0.0, 0.0, 0.0};
volatile bool joy_ready = false;
volatile unsigned long time_now;

//Enter arming sequence for ESC
void escarm(Servo& thrust1, Servo& thrust2){
  // ESC arming sequence for BLHeli S
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);

  // Sweep up
  for(int i=1100; i<1500; i++) {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Sweep down
  for(int i=1500; i<1100; i--) {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Back to minimum value
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);
} 

void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 4;
  int num_bytes = 4;
  int i, j;

  for (i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for (j = 0; j < num_bytes; j++) {
      temp[j] = buffer[4 * i + j];
    }
    dat[i] = *((float*) temp);
    // if(i == 1 || i == 3){
    //   dat[i] = -*((float*) temp);
    // } else {
    //   dat[i] = *((float*) temp);
    // }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(THRUST1, OUTPUT);
  pinMode(THRUST2, OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);// Standard 50hz servo
  servo2.setPeriodHertz(50);// Standard 50hz servo
  thrust1.setPeriodHertz(51);// Standard 50hz servo
  thrust2.setPeriodHertz(51);// Standard 50hz servo
  servo1.attach(SERVO1, 900, 2100);
  servo2.attach(SERVO2, 900, 2100);
  thrust1.attach(THRUST1, 1000, 2000);
  thrust2.attach(THRUST2, 1000, 2000);

  escarm(thrust1, thrust2);
  
  delay(500);

  servo1.write((int) 30);
  servo2.write((int) 30);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(3000);
      servo1.write((int) 180);
      servo2.write((int) 0);
      delay(3000);
      servo1.write((int) 0);
      servo2.write((int) 180);
    }
  }
  servo1.write((int) 50);
  servo2.write((int) 50);
  delay(500);

  servo1.write((int) 90);
  servo2.write((int) 90);

  // set the motor out pins as outputs
  //time_loop = millis();


  if (udp.listen(1333)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());

    // setup callback functions of the udp
    udp.onPacket([](AsyncUDPPacket packet) {
      joy_ready = false;
      time_now = millis();
      unsigned char *buffer = packet.data();
      unpack_joystick(joy_data, buffer);
      joy_ready = true;
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
  servo1.write((int) 110);
  servo2.write((int) 110);
  delay(500);

  servo1.write((int) 150);
  servo2.write((int) 150);
  delay(500);
}

void loop() {
  if (joy_ready && millis() - time_now <= 1000) {
    //time_loop = millis();
    servo1.write((int) joy_data[2]); //joy_data[2]
    servo2.write((int) joy_data[3]); //joy_data[3]
//    thrust1.write((int) floor(180.0*joy_data[0]/255));
//    thrust1.write((int) floor(180.0*joy_data[1]/255));

    thrust1.writeMicroseconds(1000 + joy_data[0]);
    delay(5);
    thrust2.writeMicroseconds(1000 + joy_data[0]);
    delay(5);

//    analogWrite(THRUST1, (int) floor(joy_data[0]));
//    analogWrite(THRUST2, (int) floor(joy_data[1]));
    //if (millis()/1000%1<.1){
    Serial.print((int) joy_data[2]);
    Serial.print(' ');
    Serial.print((int) joy_data[3]);
    Serial.print(' ');
    Serial.print((int) joy_data[0]);
    Serial.print(' ');
    Serial.print((int) joy_data[1]);
    Serial.println(' ');
    //}
  }
}
