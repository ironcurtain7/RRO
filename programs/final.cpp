#include <Arduino.h>
#include <Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <OPT3101.h>
#include <Wire.h>
#include "BasicOTA.hpp"
int flagcube = 1;
int cx = 0, ch = 0, iwhile = 0, iexit = 0, els = 0, I;
long readsens;
int error, color, delta = 0, flag5 = 0;
int dist = 500, errorold = 0, i = 0, speedwhile = 0;
int LigR, sum, count, timold = 0, countline = 0;
#define SSID      "DIR-615-effb"
#define PASSWORD  "18050523"
#define RXD2 16
#define TXD2 17
float Kp = 0.065, Ki = 0.01, Kd = 0.05;
int white = 3420;
int blue = 3270;
int timing = 52;
int speed = 70;
int tim1 = 0, tim2 = 0, tim3 = 0, ang1 = 25, ang2 = 25, ang3 = 50;
  int speed1 = speed;
String Kp_text = String(Kp, 5);
String Ki_text = String(Ki, 5);
String Kd_text = String(Kd, 5);
String white_text = String(white);
String blue_text = String(blue);
String speed_text = String(speed);
String timing_text = String(timing);

String tim1_text = String(tim1);
String tim2_text = String(tim2);
String tim3_text = String(tim3);

String ang1_text = String(ang1);
String ang2_text = String(ang2);
String ang3_text = String(ang3);
uint16_t amplitudes[3];
int16_t distances[3];
AsyncWebServer server(80);
Servo servo1;
OPT3101 sensor;
BasicOTA OTA;
TaskHandle_t Task1;

int readUZ(int LR){


if (LR ==  0){ // LEFT


if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();

    amplitudes[sensor.channelUsed] = sensor.amplitude;
    distances[sensor.channelUsed] = sensor.distanceMillimeters;
    sensor.nextChannel();
    sensor.startSample();
    dist = distances[0];
  }
}
if (LR == 1){ // right

if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();

    amplitudes[sensor.channelUsed] = sensor.amplitude;
    distances[sensor.channelUsed] = sensor.distanceMillimeters;
    sensor.nextChannel();
    sensor.startSample();
    dist = distances[2];
  }

}
if (LR ==  2){ // CENTER


if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();

    amplitudes[sensor.channelUsed] = sensor.amplitude;
    distances[sensor.channelUsed] = sensor.distanceMillimeters;
    sensor.nextChannel();
    sensor.startSample();
    dist = distances[1];
  }
}

return dist;


}

void speek(int variation){
if (variation == 0){
digitalWrite(14,HIGH);
delay(50);
digitalWrite(14,LOW);
}
if (variation == 1){
digitalWrite(14,HIGH);
delay(500);
digitalWrite(14,LOW);
}
if (variation == -1){
digitalWrite(14,HIGH);
delay(10);
digitalWrite(14,LOW);
}
}

void Task1code( void * pvParameters ){

int timold1 = 0, LigR1 = 0, sum1, i1;
while (1){

LigR1=analogRead(32);
sum1 = 0;
if (LigR1<white){
  for (i1 = 0; i1 < 10; i1++) {
    LigR1=analogRead(32);
    sum1 = sum1 + LigR1;
  }

if (sum1/i1<white){
  if (millis() - timold1 > 3000){
    speek(1);
    timold1 = millis();

    countline = countline + 1;
}
    
}

}




} 
}

void arc(int speed, int dir, int angle){
int count1 = 0;

if (dir == 0){
digitalWrite(25, LOW);
analogWrite(26,speed);
}
if (dir == -1){
digitalWrite(25, HIGH);
analogWrite(26, 255-speed);
}

while (count1 < angle){
if (digitalRead(23) == 1){
while (digitalRead(23) == 1){}
count1 = count1 + 1;
}
}


}

void ride(){
cx = 0, ch = 0, iwhile = 0, iexit = 0, els = 0, I = 0;
digitalWrite(25, LOW);
analogWrite(26,speed);


  if (Serial2.available() > 0){
    String rd = Serial2.readStringUntil('\n');
    if (rd[0] == 'r'){
      if (flagcube == 1){
      while(iexit == 0){
        delay(3);
        if (Serial2.available() > 0){

          String rd = Serial2.readStringUntil('\n');
          String rc = rd.substring(1,4);
          String rh = rd.substring(4,6); 
          cx = rc.toInt();
          ch = rh.toInt();
          
          }
          
          

      int err = 20 - cx;
      int u1 = err * 0.5;
      //Serial.println(u1);
      servo1.write(80 - u1);      

      if (ch > 65) iexit = 1;

      }
      servo1.write(107);
      arc(70,0,20);
      servo1.write(67);
      arc(70,0,15);
      while (Serial2.available()) Serial2.read();

      }

    }// (rd[0] == 'r')

    else if (rd[0] == 'g'){
      if (flagcube == 1){

      while(iexit == 0){
        delay(3);
        if (Serial2.available() > 0){

          String rd = Serial2.readStringUntil('\n');
          String rc = rd.substring(1,4);
          String rh = rd.substring(4,6); 
          cx = rc.toInt();
          ch = rh.toInt();
          
          }
          
          

      int err = 100 - cx;
      int u1 = err * 0.5;
      //Serial.println(u1);
      servo1.write(87 - u1);      

      if (ch > 55) iexit = 1;

      }
      servo1.write(55);
      arc(70,0,20);
      servo1.write(115);
      arc(70,0,18);
      while (Serial2.available()) Serial2.read();


      }
    }// (rd[0] == 'g')
  }
    else {
      analogWrite(26,speed);


      readsens = readUZ(1-color);
      Serial.println(readsens);
      error = (readsens - 1200) * (color * 2 - 1) - cx * 5;



      I = I + error;
      int u = error * Kp + Kd * (error - errorold) + Ki * I;
      errorold = error;


      if (u>ang3){
      u = ang3;
      }

      if (u<ang3*-1){
        u = ang3*-1;
      }
      if (flag5 == 0){
        servo1.write(87-u);
      }

      }// (rd[0] == 'n')

          
      





}

void turn(){




LigR=analogRead(32);



sum=0;
if (LigR<white){
  for (i = 0; i < 10; i++) {
    LigR=analogRead(32);
    sum = sum + LigR;
  }

if (sum/i<white){
  if (millis() - timold > 3000){
    digitalWrite(25, LOW);
analogWrite(26,0);
delay(1000);
digitalWrite(25, LOW);
analogWrite(26,speed);
    //speek(0);
    if (color == 0){
    servo1.write(87);
    while (readUZ(2) > 250){}
    servo1.write(120);
    arc(80,-1,timing);
    servo1.write(87);
    arc(80,-1,20);
    }
    else{
    servo1.write(87);
    while (readUZ(2) > 250){}
    servo1.write(60);
    arc(80,-1,timing);
    servo1.write(87);
    arc(80,-1,20);
    }
    timold = millis();
digitalWrite(25, LOW);
analogWrite(26,0);
delay(1000);
digitalWrite(25, LOW);
analogWrite(26,speed);
    count = count + 1;
}
    
}

}

while (Serial2.available()) Serial2.read();


if (count == 111){
  int mil = millis();
while(millis() - mil < 750){
  ride();
}
digitalWrite(25, LOW);
analogWrite(26,0);
speek(1);
speek(1);
delay(10000000);
}
servo1.write(87);







}

void start(){

cx = 0, ch = 0, iwhile = 0, iexit = 0, els = 0;
int porog, flag = 0, Min = 9999;

while (Serial2.available()) Serial2.read();

digitalWrite(25, LOW);
analogWrite(26,speed);




while (flag == 0){

servo1.write(86);

if (Serial2.available() > 0){
    String rd = Serial2.readStringUntil('\n');
    if (rd[0] == 'r'){
      
      while(iexit == 0){
        delay(3);
        if (Serial2.available() > 0){

          String rd = Serial2.readStringUntil('\n');
          String rc = rd.substring(1,4);
          String rh = rd.substring(4,6); 
          cx = rc.toInt();
          ch = rh.toInt();
          
          }
          
          

      int err = 20 - cx;
      int u1 = err * 0.5;
      //Serial.println(u1);
      servo1.write(80 - u1);      

      if (ch > 65) iexit = 1;

      }
      servo1.write(107);
      arc(70,0,20);
      servo1.write(67);
      arc(70,0,15);
      while (Serial2.available()) Serial2.read();



    }// (rd[0] == 'r')

    if (rd[0] == 'g'){


      while(iexit == 0){
        delay(3);
        if (Serial2.available() > 0){

          String rd = Serial2.readStringUntil('\n');
          String rc = rd.substring(1,4);
          String rh = rd.substring(4,6); 
          cx = rc.toInt();
          ch = rh.toInt();
          
          }
          
          

      int err = 100 - cx;
      int u1 = err * 0.5;
      //Serial.println(u1);
      servo1.write(87 - u1);      

      if (ch > 60) iexit = 1;

      }
      servo1.write(62);
      arc(70,0,20);
      servo1.write(112);
      arc(70,0,15);
      while (Serial2.available()) Serial2.read();



    }// (rd[0] == 'g')
}

LigR=analogRead(32);
//Serial.print(LigR);
//Serial.print("                        ");
sum=0;

if (LigR<white){
  for (i = 0; i < 10; i++) {
    LigR=analogRead(32);
    sum = sum + LigR;
  }

porog = sum/i;
Serial.println(porog);
if (porog<white){
flag = 1;
while (porog<white){


LigR=analogRead(32);
//Serial.print(LigR);
//Serial.print("                        ");
sum=0;

  for (i = 0; i < 10; i++) {
    LigR=analogRead(32);
    sum = sum + LigR;
  }

porog = sum/i;


  if (porog<Min){
    Min = porog;
  } 
}   
}
}
}





Serial.println(Min);

if (Min > blue){
  color = 1;
Serial.print("ORANGE");
servo1.write(87);
    arc(70,0,70);
    servo1.write(60);
    arc(70,-1,timing);


}
else{
  color = 0;
Serial.print("BLUE");
servo1.write(87);
    arc(70,0,70);
    servo1.write(120);
    arc(70,-1,timing);



}

timold = millis();
speek(0);
while (Serial2.available()) Serial2.read();
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void IRAM_ATTR push_button() {

if (speed>0)
speed = 0;
else
speed = speed1;

delay(350);
}

void setup() {
pinMode(25, OUTPUT); // configure the trigger_pin(D9) as an Output
pinMode(26, OUTPUT); // Set the LED (D13) pin as a digital output
pinMode(14, OUTPUT); // configure the trigger_pin(D9) as an Output
pinMode(27, INPUT_PULLUP);
Serial.begin(115200); // Enable the serial with 9600 baud rate
Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

Wire.begin();

servo1.attach(33);
servo1.write(87);




  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(64);
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::High);

  sensor.startSample();







//WiFi.mode(WIFI_AP);
//WiFi.softAP("ESP32AP","123321456");




  Serial.println("Startup");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  OTA.begin(); // Setup settings

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

String mess = "<!DOCTYPE HTML><html><head><title>ESP Input \
    Form</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body><form action='/get'> \
    kP: <input type='text' name='Kp' value='"+Kp_text+"'><br> \
    kI: <input type='text' name='Ki' value='"+Ki_text+"'> \
    <br>kD: <input type='text' name='Kd' value='"+Kd_text+"'>\
    <br>speed: <input type='text' name='speed' value='"+speed_text+"'>\
    <br>white: <input type='text' name='white' value='"+white_text+"'>\
    <br>blue: <input type='blue' name='blue' value='"+blue_text+"'>\
    <br>timing: <input type='timing' name='timing' value='"+timing_text+"'>\
    <br>tim1: <input type='tim1' name='tim1' value='"+tim1_text+"'>\
    <br>ang1: <input type='ang1' name='ang1' value='"+ang1_text+"'>\
    <br>tim2: <input type='tim2' name='tim2' value='"+tim2_text+"'>\
    <br>ang2: <input type='ang2' name='ang2' value='"+ang2_text+"'>\
    <br>tim3: <input type='tim3' name='tim3' value='"+tim3_text+"'>\
    <br>ang3: <input type='ang3' name='ang3' value='"+ang3_text+"'>\
    <input type='submit' value='Submit'>\
  </form>\
</body></html>";
    
    request->send(200, "text/html", mess);

});

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("Kp")) {
      Kp_text = request->getParam("Kp")->value();
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    if (request->hasParam("Ki")) {
      Ki_text = request->getParam("Ki")->value();
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    if (request->hasParam("Kd")) {
      Kd_text = request->getParam("Kd")->value();
    }

    if (request->hasParam("speed")) {
      speed_text = request->getParam("speed")->value();
    }
    
    if (request->hasParam("white")) {
      white_text = request->getParam("white")->value();
    }

    if (request->hasParam("blue")) {
      blue_text = request->getParam("blue")->value();
    }

    if (request->hasParam("timing")) {
      timing_text = request->getParam("timing")->value();
    }
    if (request->hasParam("tim1")) {
      tim1_text = request->getParam("tim1")->value();
    }
    if (request->hasParam("tim2")) {
      tim2_text = request->getParam("tim2")->value();
    }
    if (request->hasParam("tim3")) {
      tim3_text = request->getParam("tim3")->value();
    }
    if (request->hasParam("ang1")) {
      ang1_text = request->getParam("ang1")->value();
    }
    if (request->hasParam("ang2")) {
      ang2_text = request->getParam("ang2")->value();
    }
    if (request->hasParam("ang3")) {
      ang3_text = request->getParam("ang3")->value();
    }



    Kp = Kp_text.toFloat();
    Ki = Ki_text.toFloat();
    Kd = Kd_text.toFloat();
    speed = speed_text.toInt();
    white = white_text.toInt();
    blue = blue_text.toInt();
    timing = timing_text.toInt();
    tim1 = tim1_text.toInt();
    tim3 = tim2_text.toInt();
    tim1 = tim3_text.toInt();
    ang1 = ang1_text.toInt();
    ang2 = ang2_text.toInt();
    ang3 = ang3_text.toInt();

    Serial.println(String(Kp,5));
    Serial.println(String(Ki,5));
    Serial.println(String(Kd,5));


int light = 0;
 sum = 0;
for (i = 0; i < 10; i++) {
    LigR=analogRead(32);
    sum = sum + LigR;
  }
light = sum/i;

int rightread = -1;
int leftread = -1;
Serial.println(rightread);
Serial.println(leftread);
String light_text = String(light);
String right_dist = String(rightread);
String left_dist = String(leftread);

String count_text=String(count);

String mess = "<!DOCTYPE HTML><html><head><title>ESP Input \
    Form</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body> \
    Kp="+Kp_text+"  Ki="+Ki_text+"  Kd="+Kd_text+"  Light="+light_text+"  Right_dist="+right_dist+"  Left_dist="+left_dist+"  Count="+count_text+" \
    <form action='/get'> \
    kP: <input type='text' name='Kp' value='"+Kp_text+"'><br> \
    kI: <input type='text' name='Ki' value='"+Ki_text+"'> \
    <br>kD: <input type='text' name='Kd' value='"+Kd_text+"'>\
    <br>speed: <input type='text' name='speed' value='"+speed_text+"'>\
    <br>white: <input type='text' name='white' value='"+white_text+"'>\
    <br>blue: <input type='blue' name='blue' value='"+blue_text+"'>\
    <br>timing: <input type='timing' name='timing' value='"+timing_text+"'>\
    <br>tim1: <input type='tim1' name='tim1' value='"+tim1_text+"'>\
    <br>ang1: <input type='ang1' name='ang1' value='"+ang1_text+"'>\
    <br>tim2: <input type='tim2' name='tim2' value='"+tim2_text+"'>\
    <br>ang2: <input type='ang2' name='ang2' value='"+ang2_text+"'>\
    <br>tim3: <input type='tim3' name='tim3' value='"+tim3_text+"'>\
    <br>ang3: <input type='ang3' name='ang3' value='"+ang3_text+"'>\
    <input type='submit' value='Submit'>\
  </form>\
</body></html>";

    request->send(200, "text/html", mess);
  });
  server.onNotFound(notFound);
  server.begin();

speek(1);
while (digitalRead(27) == 0){
OTA.handle();
}
Serial.println("START ROBOT");



start();

xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 1); // надо 
attachInterrupt(27, push_button, RISING);
}

void loop() {

ride();
turn();
}