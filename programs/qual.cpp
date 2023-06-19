#include <Arduino.h>
#include <Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <OPT3101.h>
#include <Wire.h>
#include "BasicOTA.hpp"

long readsens;
int error, color;
int dist = 500, errorold = 0, i = 0, speedwhile = 0;
int LigR, sum, count, timold = 0;
#define SSID      "DIR-615-effb"
#define PASSWORD  "18050523"
float Kp = 0.028, Ki = 0, Kd = 0.9;
int white = 3500;
int blue = 3270;
int timing = 75;
int speed = 200;
int tim1 = 0, tim2 = 0, tim3 = 0, ang1 = 15, ang2 = 15, ang3 = 15;
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
}

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
else{ // right

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

return dist;


}

void arc(int speed, int dir, int angle){
int count1 = 0;


digitalWrite(25, LOW);
analogWrite(26,speed);



while (count1 < angle){
if (digitalRead(23) == 1){
while (digitalRead(23) == 1){}
count1 = count1 + 1;
}
}


}

void ride(){

analogWrite(26,speed);


readsens = readUZ(1-color);

error = (readsens - 800) * (color * 2 - 1);



int u = error * Kp + Kd * (error - errorold);
errorold = error;


if (u>ang3){
  u = ang3;
}

if (u<ang3*-1){
  u = ang3*-1;
}

servo1.write(87-u);


}

void turn(){




LigR=analogRead(32);



sum=0;
if (LigR<3500){
  for (i = 0; i < 10; i++) {
    LigR=analogRead(32);
    sum = sum + LigR;
  }

if (sum/i<3500){
  if (millis() - timold > 500){
    speek(0);
    if (color == 0){
    
    servo1.write(62);
    arc(speed-40,0,timing);
    }
    else{
    servo1.write(107);
    arc(speed-40,0,timing);

    }
    timold = millis();

    count = count + 1;
}
    
}

}




if (count == 11){
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






}

void start(){


int sum, porog, flag = 0, Min = 9999;

int ang1 = 15,ang2 = 15, white = 3500, blue = 3200;

delay(400);

digitalWrite(25, LOW);
analogWrite(26,speed-40);




while (flag == 0){

servo1.write(86);



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

servo1.write(87 + ang1);
arc(speed,0,90);
servo1.write(87);
arc(speed,0,10);



}
else{
  color = 0;
Serial.print("BLUE");

servo1.write(87 - ang2);
arc(speed,0,90);
servo1.write(87);
arc(speed,0,10);



}


speek(0);

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

  sensor.setFrameTiming(32);
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

attachInterrupt(27, push_button, RISING);
}

void loop() {
ride();
turn();
}