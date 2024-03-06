#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <LCD_I2C.h>
#include <Adafruit_PCF8574.h>
#include <WiFi.h>
#include <MQTT.h>
////////////////////////////////////////////////////////////////////////////
LCD_I2C lcd(0x27, 16, 2);
ESP32Encoder encoder;
WiFiClient net;
MQTTClient client;
hw_timer_t *My_timer = NULL;

#define BUTTON_PIN 26 
#define ENCODER_A_PIN 16
#define ENCODER_B_PIN 17

int motorSpeed;
int baseSpeed = 0;
int maxSpeed = 45;
int sum_error = 0;
int SPEED;
int maxmotorspeed = 20;
int distance_set = 5;
int dis_set = 0;

// PID
int error = 0;
int pre_error = 0;
float integral = 0;
float LastTime = 0;
float Dt = 0.032;

int Kp = 5;
int Kd = 1;
float Ki = 0.1;

int INA1 = 18;
int INA2 = 19;
int PWMA = 5;

const int trig = 14;      //ประกาศขา trig
const int echo = 12;      //ประกาศขา echo
long duration, distance;  //ประกาศตัวแปรเก็บค่าระยะ

int encode = 0;
static long lastPosition = 0;
const float limit_up = 60;
const float limit_down = 30;
int motor_limit;
float degree;
int state = 0;
int prep_state;
double turn;

int newPosition;
////////////////////////////////////////////////
const char ssid[] = "Dxt-iphone";
const char pass[] = "magicwireless";

const char mqtt_broker[]="test.mosquitto.org";
const char status_topic[]="final1.18/status";
const char getval_topic[]="final1.18/getval";
const char mqtt_topic[]="final1.18/message";
const char distance_topic[]="final1.18/distance";
const char mqtt_client_id[]="arduino_group_final1.18"; // must change this string to a unique value
int MQTT_PORT=1883;

void messageReceived(String &topic, String &payload) {
  // Serial.println("incoming: " + topic + " - " + payload);
  if(topic == "final1.18/getval"){
    if(payload == "5"){
      dis_set = 1;
    }else if(payload == "10"){
      dis_set = 2;
    }else if(payload == "15"){
      dis_set = 3;
    }else if(payload == "20"){
      dis_set = 4;
    }else if(payload == "25"){
      dis_set = 5;
    }else if(payload == "30"){
      dis_set = 6;
    }else if(payload == "35"){
      dis_set = 7;
    }
  }
  
}

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_client_id)) {  
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe(mqtt_topic);
  client.subscribe(distance_topic);
  client.subscribe(getval_topic);
  client.subscribe(status_topic);
}

void IRAM_ATTR Timer(){
	//Code want to be timer here//
  if((state == 1) & (distance_set == 5)){
    distance_set = 10;
    prep_state = 1;
    state = 0;
  }else if((state == 1) & (distance_set == 10) & (prep_state == 1)){
    distance_set = 15;
    state = 0;
    prep_state = 2;
  }else if((state == 1) & (distance_set == 15) & (prep_state == 2)){
    distance_set = 20;
    state = 0;
    prep_state = 3;
  }else if((state == 1) & (distance_set == 20) & (prep_state == 3)){
    distance_set = 25;
    state = 0;
    prep_state = 4;
  }else if((state == 1) & (distance_set == 25) & (prep_state == 4)){
    distance_set = 30;
    state = 0;
    prep_state = 5;
  }else if((state == 1) & (distance_set == 30) & (prep_state == 5)){
    distance_set = 35;
    state = 0;
    prep_state = 6;
  }else if((state == 1) & (distance_set == 35) & (prep_state == 6)){
    distance_set = 5;
    state = 0;
    prep_state = 0;
  }
}


void motorup(int speedM) {
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  analogWrite(PWMA, speedM);
}

void motordown(int speedM) {
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  analogWrite(PWMA, speedM);
}

void pid(int setpoint) {

  if ((distance >= (setpoint-1)) && (distance <= (setpoint-2))) {
    error = -2;
  } else if (distance >= (setpoint-3) && distance <= (setpoint-4)) {
    error = -2;
  } else if (distance >= (setpoint-5) && distance <= (setpoint-6)) {
    error = -3;
  } else if (distance <= (setpoint-7)) {
    error = -4;
  }
  if (distance >= setpoint && distance <= (setpoint+1)) {
    error = 0;
  } else if (distance >= (setpoint+2) && distance <= (setpoint+3)) {
    error = 50;
  } else if (distance >= (setpoint+4) && distance <= (setpoint+5)) {
    error = 52;
  } else if (distance >= (setpoint+6) && distance <= (setpoint+7)) {
    error = 54;
  } else if (distance >= (setpoint+8) && distance <= (setpoint+9)) {
    error = 56;
  } else if (distance > (setpoint+10)) {
    error = 58;
  }

  float CurrentTime = millis() / 1000.00;
  Dt = CurrentTime - LastTime;
  LastTime = CurrentTime;
  integral += error * Dt;
  integral = constrain(integral, -10, 10);
  //  if (error == 0) integral = 0;
  double Derivative = (error - pre_error) / Dt;

  motorSpeed = Kp * error + Kd * (error - pre_error) + Ki * (integral);
  SPEED = baseSpeed + motorSpeed;

  if (SPEED > maxSpeed) SPEED = maxSpeed;
  if (SPEED < -maxSpeed) SPEED = -maxSpeed;
  if (motorSpeed > maxmotorspeed) motorSpeed = maxmotorspeed;
  if (motorSpeed < -maxmotorspeed) motorSpeed = -maxmotorspeed;
  if (error == 0) {
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, HIGH);
  analogWrite(PWMA, 0);
  }

  if (SPEED > 0) {
    motorup(SPEED);
  } else if (SPEED < 0) {
    motordown(abs(SPEED));
  }

  pre_error = error;
  sum_error += error;

  // Serial.print("       ");
  // Serial.print(error);
  // Serial.print("       ");
  // Serial.print(SPEED);
  // Serial.print("       ");
  // Serial.println(motorSpeed);
}

void sensor() {
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);           //ใช้งานขา trig
  duration = pulseIn(echo, HIGH);    //อ่านค่าของ echo
  distance = (duration / 2) / 29.1;  //คำนวณเป็น centimeters
  Serial.print(distance);
  Serial.println(" cm");
  //delay(10);
}

void LCD_Show(){
  // lcd.setCursor(1,0);
  // lcd.print("Distance: ");
  // lcd.print(distance);
  lcd.setCursor(1,0);
  lcd.print("Setpoint: ");
  if(distance_set < 10){
    lcd.print(" ");
  }
  lcd.print(distance_set);
  lcd.setCursor(1,1);
  if(!client.connected()){
    lcd.print("MQTT Disconnect!");
  }else{
    lcd.print("MQTT Connect!");
  }
  delay(2);
  lcd.clear();
}

void IRAM_ATTR button_state(){
  state = 1;
}

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, pass);
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  My_timer = timerBegin(0, 80, true);
	timerAttachInterrupt(My_timer, &Timer, true);
	timerAlarmWrite(My_timer, 250000, true);
	timerAlarmEnable(My_timer);
  pinMode(echo, INPUT);   //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(trig, OUTPUT);  //สั่งให้ขา trig ใช้งานเป็น output
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  // pinMode(25,INPUT);
  attachInterrupt(BUTTON_PIN, button_state, RISING);
  encoder.attachFullQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  connect();
}

void loop() {
  
  client.loop();
  LCD_Show();
  sensor();
  pid(distance_set);

  newPosition = encoder.getCount();

  if (newPosition != lastPosition) {
    turn = newPosition / 2400.0;
    if (int(turn) >= 1) {
      encode = (newPosition - (2400 * int(turn)));
    } else if (turn < 0) {
      encode = (newPosition + (2400 * (int(0.0 - turn) + 1)));
    } else {
      encode = newPosition;
    }
    lastPosition = newPosition;
  }
  degree = map(encode, 0, 2400, 1, 3600) / 10;

  
///////////////////////////////////////////////////////////////////
  if(dis_set == 1){
    distance_set = 5;
  }else if(dis_set == 2){
    distance_set = 10;
  }else if(dis_set == 3){
    distance_set = 15;
  }else if(dis_set == 4){
    distance_set = 20;
  }else if(dis_set == 5){
    distance_set = 25;
  }else if(dis_set == 6){
    distance_set = 30;
  }else if(dis_set == 7){
    distance_set = 35;
  }
  
  client.publish(mqtt_topic, String(distance_set));
  client.publish(distance_topic, String(distance));
  if(error == 0){
    client.publish(status_topic, "STEADY!");
  }else{
    client.publish(status_topic, "MOVING...");
  }

  if(!client.connected()){
    distance_set = 5;
    prep_state = 0;
    state = 0;
    dis_set = 0;
  }
  // Serial.print("State: ");
  // Serial.print(state);
  // Serial.print("      ");
  // Serial.print("Disance set");
  // Serial.println(distance_set);
  //LIMIT  ////////////////
}

