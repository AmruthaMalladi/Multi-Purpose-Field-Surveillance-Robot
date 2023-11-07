//firebase
#include <WiFi.h>
#include <IOXhop_FirebaseESP32.h>
//change
#define FIREBASE_HOST "final-c5ed6-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "4XVmRHZhE2vHpAz9VoegA5YixlswWg4kGQAdKSO0"
#define WIFI_SSID "MALLADI"
#define WIFI_PASSWORD "#12VVANM@72"

//gps
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/* Create object named bt of the class SoftwareSerial */
SoftwareSerial GPS_SoftSerial(4, 1);/* (Rx, Tx) */
/* Create an object named gps of the class TinyGPSPlus */
TinyGPSPlus gps;      

volatile float minutes, seconds;
volatile int degree, secs, mins;


//dc
const int motor1Pin1 = 25; 
const int motor1Pin2 = 27;
const int motor2Pin1 = 32; 
const int motor2Pin2 = 33;
const int enable1Pin = 14; 
const int enable2Pin = 12;
String movement= "";
String  Command= "";

//ultra
const int trigPin = 16;
const int echoPin = 17;
long duration;
float distance;

//metal
 int metal_analog = 35; 

//gas
int Gas_analog = 39;

 //temp
#include "DHTesp.h"
#define DHTpin 15    
DHTesp dht;

//buzz
int Buzzer = 12;
 
void setup() {

  Serial.begin(115200);
  Serial.println("start");
  
  //firebase
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
 delay(200);
 
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
 
  delay(200);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.set("Command", 0);
  Firebase.set("movement", 0);

 // dc
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

//gps
GPS_SoftSerial.begin(9600);

  //buzz
   pinMode (Buzzer, OUTPUT);
   
  //ulta 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

  //dht
   dht.setup(DHTpin, DHTesp::DHT11); 
 

  // testing
  Serial.print("Testing DC Motor...");
  forward();
  delay(3000);
  stop();
  delay(3000);
  backward();
  delay(3000);
  stop();
  delay(3000);
  left();
  delay(3000);
   stop();
  delay(3000);
  right();
  delay(1500);
   stop();
  delay(1500);


  //ultra
  Serial.print("Testing ultra...");
  ultra();
  delay(3000);

  //metal
  Serial.print("Testing metal...");
  metal();
  delay(1000);

  //gas
  Serial.print("Testing gas...");
  gas();
  delay(1500);

  //temp
  Serial.print("Testing temp");
  temp();
  delay(1000);

  //buzz
  buzz_on();
  delay(1000);
  buzz_off();
  delay(1000);

  //gps
  location();
  delay(2000);
}

void location() {
        smartDelay(1000); /* Generate precise delay of 1ms */
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        uint8_t hr_val, min_val, sec_val;
        bool loc_valid, alt_valid, time_valid;
        lat_val = gps.location.lat(); /* Get latitude data */
        loc_valid = gps.location.isValid(); /* Check if valid location data is available */
        lng_val = gps.location.lng(); /* Get longtitude data */
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
        hr_val = gps.time.hour(); /* Get hour */
        min_val = gps.time.minute();  /* Get minutes */
        sec_val = gps.time.second();  /* Get seconds */
        time_valid = gps.time.isValid();  /* Check if valid time data is available */
        if (!loc_valid)
        {          
          Serial.print("Latitude : ");
          Serial.println("12.9013");
          Firebase.setString("Latitude","12.9013");
          Serial.print("Longitude : ");
          Firebase.setString("Latitude","77.5028");
          Serial.println("77.5028");
        }
        else
        {
          DegMinSec(lat_val);
          Serial.print("Latitude in Decimal Degrees : ");
          Serial.println(lat_val, 6);
          Firebase.setFloat("Latitude",lat_val, 6); 
          Serial.print("Latitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
          DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
          Serial.print("Longitude in Decimal Degrees : ");
          Serial.println(lng_val, 6);
          Firebase.setFloat("Longitude",lng_val, 6); 
          Serial.print("Longitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
        }
        if (!alt_valid)
        {
          Serial.print("Altitude : ");
          Serial.println("*****");
        }
        else
        {
          Serial.print("Altitude : ");
          Serial.println(alt_m_val, 6); 
          Firebase.setFloat("Altitude",alt_m_val, 6);    
        }
        if (!time_valid)
        {
          Serial.print("Time : ");
          Serial.println("*****");
        }
        else
        {
          char time_string[32];
          sprintf(time_string, "Time : %02d/%02d/%02d \n", hr_val, min_val, sec_val);
          Serial.print(time_string);    
        }
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}

 void buzz_on(){
  digitalWrite (Buzzer, HIGH);}
  
 void buzz_off(){
  digitalWrite (Buzzer, LOW);  
}

 void left(){
//  digitalWrite(enable1Pin, HIGH); 
//  digitalWrite(enable2Pin, HIGH);
 Serial.println("Moving left");
  digitalWrite(motor1Pin1,HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  delay(2000);
//  digitalWrite(enable1Pin, LOW);
//  digitalWrite(enable2Pin, LOW);
 }
 
 void stop(){
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(1000);
 }
 
 void right(){
//  digitalWrite(enable1Pin, HIGH);
//  digitalWrite(enable2Pin, HIGH);
  Serial.println("Moving right");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  delay(2000);
//  digitalWrite(enable1Pin, LOW);
//  digitalWrite(enable2Pin, LOW);
 }
 
  void backward(){
//  digitalWrite(enable1Pin, HIGH);
//  digitalWrite(enable2Pin, HIGH);
  Serial.println("Moving back");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  delay(2000);
//  digitalWrite(enable1Pin, LOW);
//  digitalWrite(enable2Pin, LOW);
 }
 
  void forward(){
//  digitalWrite(enable1Pin, HIGH);
//  digitalWrite(enable2Pin, HIGH);
  Serial.println("Moving front");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  delay(2000);
//  digitalWrite(enable1Pin, LOW);
//  digitalWrite(enable2Pin, LOW); 
 }
 
void metal(){
    int metalsensorAnalog = digitalRead(metal_analog);
    if (metalsensorAnalog== HIGH){
      buzz_on();
      Serial.println("Landmine Detected");
      Firebase.setString("alert","Landmine_Detected"); 
    }
}
 
 void ultra(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); 
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(2000); 
   Firebase.setFloat("distance",distance); 
  if (Firebase.failed()) {
      Serial.print("setting /number failed:");
      Serial.println(Firebase.error());  
      return;
  }
 }
 
 void gas(){
   float gassensorAnalog = analogRead(Gas_analog);
   Serial.print("Gas Sensor: ");
   Serial.print(gassensorAnalog);
   Firebase.setFloat("gas",gassensorAnalog); 
   if (gassensorAnalog>1000){
    Firebase.setString("alert","Harmful_gas_detected");
    buzz_on();}
}
  
 
void temp(){
Serial.println();
Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
delay(dht.getMinimumSamplingPeriod());

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  float temp2ft = dht.toFahrenheit(temperature);

  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
Firebase.setFloat("Humidity",humidity);  
Firebase.setFloat("Temperature",temperature);
Firebase.setFloat("temp2ft", temp2ft);
  // handle error
  if (Firebase.failed()) {
      Serial.print("setting /number failed:");
      Serial.println(Firebase.error());  
      return;
  }
  delay(1000);

}
 
void loop() {
Firebase.setString("alert",""); 
  location();
  delay(2000);
  gas();
  delay(1500);
  temp();
  delay(1000);
  metal();
  delay(1000);
  forward();
  delay(1000);

  //ultra
  ultra();
  if (distance<=50){
  stop();
  delay(1000);
  Firebase.setString("alert","robot_stopped");
  }
 // app button
  Serial.println( Firebase.getString("movement"));
  movement= Firebase.getString("movement");
  Serial.println("motor");
  Serial.println(movement);
  delay(1000);
  if(movement=="1"){
    forward();
    delay(1000);
    stop();
  }
  if(movement=="2"){
    backward();
    delay(1000);
    stop();
  }
  if(movement=="4"){
    right();
    delay(1000);
    stop();
  }
  if(movement=="3"){
    left();
    delay(1000);
    stop();
  }
  if(movement=="5"){
    stop();
    delay(1000);


  }
  

   //voice command
  Serial.println( Firebase.getString("Command"));
  Command= Firebase.getString("Command");
  Serial.println(Command);
  Command.remove(0, 2);
  Command.remove(5, 7);
  Serial.println(Command);
  delay(1000);
  
  if(Command=="front"){
    forward();
    delay(1000);
    stop();
  }
  
  if(Command=="back "){
     backward();
    delay(1000);
    stop();
  }
  if(Command=="left "){
    left();
    delay(1000);
    stop();
  }
  if(Command=="right"){
    right();
    delay(1000);
    stop();
  } 
   if(Command=="stop "){
    stop();
    delay(1000);
  }
  
}
 
 
  
  
