//
//  Angara.Net dht22-bmp085-gprs sensor
//
//  DHT-22, BMP085, Arduino Uno R3, TinySine SIM900
//  http://github.com/maxp/rs-gsm
//

// Libraries:
//  http://playground.arduino.cc/Main/WireLibraryDetailedReference
//  http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
//  http://code.google.com/p/gsm-shield-arduino/
//  http://playground.arduino.cc/Main/DHTLib
//  https://github.com/adafruit/Adafruit-BMP085-Library
//  https://github.com/Cathedrow/Cryptosuite
//
//  http://www.tinyosshop.com/index.php?route=product%2Fproduct&product_id=464


char VERSION[] = "rs_gsm v0.2";

#define HOST      "rs.angara.net"
#define PORT       80
#define BASE_URI  "/dat?"
// #define SECRET    "$$$"

#define INTERVAL    720    // sec
// #define INTERVAL    20  // !!! remove

#define APN   ""
#define USER  ""
#define PASS  ""

// GSM.h:
//   GSM_TX    2
//   GSM_RX    3
// GSM.cpp:
//   GSM_RESET 9 (!!! TinySine sim900)
//   GSM_ON    8 (!!! TinySine sim900)

#define DHT_PIN 6

// #define ONEWIRE    10

#define LED_GREEN  11
#define LED_RED    12

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here


// cable:
// A4 - blue   (bmp sda)
// A5 - green  (bmp scl)
// D6 - grey   (dht data)
// V3 - red/white
// V5 - red
// GD - blue/green/grey-white

#include <Wire.h>
#include <dht.h>
#include <Adafruit_BMP085.h>

// #include <OneWire.h>
#include <SoftwareSerial.h>
#include "SIM900.h"
#include "inetGSM.h"
// #include "sha1.h"

int cycle;

// OneWire ds(ONEWIRE);
InetGSM inet;

#define IMEI_LEN 20

char tchar[10];
char imei[IMEI_LEN+1];

char rbuff[50];
char ubuff[120];


void setup() {
  pinMode(GSM_ON, OUTPUT);
  pinMode(GSM_RESET, OUTPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);  
  delay(1000);
  
  Serial.begin(9600);
  Serial.println(VERSION);
  cycle = 0;
  
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);  
}

void sim_power() {
  pinMode(GSM_ON, OUTPUT);
  digitalWrite(GSM_ON, LOW);
  delay(1000);
  digitalWrite(GSM_ON, HIGH);
  delay(1300);
  digitalWrite(GSM_ON, LOW);
  delay(2500);
}

void sim_reset() {
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, LOW);
  delay(100);
  digitalWrite(GSM_RESET, HIGH);
  delay(200);
  digitalWrite(GSM_RESET, LOW);
  delay(100);
}

// void read_t() {
//  byte data[12];
//  
//  digitalWrite(LED_GREEN, HIGH);
//  ds.reset(); ds.skip(); ds.write(0x44, 1); // measure
//  delay(999); // delay > 750
//  ds.reset(); ds.skip(); ds.write(0xBE); // read scratchpad
//
//  Serial.print("\nscratch:");  
//  for(int i=0; i < 9; i++) {
//    data[i] = ds.read(); Serial.print(" "); Serial.print(data[i], HEX);
//  }
//  Serial.println();
//  
//  digitalWrite(LED_GREEN, LOW);
//  
//  if(data[8] != OneWire::crc8(data, 8)) {
//    // t = "XXX"
//    tchar[0] = 'X'; tchar[1] = 'X'; tchar[2] = 'X'; tchar[3] = 0; 
//    Serial.println("!CRC\n");
//    return;
//  }
//  
//  int t = ((int)data[1] << 8) + (int)data[0]; t = t*6+t/4;  // *6.25
//  
//  byte sign;
//  if( t < 0 ) { sign =  1; t = -t; } else { sign = 0; }
//  
//  int h = t %100; t = t / 100;
//
//  if(sign) { sprintf(tchar, "-%d.%02d", t, h); }
//  else { sprintf(tchar, "%d.%02d", t, h); }
//
//  Serial.print("t: "); Serial.println(tchar); Serial.println();
//}


int read_dht() {
  dht d;
  char c[20];

  if(d.read22(DHT_PIN) == DHTLIB_OK) {
    dtostrf(d.humidity, 3, 1, c);
    strcat(ubuff, "&h="); strcat(ubuff, c);
    dtostrf(d.temperature, 3, 1, c);
    strcat(ubuff, "&t="); strcat(ubuff, c);
    return 0;
  }
  return -1;
}

int read_bmp() {
    Adafruit_BMP085 bmp;  
    char c[20];
    
    if( !bmp.begin() ) {
      return -1;  
    }
  
    dtostrf(bmp.readTemperature(), 3, 1, c);
    strcat(ubuff, "&t1="); strcat(ubuff, c);
    dtostrf(bmp.readPressure()/100., 3, 1, c);
    strcat(ubuff, "&p="); strcat(ubuff, c);
    return 0;
}


void gsm_send(int cycle) {
  Serial.print("\ngsm_send: "); Serial.println(cycle);      

  gsm.begin(9600);
  
  int rc;
  rc = gsm.SendATCmdWaitResp("AT+GSN", 500, 100, "OK", 1);  // == 1
  
  int i = 0, j = 0; 
  byte c; 
  while((c = gsm.comm_buf[i]) && i < 1000) {
    if( c <= 32 ) { i++; } else { break; }
  }
  while( (c = gsm.comm_buf[i]) && (j < IMEI_LEN) ) {
    if( c <= 32 ) { break; }
    imei[j++] = c; i++;
  }
  imei[j] = 0;
  
  Serial.print("\nimei: "); Serial.println(imei);
 
  strcat(ubuff, "&hwid=");
  strcat(ubuff, imei);
  
  // TODO: calc hash = secret . ubuff

  // strcat(ubuff, "&sha1=");
  // strcat(ubuff, "???????");

  digitalWrite(LED_GREEN, HIGH);  
  inet.attachGPRS(APN, USER, PASS);
  delay(2000);

  gsm.SimpleWriteln("AT+CIFSR");
  delay(5000);

  Serial.print("\nurl: "); Serial.println(ubuff);
  
  inet.httpGET(HOST, PORT, ubuff, rbuff, 50); 
  digitalWrite(LED_GREEN, LOW);  
  Serial.print("\nresp: "); Serial.println(rbuff);
  
  sim_reset();  
}

void pause(int sec) {
  for( int i=0; i < sec; i++){
    delay(500);
    if( i % 4 == 0 ) {
      digitalWrite(LED_RED, HIGH);
      delay(5);
      digitalWrite(LED_RED, LOW);
      delay(40);
      digitalWrite(LED_GREEN, HIGH);
      delay(5);
      digitalWrite(LED_GREEN, LOW);
      delay(450);
    }
    else {
      delay(500);
    }
  }  
}

void loop() {
  char c[20];
  strcpy(ubuff, BASE_URI);
  sprintf(c, "cycle=%d", cycle);
  strcat(ubuff, c);
  // read_t();
  read_dht();
  read_bmp();
  gsm_send(cycle++);
  pause(INTERVAL);
}

//.

