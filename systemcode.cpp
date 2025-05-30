#define BLYNK_TEMPLATE_ID "TMPL3itBjW2eH"
#define BLYNK_TEMPLATE_NAME "IoT Based Aquaculture"
#define BLYNK_AUTH_TOKEN "41neT73UmL9gncTDu4q_URjHJuwLgc2x"
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Servo.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#define ONE_WIRE_BUS D5                          //D1 pin of nodemcu
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.


Servo myservo;
BlynkTimer timer;


int relay = D8;
int ph = D7;
int phval;
int water =D6;
int waterval;
int pump =D1;
bool tempstatus =false;
bool phstatus =false;
bool tdsstatus =false;
bool waterstatus =false;
float temp;
namespace pin {
    const byte tds_sensor = A0;
}

namespace device {
    float aref = 1.0; // ESP8266 ADC range is 0-1V, adjust if using a voltage divider.
}

namespace sensor {
    float ec = 0;
    unsigned int tds = 0;
    float waterTemp = 25.0; // Default water temperature, can use a temperature sensor for actual value.
    float ecCalibration = 1.0; // Adjust this value based on calibration.
}

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "iot";
char pass[] = "12345678";

float t;
float h;

BLYNK_WRITE(V2)
{
  int s =param.asInt();
  if(s ==1)
  {
    myservo.write(90);
   
  }
  else
  {
    myservo.write(0);
   
  }
}

BLYNK_WRITE(V3)
{
  int p =param.asInt();
  if(p ==1)
  {
    digitalWrite(pump,HIGH);
  }
  else
  {
    digitalWrite(pump,LOW);
   
  }
}
void setup() {
    Serial.begin(9600);
    sensors.begin();
    myservo.attach(D2,500,2400);
    myservo.write(0);
    pinMode(ph, INPUT);
    pinMode(relay, OUTPUT);
    pinMode(water,INPUT);
    pinMode(pump,OUTPUT);
    Blynk.begin(auth, ssid, pass);
    timer.setInterval(5000, tmp);
    timer.setInterval(1000, sen);
    timer.setInterval(60000,up);
}

void loop() {
    Blynk.run();
    timer.run();
}

void up()
{
  Blynk.virtualWrite(V0, sensor::tds);
  Blynk.virtualWrite(V1,temp);
}

void sen() {
    // Read raw ADC value
    float rawVoltage = analogRead(pin::tds_sensor) * device::aref / 1024.0;
    //Serial.print("Raw Voltage: "); Serial.println(rawVoltage, 2);

    // Temperature compensation
    float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTemp - 25.0);
    sensor::ec = (rawVoltage / temperatureCoefficient) * sensor::ecCalibration;

    // Convert EC to TDS
    sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * pow(sensor::ec, 2) + 857.39 * sensor::ec) * 0.5;

    Serial.print(F("TDS: ")); Serial.println(sensor::tds);
    Serial.print(F("EC: ")); Serial.println(sensor::ec, 2);

   
   // Blynk.virtualWrite(V1, sensor::ec);

    if (sensor::tds >= 250  && !tdsstatus)
    {
      tdsstatus =true;
      digitalWrite(relay,HIGH);
        Serial.println("Not good");
        Blynk.logEvent("tdsstatus","WATER TDS HIGH!!");
    }
    else if(sensor::tds <250 && tdsstatus)
    {
      tdsstatus =false;
      digitalWrite(relay,LOW);
     
    }
}

void tmp() {
    phval = digitalRead(ph);
    waterval =digitalRead(water);
    Serial.println("Water:"+String(waterval));
    sensors.requestTemperatures();  
    Serial.println(sensors.getTempCByIndex(0));  
  temp = sensors.getTempCByIndex(0);                
  Serial.println("Temperature is: ");
   Serial.println(temp);
  
 
   
    if (temp>= 30  && !tempstatus)
    {
      tempstatus =true;
      Serial.println("TEMP  HIGH");
      digitalWrite(relay,HIGH);
          Blynk.logEvent("tempstatus","WATER TEMP HIGH!!");
   
      }
      else if(temp<30 && tempstatus)
      {
        digitalWrite(relay,LOW);
        tempstatus =false;
        
      }
      if (phval == 1 && !phstatus) {
        phstatus=true;
        Blynk.virtualWrite(V8, "WATER PH DETECTED");
        Blynk.logEvent("phstatus","WATER pH HIGH!!");
   
    }
    else if(phval == 0 && phstatus)
    {
      phstatus=false;
        Blynk.virtualWrite(V8, "NO WATER PH DETECTED");
     
    }
    if (waterval == 1 && !waterstatus) {
        waterstatus=true;
        //digitalWrite(pump,LOW);
        Blynk.virtualWrite(V8, "WATER IS high");
                  Blynk.logEvent("waterstatus","WATER IS high!!");
      
   
    }
    else if(waterval == 0 && waterstatus)
    {
      waterstatus=false;
      //digitalWrite(pump,HIGH);
        Blynk.virtualWrite(V8, "WATER  DETECTED");
//          Blynk.logEvent("waterstatus","WATER IS LOW!!");
        Serial.println("Pump on");
     
    }
}
