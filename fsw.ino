/*
   Durum bilgileri:
   1. stabil
   2. yukselis
   3. dusus
   4. ayrilma
   5. bonus
   6. yerde
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <BMP085.h>
#include <SimpleKalmanFilter.h>
#include <neotimer.h>

// GPS
static uint32_t RXPin = 10, TXPin = 11;
static uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define BNO055_SAMPLERATE_DELAY_MS (1)
uint32_t donus_sayisi = 0;
bool tam = false, yarim = false, ceyrek = false;


//BMP085
BMP085 bmp;
float referencePressure;
int timee, previousTime = 0;
float firstHeight = 0;

SimpleKalmanFilter kalman(4, 4, 0.4);
SimpleKalmanFilter kalman_vel(0.8, 0.8, 0.1);


// Servo
Servo servo;     //D5
bool servoK = false;
bool servoB = false;
Servo esc;       //D5
uint32_t escPin = 6;  //D6

// Global Variables
const int VOLTAGE_PIN = A3 ; //A3
const uint8_t BUZZER = A2 ; //A2
#define FUSE 12  //D12
const int LDR = A7  ; //A7

uint16_t ldr ;
float batteryVoltage;
float oldHeight = 0;
short counter = 0, cnt = 0;
short state2 = 0, sayac = 0;
bool b = false, a = false , buzzer = false , dny = false , fny2 = false;


// PID Variables
double setPoint, input, output, Kp = 0.1 , Ki = 0.5 , Kd = 0.09;
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);


char command;

typedef struct {
  const char* id = "37032";
  char* video = "Hayir";
  char* state;
  unsigned short numOfTurns = 0;
  unsigned short packNo = 0;
  float pressure;
  float height;
  float velocity;
  float temperature;
  float voltage;
  float gpsLat;
  float gpsLng;
  float gpsAlt;
  float pitch;
  float roll;
  float yaw;
} t;

t telemetry;

uint32_t timer = millis();
uint32_t timer4state = millis();
Neotimer servo_timer = Neotimer(5000);
Neotimer pid_timer = Neotimer(10000);
Neotimer fny_timer = Neotimer(100);


/****** SET UP *****/
void setup() {

  servo.write(0);
  // esc.attach(escPin, 1000, 2000);
  esc.write(0);
  Serial.begin(9600);
  delay(3000);


  //PID
  pid.SetMode(AUTOMATIC);
  setPoint = 0;



  // BMP085
  while (!bmp.begin(BMP085_HIGH_RES))
  {
  }


  bmp.begin();

  //EEPROM
  controlEEPROM();


  //BNO055
  if (!bno.begin())
  {

    while (1);
  }
  bno.setExtCrystalUse(true);

  // GPS
  ss.begin(GPSBaud);


  //Buzzer
  pinMode(BUZZER, OUTPUT);
  //LDR
  pinMode(LDR, INPUT);

  //VOLTAGE
  pinMode(VOLTAGE_PIN, INPUT);

  //FUSE
  pinMode(FUSE , OUTPUT);


}

/****** LOOP *****/
void loop() {

  // GET SENSOR VALUES
  runBMP085();
  runBNO055();
  computeVoltage();
  if (ss.available()) {
    getGPS();
  }



  // START TELEMETRY
  if (millis() - timer > 1000) { // every 1 min
    timer = millis();

    Serial.println("OK");
    printTelemetry();
  }


  // STATUS & FUNCTIONS

  if ((telemetry.height >= -0.5 && telemetry.height <= 1 ) && state2 == 1 ) {   // stabil
    state2 = 1;
    setState(state2);
  }
  if (telemetry.height > 7 && (state2 == 1 || state2 == 2))  // stabil veya yukselis  //  >7
  {
    state2 = 2;
    setState(state2);

  }
  if ( telemetry.height >= 600 && telemetry.height <= 700   && (state2 == 2 || state2 == 3)) { // yukselis veya dusus  //600 ve 700
    state2 = 3;
    setState(state2);

  }
  if ((telemetry.height <= 410 && telemetry.height >= 390) && (state2 == 3 || state2 == 4) )  // dusus veya ayrilma //410 ve 390
  {
    RXPin = 11;
    TXPin = 10;
    SoftwareSerial ss(RXPin, TXPin);
    servo.attach(5);
    esc.attach(escPin, 1000, 2000);
    servo.write(90);
    ldr = analogRead(LDR);
    esc.attach(escPin, 1000, 2000);
    if (ldr > 550) {
      state2 = 4;
      setState(state2);
    }

  }
  if ((telemetry.height <= 250 && telemetry.height >= 190) && (state2 == 4 || state2 == 5) )  // ayrilma veya bonus  //250 ve 190
  {
    servo.attach(5);
    servo.write(0);
    state2 = 5;
    setState(state2);
  }
  if (telemetry.height <= 2 && (state2 == 5 || state2 == 6)) { //bonus veya yerde   //  <= 0
    state2 = 6;
    RXPin = 10;
    TXPin = 11;
    SoftwareSerial ss(RXPin, TXPin);
    setState(state2);
    servo.detach();
    digitalWrite(BUZZER, HIGH);

  }

  // PID ALGORITHM

  if ((telemetry.height <= 400 && telemetry.height >= 250) && (state2 == 3 ||  state2 == 4)) {  // dusus veya ayrilma
    RXPin = 11;
    TXPin = 10;
    SoftwareSerial ss(RXPin, TXPin);
    setPoint = 9;
    input = telemetry.velocity;
    pid.Compute();
    esc.write(output);
  }
  else if ((telemetry.height <= 250 && telemetry.height >= 200) && ( state2 == 5 ||  state2 == 4)) {   // bonus veya ayrilma
    RXPin = 11;
    TXPin = 10;
    SoftwareSerial ss(RXPin, TXPin);
    do {
      input = telemetry.velocity;
      pid.Compute();
      esc.write(output);
      setPoint --;
    }
    while (setPoint == 0);

    if (setPoint == 0) {
      RXPin = 11;
      TXPin = 10;
      SoftwareSerial ss(RXPin, TXPin);
      if (a == false) {
        pid_timer.start();
        a = true;
      }
      if (pid_timer.done()) {
        b = true;

      } if (pid_timer.waiting()) {
        setPoint = 0;
        input = telemetry.velocity;
        pid.Compute();
        esc.write(output);
      }
    }
   
    if ( b == true) {
      do {
        input = telemetry.velocity;
        pid.Compute();
        esc.write(output);
        setPoint ++;
      }
      while (setPoint == 9);

    }

  } else if ((telemetry.height <= 190 && telemetry.height >= 20) && ( state2 == 5 ||  state2 == 6)) {  //bonus veya yerde
    RXPin = 11;
    TXPin = 10;
    SoftwareSerial ss(RXPin, TXPin);
    setPoint = 9;
    input = telemetry.velocity;
    pid.Compute();
    esc.write(output);
  } else if ((telemetry.height <= 20 && telemetry.height >= 2) && ( state2 == 5 ||  state2 == 6)) {   //bonus veya yerde
    do {
      input = telemetry.velocity;
      pid.Compute();
      esc.write(output);
      setPoint ++;
    }
    while (setPoint == 11);
  } else if ((telemetry.height <= 5 && telemetry.height >= 0) && ( state2 == 5 ||  state2 == 6)) {    //bonus veya yerde
    RXPin = 10;
    TXPin = 11;
    SoftwareSerial ss(RXPin, TXPin);
    esc.write(0);
  }



  // GET COMMANDS
  if (Serial.available()) {
    command = Serial.read();
    getCommand(command);

  }
  if (dny == true) {
    RXPin = 11;
    TXPin = 10;
    SoftwareSerial ss(RXPin, TXPin);
    dny = false;
  }
 

  if (fny2 == true) {
    if (fny_timer.done() ) {
      digitalWrite(FUSE, LOW);
      fny2 = false;
    }
    if (fny_timer.waiting()) {
      digitalWrite(FUSE, HIGH);
    }
  }

  if (servoK == true) {
    if (servo_timer.done() ) {
      servoB = true;
      if (servoB == true) {
        servoB = false;
        servo.write(0);

      }
    }
    if (servo_timer.waiting()) {
      servo.write(90);
    }
  }

  if (telemetry.voltage < 12.0) {
    esc.detach();
    bmp.setOversampling(BMP085_ULTRA_LOW_POWER);
    bno.enterSuspendMode();

  }
  if (buzzer == true) {
    digitalWrite(BUZZER, HIGH);
  }

}





// GPS
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void getGPS() {

  telemetry.gpsLat = gps.location.lat();
  telemetry.gpsLng = gps.location.lng();
  telemetry.gpsAlt = gps.altitude.meters();

  smartDelay(0);
}

static void dateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  String a, b;
  if (!d.isValid())
  {
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.day(), d.month(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour() + 3, t.minute(), t.second());
    Serial.print(sz);
  }
  smartDelay(0);
}

// BNO055
void runBNO055() {
  delay( BNO055_SAMPLERATE_DELAY_MS);
  sensors_event_t event;
  bno.getEvent(&event);

  telemetry.roll = event.orientation.z;
  telemetry.pitch = event.orientation.y;
  telemetry.yaw = event.orientation.x;

  numOfTurns(telemetry.yaw);

}

void numOfTurns(float yaw) {
  if ( yaw > 0 && yaw < 180 && ceyrek == false) {
    ceyrek = true;
  }
  if ( yaw > 180 && yaw < 300 && ceyrek == true) {
    yarim = true;
  }
  if (yaw > 300 &&  yaw < 360 && yarim == true ) {
    yarim = false;
    ceyrek = false;
    telemetry.numOfTurns++;
  }
}


//BMP085
float velocity(float height) {
  timee = millis();
  float interval = timee - previousTime;
  float v = ((height - firstHeight) / (interval / 1000.0));
  firstHeight = height;
  telemetry.velocity = kalman_vel.updateEstimate(v);
  previousTime = timee;
  return v ;
}


void runBMP085() {
  bmp.setOversampling(BMP085_HIGH_RES);
  float temp = bmp.readTemperature();
  telemetry.temperature = temp;
  long pressure085 =  bmp.readPressure();
  telemetry.pressure = pressure085;
  float relativeAltitude = bmp.getAltitude(pressure085, referencePressure);
  float filteredAltitude = kalman.updateEstimate(relativeAltitude);
  telemetry.height = filteredAltitude;
  velocity(filteredAltitude);

}


// Battery Voltage
void computeVoltage() {
  batteryVoltage = (19 * analogRead(VOLTAGE_PIN) / 1024.0);
  telemetry.voltage = batteryVoltage;
}


// Commands
void getCommand(char c) {
  if (c == '~')
  {
    telemetry.video = "Evet";
  } else if (c == '$') {
    buzzer = true;
  }
  else if (c == '@')
  {
    dny = true;
    servo.attach(5);
    servoK = true;
    servo_timer.start();
  }
  else if (c == '^')
  { //B plani
    //(sensor yeterince hassas olcemiyor, b plani olarak set state yapabiliriz)
    state2 = 3;
    setState(state2 );
  }
  else if (c == '#')
  {
    fny2 = true;
    fny_timer.start();

  } else if (c == '|')
  {
    digitalWrite(FUSE, LOW);
  } else if (c == '[')
  {
    dny = true;
    esc.attach(escPin, 1000, 2000);
    esc.write(35);

  }
  else if (']') {
    esc.attach(escPin, 1000, 2000);
    esc.write(0);
  }

}


//EEPROM control
void controlEEPROM() {
  if (EEPROM.get(0, referencePressure) != 0) {
    referencePressure = EEPROM.get(0, referencePressure);
    telemetry.packNo = EEPROM.get(4, telemetry.packNo);
    state2 =  EEPROM.get(8, state2);
    setState(state2);
  }
  else
  {
    for (int i = 0; i <= 10 ; i++) {
      referencePressure = bmp.readPressure();
      EEPROM.put(0, referencePressure);
    }

    state2 = 1;
    setState(state2);
  }
}

//STATE
short setState(short state2) {
  if (state2 == 1)
  {
    telemetry.state = "STABIL";
    EEPROM.put(8, state2);
  }
  else if (state2 == 2)
  {
    telemetry.state = "YUKSELIS";
    EEPROM.put(8, state2);
  }
  else if (state2 == 3)
  {
    telemetry.state = "DUSUS";
    EEPROM.put(8, state2);
  }
  else if (state2 == 4)
  {
    telemetry.state = "AYRILMA";
    EEPROM.put(8, state2);
  }
  else if (state2 == 5)
  {
    telemetry.state = "BONUS";
    EEPROM.put(8, state2);
  }
  else if (state2 == 6)
  {
    telemetry.state = "YERDE";
    EEPROM.put(8, state2);
  }
  return state2;
}

//Print Telemetry
void printTelemetry() {
  telemetry.packNo++;
  EEPROM.put(4, telemetry.packNo);
  Serial.print(telemetry.id); Serial.print(','); // takim numarasi
  Serial.print(telemetry.packNo); Serial.print(',');  // paket numarasi
  dateTime(gps.date, gps.time); Serial.print(','); // gonderme zamani
  Serial.print(telemetry.pressure); Serial.print(','); //basinc
  Serial.print(telemetry.height); Serial.print(',');    // yukseklik
  Serial.print(telemetry.velocity); Serial.print(',');  // inis hizi
  Serial.print(telemetry.temperature); Serial.print(',');  //sicaklik
  Serial.print(telemetry.voltage); Serial.print(',');  //pil gerilimi
  Serial.print(telemetry.gpsLat, 6); Serial.print(','); // gps latitude
  Serial.print(telemetry.gpsLng, 6); Serial.print(','); // gps longitude
  Serial.print(telemetry.gpsAlt, 2); Serial.print(','); // gps altitude
  Serial.print(telemetry.state); Serial.print(',');  // uydu statusu
  Serial.print(telemetry.pitch); Serial.print(',');  // pitch
  Serial.print(telemetry.roll); Serial.print(',');  //roll
  Serial.print(telemetry.yaw); Serial.print(',');  // yaw
  Serial.print(telemetry.numOfTurns); Serial.print(',');  //donus sayisi
  Serial.print(telemetry.video); //video aktarim bilgisi
  Serial.print('>');

}