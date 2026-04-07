#include <QTRSensors.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <Adafruit_NeoPixel.h>

// --- PARAMETRY ---
float Kp = 0.03;
float Ki = 0;
float Kd = 1;
float BaseSpeed  = 80;
float MaxSpeed   = 120;
float TurnSpeed  = 60;
float lost_threshold = 400;

// --- PINY ---
#define NUM_SENSORS 8
#define EMITTER_PIN 16

#define LEFT_MOTOR_FORWARD   11
#define LEFT_MOTOR_BACKWARD  12
#define RIGHT_MOTOR_FORWARD  13
#define RIGHT_MOTOR_BACKWARD 14

#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

// --- WIFI / UDP ---
const char* ssid     = "LF";
const char* password = "1234";

AsyncUDP udp;
QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS];
int ready = 0;

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- STAN LINII ---
int   last_sighted = 0;
bool  line_lost    = false;
bool  intersection = false;
float linePosition = 3500;

// --- PID ---
float lastError = 0;

// ==================== SILNIKI ====================

void setMotor(uint8_t pin_fwd, uint8_t pin_bck, int speed)
{
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pin_fwd, speed);
    analogWrite(pin_bck, 0);
  } else if (speed < 0) {
    analogWrite(pin_fwd, 0);
    analogWrite(pin_bck, -speed);
  } else {
    analogWrite(pin_fwd, 0);
    analogWrite(pin_bck, 0);
  }
}

void stopMotors()
{
  setMotor(LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD,  0);
  setMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, 0);
}

// ==================== PID ====================

void pidControl()
{
  // --- zgubiona linia ---
  if (line_lost) {
    lastError = 0;
    if (last_sighted == 1) {
      setMotor(LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD,  -(int)TurnSpeed);
      setMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD,  (int)TurnSpeed);
    } else {
      setMotor(LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD,   (int)TurnSpeed);
      setMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, -(int)TurnSpeed);
    }
    return;
  }

  // --- skrzyżowanie: jedź prosto ---
  if (intersection) {
    setMotor(LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD,  (int)BaseSpeed);
    setMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, (int)BaseSpeed);
    lastError = 0;
    return;
  }

  // --- normalny PD ---
  float error      = linePosition - 3500.0f;
  float derivative = error - lastError;
  lastError        = error;

  float correction = Kp * error + Kd * derivative;

  int leftSpeed  = (int)(BaseSpeed + correction);
  int rightSpeed = (int)(BaseSpeed - correction);

  leftSpeed  = constrain(leftSpeed,  -(int)MaxSpeed, (int)MaxSpeed);
  rightSpeed = constrain(rightSpeed, -(int)MaxSpeed, (int)MaxSpeed);

  setMotor(LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD,  leftSpeed);
  setMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, rightSpeed);
}

// ==================== WYKRYWANIE ====================

bool isIntersection()
{
  if (line_lost) return false;
  int strong = 0;
  for (int i = 1; i <= 6; i++)
    if (sensorValues[i] > 800) strong++;
  return (strong >= 5);
}

int detectDirection()
{
  long left_sum = 0, right_sum = 0;
  for (int i = 0; i < 4; i++) left_sum  += sensorValues[i];
  for (int i = 4; i < 8; i++) right_sum += sensorValues[i];
  long diff = left_sum - right_sum;
  if (diff >  500) return 1;
  if (diff < -500) return 2;
  return 0;
}

// ==================== CZUJNIKI + LED ====================

void updateSensors()
{
  linePosition = qtr.readLineBlack(sensorValues);

  int lost_count = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (sensorValues[i] <= lost_threshold) lost_count++;

  line_lost    = (lost_count == NUM_SENSORS);
  intersection = isIntersection();

  if (!line_lost)
    last_sighted = detectDirection();

  if (intersection) { pixels.setPixelColor(0, pixels.Color(180, 0, 255));
  } else if (line_lost) {
    if      (last_sighted == 2) pixels.setPixelColor(0, pixels.Color(80, 0, 0));
    else if (last_sighted == 1) pixels.setPixelColor(0, pixels.Color(0, 0, 80));
    else                          pixels.setPixelColor(0, pixels.Color(0, 80, 0));
  } else if (last_sighted == 2)   pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  else if   (last_sighted == 1)   pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  else                            pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
}

// ==================== SETUP ====================

void setup()
{
  pixels.begin();

  pinMode(LEFT_MOTOR_FORWARD,   OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD,  OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD,  OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10,9,3,8,7,6,5,4}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  qtr.setTimeout(2000);

  Serial.begin(9600);

  WiFi.softAP(ssid, password);

  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.softAPIP());
  }

  udp.onPacket([](AsyncUDPPacket packet) {
    char* tmpStr = (char*) malloc(packet.length() + 1);
    memcpy(tmpStr, packet.data(), packet.length());
    tmpStr[packet.length()] = '\0';
    String message = String(tmpStr);
    String firstTwo = message.substring(0, 2);
    free(tmpStr);

    Serial.println(message);

    if (message == "Cal")     { calibrate(); }
    if (message == "Reset")   { ESP.restart(); }
    if (message == "Start")   { ready = 1; }
    if (message == "Stop")    { ready = 0; stopMotors(); }
    if (message == "Sensors") { request_sensorsRaw(); }
    if (message == "Params")  { request_params(); }

    if (firstTwo == "Kp") Kp             = message.substring(4).toFloat();
    if (firstTwo == "Ki") Ki             = message.substring(4).toFloat();
    if (firstTwo == "Kd") Kd             = message.substring(4).toFloat();
    if (firstTwo == "Ma") MaxSpeed       = message.substring(4).toFloat();
    if (firstTwo == "Ba") BaseSpeed      = message.substring(4).toFloat();
    if (firstTwo == "Tu") TurnSpeed      = message.substring(4).toFloat();
    if (firstTwo == "Th") lost_threshold = message.substring(4).toFloat();
  });
}

// ==================== LOOP ====================

void loop()
{
  updateSensors();

  if (ready) {
    pidControl();
  } else {
    stopMotors();
  }

  static unsigned long lastMillis = 0;
  const int interval = 200;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;

    String pos = "Position: " + String((int)linePosition);
    udp.broadcast(pos.c_str());

    request_sensorsRaw();
  }
}

// ==================== KALIBRACJA ====================

void calibrate()
{
  qtr.resetCalibration();
  for (uint16_t i = 0; i < 120; i++) {
    qtr.calibrate();
    delayMicroseconds(150);
    yield();
  }
  udp.broadcast("C");
}

// ==================== TELEMETRIA ====================

void request_sensorsRaw()
{
  String sensors = "Sensor ";
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    sensors += String(sensorValues[i]) + " ";
  sensors += "Last:" + String(last_sighted == 1 ? "L" : last_sighted == 2 ? "R" : "0");
  sensors += " Lost:" + String(line_lost);
  sensors += " Int:"  + String(intersection);
  sensors += " Pos:"  + String(linePosition);
  udp.broadcast(sensors.c_str());
}

void request_params()
{
  String params = "Kp: "      + String(Kp)
                + " Ki: "     + String(Ki)
                + " Kd: "     + String(Kd)
                + " Max: "    + String(MaxSpeed)
                + " Base: "   + String(BaseSpeed)
                + " Turn: "   + String(TurnSpeed)
                + " Lost_th: "+ String(lost_threshold);
  udp.broadcast(params.c_str());
}