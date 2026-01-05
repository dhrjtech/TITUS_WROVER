/*
 Project: TITUS WROVER Line Follower Robot
 Controller: ESP32 WROVER
 Features:
 - PID-based line following
 - Web-based real-time PID tuning
 - Line-loss recovery
 Author: Dheeraj T
*/




#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ================= WIFI =================
const char* ssid = "TITUS";
const char* password = "12345678";

// ================= WEB SERVER =================
AsyncWebServer server(80);

// ================= MOTOR PINS =================
const int IN1_PIN = 23; // MOTOR A
const int IN2_PIN = 22;
const int ENA_PIN = 19;

const int IN3_PIN = 21; // MOTOR B
const int IN4_PIN = 18;
const int ENB_PIN = 5;

// ================= PWM SETTINGS =================
const int PWM_FREQUENCY = 10000;
const int PWM_RESOLUTION = 8;

// ================= SENSOR PINS =================
const int SENSOR_PINS[8] = {32, 33, 25, 26, 27, 14, 12, 13};
const int n = 8;

// ================= SPEED =================
int baseSpeed = 150;
int maxSpeed = 255;

// ================= PID =================
double Kp = 155.0;
double Ki = 0;
double Kd = 3;

float error = 0;
float previousError = 0;
float integral = 0;

int SENSOR_VAL[8];

// ================= STOP FLAG =================
volatile bool stopMotors = false;

// ================= PID FUNCTION =================
float computePID(float error) {
  integral += error;
  integral = constrain(integral, -100, 100);
  float derivative = error - previousError;
  previousError = error;
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Web app in html 

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 PID Tuner</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial;
      text-align: center;
      background: #111;
      color: white;
    }
    h2 { color: #0f0; }
    .box {
      border: 1px solid #333;
      padding: 15px;
      margin: 10px;
    }
    input[type=range] {
      width: 90%;
    }
    input[type=number] {
      width: 120px;
      font-size: 18px;
      text-align: center;
    }
    button {
      font-size: 18px;
      padding: 8px 20px;
      margin: 6px;
    }
  </style>
</head>
<body>

<h2>PID TUNER</h2>

<div class="box">
  <h3>Base Speed</h3>
  <input type="range" min="0" max="255" step="1"
    id="baseSlider" value="150"
    oninput="syncBase(this.value)">
  <br>
  <input type="number" min="0" max="255"
    id="baseInput" value="150"
    onchange="syncBase(this.value)">
</div>

<div class="box">
  <h3>Kp</h3>
  <input type="range" min="0" max="300" step="0.5"
    id="kpSlider" value="155"
    oninput="syncPID('kp', this.value)">
  <br>
  <input type="number" step="0.5"
    id="kpInput" value="155"
    onchange="syncPID('kp', this.value)">
</div>

<div class="box">
  <h3>Ki</h3>
  <input type="range" min="0" max="50" step="0.01"
    id="kiSlider" value="0"
    oninput="syncPID('ki', this.value)">
  <br>
  <input type="number" step="0.01"
    id="kiInput" value="0"
    onchange="syncPID('ki', this.value)">
</div>

<div class="box">
  <h3>Kd</h3>
  <input type="range" min="0" max="50" step="0.1"
    id="kdSlider" value="3"
    oninput="syncPID('kd', this.value)">
  <br>
  <input type="number" step="0.1"
    id="kdInput" value="3"
    onchange="syncPID('kd', this.value)">
</div>

<br>

<button style="background:#0f0;" onclick="fetch('/start')">START</button>
<button style="background:#f00;color:white;" onclick="fetch('/stop')">STOP</button>

<script>
function syncPID(type, val) {
  document.getElementById(type + "Slider").value = val;
  document.getElementById(type + "Input").value  = val;
  fetch(`/set?${type}=${val}`);
}

function syncBase(val) {
  document.getElementById("baseSlider").value = val;
  document.getElementById("baseInput").value  = val;
  fetch(`/setSpeed?base=${val}`);
}
</script>

</body>
</html>
)rawliteral";

// ================= SETUP =================
void setup() {

  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  ledcAttach(ENA_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(ENB_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  for (int i = 0; i < n; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
    request->send(200, "text/plain", "OK");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stopMotors = true;
    ledcWrite(ENA_PIN, 0);
    ledcWrite(ENB_PIN, 0);
    request->send(200, "text/plain", "STOPPED");
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    stopMotors = false;
    integral = 0;
    previousError = 0;
    request->send(200, "text/plain", "RUNNING");
  });

  server.begin();
}

// ================= LOOP =================
void loop() {

  if (stopMotors) {
    ledcWrite(ENA_PIN, 0);
    ledcWrite(ENB_PIN, 0);
    return;
  }

  for (int i = 0; i < n; i++) {
    SENSOR_VAL[i] = digitalRead(SENSOR_PINS[i]);
  }

  float x = 0, y = 0;
  for (int i = 0; i < n; i++) {
    x += i * SENSOR_VAL[i];
    y += SENSOR_VAL[i];
  }

  // if (y == 0) {
  //   ledcWrite(ENA_PIN, 0);
  //   ledcWrite(ENB_PIN, 0);
  //   integral = 0;
  //   previousError = 0;
  //   return;
  // }
  if (y == 0) {
  // ===== LINE LOST → MOVE BACKWARD =====

  // Reverse both motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);

  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  // Moderate reverse speed
  ledcWrite(ENA_PIN, 120);
  ledcWrite(ENB_PIN, 120);

  // Reset PID to avoid jump
  integral = 0;
  previousError = 0;

  return; // keep reversing until line is found
}


  error = (x / y - 3.5) * 4.0;

  float pid = computePID(error);
  pid = constrain(pid, -255, 255);

  float leftSpeed  = baseSpeed + pid;
  float rightSpeed = baseSpeed - pid;

  digitalWrite(IN1_PIN, leftSpeed >= 0);
  digitalWrite(IN2_PIN, leftSpeed < 0);
  digitalWrite(IN3_PIN, rightSpeed >= 0);
  digitalWrite(IN4_PIN, rightSpeed < 0);

  leftSpeed  = constrain(abs(leftSpeed), 0, maxSpeed);
  rightSpeed = constrain(abs(rightSpeed), 0, maxSpeed);

  ledcWrite(ENA_PIN, leftSpeed);
  ledcWrite(ENB_PIN, rightSpeed);
}
