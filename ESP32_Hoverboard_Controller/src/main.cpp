#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Hoverboard communication defines
#define HOVER_SERIAL_BAUD   115200
#define START_FRAME         0xABCD
#define TIME_SEND           100

// WiFi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Web server
AsyncWebServer server(80);

// Serial communication with hoverboard
HardwareSerial HoverSerial(2); // Use UART2 (pins 16, 17)

// Global variables
uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// Current commands
int16_t currentSteer = 0;
int16_t currentSpeed = 0;

// HTML for the web interface
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Hoverboard Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
    .slidecontainer { width: 100%; }
    .slider { -webkit-appearance: none; width: 100%; height: 15px; border-radius: 5px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s;}
    .slider:hover { opacity: 1; }
    .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
    .slider::-moz-range-thumb { width: 25px; height: 25px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
    .button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2 {background-color: #555555;}
    .feedback { margin-top: 20px; }
  </style>
</head>
<body>
  <h1>Hoverboard Motor Controller</h1>
  <div class="slidecontainer">
    <p>Speed: <span id="speedValue"></span></p>
    <input type="range" min="-1000" max="1000" value="0" class="slider" id="speedSlider" oninput="updateSpeed(this.value)">
  </div>
  <div class="slidecontainer">
    <p>Steer: <span id="steerValue"></span></p>
    <input type="range" min="-1000" max="1000" value="0" class="slider" id="steerSlider" oninput="updateSteer(this.value)">
  </div>
  <button class="button" onclick="stopMotors()">STOP</button>
  <div class="feedback">
    <p>Left Speed: <span id="leftSpeed">0</span></p>
    <p>Right Speed: <span id="rightSpeed">0</span></p>
    <p>Battery Voltage: <span id="batVoltage">0</span></p>
    <p>Board Temp: <span id="boardTemp">0</span></p>
  </div>
  <script>
    function updateSpeed(val) {
      document.getElementById('speedValue').innerHTML = val;
      sendCommand();
    }
    function updateSteer(val) {
      document.getElementById('steerValue').innerHTML = val;
      sendCommand();
    }
    function stopMotors() {
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steerSlider').value = 0;
      document.getElementById('speedValue').innerHTML = 0;
      document.getElementById('steerValue').innerHTML = 0;
      sendCommand();
    }
    function sendCommand() {
      var speed = document.getElementById('speedSlider').value;
      var steer = document.getElementById('steerSlider').value;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/set?speed=" + speed + "&steer=" + steer, true);
      xhr.send();
    }
    setInterval(function() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var data = JSON.parse(this.responseText);
          document.getElementById('leftSpeed').innerHTML = data.leftSpeed;
          document.getElementById('rightSpeed').innerHTML = data.rightSpeed;
          document.getElementById('batVoltage').innerHTML = data.batVoltage;
          document.getElementById('boardTemp').innerHTML = data.boardTemp;
        }
      };
      xhr.open("GET", "/feedback", true);
      xhr.send();
    }, 500);
  </script>
</body>
</html>
)rawliteral";

void Send(int16_t uSteer, int16_t uSpeed) {
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

void Receive() {
  if (HoverSerial.available()) {
    incomingByte = HoverSerial.read();
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
  } else {
    return;
  }

  if (bufStartFrame == START_FRAME) {
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
    *p++ = incomingByte;
    idx++;
  }

  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                        ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    }
    idx = 0;
  }
  incomingBytePrev = incomingByte;
}

void setup() {
  Serial.begin(115200);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("speed") && request->hasParam("steer")) {
      currentSpeed = request->getParam("speed")->value().toInt();
      currentSteer = request->getParam("steer")->value().toInt();
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  server.on("/feedback", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"leftSpeed\":" + String(Feedback.speedL_meas) + ",";
    json += "\"rightSpeed\":" + String(Feedback.speedR_meas) + ",";
    json += "\"batVoltage\":" + String(Feedback.batVoltage) + ",";
    json += "\"boardTemp\":" + String(Feedback.boardTemp);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();
}

unsigned long iTimeSend = 0;

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(currentSteer, currentSpeed);
}