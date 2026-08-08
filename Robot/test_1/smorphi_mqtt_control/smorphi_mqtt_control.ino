#include <WiFi.h>
#include <PubSubClient.h>
#include <smorphi_single.h>

// ---------- WiFi credentials ----------
const char* ssid     = "Dinindu";
const char* password = "20020730d";

// ---------- MQTT (HiveMQ public broker) ----------
const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 1883;

// IMPORTANT: change "yourname123" to something unique to you.
// The public broker is shared by everyone, so a generic topic name
// like "smorphi/control" could receive commands from strangers.
const char* mqtt_topic  = "smorphi/yourname123/control";
const char* client_id   = "smorphi-esp32-yourname123"; // must also be unique on the broker

const int DRIVE_SPEED = 110;

// ---------- Circle motion tuning ----------
const int CIRCLE_SEGMENTS   = 8;
const int CIRCLE_FORWARD_MS = 400; // nudge forward
const int CIRCLE_TURN_MS    = 250; // pivot angle adjustment

WiFiClient espClient;
PubSubClient client(espClient);
Smorphi_single my_robot;

String currentCommand = "stop";
volatile bool newCommandReceived = false;

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
}

// Waits while still servicing MQTT, so a long multi-step move (like the
// circle routine) doesn't drop the connection or ignore new commands.
void waitAndServiceMQTT(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    client.loop();
  }
}

void doCircle() {
  Serial.println("Starting circle routine");
  for (int i = 0; i < CIRCLE_SEGMENTS; i++) {
    // Bail out early if a new command arrived mid-circle (e.g. "stop")
    if (newCommandReceived) {
      Serial.println("Circle interrupted by new command");
      return;
    }

    my_robot.MoveForward(DRIVE_SPEED);
    waitAndServiceMQTT(CIRCLE_FORWARD_MS);

    my_robot.MoveRight(DRIVE_SPEED);
    waitAndServiceMQTT(CIRCLE_TURN_MS);
  }
  my_robot.stopSmorphi_single();
  Serial.println("Circle complete, stopped at origin");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "forward") {
    Serial.println("Command: forward");
    my_robot.MoveForward(DRIVE_SPEED);
  } else if (cmd == "backward") {
    Serial.println("Command: backward");
    my_robot.MoveBackward(DRIVE_SPEED);
  } else if (cmd == "stop") {
    Serial.println("Command: stop");
    my_robot.stopSmorphi_single();
  } else if (cmd == "diagonal_left") {
    Serial.println("Command: diagonal_left");
    my_robot.MoveDiagUpLeft(DRIVE_SPEED);
  } else if (cmd == "diagonal_right") {
    Serial.println("Command: diagonal_right");
    my_robot.MoveDiagUpRight(DRIVE_SPEED);
  } else if (cmd == "circle") {
    doCircle();
  } else {
    Serial.println("Unknown command: " + cmd);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("MQTT message [" + String(topic) + "]: " + message);

  currentCommand = message;
  currentCommand.trim();
  currentCommand.toLowerCase();
  newCommandReceived = true;
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect(client_id)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
      Serial.println("Subscribed to: " + String(mqtt_topic));
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2s");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  my_robot.BeginSmorphi_single();

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  if (newCommandReceived) {
    newCommandReceived = false;
    handleCommand(currentCommand);
  }
}
