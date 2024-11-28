#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

///****************** CONFIGURATION ******************
#define wifi_ssid "Wastefull_Insights"
#define wifi_password "wi_2107#"
#define mqtt_server "192.168.31.230" //"192.168.1.108" // your HA IP, example 192.XX.XXX.XX 
#define mqtt_user "hopev1"
#define mqtt_password "2905"
#define encoder_topic "sensor/encoder"

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 500

// Encoder output to Arduino Interrupt pin
#define ENC_IN_A 10
#define ENC_IN_B 5

#define trigger 16
//#define LED 4
//#define power 0

StaticJsonBuffer<32> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
char mqtt_message[100];
//****************** CONFIGURATION ******************

WiFiClient espClient;
PubSubClient client(espClient);

// Pulse count from encoder
volatile long encoderValue = 0;

// Variables for RPM measurement
int previousEncoderValue = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
long lastMsg = 0;
// Variable for RPM measuerment
int rpm = 0;
//int mmps = 0;
int Radius = 35.0; // mm
int direction = 1;

//*****************************************************

void setup()
{
  // Setup Serial Monitor
  Serial.begin(115200); 
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN_A, INPUT_PULLUP); 
  pinMode(ENC_IN_B, INPUT_PULLUP);  
  pinMode(trigger,OUTPUT); 
//  pinMode(LED,OUTPUT);
//  digitalWrite(LED,LOW);
  digitalWrite(trigger,LOW);
  
//  digitalWrite(trigger,LOW);
//  pinMode(power,OUTPUT);
  
  // Attach interrupts 
  attachInterrupt(digitalPinToInterrupt(ENC_IN_A), updateEncoder, RISING);
//  attachInterrupt(digitalPinToInterrupt(ENC_IN_B), checkDirection, CHANGE);
  
  // Setup initial values for timer
  previousMillis = millis();
}

void setup_wifi() {
  // Connect to a WiFi network
  Serial.println();
  Serial.print("Connecting Wi-Fi...");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connected:");
  Serial.println(WiFi.localIP());
//  digitalWrite(LED,HIGH);
}

void reconnect() {
  // Loop until we're reconnected
  digitalWrite(trigger,LOW);
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    // Attempt to connect

    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  digitalWrite(trigger,HIGH);
  
}
void loop()
{
   if (!client.connected()) {
    reconnect();
  }
//  digitalWrite(trigger,HIGH);
   client.loop();
//   digitalWrite(trigger,HIGH);
  // Update RPM value every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    long now = millis();
    
    // Calculate RPM
    rpm = (int)(encoderValue * 60 / ENC_COUNT_REV);
    previousEncoderValue = encoderValue;
    
   // mmps = (int)((rpm*2*3.14*Radius)/60.0);
    root["data"] = rpm;
    root.printTo(mqtt_message,sizeof(mqtt_message));
    // Only update display when there is a reading

    Serial.print(" PULSES: ");
    Serial.print(encoderValue);
    Serial.print('\t');
    Serial.print(" SPEED: ");
    Serial.print(rpm);
    Serial.print(" RPM");
    Serial.print('\t');
    Serial.print('\n');
//      Serial.print(mmps);
//      Serial.println(" mm/s");
    if (now - lastMsg > 1000) { // publish frequency
      lastMsg = now;
      client.publish(encoder_topic, mqtt_message, true);
    }
  
    
    encoderValue = 0;
  }
}

ICACHE_RAM_ATTR void updateEncoder()
{
  // Increment or decrement value for each change in phase A
  int stateA = digitalRead(ENC_IN_A);
  int stateB = digitalRead(ENC_IN_B);

  if (stateA == HIGH) {
    encoderValue += (stateB == HIGH) ? -1 : 1;
  } else {
    encoderValue += (stateB == HIGH) ? 1 : -1;
  }
}

//ICACHE_RAM_ATTR void checkDirection()
//{tr
//  // Read the state of phase A and phase B
//  int stateA = digitalRead(ENC_IN_A);
//  int stateB = digitalRead(ENC_IN_B);
//
//  // Determine the direction based on the state changes
//  if (stateA == HIGH && stateB == LOW) {
//    direction = 1; // Forward
//  } else if (stateA == LOW && stateB == HIGH) {
//    direction = -1; // Reverse
//  }
//}
