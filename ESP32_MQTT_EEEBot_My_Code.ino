

//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  ESP32 MQTT EEEBot Template                          *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code is modified from https://randomnerdtutorials.com by Rui Santos

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <NewPing.h>
#include <math.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_MPU6050.h>
#include <MPU6050_light.h>
#include <unistd.h>
#include <time.h>

#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal
#define TRIGGER_PIN 14
#define ECHO_PIN 27
#define MAX_DISTANCE 150
#define ENCODER_PULSES_PER_REV 24
#define sensor0_max 4095
#define sensor1_max 4095
#define sensor2_max 4095
#define sensor3_max 3050
#define sensor4_max 3870
#define sensor5_max 4095


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
MPU6050 mpu(Wire);


// replace the next variables with your SSID/Password combination
const char* ssid = "B01-17-V2";     
const char* password = "FA1717VK";  

// add your MQTT Broker IP address, example:
const char* mqtt_server = "B01-17-V2.local";  

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

int sensorDistance, velocityChange;
float temperature, velocity, cpuTemp, yawAngle;
long timer = 0;

const int motor_offset = 25;
const int servo_offset = 15;
const int ledPin = 4;
const int pin0 = 36;
const int pin1 = 39;
const int pin2 = 34;
const int pin3 = 35;
const int pin4 = 32;
const int pin5 = 33;


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens
  }

  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // gyro and accelero
  Serial.println("Done!\n");

  setup_wifi();
  client.setServer(mqtt_server, 1883); // mqtt  and port for the IP
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // we start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println("RPI_ADDRESS");
}

// three integer values are sent to the slave device
int leftMotor_speed, rightMotor_speed, servoAngle;


void callback(char* topic, byte* message, unsigned int length)
{

  //int16_t encRight_count = 0;
  //int16_t encLeft_count = 0;

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
  if (String(topic) == "esp32_falu/output") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("LED = ON\n");
      digitalWrite(ledPin, HIGH);
    } else {
      Serial.println("LED = OFF\n");
      digitalWrite(ledPin, LOW);
    }
  }

  if (String(topic) == "esp32_falu/forward") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING FORWARD\n");
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 250;
      rightMotor_speed = 175;
      //driveStraight(encRight_count, encLeft_count);
      servoAngle = 93;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    } 
    else {


      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/reverse") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING BACKWARD\n");
      leftMotor_speed = -250;
      rightMotor_speed = -175;
      //driveStraight(encRight_count, encLeft_count);
      servoAngle = 93;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting

    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/forwardRight") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING FORWARD AND RIGHT\n");
      leftMotor_speed = 250;
      rightMotor_speed = 175 - motor_offset;
      servoAngle = 93 + servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting

    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93 + servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/forwardLeft") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING FORWARD AND LEFT\n");
      leftMotor_speed = 250 - motor_offset;
      rightMotor_speed = 175;
      servoAngle = 93 - servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting

    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93 - servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/reverseRight") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING BACKWARD AND RIGHT\n");
      leftMotor_speed = -250;
      rightMotor_speed = -175 + motor_offset;
      servoAngle = 93 + servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting

    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93 + servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/reverseLeft") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("EEEBOT GOING BACKWARD AND LEFT\n");
      leftMotor_speed = -250 + motor_offset;
      rightMotor_speed = -175;
      servoAngle = 93 - servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting

    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93 - servo_offset;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }

  if (String(topic) == "esp32_falu/centre") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      if (yawAngle < 0) {
        String(topic) == "esp32/forwardRight";
        messageTemp == "off";
      }
    } else if (yawAngle > 0) {
      String(topic) == "esp32/forwardLeft";
      messageTemp == "off";
    } else {
      Serial.print("EEEBOT STOPPED\n");
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      servoAngle = 93;

      Wire.beginTransmission(I2C_SLAVE_ADDR);                    // transmit to device #4
      Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));   // first byte of leftMotor_speed, containing bits 16 to 9
      Wire.write((byte)(leftMotor_speed & 0x000000FF));          // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of rightMotor_speed, containing bits 16 to 9
      Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1         // second byte of x, containing the 8 LSB - bits 8 to 1
      Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of servoAngle, containing bits 16 to 9
      Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
      Wire.endTransmission();                                    // stop transmitting
    }
  }
  // --
}

void reconnect() {
  // loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");

      // add your subscribe topics here
      // --
      client.subscribe("esp32_falu/output");
      client.subscribe("esp32_falu/forward");
      client.subscribe("esp32_falu/reverse");
      client.subscribe("esp32_falu/forwardRight");
      client.subscribe("esp32_falu/forwardLeft");
      client.subscribe("esp32_falu/reverseRight");
      client.subscribe("esp32_falu/reverseLeft");
      client.subscribe("esp32_falu/centre");
      // --
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



float totalDistance(int16_t encRight_count, int16_t encLeft_count)
{
  float averageCount, PPM, distanceTravelled;
  averageCount = (encRight_count + encLeft_count) / 2;
  PPM = 24 / (M_PI * 0.059);
  distanceTravelled = (averageCount / PPM);

  return distanceTravelled;
}

float currentVelocity(int16_t encRight_count, int16_t encLeft_count)
{
  // Declare variables
  long interval = 1000;
  long previousMillis = 0;
  long currentMillis = 0;
  float RPM, PPR, averageCount;

  // Calculation of mean count between left and right encoder
  averageCount = (encRight_count + encLeft_count) / 2;

  // Calculation of Pulses Per Rotation
  PPR = 24 / (M_PI * 0.059);

  // Update RPM value every second
  currentMillis = millis();
  if ((currentMillis - previousMillis) > interval)
  {
    previousMillis = currentMillis;
  

  // Revolutions per minute (RPM) = Encoder count per min / pulses per rotation
  RPM = (float)((averageCount * 60) / PPR);
  }
  
  averageCount = 0;

  return RPM;
}

/*void driveStraight(int16_t encRight_count, int16_t encLeft_count)
{
  // Declare variables
  int16_t encoder_diff = 0;

  // Loop continously
  while(1)
  {
   
    encoder_diff = encRight_count - encLeft_count;

    // If right is faster, slow it down and speed up left
    if (encoder_diff > 0)
    {
      rightMotor_speed -= motor_offset;
      leftMotor_speed += motor_offset;
    }   
    // If left is faster, slow it down and speed up right
    else if (encoder_diff < 0)
    {
      rightMotor_speed += motor_offset;
      leftMotor_speed -= motor_offset;
    }
    // Brief pause to let the motors respond
    delay(20);
  }
}

void driveStraight_yaw(float yawAngle)
{
  while (1)
  {
  while ((yawAngle < 90)&&(yawAngle > -90))
  {
    // If curving right, start curving left
    if ((yawAngle > 0)&&(yawAngle < 90))
    {
      servoAngle -= motor_offset;
    }
    // If curving left, start curving right
    else if ((yawAngle < 0)&&(yawAngle > -90))
    {
      servoAngle -= motor_offset;
    }
  }

  while ((yawAngle > 90)&&(yawAngle < -90))
  {
    yawAngle = fmod(yawAngle, 90);
  }
  }
}*/


/*void weighted_average(int sensor0, int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
  aligned_weight = ;

  if sensor0 != sensor1_max
  {

  }
  else if sensor0 == pin_min
  {
    serial.print("Turning Left")
  }
}*/
void loop() 
{


  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 100) {
    lastMsg = now;

    // add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --

    mpu.update();

    if ((millis() - timer) > 500)  // print data every 500ms
    {
      temperature = mpu.getTemp();
      yawAngle = ((mpu.getAngleZ()) * -1);
      timer = millis();
    }
    //Serial.print("Yaw Angle = ");
    //Serial.println(yawAngle);
    //Serial.print("Temperature = ");
    //Serial.println(temperature);

    sensorDistance = sonar.ping_cm();
    //Serial.print("DISTANCE = ");
    //Serial.println(sensorDistance);


    // two 16-bit integer values are requested from the slave
    float distanceTravelled = 0;
    int16_t encRight_count = 0;
    int16_t encLeft_count = 0;
    float distanceGap = 0;
    uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
    uint8_t encRight_count16_9 = Wire.read();                     // receive bits 16 to 9 of a (one byte)
    uint8_t encRight_count8_1 = Wire.read();                      // receive bits 8 to 1 of a (one byte)
    uint8_t encLeft_count16_9 = Wire.read();                      // receive bits 16 to 9 of b (one byte)
    uint8_t encLeft_count8_1 = Wire.read();                    // receive bits 8 to 1 of b (one byte)

    encRight_count = (encRight_count16_9 << 8) | encRight_count8_1;  // combine the two bytes into a 16 bit number
    encLeft_count = (encLeft_count16_9 << 8) | encLeft_count8_1;     // combine the two bytes into a 16 bit number
    //Serial.print("RIGHT ENCODER COUNT = ");
    //Serial.print(encRight_count);
    //Serial.print("LEFT ENCODER COUNT = ");
    //Serial.print(encLeft_count);

    distanceTravelled = totalDistance(encRight_count, encLeft_count);

    //distanceGap = (*distanceArray[0] - distanceArray[0]);
    velocity = currentVelocity(encRight_count, encLeft_count);

    int sensor0 = analogRead(pin0);
    int sensor1 = analogRead(pin1);
    int sensor2 = analogRead(pin2);
    int sensor3 = analogRead(pin3);
    int sensor4 = analogRead(pin4);
    int sensor5 = analogRead(pin5);

    char distString[8];
    dtostrf(sensorDistance, 1, 2, distString);
    // Serial.print("Sensor Distance = ");
    // Serial.println(distString);
    client.publish("esp32_falu/sensorDistance", distString);

    char encRight_string[8];
    dtostrf(encRight_count, 1, 2, encRight_string);
    //Serial.print("Right Encoder Distance = ");
    //Serial.println(encRight_string);
    client.publish("esp32_falu/encRight_count", encRight_string);

    char encLeft_string[8];
    dtostrf(encLeft_count, 1, 2, encLeft_string);
    //Serial.print("Left Encoder Distance = ");
    //Serial.println(encLeft_string);
    client.publish("esp32_falu/encLeft_count", encLeft_string);

    char distTravelled[8];
    dtostrf(distanceTravelled, 1, 2, distTravelled);
    //Serial.print("Total Distance Travelled = ");
    //Serial.println(distTravelled);
    client.publish("esp32_falu/distanceTravelled", distTravelled);

    char velocity_string[8];
    dtostrf(velocity, 1, 2, velocity_string);
    //Serial.print("Velocity = ");
    //Serial.println(velocity_string);
    client.publish("esp32_falu/velocity", velocity_string);

    char temp_string[8];
    dtostrf(temperature, 1, 2, temp_string);
    //Serial.print("Temperature =  ");
    //Serial.println(tempString);
    client.publish("esp32_falu/temperature", temp_string);

    char yaw_string[8];
    dtostrf(yawAngle, 1, 2, yaw_string);
    //Serial.print("Yaw Angle = ");
    //Serial.println(yawAngle);
    client.publish("esp32_falu/yawAngle", yaw_string);

    char IR0[8];
     dtostrf(sensor0, 1, 2, IR0);
    Serial.print("Sensor 0 = ");
    Serial.println(sensor0);
    client.publish("esp32_falu/sensor0", IR0);

    char IR1[8];
    dtostrf(sensor1, 1, 2, IR1);
    Serial.print("Sensor 1 = ");
    Serial.println(sensor1);
    client.publish("esp32_falu/sensor1", IR1);

    char IR2[8];
    dtostrf(sensor2, 1, 2, IR2);
    Serial.print("Sensor 2 = ");
    Serial.println(sensor2);
    client.publish("esp32_falu/sensor2", IR2);

    char IR3[8];
    dtostrf(sensor3, 1, 2, IR3);
    Serial.print("Sensor 3 = ");
    Serial.println(sensor3);
    client.publish("esp32_falu/sensor3", IR3);

    char IR4[8];
    dtostrf(sensor4, 1, 2, IR4);
    Serial.print("Sensosr 4 = ");
    Serial.println(sensor4);
    client.publish("esp32_falu/sensor4", IR4);

    char IR5[8];
    dtostrf(sensor5, 1, 2, IR5);
    Serial.print("Sensor 5 = ");
    Serial.println(sensor5);
    client.publish("esp32_falu/sensor5", IR5);

    delay(100);
    /*char cpuTemp_string[8];
    dtostrf(cpuTemp, 1, 2, cpuTemp_string);
    Serial.print("CPU TEMPERATURE =  ");
    Serial.println(cpuTemp);
    client.publish("esp32/cpuTemp", cpuTemp_string);*/

    /*char voltage_string[8];
    dtostrf(voltage, 1, 2, voltage_string);
    Serial.print("Voltage =  ");
    Serial.println(voltage);
    client.publish("esp32/voltage", voltage_string);

   /*  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
   depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */

    /*
    //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
    //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
    Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
    Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of leftMotor_speed, containing the 8 LSB - bits 8 to 1
    //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
    //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
    Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    // first byte of rightMotor_speed, containing bits 16 to 9
    Wire.write((byte)(rightMotor_speed & 0x000000FF));           // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of servoAngle, containing bits 16 to 9
    Wire.write((byte)(servoAngle & 0x000000FF));           // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
    Wire.endTransmission();   // stop transmitting
  
    delay(100);
    */

    // --
  }
}
