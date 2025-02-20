#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

// MQTT服务器配置
const char* ssid = "moi";
const char* password = "pikaqiuyeye";
const char* mqtt_server = "112.4.115.127";
const int mqtt_port = 1883;
//const char* mqtt_user = "xixixi";
//const char* mqtt_password = "MQTT_PASSWORD";
const char* mqtt_topic = "your/topic";

// 485通信配置
SoftwareSerial mySerial(3, 1);  // RX, TX（GPIO3，GPIO1）

WiFiClient espClient;
PubSubClient client(espClient);

// 初始化485通信
void init485() {
  mySerial.begin(115200);  // 设置485波特率
}

// 连接WiFi
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

// 连接MQTT
void connectMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    if (client.connect("ESP8266Client")) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed to connect to MQTT, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// 发送数据到485从机
void send485Data(const char* data) {
  mySerial.print(data);  // 直接通过软件串口发送数据
  delay(10);  // 等待发送完成
}

// 接收485从机返回的数据
String receive485Data() {
  String receivedData = "";
  if (mySerial.available()) {
    while (mySerial.available()) {
      char c = mySerial.read();
      receivedData += c;
      delay(3);  // 等待下一个字符
    }
  }
  return receivedData;
}

void setup() {
  Serial.begin(115200);
  init485();
  connectWiFi();
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // 每5秒发送一次数据到485从机
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 1000) {
    lastSendTime = millis();
    send485Data("666");  // 发送数据"666"到485从机
    delay(100);  // 等待从机响应
    String receivedData = receive485Data();  // 接收从机返回的数据
    if (receivedData.length() > 0) {
      Serial.print("Received from 485: ");
      Serial.println(receivedData);
      client.publish(mqtt_topic, receivedData.c_str());  // 将数据发送到MQTT服务器
    }
  }
}
