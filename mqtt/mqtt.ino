#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi 配置
const char* ssid = "moi";  // 替换为你的 WiFi 名称
const char* password = "pikaqiuyeye";  // 替换为你的 WiFi 密码

// MQTT 配置
const char* mqtt_server = "112.4.115.127";  // MQTT 服务器地址
const int mqtt_port = 1883;  // MQTT 服务器端口
const char* mqtt_client_id = "ESP8266Client";  // MQTT 客户端 ID

// MQTT 发布的主题
const char* mqtt_topic = "machines/1001/2333/value";  // 修改为主题 "machines/1001/2333/value"

WiFiClient espClient;
PubSubClient client(espClient);

void reconnect() {
    while (!client.connected()) {
        Serial.print("尝试连接 MQTT...");
        if (client.connect(mqtt_client_id)) {
            Serial.println("连接成功！");
        } else {
            Serial.print("连接失败，错误码：");
            Serial.print(client.state());
            delay(5000);
        }
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("连接到 WiFi：");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        Serial.print(WiFi.status());  // 打印WiFi状态码
    }
    Serial.println("");
    Serial.println("WiFi 连接成功！");
    Serial.println(WiFi.localIP());
}

void setup() {
    Serial.begin(115200);
    Serial.print("------");
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    reconnect();
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }

    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 500) {
        lastSendTime = millis();

        String data = "Hello, wzdddddddd";  // 发送的消息内容
        Serial.print("发送数据：");
        Serial.println(data);

        if (!client.publish(mqtt_topic, data.c_str())) {  // 使用新的主题发送消息
            Serial.println("消息发送失败！");
        } else {
            Serial.println("消息发送成功！");
        }
    }
}
