#include <ESP8266WiFi.h>          // ESP8266专用WiFi库
#include <PubSubClient.h>
#include <Ticker.h>               // 替代ESP32的定时器

// WiFi配置
#define WIFI_SSID       "moi"
#define WIFI_PASSWORD   "pikaqiuyeye"

// MQTT配置
#define MQTT_BROKER     "112.4.115.127"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "ESP8266_Client"

// RS485配置
#define RS485_ENABLE_PIN 14       // RS485使能引脚（GPIO14）
#define MAX_MACHINE      32       // 最大从机数目
#define MAX_SENSOR       4        // 每个从机的传感器数目


#define TXD_PIN         1       
#define RXD_PIN         3       
// UART DMA配置
#define UART_NUM        UART_NUM_0

#define BUF_SIZE        256  // DMA接收缓冲区
#define DMA_BUF_SIZE    128  // 单个DMA缓冲区大小
#define DMA_BUF_COUNT   2    // DMA缓冲区数量
// 数据帧定义
#define FRAME_HEADER    0xAD
#define FRAME_FOOTER    0xFF
#define FRAME_MIN_LENGTH 7


WiFiClient espClient;
PubSubClient mqttClient(espClient);
Ticker stateTicker;               // 状态切换定时器

uint16_t sensor_voltages[MAX_MACHINE][MAX_SENSOR]; // 传感器数据

// 使用状态机表示主机的工作状态
enum States {
  wait = 0, query, upload
} state = wait;

// 接收到完整数据帧的标志
bool frame_ready = false;

// CRC16校验函数（Modbus）
uint16_t crc16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// RS485使能端控制（发送时置高，接收时置低）
void rs485_enable_tx() { digitalWrite(RS485_ENABLE_PIN, HIGH); }
void rs485_enable_rx() { digitalWrite(RS485_ENABLE_PIN, LOW); }

// 发送数据帧
void send_rs485_frame(uint8_t addr, uint8_t func_code, uint8_t data_len) {
    uint8_t tx_buffer[6];
    tx_buffer[0] = FRAME_HEADER;
    tx_buffer[1] = addr;
    tx_buffer[2] = func_code;
    tx_buffer[3] = data_len;

    // 计算CRC
    uint16_t crc = crc16(&tx_buffer[1], 3 + data_len);
    tx_buffer[4 + data_len] = crc & 0xFF;
    tx_buffer[5 + data_len] = crc >> 8;
    tx_buffer[6 + data_len] = FRAME_FOOTER;

    rs485_enable_tx();
    Serial.write(tx_buffer, 7 + data_len);  // 使用Serial1发送
    delay(10);                              // 等待发送完成
    rs485_enable_rx();
}

// 解析 RS485 接收的数据
void parse_rs485_data() {
    static uint8_t data[BUF_SIZE];
    static int index = 0;

    while (Serial.available()) {
        uint8_t c = Serial.read();
        if (index < BUF_SIZE) data[index++] = c;

        // 检查帧尾
        if (c == FRAME_FOOTER && index >= FRAME_MIN_LENGTH) {
            if (data[0] == FRAME_HEADER) {
                uint16_t crc_recv = (data[index - 3] << 8) | data[index - 2];
                uint16_t crc_calc = crc16(&data[1], index - 4);
                if (crc_recv == crc_calc) {
                    uint8_t addr = data[1];
                    if (addr < MAX_MACHINE) {
                        for (int i = 4; i < index - 3; i += 3) {
                            uint8_t sensor_id = data[i];
                            sensor_voltages[addr][sensor_id] = (data[i + 1] << 8) | data[i + 2];
                        }
                    }
                    frame_ready = true;
                } else {
                    Serial.println("CRC 校验失败!");
                }
            }
            index = 0;  // 重置缓冲区
        }
    }
}

// WiFi 初始化
void setup_wifi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
}

// MQTT 初始化
void setup_mqtt() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT...");
        if (mqttClient.connect(MQTT_CLIENT_ID)) {
            Serial.println("MQTT connected");
        } else {
            delay(5000);
        }
    }
}

// 发布传感器数据到MQTT
void publish_sensor_data(uint8_t machine_uid, uint8_t sensor_id, float voltage) {
    char topic[50], payload[20];
    snprintf(topic, sizeof(topic), "machines/%d/%d/value", machine_uid, sensor_id);
    snprintf(payload, sizeof(payload), "%.2f", voltage);
    mqttClient.publish(topic, payload);
}

// 定时器回调（修复状态判断错误）
void on_timer() {
    if (state == wait) {
        state = query;
    } else if (state == query) {
        state = upload;
    } else {
        state = wait;
    }
}

// 初始化
void setup() {
    Serial.begin(115200);
    //Serial.begin(115200, SERIAL_8N1, RXD_PIN,TXD_PIN );  // 初始化UART1
    pinMode(RS485_ENABLE_PIN, OUTPUT);
    rs485_enable_rx();

    setup_wifi();
    setup_mqtt();

    // 初始化定时器（每10秒切换状态）
    stateTicker.attach(10, on_timer);
}

void loop() {
    mqttClient.loop();
    parse_rs485_data();  // 持续解析数据

    if (state == query) {
        for (int addr = 0; addr < MAX_MACHINE; addr++) {
            frame_ready = false;
            send_rs485_frame(addr, 1, 0);
            while (!frame_ready) delay(10);  // 非阻塞等待
        }
    } else if (state == upload) {
        for (int addr = 0; addr < MAX_MACHINE; addr++) {
            for (int sensor_id = 0; sensor_id < MAX_SENSOR; sensor_id++) {
                float voltage = (float)sensor_voltages[addr][sensor_id] / 65536.0;
                publish_sensor_data(addr, sensor_id, voltage);
            }
        }
    }
}
