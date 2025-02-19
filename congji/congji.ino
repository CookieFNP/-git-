#include <SoftwareSerial.h>

// RS485配置
#define RS485_ENABLE_PIN 2    // RE和DE引脚接法
#define DEVICE_ADDRESS   0    // 本机设备地址

// 帧格式定义
#define FRAME_HEADER     0xAD
#define FRAME_FOOTER     0xFF
#define FRAME_TIMEOUT    50   // 帧接收超时(ms)
#define FRAME_MIN_LENGTH 7
uint8_t rxBuffer[32];        // 接收缓冲区
uint8_t txBuffer[32];        // 发送缓冲区

SoftwareSerial rs485(0, 1); // RX, TX (根据实际接线调整)

// 模拟传感器数据（实际使用时可替换为真实传感器读取）
uint16_t sensorValues[4] = { 
  analogRead(A0) * 64,      // 10bit转16bit模拟
  analogRead(A1) * 64,
  analogRead(A2) * 64,
  analogRead(A3) * 64
};

// Modbus CRC16计算
uint16_t crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// 设置RS485发送模式
void enableTX() {
  digitalWrite(RS485_ENABLE_PIN, HIGH);
  delayMicroseconds(50);
}

// 设置RS485接收模式
void enableRX() {
  delayMicroseconds(50);
  digitalWrite(RS485_ENABLE_PIN, LOW);
}

// 发送响应帧
void sendResponse() {
  uint8_t dataLen = 4 * 2; // 4个传感器 x 2字节
  txBuffer[0] = FRAME_HEADER;
  txBuffer[1] = DEVICE_ADDRESS;
  txBuffer[2] = 0x01;       // 功能码
  txBuffer[3] = dataLen;    // 数据长度
  
  // 填充传感器数据
  for(int i=0; i<4; i++){
    txBuffer[4 + i*2] = (sensorValues[i] >> 8) & 0xFF;
    txBuffer[5 + i*2] = sensorValues[i] & 0xFF;
  }
  
  // 计算CRC
  uint16_t crc = crc16(&txBuffer[1], 3 + dataLen);
  txBuffer[4 + dataLen] = crc & 0xFF;
  txBuffer[5 + dataLen] = crc >> 8;
  txBuffer[6 + dataLen] = FRAME_FOOTER;

  enableTX();
  rs485.write(txBuffer, 7 + dataLen);
  rs485.flush();
  enableRX();
}

// 处理接收数据
void processFrame() {
  static uint8_t state = 0;
  static uint8_t index = 0;
  static uint32_t lastByteTime = 0;
  
  while(rs485.available()){
    uint8_t c = rs485.read();
    uint32_t now = millis();
    
    // 超时处理
    if(now - lastByteTime > FRAME_TIMEOUT) index = 0;
    lastByteTime = now;

    // 有限状态机解析
    switch(state){
      case 0: // 等待帧头
        if(c == FRAME_HEADER){
          rxBuffer[index++] = c;
          state = 1;
        }
        break;
        
      case 1: // 接收数据
        rxBuffer[index++] = c;
        if(c == FRAME_FOOTER && index >= FRAME_MIN_LENGTH){
          // 验证地址
          if(rxBuffer[1] != DEVICE_ADDRESS) break;
          
          // 校验CRC
          uint16_t recvCRC = (rxBuffer[index-3] << 8) | rxBuffer[index-2];
          uint16_t calcCRC = crc16(&rxBuffer[1], index-4);
          
          if(recvCRC == calcCRC && rxBuffer[2] == 0x01){
            sendResponse(); // 发送响应
          }
          
          state = 0;
          index = 0;
        }
        break;
    }
    
    if(index >= sizeof(rxBuffer)) index = 0;
  }
}

void setup() {
  pinMode(RS485_ENABLE_PIN, OUTPUT);
  enableRX();
  
  // 初始化硬件串口用于调试
  Serial.begin(115200);
  
  // 初始化软件串口用于RS485
  rs485.begin(115200);
  
  Serial.println("RS485 Slave Ready");
}

void loop() {
  processFrame();
  
  // 更新模拟数据（示例）
  static uint32_t lastUpdate = 0;
  if(millis() - lastUpdate > 1000){
    for(int i=0; i<4; i++){
      sensorValues[i] = analogRead(i) * 64; // 模拟值放大
    }
    lastUpdate = millis();
  }
}
