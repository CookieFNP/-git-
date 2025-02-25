
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; // 创建ADS1115对象


// 自定义CRC16校验函数
uint16_t calculateCRC16(uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;  // 初始值
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;  // CRC-16-Modbus多项式
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 定义Modbus帧的结构
const int FRAME_HEADER = 0xAD;  // 字头
const int FRAME_TAIL = 0xAF;    // 字尾
const int DEVICE_ADDRESS = 0x01;  // 设备地址
const int FUNCTION_CODE = 0x03;  // 功能码（轮询功能码示例）
const int DATA_LENGTH = 0x00;    // 数据长度，轮询时为0

// 模拟的电压值（单位：V）
const float simulatedVoltages[] = {2.5, 3.0, 1.8, 2.2};  // 示例电压值
const int TRANSMITTER_COUNT = sizeof(simulatedVoltages) / sizeof(simulatedVoltages[0]);  // 变送器数量

// ADC相关参数
const float referenceVoltage = 3.3;  // ADC参考电压
const int adcResolution = 65535;  // 16位ADC分辨率


void setup() {
    // 初始化串口通信，波特率设置为 115200
    Serial.begin(115200);
    ads.begin(0x48);    // ADS1115默认地址0x48
    ads.setGain(GAIN_ONE); // 设置量程为±4.096V（适合1-5V输入）
}

void loop() {
    // 接收数据并解析
    if (Serial.available() > 0) {
        // 读取接收到的数据
        uint8_t receivedByte = Serial.read();

        // 检查是否为帧头
        static bool frameStart = false;
        static uint8_t frameBuffer[256];  // 帧缓冲区
        static int frameIndex = 0;

        if (receivedByte == FRAME_HEADER) {
            // 如果检测到帧头，重置缓冲区
            frameStart = true;
            frameIndex = 0;
            frameBuffer[frameIndex++] = receivedByte;
        } else if (frameStart) {
            // 如果正在接收帧，继续填充缓冲区
            frameBuffer[frameIndex++] = receivedByte;

            // 检查是否为帧尾
            if (receivedByte == FRAME_TAIL) {
                // 帧接收完成，解析帧
                frameStart = false;
                sendModbusResponse();///
                // 解析帧并验证CRC
                /*
                if (validateFrame(frameBuffer, frameIndex)) {
                    Serial.println("Frame received and validated successfully.");
                    // 发送响应帧
                    sendModbusResponse();
                } else {
                    Serial.println("Invalid frame received.");
                }
                */
            }
        }
    }

    // 等待一段时间
    delay(100);  // 短暂延时，避免过度占用CPU
}

// 验证接收到的Modbus帧
bool validateFrame(uint8_t* frame, int length) {
    // 确保帧长度至少包含头、地址、功能码、数据长度、CRC和尾
    if (length < 9) return false;

    // 提取帧的有效部分（不包括帧尾）
    int dataLength = length - 1;  // 排除帧尾
    uint16_t receivedCRC = (frame[dataLength - 1] << 8) | frame[dataLength - 2];
    uint16_t calculatedCRC = calculateCRC16(frame, dataLength - 2);  // 不包括CRC本身

    // 比较接收到的CRC和计算的CRC
    return (receivedCRC == calculatedCRC);
}
void sendModbusResponse() {
    uint8_t responseFrame[256];  // 响应帧缓冲区
    int responseLength = 0;

    // 构造响应帧
    responseFrame[responseLength++] = FRAME_HEADER;  // 字头
    responseFrame[responseLength++] = DEVICE_ADDRESS;  // 设备地址
    responseFrame[responseLength++] = FUNCTION_CODE;  // 功能码

    // 添加字节数（变送器数量 * 3）
    responseLength += 1;
    responseFrame[3] = TRANSMITTER_COUNT * 3;

    // 模拟电压值并转换为16位ADC值
    for (int i = 0; i < TRANSMITTER_COUNT; i++) {
        int16_t adcValue = ads.readADC_SingleEnded(i); // 读取第i个通道的ADC值

        // 调试输出
        Serial.print("Transmitter ");
        Serial.print(i + 1);
        Serial.print(": Voltage = ");
        Serial.print(voltage, 3);  // 保留3位小数
        Serial.print(" V -> ADC Value = ");
        Serial.println(adcValue);

        responseFrame[responseLength++] = i + 1;  // 变送器序号
        responseFrame[responseLength++] = adcValue >> 8;  // ADC值高字节
        responseFrame[responseLength++] = adcValue & 0xFF;  // ADC值低字节
    }

    

    // 计算CRC校验码
    uint16_t crc = calculateCRC16(responseFrame, responseLength);
    responseFrame[responseLength++] = crc & 0xFF;  // CRC低字节
    responseFrame[responseLength++] = (crc >> 8) & 0xFF;  // CRC高字节
    responseFrame[responseLength++] = FRAME_TAIL;  // 字尾

    // 发送响应帧
    Serial.write(responseFrame, responseLength);
    Serial.println("Response frame sent.");
}
