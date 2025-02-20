#include <SoftwareSerial.h>

// 定义软件串口的RX和TX引脚
const int rxPin = 7;  // RX引脚，连接到485模块的TXD
const int txPin = 8;  // TX引脚，连接到485模块的RXD

// 创建软件串口对象
SoftwareSerial mySerial(rxPin, txPin);

void setup() {
  // 初始化硬件串口，用于调试
  Serial.begin(115200);
  Serial.println("485 Slave Initialized");

  // 初始化软件串口，波特率设置为9600
  mySerial.begin(115200);
}

void loop() {
  // 检查软件串口是否有数据可读
  if (mySerial.available() > 0) {
    String receivedData = "";
    while (mySerial.available() > 0) {  // 读取所有可用的数据
      char c = mySerial.read();
      receivedData += c;  // 将接收到的字符追加到字符串
      delay(3);  // 等待下一个字符
    }

    // 如果接收到数据，将其发送回发送方
    if (receivedData.length() > 0) {
      mySerial.println(receivedData);  // 通过软件串口发送回数据
      Serial.println("Echo: " + receivedData);  // 通过硬件串口打印到调试监视器
    }
  }
}
