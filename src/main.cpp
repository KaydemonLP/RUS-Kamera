#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "soc/rtc_cntl_reg.h"
#include <esp_camera.h>
#include "color_util.h"
#include "camera_util.h"
#include "img_converters.h"
#include <Wire.h>
#include <mutex>

// ESP32 CAM
#define SDA0_Pin 14
#define SCL0_Pin 15

#define I2C_Freq 100000
#define I2C_DEV_ADDR 0x10


typedef struct
{
  int requestCount;
  byte command;
} __attribute__((packed)) req;

void onReceive(int size);
void onRequest();

req requestCommand;

// ESP NOW
const uint8_t g_iRecieverAdress[] = {0xE0, 0xE2, 0xE6, 0xB0, 0x06, 0x84};

esp_now_peer_info_t g_peerInfo; // information about the peer

typedef struct struct_message {
    int m_iCommand; // this variable's value is sent to the reciever
} struct_message;

struct_message g_messageToSend;
// ESP NOW

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool g_bScanButtonPressed = false;

void setup() 
{
  Serial.begin(115200);
  while(!Serial)
    delay(1);


  if(!Wire.begin((uint8_t)I2C_DEV_ADDR, SDA0_Pin, SCL0_Pin, I2C_Freq))
  {
    Serial.println("I2C Wire Error. Going idle.");
    while(true)
      delay(1);
  }
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest); 

  cameraSetup();
}

#define SCANS_NEEDED 4

bool g_Scanning = false;
int g_iScanCount = 0;
float g_flWastePercentage = 0;

float g_flLastWastePercentage = -1;

void ResetScan()
{
  g_flLastWastePercentage = g_flWastePercentage;
  g_Scanning = false;
  g_iScanCount = 0;
  g_flWastePercentage = 0;
}

void loop() 
{
  if( g_bScanButtonPressed )
  {
      g_Scanning = true;
      g_bScanButtonPressed = false;
  }

  if( g_Scanning )
  {
    Serial.printf("Taking pic %d\n", g_iScanCount+1);
    g_flWastePercentage += takePic();

    Serial.println(g_flWastePercentage);

    g_iScanCount++;
    if( g_iScanCount >= SCANS_NEEDED )
    {
      g_flWastePercentage /= (float) SCANS_NEEDED;

      Serial.printf("Final result: %f\n", g_flWastePercentage);

      ResetScan();
    }
  }
}

void onReceive(int size)
{
  if(size==sizeof(req))
    Wire.readBytes((char *)&requestCommand, sizeof(req));
}

enum {
  CMD_IS_SCANNING = 2,
  CMD_START_SCAN,
  CMD_GET_RESULT
};

void onRequest()
{
  switch(requestCommand.command)
  {
    case CMD_IS_SCANNING: 
    {
      Wire.write((uint8_t *)&g_Scanning,sizeof(bool));
      break;
    }
    case CMD_START_SCAN:
    g_bScanButtonPressed = true;
    break;
    case CMD_GET_RESULT: 
    {
      Wire.write((uint8_t *)&g_flLastWastePercentage,sizeof(float));
      g_flLastWastePercentage = -1;
      break;
    }
    //case 5: Wire.write((uint8_t *)registers.reg5,5); break;
    default: Wire.write(0); //single byte for dummy
  }
}