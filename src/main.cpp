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
//#define LED_PIN 5 //ESP32
#define FLASH_PIN 4 //ESP32 CAM Flash
#define LED_PIN 33 //ESP32 CAM LED

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

#define START_SCAN_BUTTON 4

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

// Set device as a Wi-Fi Station
/*  WiFi.mode(WIFI_STA);

  // initialise ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(g_peerInfo.peer_addr, g_iRecieverAdress, 6);
  g_peerInfo.channel = 0;
  g_peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&g_peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
*/
  pinMode( 12, OUTPUT );
  pinMode( 13, OUTPUT );

  cameraSetup();
  //attachInterrupt(START_SCAN_BUTTON, scanButtonInterrupt, RISING);
}

#define SCANS_NEEDED 4

bool g_Scanning = false;
int g_iScanCount = 0;
float g_flWastePercentage = 0;

float g_flLastWastePercentage = 0;

void ResetScan()
{
  g_flLastWastePercentage = g_flWastePercentage;
  g_Scanning = false;
  g_iScanCount = 0;
  g_flWastePercentage = 0;
}

void loop() 
{
  //g_messageToSend.m_iCommand = 40;
  //esp_err_t result = esp_now_send(g_iRecieverAdress, (uint8_t *) &g_messageToSend, sizeof(g_messageToSend));

  if( g_bScanButtonPressed )
  {
      g_Scanning = true;
      g_bScanButtonPressed = false;
  }

  if( g_Scanning )
  {
    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
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
  else
  {
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
  }
  delay(10);
}

void onReceive(int size)
{
  if(size==sizeof(req))
    Wire.readBytes((char *)&requestCommand, sizeof(req));
}

void onRequest()
{
  switch(requestCommand.command)
  {
    case 3:
    g_bScanButtonPressed = true;
    break;
    case 4: 
    {
      Wire.write((uint8_t *)&g_flLastWastePercentage,sizeof(float)); 
      break;
    }
    //case 5: Wire.write((uint8_t *)registers.reg5,5); break;
    default: Wire.write(0); //single byte for dummy
  }
}