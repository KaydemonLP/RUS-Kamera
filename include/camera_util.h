#pragma once

#include <vector>

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

void cameraSetup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}


float takePic( void ) 
{
  // Dispose first pictures because of bad quality
  camera_fb_t* fb = NULL;
  // Skip first 3 frames (increase/decrease number as needed).
  for (int i = 0; i < 3; i++) {
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    fb = NULL;
  }
    
  // Take a new photo
  fb = NULL;  
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }  

  uint8_t *outbuf = new uint8_t[fb->width * fb->height * 3];

  fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, outbuf);

  std::vector<std::vector<bool>> plate_shape;
  plate_shape.push_back(std::vector<bool>());

  int xCoord = 0;
  int yCoord = 0;

  for( int iPixel = 0; iPixel < fb->width * fb->height * 3; iPixel += 3 )
  {
    //Serial.printf( "R: %d G: %d B: %d\n", outbuf[iPixel], outbuf[iPixel+1], outbuf[iPixel+2] );
    if( IsPixelWhite( outbuf[iPixel], outbuf[iPixel+1], outbuf[iPixel+2] ) )
    {
      plate_shape[yCoord].push_back(true);
    }
    else
    {
      //Serial.println("Pixel is not white"); 
      plate_shape[yCoord].push_back(false);
    }

    xCoord++;
    if( xCoord >= fb->width )
    {
      xCoord = 0;
      yCoord++;
      plate_shape.push_back(std::vector<bool>());
    }
  }

  String mask;

  int iWhitePercentage = 0;

  for( int x = 0; x < fb->height; x++ )
  {
    for( int y = 0; y < fb->width; y++ )
    {
      mask += plate_shape[x][y] ? "1" : "0";
      if( plate_shape[x][y] )
        iWhitePercentage++;
    }
    mask += '\n';
  }

  plate_shape.clear();

  float flPercentage = (float)(iWhitePercentage) / ((float)(fb->width*fb->height));


  esp_camera_fb_return(fb);

  return flPercentage;
}