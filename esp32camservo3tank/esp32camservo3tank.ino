#include <driver/adc.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <math.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "index_html_gz.h"
//#include <Wire.h>
//#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//enable OTA for esp32cam: https://arduino.stackexchange.com/questions/75198/why-doesnt-ota-work-with-the-ai-thinker-esp32-cam-board

const char* ssid = "espcam";
const char* password = "leoncoolmoon";
 
long runtime = 0;
long runtimeDelay = 300;
boolean isrunning = false;
boolean ledon = false;
long teltime = 0;
long teltimeDelay = 2000;
boolean tlconnected = false;

int res = 4;
int aec = 204;
int ael =0;
int thr = 32;
int fix = 5;
int bal = 50; // left right balance
int pwr = 255;//key input power for running
int pwt = 255;//key input power for turning
int bri = 255;//led brightness
int bka =  757;//convert analog read to votage
int scr = 1;//screen on 1 ,screen off 0

#define I2C_SDA 15
#define I2C_SCL 14
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
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
#define LED               4
#define LED_C             9
#define r_LED             33
#define BATT              33

//#define PAN_TILT_ONLY
#define PAN_TILT_TANK
//#define PAN_TILT_CAR

//#if defined(PAN_TILT_ONLY)

#define P_SERVO     3 //PIN 1，3，12 can be used
#define T_SERVO     12
#define P_SERVO_C     3
#define T_SERVO_C     2
#define P_SERVO_D    45
#define T_SERVO_D    45
//#endif

#if defined(PAN_TILT_TANK)
#define WHEEL_L_A     13
#define WHEEL_L_B     15
#define WHEEL_R_A     2
#define WHEEL_R_B     14
#define WHEEL_L_A_C     5
#define WHEEL_L_B_C     6
#define WHEEL_R_A_C     7
#define WHEEL_R_B_C     8
#endif

#if defined(PAN_TILT_CAR)
#define FRONT_W     13
#define WHEEL_A     14
#define WHEEL_B     15
#define FRONT_W_C     6
#define WHEEL_A_C     7
#define WHEEL_B_C     8
#define FRONT_W_D     45
int wheelDir = 45;
#endif

#define sFreq     50
#define sResolution  16
#define wFreq     8000
#define wResolution  8

//TwoWire i2c = TwoWire(0);

camera_fb_t * fb = NULL;

WebSocketsServer webSocket(82);
//WebsocketsServer WSserver;
AsyncWebServer webserver(80);
//Adafruit_SSD1306 oled;
// Arduino like analogWrite
// value has to be between 0 and valueMax
void pwmOutPut(uint8_t channel, uint32_t value, uint32_t valueMax = 180) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
  /*
    1. First, you need to choose a PWM channel. There are 16 channels from 0 to 15.
    2. Then, you need to set the PWM signal frequency. For an LED, a frequency of 5000 Hz is fine to use.
    3. You also need to set the signal’s duty cycle resolution: you have resolutions from 1 to 16 bits.  We’ll use 8-bit resolution, which means you can control the LED brightness using a value from 0 to 255.
    4. Next, you need to specify to which GPIO or GPIOs the signal will appear upon. For that you’ll use the following function:
    "ledcAttachPin(GPIO, channel)"
    This function accepts two arguments. The first is the GPIO that will output the signal, and the second is the channel that will generate the signal.
    5. Finally, to control the LED brightness using PWM, you use the following function:
    "ledcWrite(channel, dutycycle)"
    This function accepts as arguments the channel that is generating the PWM signal, and the duty cycle
  */
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  //i2c.begin(I2C_SDA, I2C_SCL);
  //oled = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &i2c);
   //if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
   // Serial.println(F("SSD1306 allocation failed"));
   // for(;;); // Don't proceed, loop forever
  //}



  /*//station mode
   * 
   * WiFi.mode(WIFI_STA);
   * WiFi.begin(ssid, password);
   */
  //softap mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
   String ip = WiFi.localIP().toString();  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.print("IP address: ");
  Serial.println(ip);
  //oled.clearDisplay();
  //oled.setTextColor(WHITE);
  //oled.setCursor(1, 10);
  //oled.println("IP:"+ip);
  //oled.display();

  ArduinoOTA.setHostname("esp32-tank");
  ArduinoOTA.setPassword(password);
   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("ATO ready!");
  //oled.setCursor(1, 21);
  //oled.println("ATO ready!");
  //oled.display();
  
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
 if (err != ESP_OK) {
   Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  //s->set_vflip(s, 1);//flip it back
  //s->set_brightness(s, 1);//up the blightness just a bit
  //s->set_saturation(s, -2);//lower the saturation

  //oled.fillRect(1, 21, 128, 32, BLACK);
  //oled.display(); 
  //oled.setCursor(1, 21);
  //oled.println("Camera ready!");
  //oled.display();

 //LED
  pinMode(r_LED, OUTPUT);
  pinMode(LED, OUTPUT);
  ledcSetup(LED,  wFreq, wResolution);
  ledcAttachPin(LED, LED_C);

  // Ai-Thinker: pins 2 and 16 pan_tilt
   
  //servo1
  pinMode(P_SERVO, OUTPUT);
  ledcSetup(P_SERVO_C, sFreq, sResolution); //channel, freq, resolution
  ledcAttachPin(P_SERVO, P_SERVO_C); // pin, channel
//  //servo2  pin 16 crash
  pinMode(T_SERVO, OUTPUT);
  ledcSetup(T_SERVO_C, sFreq, sResolution);
  ledcAttachPin(T_SERVO, T_SERVO_C);


  
#if defined(PAN_TILT_CAR)
  pinMode(FRONT_W, OUTPUT);
  ledcSetup(FRONT_W_C,  sFreq, sResolution);
  ledcAttachPin(FRONT_W, FRONT_W_C);
  ledcWrite(FRONT_W_C, FRONT_W_D);
  pinMode(WHEEL_A, OUTPUT);
  pinMode(WHEEL_B, OUTPUT);
  ledcSetup(WHEEL_A_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_A, WHEEL_A_C); // pin, channel
  ledcSetup(WHEEL_B_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_B, WHEEL_B_C); // pin, channel
  ledcWrite(WHEEL_A_C, 0);
  ledcWrite(WHEEL_B_C, 0);
#endif

#if defined(PAN_TILT_TANK)
  Serial.println("setup pwm pins");
  pinMode(WHEEL_L_A, OUTPUT);
  pinMode(WHEEL_L_B, OUTPUT);
  pinMode(WHEEL_R_A, OUTPUT);
  pinMode(WHEEL_R_B, OUTPUT);

  ledcSetup(WHEEL_L_A_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_L_A, WHEEL_L_A_C); // pin, channel
  ledcSetup(WHEEL_L_B_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_L_B, WHEEL_L_B_C); // pin, channel
  ledcSetup(WHEEL_R_A_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_R_A, WHEEL_R_A_C); // pin, channel
  ledcSetup(WHEEL_R_B_C, wFreq, wResolution); //channel, freq, resolution
  ledcAttachPin(WHEEL_R_B, WHEEL_R_B_C); // pin, channel
  ledcWrite(WHEEL_L_A_C, 0);
  ledcWrite(WHEEL_L_B_C, 0);
  ledcWrite(WHEEL_R_A_C, 0);
  ledcWrite(WHEEL_R_B_C, 0);
#endif

  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
#if defined(PAN_TILT_CAR)
    ledcWrite(WHEEL_A_C, 0);
    ledcWrite(WHEEL_B_C, 0);
    pwmOutPut(FRONT_W_C, FRONT_W_D);
#endif
#if defined(PAN_TILT_TANK)
    ledcWrite(WHEEL_L_A_C, 0);
    ledcWrite(WHEEL_L_B_C, 0);
    ledcWrite(WHEEL_R_A_C, 0);
    ledcWrite(WHEEL_R_B_C, 0);
#endif
    // #if defined(PAN_TILT_ONLY)
    pwmOutPut(P_SERVO_C, P_SERVO_D); // channel, value
    pwmOutPut(T_SERVO_C, T_SERVO_D);
    //  #endif
  });
  //oled.fillRect(1, 21, 128, 32, BLACK);
  //oled.display(); 
  //oled.setCursor(1, 21);
  //oled.println("LED+motor ready!");
  //oled.display();
  
  webserver.begin();
  webSocket.begin();
  //oled.fillRect(1, 21, 128, 32, BLACK);
  //oled.display(); 
  //oled.setCursor(1, 21);
  //oled.println("Web UI ready!");
  //oled.display();
  webSocket.onEvent(webSocketEvent);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
    tlconnected = false;
    pinMode(r_LED, OUTPUT);
      break;
    case WStype_CONNECTED:               // if a new websocket connection is established
    pinMode(BATT, INPUT);
    tlconnected = true;
    teltime = millis();
      break;
    case WStype_TEXT:  {                   // if new text data is received
        String msg((char*)payload);
        int commaIndex = msg.indexOf(',');
        int semIndex = msg.indexOf(';');

        if (commaIndex != -1) {
          int panValue = msg.substring(0, commaIndex).toInt();
          int tiltValue = msg.substring(commaIndex + 1).toInt();
          panValue = map(panValue, -255, 255, 0, 180); // 0-180
          tiltValue = map(tiltValue, -255, 255, 0, 180); // 0-180 reversed
          Serial.print("p: ");
          Serial.print(panValue);
          Serial.print("t: ");
          Serial.println(tiltValue);
          pwmOutPut(P_SERVO_C, panValue); // channel, value
          pwmOutPut(T_SERVO_C, tiltValue);
        } else if (semIndex != -1) {
          int panValue = msg.substring(0, semIndex).toInt();
          int tiltValue = msg.substring(semIndex + 1).toInt();
#if defined(PAN_TILT_TANK)
          //panValue = map(panValue, -90, 90, -255 , 255); // -255~255
          //tiltValue = map(tiltValue, -90, 90,  -255 , 255); //
          Serial.print("Tx: ");
          Serial.print(panValue);
          Serial.print("    -Ty: ");
          Serial.println(tiltValue);
          wheel(panValue, tiltValue); // x, y
#endif
#if defined(PAN_TILT_CAR)
          //tiltValue = map(tiltValue, -90, 90, -255, 255); // 0-255
          panValue = 90 - map(panValue, -255, 255, 0, 90); //
          Serial.print("Cx: ");//-68>0>89,^
          Serial.print(tiltValue);
          Serial.print("    -Cy: ");
          Serial.println(panValue);//65>90>125  <->
          wheel(tiltValue); // x,
          wheelDir = panValue;
          pwmOutPut(FRONT_W_C, wheelDir);
#endif
        } else  if (!msg.compareTo( "reflash")) {
          fb = esp_camera_fb_get();
          Serial.print(">");
          webSocket.sendBIN(num, (const uint8_t *)fb->buf, fb->len);
          esp_camera_fb_return(fb);
          fb = NULL;
        } else if (!msg.compareTo("ledoff")) {
          Serial.println("0");
          ledon = false;
          ledcWrite(LED_C, 0);
        } else if (!msg.compareTo( "ledon")) {
          ledon = true;
          Serial.println(bri);
          ledcWrite(LED_C, bri);
        } else if (msg.startsWith("thr:")) {
          thr = msg.substring(4).toInt();
          Serial.print("thr:");
          Serial.println(thr);
        }else if (msg.startsWith("fix:")) {
          fix = msg.substring(4).toInt();
          Serial.print("fix:");
          Serial.println(fix);
        }else if (msg.startsWith("bal:")) {
          bal = msg.substring(4).toInt();
          Serial.print("bal:");
          Serial.println(bal);
        }else if (msg.startsWith("pwr:")) {
          pwr = msg.substring(4).toInt()*2.55;
          Serial.print("pwr:");
          Serial.println(pwr);
        }else if (msg.startsWith("pwt:")) {
          pwt = msg.substring(4).toInt()*2.55;
          Serial.print("pwt:");
          Serial.println(pwt);
        }else if (msg.startsWith("bri:")) {
          bri = msg.substring(4).toInt();
          Serial.print("bri:");
          Serial.println(bri);
          ledcWrite(LED_C, bri);
        }else if (msg.startsWith("bka:")) {
          bka = msg.substring(4).toInt();
          Serial.print("bka:");
          Serial.println(bka);
        }else if (msg.startsWith("res:")) {
          res = msg.substring(4).toInt();
          Serial.print("res:");
          Serial.println(res);
            sensor_t * s = esp_camera_sensor_get();
            s->set_framesize(s, (framesize_t)res);
        }else if (msg.startsWith("aec:")) {
            aec = msg.substring(4).toInt();
          Serial.print("aec:");
          Serial.println(aec);
            sensor_t * s = esp_camera_sensor_get();
            s->set_exposure_ctrl(s, aec);
        }else if (msg.startsWith("ael:")) {
           ael = msg.substring(4).toInt();
          Serial.print("ael:");
          Serial.println(ael);
            sensor_t * s = esp_camera_sensor_get();
            s->set_ae_level(s, ael);
        } else  if (msg.startsWith("scr:")) {
          scr = msg.substring(4).toInt();
          Serial.print("scr:");
          Serial.println(scr);
          //0:turn off screen,1:turn on screen
            // oled.ssd1306_command(scr == 0?SSD1306_DISPLAYOFF:SSD1306_DISPLAYON);      
        }else if(msg.startsWith("key:")){   
          int keyCode = msg.substring(4).toInt();
          Serial.print("keyCode:");
          Serial.println(keyCode);
          keyMove(keyCode); 
        }else if(msg.startsWith("ser:")){   
          int servo = msg.substring(4).toInt()+90;
          Serial.print("angle:");
          Serial.println(servo);
          Serial.print("servo:");
          Serial.println(T_SERVO);
          Serial.print("servo chennal:");
          Serial.println(T_SERVO_C);
          pwmOutPut(T_SERVO_C, servo);
        } else  if (!msg.compareTo( "greetings")) {
          //"&sig=&bat=&dir="
          //res, bri, ledon,thr,fix,bal,aec,ael,bka,scr...
          String initx=String("&res=")+res+"&bri="+bri+"&ledon="+(ledon?"1":"0")+"&thr="+thr+"&fix="+fix+"&bal="+bal+"&aec="+aec+"&ael="+ael+"&bka="+bka+"&scr="+scr;
          webSocket.broadcastTXT(initx);
          }
      }
      break;
    case  WStype_ERROR:
      break;
  }
}

#if defined(PAN_TILT_CAR)
void wheel(int x) {//-255~255
  if (abs(x) > thr ) {
    wheelg(x);
    isrunning = true;
    runtime = millis();
  } else {
    wheelg(0);
    isrunning = false;
  }
}

void wheelg(int s) {
  if (s == 0) {
    ledcWrite(WHEEL_A_C, 0);
    ledcWrite(WHEEL_B_C, 0);
  } else if (s > 0) {
    ledcWrite(WHEEL_A_C, 0);
    ledcWrite(WHEEL_B_C, s);
  } else if (s < 0) {
    ledcWrite(WHEEL_A_C, s);
    ledcWrite(WHEEL_B_C, 0);
  }
}

#endif
#if defined(PAN_TILT_TANK)
void wheel(int x, int y) { //x|y -255~255
  double a, k, ss, dx, dy;
  ss = sqrt(x * x + y * y);
  dx = x;
  dy = y;
  Serial.print("ss:");
  Serial.println(ss);
  if (ss > thr) {
    if (abs(y) < fix) {
      if (x = 0) {
        wheelMove(0, true);
        wheelMove(0, false);
        isrunning = false;
      } else {
        wheelMove(ss / 2, (x > 0));
        wheelMove(0, (x < 0));
        isrunning = true;
        runtime = millis();
      }
    } else {//y<-5||y>5 fixed area
      Serial.print("x: ");
      Serial.print(dx);
      Serial.print("    -y:");
      Serial.print(dy);
      a = (atan(dx / dy));  
      k = abs(x)<fix? 1 :((1.5708 - abs(a)) / (1.5708 + abs(a)));
      Serial.print("    -prek:");
      Serial.println(k);
      k = k*(bal+50)/100;
      //0.7854 = 45
      //0.5236 = 30
      Serial.print("    -a: ");
      Serial.print(a);
      Serial.print("    -k:");
      Serial.println(k);
      wheelMove((-y) / abs(y)*ss * k, (a > 0));
      wheelMove((-y) / abs(y)*ss, (a < 0));
      isrunning = true;
      runtime = millis();
    }
  } else {
    wheelMove(0, true);
    wheelMove(0, false);
    isrunning = false;
  }
}

void wheelMove(int mSpeed, boolean rSide) {
  Serial.print("speed: ");
  Serial.print(mSpeed);
  Serial.print("   -side: ");
  Serial.println(rSide ? "right" : "left");
  if (abs(mSpeed) <= 255 && mSpeed != 0) {
    Serial.print("pinA: ");
    Serial.print(max(mSpeed, 0));
    Serial.print("   -pinB: ");
    Serial.println(abs(min(mSpeed, 0)));
    if (rSide) {
      ledcWrite(WHEEL_R_A_C, max(mSpeed, 0));
      ledcWrite(WHEEL_R_B_C, abs(min(mSpeed, 0)));
    } else {
      ledcWrite(WHEEL_L_A_C, max(mSpeed, 0));
      ledcWrite(WHEEL_L_B_C, abs(min(mSpeed, 0)));
    }
  } else {
    if (rSide) {
      ledcWrite(WHEEL_R_A_C, 0);
      ledcWrite(WHEEL_R_B_C, 0);
    } else {
      ledcWrite(WHEEL_L_A_C, 0);
      ledcWrite(WHEEL_L_B_C, 0);
    }
  }
}

#endif

String getRSS(){
  long rssi = WiFi.RSSI();
  char buf[16];
  String rt=ltoa(rssi,buf,10);
  Serial.print("RSSI: ");
  Serial.println(rt);
  return rt;
  }
String getBAT(){
  adc1_config_width(ADC_WIDTH_BIT_10);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11); 
  String rt=String(analogRead(BATT)/bka, DEC);//0-4095 0-5v
  Serial.print("BATTERY: ");
  Serial.println(rt);
  return rt;
  }
String getDIR(){
return "0";
}
void keyMove(int keyCode){
#if defined(PAN_TILT_CAR) 
 if(keyCode == 39){
  wheelDir=wheelDir<60?wheelDir+30:90;
   pwmOutPut(FRONT_W_C, wheelDir);
    //ws.send("key:rt");
    } else if(keyCode==38){
      wheel(64);
      //ws.send("key:up");
      } else if(keyCode==37){
  wheelDir=wheelDir>30?wheelDir-30:0;
   pwmOutPut(FRONT_W_C, wheelDir);
        //ws.send("key:lt");
        } else if(keyCode==40){
  wheel(-64);
          //ws.send("key:dw");
         }else if(keyCode==0){  
  wheelDir = FRONT_W_D;      
pwmOutPut(FRONT_W_C, wheelDir);
wheel(0);
          //ws.send("key:stop");
         }
#endif
#if defined(PAN_TILT_TANK)  
  if(keyCode == 39){
    wheelMove(pwt, true);
    wheelMove(0, false);
    //ws.send("key:rt");
    } else if(keyCode==38){
    wheelMove(pwr, true);
    wheelMove(pwr, false);
      //ws.send("key:up");
      } else if(keyCode==37){
    wheelMove(0, true);
    wheelMove(pwt, false);
        //ws.send("key:lt");
        } else if(keyCode==40){
    wheelMove(-1*pwr, true);
    wheelMove(-1*pwr, false);
          //ws.send("key:dw");
         }else if(keyCode==0){
    wheelMove(0, true);
    wheelMove(0, false);
          //ws.send("key:stop");
         }
 #endif

  }

void loop() {
  long t = millis();
  if(tlconnected){
   // pinMode(BATT, INPUT);
    //digitalWrite(r_LED, HIGH);
   if((t - teltime > teltimeDelay)){
    //update telemtry
    teltime = t;
    
   String tlmtx= "S:"+getRSS()+"db |B:"+getBAT();

  //oled.fillRect(1, 21, 128, 32, BLACK);
  //oled.display(); 
  //oled.setCursor(1, 21);
  //oled.println(tlmtx+"V");//ip.c_str()
  //oled.display(); 
     tlmtx=tlmtx+"&dir="+getDIR();
     tlmtx.replace("S:","&sig=");
     tlmtx.replace("db |B:","&bat=");
     
    //"&sig=&bat=&dir="
     webSocket.broadcastTXT(tlmtx);
    }
    digitalWrite(r_LED, LOW);
      }
  
  if (isrunning && (t - runtime > runtimeDelay)) {
    runtime = t;
    isrunning = false;
    Serial.println("auto stop");
#if defined(PAN_TILT_CAR)
    wheelg(0);
#endif
#if defined(PAN_TILT_TANK)
    wheel(0, 0);
#endif
  }
  webSocket.loop();//need to know if it is blocking
   ArduinoOTA.handle();
}
