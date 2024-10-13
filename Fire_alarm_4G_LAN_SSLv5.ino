#include <Arduino.h>
#include <Stream.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESP32Time.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ESP_SSLClient.h>
#include <EthernetUdp.h>

#define POW_pin 10  //4G Power Enable
#define SLP_pin 11  //4G Sleep pin
#define PWR_pin 12  //4G PWR pin
#define NTX_pin 13  //4G UART pin
#define NRX_pin 14  //4G UART pin

#define ETH_PHY_CS 15   //  15
#define ETH_PHY_IRQ 16  //  16
#define ETH_PHY_RST 5   //  5
#define ETH_SPI_SCK 7   //  7
#define ETH_SPI_MISO 4  //  4
#define ETH_SPI_MOSI 6  //  6

#define faultAlarm_pin 17
#define fireAlarm_pin 18

#define fireAlarm_led 42    //Red LED
#define faultAlarm_led 41   //Yellow LED
#define netlight_led 40     //Red LED
#define userLed_pin 39      //Blink LED
#define userTest_button 38  //User Test Button

bool isFirstreport = false;
bool isupdatetime = false;
bool isFirst = false;
bool pLock = false;
bool ledState = false;
bool isModemready = false;
bool systemfault = false;
bool isModemfound = false;
bool isLanready = false;
bool stateChange = false;
bool isSendsuccess = true;
bool startSenddata = false;
bool startHttpservice = false;
bool dataStore_checkList[5] = { false, false, false, false, false };

int count = 0;
int faultAlarm = 0;
int fireAlarm = 0;
int oldfaultAlarm = 0;
int oldfireAlarm = 0;
int respondlength = 0;
int cmm_state = 0;
int dataIndex = 0;
int sendIndex = 0;
int failSendcount_4G = 0;
int failSendcount_LAN = 0;
bool dataStore_priority[5] = { 0, 0, 0, 0, 0 };

uint8_t eth_MAC[] = { 0x00, 0x4d, 0x45, 0x00, 0x00, 0x01 };

const uint16_t localPort = 55432;          // Local port for UDP packets.
const char timeServer[] = "pool.ntp.org";  // Default NTP server pool.
const int NTP_PACKET_SIZE = 48;            // NTP time stamp is in the first 48 bytes of the message.
byte packetBuffer[NTP_PACKET_SIZE];        // Buffer for both incoming and outgoing packets.

uint32_t chipId = 0;

const int rand_pin = A0;

unsigned long dataSendtime = 0;
const long dataSendtimeout = 10000;

unsigned long loopTime = 0;
const long loopTimeout = 10000;

unsigned long lanchk_previousMillis = 0;
const long lanchk_interval = 1000;

unsigned long healthReportpreviousMillis = 0;
const long healthReportinterval = 300000;

unsigned long event_previousMillis = 0;
const long event_interval = 180000;

const long syncTimeinterval = 3678000LL;
unsigned long syncTimepreviousMillis = 0;

unsigned long uled_previousMillis = 0;
const long uled_interval = 250;

unsigned long PacketCount = 0;

String deviceID = "12EF2535A0";
String preData = "";
String dataStore[5] = { "", "", "", "", "" };

// String serverURL = "node52787-pladaw.proen.app.ruk-com.cloud";
// String serverPath = "/data_handler.php";

String serverURL = "node61107-fire-alarm-monitoring.proen.app.ruk-com.cloud";
String serverPath = "/api/v1/event";

Stream *_Serial;
HardwareSerial modem_serial(1);
ESP32Time rtc;

EthernetUDP Udp;

ESP_SSLClient ssl_client;

EthernetClient basic_client;

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  deviceID = String(chipId);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  Com_initial();
  IO_initial();

  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    0);
  delay(100);
  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    10000,
    NULL,
    1,
    &Task2,
    1);
  delay(100);

  resetA7682module();
  resetW5500module();

  Serial.println("Starting ETHERNET connection...");
  if (Ethernet.begin(eth_MAC) == 1) {
    Ethernet.setRetransmissionTimeout(50);  // set the Ethernet controller's timeout to 50 ms
    isLanready = true;
    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());
  }
  ssl_client.setInsecure();

  // Set the receive and transmit buffers size in bytes for memory allocation (512 to 16384).
  ssl_client.setBufferSizes(1024 /* rx */, 512 /* tx */);

  /** Call setDebugLevel(level) to set the debug
     * esp_ssl_debug_none = 0
     * esp_ssl_debug_error = 1
     * esp_ssl_debug_warn = 2
     * esp_ssl_debug_info = 3
     * esp_ssl_debug_dump = 4
     */
  ssl_client.setDebugLevel(0);

  // Assign the basic client
  // Due to the basic_client pointer is assigned, to avoid dangling pointer, basic_client should be existed
  // as long as it was used by ssl_client for transportation.
  ssl_client.setClient(&basic_client);

  Udp.begin(localPort);
  
  if (isLanready == true) {
    if (isupdatetime == false) {
      isupdatetime = true;
      updateTime();
      if (isFirstreport == false) {
        isFirstreport = true;
        sendNormaldata();
      }
    }
  }
  Serial.println("Device ID : " + deviceID);
}
void Task1code(void *pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    if (digitalRead(userTest_button) == 0) {
      digitalWrite(faultAlarm_led, HIGH);
      digitalWrite(fireAlarm_led, HIGH);
      while (digitalRead(userTest_button) == 0) delay(5);
      sendEventdata(-99, -99);
      digitalWrite(faultAlarm_led, LOW);
      digitalWrite(fireAlarm_led, LOW);
    }
    if (digitalRead(faultAlarm_pin) == false) {
      faultAlarm = 1;
    } else if (digitalRead(faultAlarm_pin) == true) {
      faultAlarm = 0;
    }
    if (digitalRead(fireAlarm_pin) == false) {
      fireAlarm = 1;
    } else if (digitalRead(fireAlarm_pin) == true) {
      fireAlarm = 0;
    }

    if (oldfaultAlarm != faultAlarm) {
      oldfaultAlarm = faultAlarm;
      isFirst = true;
      stateChange = true;
    }
    if (oldfireAlarm != fireAlarm) {
      oldfireAlarm = fireAlarm;
      isFirst = true;
      stateChange = true;
    }
    if (stateChange == true and (faultAlarm != 0 or fireAlarm != 0)) {  //faultAlarm != 0 or fireAlarm != 0
      if (isFirst == true) {
        isFirst = false;
        healthReportpreviousMillis = millis();
        syncTimepreviousMillis = millis();
        event_previousMillis = millis();
        //Serial.println("==== F =====");
        sendEventdata(faultAlarm, fireAlarm);
      } else {
        unsigned long event_currentMillis = millis();
        if (fabs(event_currentMillis - event_previousMillis) >= event_interval) {
          event_previousMillis = event_currentMillis;
          healthReportpreviousMillis = millis();
          syncTimepreviousMillis = millis();
          //Serial.println("==== S =====");
          sendEventdata(faultAlarm, fireAlarm);
        }
      }
    } else if (stateChange == true and (faultAlarm == 0 and fireAlarm == 0)) {
      stateChange = false;
      sendEventdata(0, 0);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void Task2code(void *pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    unsigned long healthReportcurrentMillis = millis();
    if (fabs(healthReportcurrentMillis - healthReportpreviousMillis) >= healthReportinterval) {
      healthReportpreviousMillis = healthReportcurrentMillis;
      sendNormaldata();
    }
    unsigned long syncTimecurrentMillis = millis();
    if (fabs(syncTimecurrentMillis - syncTimepreviousMillis) >= syncTimeinterval) {
      syncTimepreviousMillis = syncTimecurrentMillis;
      if (isLanready == true) {
        updateTime();
      } else if (isModemready == true) {
        syncTime();
      }
    }
    if (isFirstreport == true and fabs(millis() - loopTime) >= loopTimeout) {
      ESP.restart();
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void loop() {
  loopTime = millis();
  if (pLock == false and _Serial->available()) {
    while (_Serial->available() > 0) {
      String input = _Serial->readStringUntil('\n');
      //Serial.println(input);
      if (input.indexOf("PB DONE") != -1) {
        setNBIOT_echoOff();
        isModemready = true;
        systemfault = false;
        failSendcount_4G = 0;
        failSendcount_LAN = 0;
        syncTime();
        Serial.println("Module Ready!!!");
        if (isFirstreport == false) {
          isFirstreport = true;
          sendNormaldata();
        }
      }
      if (input.indexOf("+CPIN: SIM REMOVED") != -1) {
        systemfault = true;
        isModemready = false;
        Serial.println("No Sim install!!!");
      }
      if (input.indexOf("+HTTPACTION: 1,200") != -1) {
        isSendsuccess = true;
        startSenddata = false;
        failSendcount_4G = 0;
        failSendcount_LAN = 0;
        dataStore_checkList[sendIndex] = false;
        respondlength = input.substring(19).toInt();
        //Serial.println("Respond count : " + String(respondCount));
        Serial.println("Respond data : " + get_HTTPresponse(respondlength));
        //Serial.println("Send data success!!!");
      } else {
        isSendsuccess = false;
      }
    }
  }
  unsigned long lanchk_currentMillis = millis();
  if (fabs(lanchk_currentMillis - lanchk_previousMillis) >= lanchk_interval) {
    lanchk_previousMillis = lanchk_currentMillis;
    if ((Ethernet.hardwareStatus() == EthernetNoHardware) || (Ethernet.linkStatus() == LinkOFF)) {
      isLanready = false;
    } else {
      if (isLanready == false and Ethernet.begin(eth_MAC) == 1) {
        Ethernet.setRetransmissionTimeout(50);  // set the Ethernet controller's timeout to 50 ms
        if (isupdatetime == false) {
          isupdatetime = true;
          updateTime();
          if (isFirstreport == false) {
            isFirstreport = true;
            sendNormaldata();
          }
          failSendcount_4G = 0;
          failSendcount_LAN = 0;
        }
        Serial.print("Ethernet IP is: ");
        Serial.println(Ethernet.localIP());
        isLanready = true;
      }
    }
  }
  if (dataStore_checkList[0] || dataStore_checkList[1] || dataStore_checkList[2] || dataStore_checkList[3] || dataStore_checkList[4]) {
    for (int i = 0; i < 5; i++) {
      if (dataStore_checkList[i] == true) {
        if (count >= 3 and isModemready == true and startSenddata == false) {
          count = 0;
          Serial.println("Send data using 4G...");
          sendVia4G(dataStore[i]);
          sendIndex = i;
        } else if (count >= 3 and isModemready == false and isLanready == true and startSenddata == false) {
          Serial.println("Send data using LAN...");
          if (sendVialan(dataStore[i]) == true) {
            dataStore_checkList[i] = false;
          }
        } else if (count != 3 and isLanready == true and startSenddata == false) {
          Serial.println("Send data using LAN...");
          count = count + 1;
          if (sendVialan(dataStore[i]) == true) {
            dataStore_checkList[i] = false;
          } else {
            if (failSendcount_LAN > 4 and isModemready == true and startSenddata == false) {  //and isSendsuccess == true
              Serial.println("Send data using 4G...");
              sendVia4G(dataStore[i]);
              sendIndex = i;
            }
          }
        } else if (isLanready == false) {
          if (isModemready == true and startSenddata == false) {  //and isSendsuccess == true
            Serial.println("Send data using 4G...");
            sendVia4G(dataStore[i]);
            sendIndex = i;
          }
        } else if (isModemready == false) {
          Serial.println("Send data using LAN...");
          //count = count + 1;
          if (sendVialan(dataStore[i]) == true) {
            dataStore_checkList[i] = false;
          }
        }
      }
    }
  }
  //-----------------------------------------------------------------------------------------------------------------
  if ((fabs(millis() - dataSendtime) > dataSendtimeout) and isModemready == true and startSenddata == true and isSendsuccess == false) {
    Serial.println("Server not respond TRY again!!!");
    if (failSendcount_4G < 2) {
      failSendcount_4G++;
      if (isModemready == true and systemfault == false) {
        sendVia4G(preData);
      }
    } else if (failSendcount_4G >= 2) {
      Serial.println("Reboot A7683...");
      digitalWrite(netlight_led, LOW);
      isModemready = false;
      systemfault = true;
      if (isLanready == true) {
        Serial.println("Send data using LAN...");
        if (sendVialan(dataStore[sendIndex]) == true) {
          startSenddata = false;
          isSendsuccess = true;
          dataSendtime = millis();
          count = 0;
          dataStore_checkList[sendIndex] = false;
        }
      }
      resetA7682module();
      // failSendcount_4G = 0;
      // failSendcount_LAN = 0;
      startHttpservice = false;
      startSenddata = true;
      isSendsuccess = false;
    }
  }
  if (failSendcount_LAN > 2 and isLanready == true) {
    Serial.println("Reboot W5500...");
    digitalWrite(netlight_led, LOW);
    resetW5500module();
    // failSendcount_LAN = 0;
  }
  if (isModemready == false and isLanready == false) {
    digitalWrite(netlight_led, LOW);
  }
  //-----------------------------------------------------------------------------------------------------------------
  else if ((failSendcount_4G + failSendcount_LAN) <= 1 and digitalRead(userTest_button) == 1) {  // and systemfault == false
    digitalWrite(faultAlarm_led, faultAlarm);
    digitalWrite(fireAlarm_led, fireAlarm);
    digitalWrite(netlight_led, HIGH);
  }
  //================================================================================================================
  unsigned long uled_currentMillis = millis();
  if (fabs(uled_currentMillis - uled_previousMillis) >= uled_interval) {
    uled_previousMillis = uled_currentMillis;
    ledState = !ledState;
    digitalWrite(userLed_pin, ledState);
    if ((failSendcount_4G + failSendcount_LAN) > 1) {  // or isModemready == false or systemfault == false or isLanready == false
      digitalWrite(netlight_led, ledState);
    }
  }
  //================================================================================================================
  delay(5);
}
void sendEventdata(int fa, int fi) {
  Serial.println("Create Data...");
  DynamicJsonDocument doc(1024);
  String payload = "";
  doc["id"] = deviceID;
  //doc["ba"] = vBatt;
  doc["Fa"] = fa;
  doc["Fi"] = fi;
  doc["dt"] = rtc.getTime("%F %T");
  serializeJson(doc, payload);
  payload = payload + "\r\n";
  writeQueue(payload);
}
void sendNormaldata(void) {
  Serial.println("Create Data...");
  PacketCount = PacketCount + 1;
  DynamicJsonDocument doc(1024);
  String payload = "";
  doc["id"] = deviceID;
  doc["pi"] = PacketCount;
  doc["dt"] = rtc.getTime("%F %T");
  serializeJson(doc, payload);
  payload = payload + "\r\n";
  writeQueue(payload);
}
bool sendVialan(String data) {
  // delay(250);
  // ssl_client.stop();
  // delay(250);
  Serial.println("------------------------------------------------------");
  Serial.print("connect to Server...");
  if (ssl_client.connect(serverURL.c_str(), 443)) {
    Serial.println("connected");
    Serial.println("Begin POST Request");

    ssl_client.print("POST " + serverPath + " HTTP/1.1\r\n");
    ssl_client.print("Host: " + serverURL + "\r\n");
    ssl_client.print("User-Agent: Arduino/1.0\r\n");
    ssl_client.print("Content-Type: application/json\r\n");
    ssl_client.print("Connection: keep-alive\r\n");
    ssl_client.print("Content-Length: ");
    ssl_client.print(data.length());
    ssl_client.print("\r\n\r\n");
    ssl_client.print(data);
    Serial.print("Read response...");

    unsigned long ms = millis();
    while (!ssl_client.available() && millis() - ms < 3000) {
      delay(0);
    }
    Serial.println();
    while (ssl_client.available()) {
      char c = ssl_client.read();
      Serial.print(c);
    }
    // ssl_client.print("Connection: close\r\n");
    // ssl_client.print("\r\n");
    Serial.print("\r\n");
    failSendcount_4G = 0;
    failSendcount_LAN = 0;
    ssl_client.stop();
    return (1);
  } else {
    Serial.print("Post Fail...\r\n");
    failSendcount_LAN++;
    ssl_client.stop();
    return (0);
  }
}
void writeQueue(String data) {
  dataStore[dataIndex] = data;
  dataStore_checkList[dataIndex] = true;
  dataStore_priority[dataIndex] = 1;
  dataIndex = dataIndex + 1;
  if (dataIndex > 4) dataIndex = 0;
}
void IO_initial(void) {

  pinMode(faultAlarm_pin, INPUT_PULLUP);
  pinMode(fireAlarm_pin, INPUT_PULLUP);
  pinMode(userTest_button, INPUT_PULLUP);

  pinMode(POW_pin, OUTPUT);  //4G Power Enable pin

  digitalWrite(POW_pin, HIGH);
  delay(200);

  pinMode(SLP_pin, INPUT_PULLUP);
  pinMode(PWR_pin, INPUT_PULLUP);

  if (digitalRead(SLP_pin) == false and digitalRead(PWR_pin) == false) {
    isModemfound = true;
  } else {
    isModemfound = false;
  }

  Serial.println("4G Module Detect : " + String(isModemfound));

  digitalWrite(POW_pin, LOW);
  digitalWrite(SLP_pin, HIGH);
  digitalWrite(PWR_pin, HIGH);
  digitalWrite(netlight_led, HIGH);
  digitalWrite(faultAlarm_led, HIGH);
  digitalWrite(fireAlarm_led, HIGH);
  digitalWrite(userLed_pin, LOW);

  pinMode(POW_pin, OUTPUT);         //4G Power Enable pin
  pinMode(SLP_pin, OUTPUT);         //NB-IOT Reset pin
  pinMode(PWR_pin, OUTPUT);         //NB-IOT Key pin
  pinMode(netlight_led, OUTPUT);    //Onboard netlight LED pin
  pinMode(faultAlarm_led, OUTPUT);  //Onboard faultAlarm LED pin
  pinMode(fireAlarm_led, OUTPUT);   //Onboard fireAlarm LED pin
  pinMode(ETH_PHY_RST, OUTPUT);     //W5500 Reset pin
  pinMode(userLed_pin, OUTPUT);     //Onboard Uset LED pin
}
void Com_initial(void) {
  Serial.begin(115200);
  modem_serial.begin(115200, SERIAL_8N1, NRX_pin, NTX_pin);
  _Serial = &modem_serial;
}
void resetW5500module() {
  isLanready == false;
  Serial.print("Resetting  Ethernet Module...  ");

  digitalWrite(ETH_PHY_RST, HIGH);
  delay(250);
  digitalWrite(ETH_PHY_RST, LOW);
  delay(50);
  digitalWrite(ETH_PHY_RST, HIGH);
  delay(350);

  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI);
  Ethernet.init(ETH_PHY_CS);

  Serial.println("Done.");
}
void resetA7682module() {
  if (isModemfound == true) {
    Serial.print("Resetting 4G Module...  ");
    startHttpservice = false;
    digitalWrite(POW_pin, LOW);
    delay(2000);
    digitalWrite(POW_pin, HIGH);
    delay(1000);
    digitalWrite(PWR_pin, LOW);
    delay(1000);
    digitalWrite(PWR_pin, HIGH);
    delay(3000);
    digitalWrite(PWR_pin, LOW);
    delay(1000);
    Serial.println("Done.");
  } else {
    Serial.println("Trun Modem Power Off...");
    digitalWrite(POW_pin, LOW);
    systemfault = true;
  }
}
bool setNBIOT_echoOn(void) {
  sendCommand("ATE1", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
bool setNBIOT_echoOff(void) {
  sendCommand("ATE0", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
String HexString2ASCIIString(String hexstring) {
  String temp = "", sub = "", result;
  char buf[3];
  for (int i = 0; i < hexstring.length(); i += 2) {
    sub = hexstring.substring(i, i + 2);
    sub.toCharArray(buf, 3);
    char b = (char)strtol(buf, 0, 16);
    if (b == '\0')
      break;
    temp += b;
  }
  return temp;
}
String str2HexStr(String strin) {
  int lenuse = strin.length();
  char charBuf[lenuse * 2 - 1];
  char strBuf[lenuse * 2 - 1];
  String strout = "";
  strin.toCharArray(charBuf, lenuse * 2);
  for (int i = 0; i < lenuse; i++) {
    sprintf(strBuf, "%02X", charBuf[i]);
    if (String(strBuf) != F("00")) {
      strout += strBuf;
    }
  }
  return strout;
}
String sendCommand(String cmd, long tout, String str_wait) {
  pLock = true;
  String Sim_res = "";
  _Serial->println(cmd);
  Sim_res = moduleRes(tout, str_wait);
  Sim_res.replace("OK", "");
  pLock = false;
  return (Sim_res);
}
String moduleRes(long tout, String str_wait) {
  unsigned long pv_ok = millis();
  unsigned long current_ok = millis();
  unsigned char flag_out = 1;
  cmm_state = 0;
  String input = "";
  String temp = "";
  while (flag_out) {
    if (_Serial->available()) {
      input = _Serial->readStringUntil('\n');
      temp += input;
      //Serial.println(input);
      if (input.indexOf(str_wait) != -1) {
        cmm_state = 1;
        flag_out = 0;
        return (temp);
      } else if (input.indexOf(F("ERROR")) != -1) {
        cmm_state = 0;
        flag_out = 0;
        return (temp);
      }
    }
    current_ok = millis();
    if (current_ok - pv_ok >= tout) {
      flag_out = 0;
      cmm_state = 0;
      pv_ok = current_ok;
    }
  }
  return (temp);
}
bool startHTTPservice(void) {
  sendCommand("AT+HTTPINIT", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
bool connecServer(void) {
  sendCommand("AT+HTTPPARA=\"URL\",\"https://" + serverURL + serverPath + "\"", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
bool sendPostrequest(int datalength) {
  sendCommand("AT+HTTPDATA=" + String(datalength) + ",3000", 3000, "DOWNLOAD");
  return ((cmm_state == 1) ? true : false);
}
bool sendData(String data) {
  sendCommand(data, 400, "OK");
  return ((cmm_state == 1) ? true : false);
}
bool sendData2(void) {
  sendCommand("AT+HTTPACTION=1", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
void sendVia4G(String data) {
  startSenddata = true;
  isSendsuccess = false;
  preData = data;
  int datalength = data.length();
  if (startHttpservice == false) {
    startHttpservice = true;
    Serial.println("Module start HTTP service : " + String(startHTTPservice()));
    Serial.println("Module Connect to server : " + String(connecServer()));
  } else if (fabs(millis() - dataSendtime) >= 3600000LL) {
    Serial.println("Module start HTTP service : " + String(startHTTPservice()));
    Serial.println("Module Connect to server : " + String(connecServer()));
  }
  Serial.println("Module Send POST request : " + String(sendPostrequest(datalength)));
  Serial.println("Module Send DATA : " + String(sendData(data)));
  Serial.println("Module Send DATA 2 : " + String(sendData2()));
  dataSendtime = millis();
}
String get_HTTPresponse(int r) {
  String temp = sendCommand("AT+HTTPREAD=0," + String(r), 3000, "+HTTPREAD: 0");
  String tmp = temp.substring(temp.indexOf(String(r)) + String(r).length(), temp.indexOf("+HTTPREAD: 0"));
  return (tmp);
}
void updateTime(void) {
  Serial.println("Update System time...");
  sendNTPpacket(timeServer);
  delay(100);
  if (Udp.parsePacket()) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);  // Read the packet into the buffer.
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    rtc.setTime(epoch + 25200);
    Serial.println("Update success...");
  } else {
    syncTime();
  }
}
bool setNTPserver(void) {
  sendCommand("AT+CNTP=\"1.th.pool.ntp.org\",28", 100, "OK");
  return ((cmm_state == 1) ? true : false);
}
void syncTime(void) {
  Serial.println("Set NTP Server : " + String(setNTPserver()));
  Serial.println("NTP Server Respond : " + String(sendCommand("AT+CNTP", 2000, "+CNTP: 0")));
  //Serial.println("NTP Server Auto Update : " + String(sendCommand("AT+CTZU=1", 100, "OK")));
  delay(3000);
  String tmp = sendCommand("AT+CCLK?", 100, "OK");
  //Serial.println("Time String : " + tmp);
  if (tmp.indexOf(F("+CCLK:")) != -1 and tmp.indexOf(F("+28")) != -1) {
    tmp = tmp.substring(9, 26);
    //Serial.println("Time String : " + tmp);
    if (tmp.length() == 17) {
      int dd = 0, mo = 0, yy = 0, hh = 0, mi = 0, ss = 0;
      dd = tmp.substring(6, 8).toInt();
      mo = tmp.substring(3, 5).toInt();
      yy = tmp.substring(0, 2).toInt() + 2000;
      hh = tmp.substring(9, 11).toInt();
      mi = tmp.substring(12, 14).toInt();
      ss = tmp.substring(15, 17).toInt();  //ss = tmp.substring(15, 17).toInt() + 2;
      rtc.setTime(ss, mi, hh, dd, mo, yy);
      //Serial.println("Time String : " + tmp);
      //Serial.println("Sync Time OK");
      Serial.println("Update success...");
    }
  }
}
// Send an NTP request to the time server at the given address (defined in local_conf.h).
void sendNTPpacket(const char *address) {
  // Set all bytes in the buffer to 0.
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Initialize values needed to form NTP request
  // (see http://en.wikipedia.org/wiki/Network_Time_Protocol).
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // All NTP fields have been given values, now
  // send a packet requesting a timestamp.
  Udp.beginPacket(address, 123);  // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}