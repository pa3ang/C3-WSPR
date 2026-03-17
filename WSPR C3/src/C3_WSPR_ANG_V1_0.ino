// WSPR beacon with SEEED XIAO ESP32 C3, SI5351.

// Build-in webserver to configure and NTP connection in 'online' configuration
// AP connect to enter WiFi Credentials or Webservice and manual time sync with button in 'offline' configuration

// Re-adopted from the original idea from Benno PA3FBX and adapted by PD8GB, PA3FEX and PA3ANG
// with ESP8266 and ESP_WSPR_ANG_V1_7 firmware.

/*
Using a SEEED ESP32 C3 and generic 25MHz Si5351 synthesizer board.
With this, we can build a WSPR transmitter with:
  - Up to 3 bands with one LPF (20,30,40m)  40 meter with limited suppresion
  - Led with status indication
  - Button to control configuration and when 'offline' synchronise timestart on even minute.
  - If 'online' time sync. using NTP when connected to Internet
  - Provide a configuration through WiFi accessable webwerver

On first startup, if no WiFi credentials are stored, the ESP will switch into Access Point mode.  
SSID = "C3-WSPR"  no password.  

Connect your phone/laptop to this network the captive portal is available at 192.168.4.1  
A configuration page appears after can enter or select the avilable WiFi SSID and enter the password.
Click “Save”. The ESP will then connect to your local WiFi.  
 
The unit will restart. Check your router for the assign IP adress to log-in to the webserer.
When 'offline' connect the iPhone to the WiFi AP called "C3-WSPR" and go to IP: 192.168.4.1 in your brwoser.
 
You will land on the status page and can advance to Configuration and the other pages:
  - /           = status
  - /config     = configuration
  - /calibrate  = to caibrate the Si5351 against 10 kHz.
  - /bands      = to select the bands based on the applied LPF.
  - /txtest     = carrier on defined bands

On fresh start the firware will create the default values into EEPROM. A magic value in the EEPROM will guide this.
The firmware has an Easter Egg : Put ERASE in Locator for a full reset!

Used libraries  
  - Etherkit Si5351 by Jason Mildrum, version 2.2.0  
  - Etherkit JTEncode by Jason Mildrum, version 1.3.1  
  - NTPtimeESP  
  - TimeLib by Michael Margolis, version 1.6.1  
  - WiFiManager by tzapu, version 2.0.17  
  - Adafruit BusIO (I2C) 
   

Version: 2026-03-12
by Johan, PA3ANG
https://github.com/pa3ang
*/

#include <si5351.h>
#include "Wire.h"
#include <JTEncode.h>
#include <stdint.h>
#include <TimeLib.h>
#include <NTPtimeESP.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>

// WSPR timing
#define TONE_SPACING 146
#define WSPR_DELAY 683
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT

// NTP server
#define SEND_INTV 10
#define RECV_TIMEOUT 10
#define TIME_ZONE -1.0f  // For Netherlands

// EEPROM definition
#define EEPROM_SIZE 512
#define CONFIG_MAGIC 0xAEEF

// LPF configuration
#define MAX_ACTIVE_BANDS 2           // Low Pass Filter (LPF) defined

struct LPFOption {
  const char* name;
  const char* bands[2];
  uint8_t bandCount;
};

LPFOption lpfOptions[] = {
  { "160m",  {"160m"}, 1 },
  { "80m",   {"80m"}, 1 },
  { "60/40", {"60m","40m"}, 2 },
  { "30/20", {"30m","20m"}, 2 },
  { "17/15", {"17m","15m"}, 2 },
  { "12/10", {"12m","10m"}, 2 },
  { "2m",    {"2m"}, 1 }
};

const int numLPFOptions = sizeof(lpfOptions)/sizeof(lpfOptions[0]);

// --- Band structure ---
struct BandConfig {
    uint8_t nr;             // band number
    char bandname[5];       // 4 characters + null-terminator
    bool active;            // true/false
};

// --- Main configuration ---
struct Config {
    uint16_t magic;         // EEPROM validation
    char call[10];          // Callsign
    char loc[10];           // Maidenhead locator
    int dbm;                // TX power in dBm
    uint8_t lpfIndex;       // LPF selection
    int cal;                // Si5351 calibration factor
    int band;               // band index
    int interval;           // TX interval
    int ssid;               // WiFi reset flag
    BandConfig bands[2];    // Max 2 active bands
} config;

// Connection variables
const char* WiFi_hostname = "C3-WSPR";
const char* NTP_Server    = "time.google.com";

struct BandFreq {
    const char* name;
    uint64_t freqHz;
};

// 12 bands with WSPR center frequency defined
const BandFreq standardBands[] = {
    {"160m", 1838100},
    {"80m",  3570100},
    {"60m",  5366200},
    {"40m",  7040100},
    {"30m", 10140200},
    {"20m", 14097100},
    {"17m", 18106100},
    {"15m", 21096100},
    {"12m", 24926100},
    {"10m", 28126100},
    {"6m",  50294500},
    {"2m", 144490000},
};
const int numStandardBands = sizeof(standardBands) / sizeof(standardBands[0]);

// WSPR and state-machine parameters
int bandStep = 0;               // pointer for next band in multiband
int activeBands = 0;            // number is active bands
const char* nextBandName;       // holds band name  
int startMinute = -1;
int offset;
int offLine = 0;

// used during calibration
uint64_t freq;                  // freq for encode() routine  
uint32_t rx_freq;               // used in calibration routine
const uint32_t calibration_freq = 1000000000ULL;  // 10 MHz, in hundredths of hertz
int32_t cal_factor; 
bool maintenance = false;
bool cal_init = false;

// used during txtest
bool freqRunning = false;
int currentFreqIndex = -1;    // -1 = geen actieve frequentie

// json variables for status page
String statusLine1;
String statusLine2;
String statusLine3;
String statusLine4;

// used in for encode
uint8_t tx_buffer[SYMBOL_COUNT];

// define hardare / software
Si5351 si5351;
JTEncode jtencode;

NTPtime NTPch(NTP_Server);
strDateTime dateTime;

WebServer server(80);
WiFiManager wm;

// led
int StatusLed = 2;
int lastSec = -1;

// button
int buttonPin = 4;

// EEPROM configuration and routines
void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, config);
  EEPROM.end();

  // if EEPROM has eronous values it's probably an absolute first start or EEPROM has been erased
  if (config.magic != CONFIG_MAGIC) {
    strcpy(config.call, "NOCALL");
    strcpy(config.loc, "AA00AA");
    config.dbm = 23;
    config.cal = 0;
    config.interval = 4;
    config.ssid = 0;
    config.lpfIndex = 3;
    strcpy(config.bands[1].bandname, "30m");  config.bands[1].nr = 1; config.bands[1].active = true;
    strcpy(config.bands[2].bandname, "20m");  config.bands[2].nr = 2; config.bands[2].active = true;
  }
  // Easter egg to force reset 
  if (strcmp(config.loc, "ERASE") == 0) {
    Serial.println("Manual reset triggered");
    config.magic = 0;
    saveConfig();
    ESP.restart();
  }
}

void saveConfigToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, config);   // save the whole struct
    EEPROM.commit();
    EEPROM.end();
    Serial.println(F("Config including bands saved to EEPROM"));
}

void saveConfig() {
  config.magic = CONFIG_MAGIC;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, config);
  EEPROM.commit();
  EEPROM.end();
}

// Webpage general strings
const char* STYLE =
  "<style>"
  "body{text-align:center;font-family:verdana}"
  "div,input,select,.statusbox{padding:5px;margin:5px 0 5px 0;box-sizing:border-box;font-size:1em}"
  "input,button,select,.statusbox{border-radius:.3rem;width:100%}"
  "input,button,select,.statusbox{border:1px solid #000}"
  "button,input[type=button],input[type=submit]{cursor:pointer;border:0;background:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%}"
  ".statusbox{text-align:center}"
  ".wrap{text-align:left;display:inline-block;min-width:260px;max-width:480px;margin:auto}"
  "</style>";

const char* HEAD = 
  "<!DOCTYPE html>"
  "<html lang='en'>"
  "<head>"
  "<title>C3-WSPR</title>"
  "<meta name='format-detection' content='telephone=no'>"
  "<meta charset='UTF-8'>"
  "<meta  name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'/>"
  // Anti-cache via HTML (voor de browser)
  "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>"
  "<meta http-equiv='Pragma' content='no-cache'>"
  "<meta http-equiv='Expires' content='0'>"
  "</head>" ;

const char* FOOTER =
  "<p style='font-size:10px; color:grey; text-align:center'>XIAO ESP32-C3, Version : ANG-V1.0, Mar-2026</p></body></html>";

// Individual html pages
String htmlConfig() {
  String html = HEAD;
  html += STYLE;
  html += "<script>";
  html += "function changeLPF(v){";
  html += "window.location='/config?lpf='+v;";
  html += "}";
  html += "</script>";
  html += "<body><div class='wrap'>";
  html += "<h1>C3-WSPR Config</h1>";
  html += "<form method='POST' action='/save'>";
  html += "Callsign<br><input name='call' value='" + String(config.call) + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "Locator<br>  <input name='loc' value='" + String(config.loc)  + "' oninput='this.value = this.value.toUpperCase();'><br>";
  html += "dBm<br><input name='dbm' value='" + String(config.dbm) + "'><br>";
  html += "LPF Filter<br>";
  html += "<select name='lpf' onchange='changeLPF(this.value)'>";
  for (int i = 0; i < numLPFOptions; i++) {
      html += "<option value='" + String(i) + "'";
      if (config.lpfIndex == i) html += " selected";
      html += ">" + String(lpfOptions[i].name) + "</option>";
  }
  html += "</select><br>";
  html += "Active bands<br>";
  html += "<style>"
        "#status { display: flex; flex-direction: column; align-items: flex-start; gap: 5px; text-align: left; }"
        ".band-label { display: flex; align-items: center; gap: 5px; }"
        "@media (min-width: 600px) { "
        "  #status { flex-direction: row; flex-wrap: wrap; gap: 10px 15px; } "
        "}"
        "</style>";
  LPFOption &lpf = lpfOptions[config.lpfIndex];
  html += "<div class='statusbox' id='status'>";
  for (int i = 0; i < lpf.bandCount; i++) {
    const char* band = lpf.bands[i];
    bool active = false;
    for (int j = 0; j < MAX_ACTIVE_BANDS; j++) {
        if (strcmp(config.bands[j].bandname, band) == 0 && config.bands[j].active)
            active = true;
    }
    html += "<label class='band-label'>";
    html += "<input type='checkbox' name='band_" + String(band) + "'";
    if (active) html += " checked";
    html += ">";
    html += String(band);
    html += "</label>";
  }
  html += "</div>";
  html += "Interval (2,4,6, ... max 16 )<br><input name='interval' value='" + String(config.interval) + "'><br>";
  html += "Calibration factor<br><input name='cal' value='" + String(config.cal) + "'><br>";
  html += "Reset WiFi (9)<br><input name='ssid' value='" + String(config.ssid) + "'><br>";
  html += "<input type='submit' value='Save & Reboot'></form>";
  html += "<form method='POST' action='/calibrate'>";
  html += "<input type='submit' value='Calibration'></form>";
  html += "<form method='POST' action='/txtest'>";
  html += "<input type='submit' value='TX Test'></form>";
  html += "<form method='POST' action='/'>";
  html += "<input type='submit' value='Back to Status'></form>";
  html += FOOTER;
  return html;
}

// Config handling page
void handleConfig() {
  if (server.hasArg("lpf")) {
    config.lpfIndex = server.arg("lpf").toInt();
  }
  server.send(200, "text/html", htmlConfig());
}

// Save config handling page
void handleSave() {
  // read values from form
  strncpy(config.call, server.arg("call").c_str(), sizeof(config.call));
  strncpy(config.loc, server.arg("loc").c_str(), sizeof(config.loc));
  config.dbm      = server.arg("dbm").toInt();
  config.cal      = server.arg("cal").toInt();
  String input    = server.arg("band");
  config.interval = server.arg("interval").toInt();
  config.ssid     = server.arg("ssid").toInt();

  // intercept wrong values
  if (config.ssid  != 9) { config.ssid = 0; } 
  if (config.interval < 2 || config.interval > 16 || (config.interval % 2 != 0)) { config.interval = 8; }
  if (server.hasArg("lpf")) {
    config.lpfIndex = server.arg("lpf").toInt();
  }
  LPFOption &lpf = lpfOptions[config.lpfIndex];
  for (int i = 0; i < lpf.bandCount; i++) {
    const char* band = lpf.bands[i];
    bool active = server.hasArg(String("band_") + band);
    strcpy(config.bands[i].bandname, band);
    config.bands[i].nr = i;
    config.bands[i].active = active;
  }
  saveConfig();
 
  String html =
  "<!DOCTYPE html><html><head>"
  "<meta http-equiv='refresh' content='5;url=/' />"
  "<title>C3-WSPR</title>"
  "</head><body><center>"
  "<h1>Saved! Rebooting</h1>"
  "<p>Restarting in 5 seconds.</p>"
  "</center></body></html>";

  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

// Calibration page
String htmlCalibrate() {
  String html = HEAD;
  html += STYLE;
  html += "<body><div class='wrap'>";
  html += "<h1>Si5351 Calibrate</h1>";
  html += "<p>Target frequency: " + String(calibration_freq / 1000.0, 3) + " kHz</p>";
  html += "<p>Current cal_factor: " + String(cal_factor) + "</p>";
  html += "<form method='POST'>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='i'>+1 Hz</button>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='k'>-1 Hz</button><br>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='o'>+10 Hz</button>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='l'>-10 Hz</button><br>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='p'>+100 Hz</button>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value=';'>-100 Hz</button><br><br>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='save'>Save</button>";
  html += "<button style='width:140px;height:45px;margin:5px' name='step' value='cancel'>Cancel</button>";
  html += "</form>";
  html += FOOTER;
  return html;
}

// Calibration handling page with live interaction with webpage
void handleCalibrate() {
  if (!cal_init) {
    rx_freq = calibration_freq;
    cal_factor = (int32_t)(calibration_freq - rx_freq) + config.cal;

    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

    maintenance = true;
    cal_init = true;
  }

  // Process POST requests
  if (server.method() == HTTP_POST && server.hasArg("step")) {
    String step = server.arg("step");

    if (step == "save" || step == "cancel") {
      if (step == "save") {
        config.cal = cal_factor;
        saveConfig();
      }
      if (step == "cancel") {
        cal_factor = config.cal;
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
      }
      si5351.set_clock_pwr(SI5351_CLK0, 0);
      server.sendHeader("Location", "/");
      server.send(303);         // POST → GET redirect
      maintenance = false;      // cancel calibration
      cal_init = false;
      startMinute = -1;         // reset start
      return;
    }

    // step adjustment
    char c = step.charAt(0);
    int32_t stepVal = 0;
    switch (c) {
      case 'i': stepVal = 100; break;
      case 'k': stepVal = -100; break;
      case 'o': stepVal = 1000; break;
      case 'l': stepVal = -1000; break;
      case 'p': stepVal = 10000; break;
      case ';': stepVal = -10000; break;
    }

    rx_freq += stepVal;
    int32_t diff = (int32_t)(calibration_freq - rx_freq);
    cal_factor = diff + config.cal;

    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.pll_reset(SI5351_PLLA);
    si5351.set_freq(calibration_freq, SI5351_CLK0);
  }

  // Always send fresh HTML (safe for iPhone Safari)
  String html = htmlCalibrate();
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send(200, "text/html", html);
}

String htmlTXTest() {
  String html = HEAD;
  html += STYLE;
  html += "<body><div class='wrap'>";
  html += "<h1>C3-WSPR TX Test</h1>";
  html += "<form method='POST'>";

  // Buttons per actieve band
  for (int i = 0; i < MAX_ACTIVE_BANDS; i++) {
    if (!config.bands[i].active) continue;
    // frequentie lookup
    uint64_t freq = 0;
    for (int j = 0; j < numStandardBands; j++) {
      if (strcmp(config.bands[i].bandname, standardBands[j].name) == 0) {
        freq = standardBands[j].freqHz;
        break;
      }
    }
    html += "<button name='bandIndex' value='" + String(i) + "' type='submit'>";
    html += String(config.bands[i].bandname) + " (" + String(freq / 1000000.0, 6) + " MHz)";
    html += "</button><br><br>";
  }
  html += "<br><button name='action' value='stoptx' type='submit'>Stop TX</button>";
  html += "<br><br><button name='action' value='exit' type='submit'>Exit</button>";
  html += "</form>";

  if (freqRunning && currentFreqIndex >= 0) {
    uint64_t freq = 0;
    for (int j = 0; j < numStandardBands; j++) {
      if (strcmp(config.bands[currentFreqIndex].bandname, standardBands[j].name) == 0) {
        freq = standardBands[j].freqHz;
        break;
      }
    }
    html += "<p>Transmitting on : " + String(config.bands[currentFreqIndex].bandname) + "</p>";
  } else {
    html += "<p>No active transmission</p>";
  }

  html += FOOTER;
  return html;
}

void handleTXTest() {
    if (server.method() == HTTP_POST) {
    maintenance = true;

    // Cancel / Reset knob
    if (server.hasArg("action") && (server.arg("action") == "cancel" || server.arg("action") == "stoptx")) {
      freqRunning = false;
      currentFreqIndex = -1;
      si5351.set_clock_pwr(SI5351_CLK0, 0);
      digitalWrite(StatusLed, LOW);
    }
    if (server.hasArg("action") && server.arg("action") == "exit") {
      maintenance = false;
      startMinute = -1;         // reset start
      server.sendHeader("Location", "/");
      server.send(303);         
      return;
    }

    // Knobs for active bands
    if (server.hasArg("bandIndex")) {
      currentFreqIndex = server.arg("bandIndex").toInt();
      freqRunning = true;
      // frequency lookup
      uint64_t freq = 0;
      for (int j = 0; j < numStandardBands; j++) {
        if (strcmp(config.bands[currentFreqIndex].bandname, standardBands[j].name) == 0) {
          freq = standardBands[j].freqHz;
          break;
        }
      }
      // Set Si5351
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      si5351.set_freq(freq * 100, SI5351_CLK0);
      digitalWrite(StatusLed, HIGH);
    }
  }
  // always send page
  server.send(200, "text/html", htmlTXTest());
}

// Status page
String htmlRoot() {
  String html = HEAD;
  html += STYLE;
  html += "<body><div class='wrap'>";
  html += "<h1>C3-WSPR Status</h1>";
  html += "<div class='statusbox' id='status'>";
  html += "Line 1<br>Line 2<br>Line 3<br>Line 4";
  html += "</div>";
  html += "<br><button onclick=\"location.href='/config'\">Configure</button>";
  html += "<script>";
  html += "function updateStatus() {";
  html += "  fetch('/status?_=' + new Date().getTime())";  
  html += "  .then(response => response.json())";
  html += "  .then(data => {";
  html += "    document.getElementById('status').innerHTML = data.line1 + '<br>' + data.line2 + '<br>' + data.line3 + '<br>' + data.line4;";
  html += "  })";
  html += "  .catch(err => console.log('Error fetching status:', err));";
  html += "}";
  html += "setInterval(updateStatus, 1000);";   // update elke seconde
  html += "updateStatus();";                    // meteen bij laden
  html += "</script>";
  html += FOOTER;
  return html;
}

// Handle the root page
void handleRoot() {
  server.send(200, "text/html", htmlRoot());
}

// AJAX update the root (/status) page
void handleStatus() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  String json = "{";
  json += "\"line1\":\"" + statusLine1 + "\",";
  json += "\"line2\":\"" + statusLine2 + "\",";
  json += "\"line3\":\"" + statusLine3 + "\",";  
  json += "\"line4\":\"" + statusLine4 + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// general routine for led blinking x times
void blink(int n) { 
  while(n--) digitalWrite(StatusLed,HIGH),delay(100),digitalWrite(StatusLed,LOW),delay(100); 
}


// start script and general setup routines
void setup() {
  Serial.begin(115200);

  pinMode(StatusLed, OUTPUT);
  digitalWrite(StatusLed, LOW);
  pinMode(buttonPin, INPUT_PULLUP);

  // first load config from EEPROM
  loadConfig();
  Serial.println("Config loaded:");
  Serial.printf("Call=%s, Loc=%s, dBm=%d, Interval=%d, Cal=%d, WiFi Reset=%d\n",config.call, config.loc, config.dbm, config.interval, config.cal, config.ssid);
  
  // initialise the Si5351
  cal_factor = config.cal; 
  si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(1000000000ULL, SI5351_CLK0);           // dummy 10 MHz
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);  // SI5351_DRIVE_8MA=5 mw / SI5351_DRIVE_2MA= 1mw/ SI5351_DRIVE_4MA= 2mw
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.set_clock_pwr(SI5351_CLK1, 0);
  si5351.set_clock_pwr(SI5351_CLK2, 0);
  Serial.println("SI5351 initialised");

  // create offset + or - max 50 Hz in final tx frequency.
  randomSeed(esp_random());
  do {
    offset = random(-50, 51);
  } while (offset == 0);
  
  // WiFi routines and start with WiFiManager or embedded WiFi
  // SSID wipe out if asked for in configuration panel
  if (config.ssid == 9) { 
    wm.resetSettings();
    config.ssid = 0;
    saveConfig();
  }

  // WIFI setup and start either local, network, or captive 
  // which mode will be used online or offline?
  // first check if user button is pressed indicating off line use 
  if (digitalRead(buttonPin) == LOW) {  
    // button pressed so off line
    offLine = 1;
    digitalWrite(StatusLed, HIGH);
    // wait till button released with status led on
    while (digitalRead(buttonPin) == LOW);
    // released so statusled off and wait shortly for convinience  
    digitalWrite(StatusLed, LOW);
    delay(1000);
  }

  if (offLine) {
    // WiFi AP on ESP device will be used
    WiFi.mode(WIFI_AP);
    WiFi.softAP("C3-WSPR", "");
    Serial.println("WiFi AP created");

    // and wait till user button is pressed indicating start of even minute
    while (digitalRead(buttonPin) == HIGH) {
      unsigned long m = millis() % 1000;   // positie binnen 1 seconde
      digitalWrite(StatusLed,(m < 100 || (m > 200 && m < 300)) ? HIGH : LOW);
    }
    setTime(0,0,0,1,1,2020);
    while (digitalRead(buttonPin) == LOW); // wachten tot knop los
   
  } else {
    // Try to find stored SSID on local network
    // if not found or empty credentials then start captive AP
    WiFi.mode(WIFI_STA);
    if (!wm.autoConnect("C3-WSPR")) {
      Serial.println("WiFi connect failed, reboot...");
      ESP.restart();
    }

    // get the time from the given NTP server
    epochUnixNTP();   
    Serial.println("Connected to WiFi!"); Serial.print("IP address: "); Serial.println(WiFi.localIP());
  }
   
  // Start WebServer and point to the individual pages 
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/config", handleConfig);
  server.on("/status", handleStatus);
  server.on("/txtest", handleTXTest);
  server.begin();
  Serial.println("Webserver started. Access via http://" + WiFi.localIP().toString());
  Serial.println("Setup complete.");

  // blink led 3 times indicating setup completed
  blink(3);
}

void loop() {
  server.handleClient();
  yield();

  // there must be a valid callsign
  if (strcmp(config.call, "NOCALL") == 0) {
      statusLine1 = "NOCALL";
      statusLine2 = "";
      statusLine3 = ">> Go to Configure <<";
      statusLine4 = "";
      blink(1);
      return;
  }
  // do not start when in Calibrate mode
  if (!maintenance) {  // alleen als niet in calibratie
    // get current time 
    int h = hour(); int m = minute(); int s = second();
    
    // Initialize StartMinute; this is immediately the first even minute after startup.
    if (startMinute < 0 ) {  
      //This routine is called at startup so calculate next start with 2 minutes idle time at least to permit web access
      startMinute = (m + 2);
      if (startMinute % 2 != 0) startMinute++;  // round to next  even minute
      if (startMinute >= 60) startMinute -= 60; // wrap around the hour
    }
    
    // find next band to transmit to show on OLED
    // Count active bands
    activeBands=0;
    for (int i = 0; i < MAX_ACTIVE_BANDS; i++) {
      if (config.bands[i].active) activeBands++;
    }

    if (activeBands == 0) {
      Serial.println("No active bands configured!");
    } else {

      // Wrap if bandStep exceeds activeBands means cycle ended and reset to first active band
      if (bandStep >= activeBands) bandStep = 0;

      // Find the next active band and get the frequency 
      int stepCounter = 0; 
      for (int i = 0; i < MAX_ACTIVE_BANDS; i++) {
        BandConfig band = config.bands[i];
        if (band.active) {
          if (stepCounter == bandStep) {
            nextBandName = band.bandname;
            break;
          }
          stepCounter++;
        }
      }
      // Find frequency for this band from standardBands[]
      freq = 0;
      for (int j = 0; j < numStandardBands; j++) {
        if (strcmp(standardBands[j].name, nextBandName) == 0) {
          freq = standardBands[j].freqHz;
          break;
        }
      }   
      // so all parameters are set and can be changed without restart.
    }

    // Start TX on second 0
    if (s == 0 && m == startMinute && activeBands !=0) {
      encode(); 
      bandStep++;
      
      // calculate next slot based on the interval set by the user
      // if bandStep is greater than actve bands then the cycle restartes after de set interval
      if (bandStep >= activeBands) {
        startMinute = (startMinute +2) + config.interval;
      } else {
        startMinute = (startMinute +2);
      }
      if (startMinute >= 60) startMinute -= 60; // wrap rond uur
    }

    int deltaMin = startMinute - m;
    if (deltaMin < 0) deltaMin += 60;
    int countdownSec = deltaMin * 60 - s;
    
    // Status page update 
    if (activeBands == 0) {
      statusLine1 = "!! NO BAND !!";
      statusLine2 = "check";
      statusLine3 = "Band Config";
      statusLine4 = "";
      blink(1);
    } else {
      if (countdownSec <= 1) {
        statusLine1 = "TRANSMITTING";
        statusLine2 = String((freq+offset) / 1000000.0, 6) + " MHz.";
        statusLine3 = ">> no interupt possible <<";
      } else {
        statusLine1 = "Idle for : " + String(countdownSec) + " s";
        statusLine2 = (offLine ? "Clock since start : " : "Current UTC time : ")
              + String(h) + ":" + (m < 10 ? "0" : "") + String(m) + ":" + (s < 10 ? "0" : "") + String(s);
        statusLine3 = String("Next Band : ") + nextBandName ;
      }
      statusLine4 = String(config.call) + " | " + String(config.loc) + " | " + String(config.dbm) + "dBm";
      // Statusled blinking op seconde basis
      if (countdownSec != lastSec) {   
        lastSec = countdownSec;
        // elke 2 seconden en de laatste 5 voor tx start
        if (countdownSec < 5 || countdownSec % 2 == 0) {  
          blink(1);
        }
      }
    }
  }
}

time_t epochUnixNTP() {
  // message on OLED
  Serial.print("Connecting to NTP Server: ");
  Serial.println(NTP_Server);

  NTPch.setSendInterval(SEND_INTV);
  NTPch.setRecvTimeout(RECV_TIMEOUT);

  unsigned long startTime = millis();  // start for timeout
  const unsigned long TIMEOUT = 60000; // 60 seconds

  do {
    dateTime = NTPch.getNTPtime(TIME_ZONE, 1);
    delay(1);
    // check timeout
    if (millis() - startTime >= TIMEOUT) {
      Serial.println("ERROR: NTP connection timeout after 60 seconds.");
      return 0;  
    }
  } while (!dateTime.valid);
  setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);
  char buf[60];
  sprintf(buf, "%02d:%02d:%02d  %02d-%02d-%04d",
        hour(), minute(), second(),
        day(), month(), year());
  Serial.println(buf); 
  return 0;
}

void encode() {
  // statusled on during transmission
  digitalWrite(StatusLed, HIGH);
  
  // encode call, locator and power into a wspr arrysymbols
  jtencode.wspr_encode(config.call, config.loc, config.dbm, tx_buffer);
  
  char msg[128];
  sprintf(msg, "- TX ON - STARTING TRANSMISSION AT : %02d:%02d - %luHz %s | %s | %ddbm",   hour(), minute(), (unsigned long)freq+offset, config.call, config.loc, config.dbm);
  Serial.println(msg);

  // initialise Si5351 CLK0
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

  unsigned long mStart = millis();

  // transmit WSPR message
  uint32_t next = micros();
  for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
    si5351.set_freq(((freq+offset) * 100) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
    next += WSPR_DELAY * 1000;
    while ((int32_t)(micros() - next) < 0);
  }

  // TX off
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  //debug  
  unsigned long mEnd = millis();              // finish
  Serial.print(F("WSPR doorlooptijd: "));
  Serial.print((mEnd - mStart) / 1000.0, 3);  // total wspr message in seconds /w 3 decimals
  Serial.println(F(" sec"));
  Serial.println("- TX OFF - END OF TRANSMISSION...");
  
  // statusled off
  digitalWrite(StatusLed, LOW);
}
