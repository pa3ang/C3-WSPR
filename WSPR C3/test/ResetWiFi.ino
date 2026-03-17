#include <WiFiManager.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFiManager wm;

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(true);
  delay(1000);

  wm.setConfigPortalBlocking(true);
  wm.startConfigPortal("TestAP");

  bool res = wm.autoConnect("TestAP");

  if (!res) {
    Serial.println("Geen verbinding, AP actief");
  } else {
    Serial.println("Verbonden met WiFi");
  }
}

void loop() {}
