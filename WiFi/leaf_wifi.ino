// Import required libraries
#ifdef ESP32
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#else
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#endif
#include <Wire.h>

// declaration of a variable
String A, B, C, D, E, F;

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String getA() {
  return A;
}
String getB() {
  return B;
}
String getC() {
  return C;
}
String getD() {
  return D;
}
String getE() {
  return E;
}
String getF() {
  return F;
}

String getBGcolor() {
  File BGcolor = SPIFFS.open("/BGcolor.txt", "r");
  String value = BGcolor.readString();
  BGcolor.close();
  return (value);
}

String getHeading() {
  File Heading = SPIFFS.open("/Heading.txt", "r");
  String value = Heading.readString();
  Heading.close();
  return (value);
}

String getSsid() {
  File ssid = SPIFFS.open("/ssid.txt", "r");
  String value = ssid.readString();
  ssid.close();
  return (value);
}

String getPassword() {
  File password = SPIFFS.open("/password.txt", "r");
  String value = password.readString();
  password.close();
  return (value);
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(19200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println(getSsid());
  Serial.println(getPassword());
  //Start WiFi AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(getSsid(), getPassword());

  // Route for root / web pages
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html");
  });
  server.on("/admin", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/admin.html");
  });
  server.on("/highcharts.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts.js", "text/javascript");
  });
  server.on("/highcharts-more.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/highcharts-more.js", "text/javascript");
  });
  server.on("/solid-gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/solid-gauge.js", "text/javascript");
  });
  server.on("/getA", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getA().c_str());
  });
  server.on("/getB", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getB().c_str());
  });
  server.on("/getC", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getC().c_str());
  });
  server.on("/getD", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getD().c_str());
  });
  server.on("/getE", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getE().c_str());
  });
  server.on("/getF", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getF().c_str());
  });

  server.on("/getBGcolor", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getBGcolor().c_str());
  });
  server.on("/getSsid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getSsid().c_str());
  });
  server.on("/getPassword", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getPassword().c_str());
  });
  server.on("/getHeading", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getHeading().c_str());
  });
  server.on("/setBGcolor", HTTP_GET, [](AsyncWebServerRequest * request) {
    //Serial.println(request->getParam("favcolor")->value());
    File BGcolor = SPIFFS.open("/BGcolor.txt", "w");
    BGcolor.print(request->getParam("favcolor")->value());
    BGcolor.close();

    File Heading = SPIFFS.open("/Heading.txt", "w");
    Heading.print(request->getParam("heading")->value());
    Heading.close();

    File ssid = SPIFFS.open("/ssid.txt", "w");
    ssid.print(request->getParam("ssid")->value());
    ssid.close();

    File password = SPIFFS.open("/password.txt", "w");
    password.print(request->getParam("password")->value());
    password.close();

    request->redirect("/");
  });

  server.begin();
}

void loop() {
  String justRates = Serial.readStringUntil('\n');
  //split justrate variable from begining to first "," charactor
  int aSpaceIndex = justRates.indexOf(",");
  int bSpaceIndex = justRates.indexOf(",", aSpaceIndex + 1);
  int cSpaceIndex = justRates.indexOf(",", bSpaceIndex + 1);
  int dSpaceIndex = justRates.indexOf(",", cSpaceIndex + 1);
  int eSpaceIndex = justRates.indexOf(",", dSpaceIndex + 1);
  int fSpaceIndex = justRates.indexOf(",", eSpaceIndex + 1);
  int gSpaceIndex = justRates.indexOf(",", fSpaceIndex + 1);
  int hSpaceIndex = justRates.indexOf(",", gSpaceIndex + 1);
  int iSpaceIndex = justRates.indexOf(",", hSpaceIndex + 1);
  int jSpaceIndex = justRates.indexOf(",", iSpaceIndex + 1);
  int kSpaceIndex = justRates.indexOf(",", jSpaceIndex + 1);
  int lSpaceIndex = justRates.indexOf(",", kSpaceIndex + 1);
  A = (justRates.substring(1, aSpaceIndex));
  B = (justRates.substring(aSpaceIndex + 2, bSpaceIndex));
  C = (justRates.substring(bSpaceIndex + 2, cSpaceIndex));
  D = (justRates.substring(cSpaceIndex + 2, dSpaceIndex));
  E = (justRates.substring(dSpaceIndex + 2, eSpaceIndex));
  F = (justRates.substring(eSpaceIndex + 2));

  Serial.println();
  Serial.print("Serial input - ");
  Serial.println(justRates);
  Serial.print("Split values - ");
  Serial.println("A : " + String(A) + " B : " + String(B) + " C : " + (C) + " D : " + String(D) + " E : " + String(E) + " F : " + String(F));
}
