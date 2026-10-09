// Copy this file to secrets.h (same folder) and fill in your real
// network details. secrets.h is listed in .gitignore and is never
// committed — only this .example file is.
//
// Both anchor.ino and tag.ino #include "../secrets.h" and read
// WIFI_SSID / WIFI_PASS / MQTT_HOST from it.

#pragma once

const char* WIFI_SSID = "your-wifi-ssid";
const char* WIFI_PASS = "your-wifi-password";
const char* MQTT_HOST = "192.168.1.100";   // IP of the machine running server.js
const int   MQTT_PORT = 1883;
