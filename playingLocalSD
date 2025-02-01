/* ESP32-S3 EXAMPLE */

#include "Arduino.h"
#include "Audio.h"
#include "WiFi.h"
#include "SD_MMC.h"
#include <SPI.h>
#include "driver/i2s_std.h" // or another appropriate I2S header


// Digital I/O used
#define I2S_BCLK 14
#define I2S_DOUT 13
#define I2S_LRC 12

#define SD_MISO            4
#define SD_MOSI            5
#define SD_SCK             12
#define SD_CS               11

#define I2S_WS 16
#define I2S_SD 15
#define I2S_SCK 2

Audio audio;

String ssid =     "MyCharterWiFic1-2G";
String password = "luckyjade033";

void setup() {
    Serial.begin(115200);

    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");

    // Initialize SD Card
    Serial.print("Initializing SD card...");
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH); // Ensure SD card is deselected
    delay(10); // Small delay to ensure pin state is set
    
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Set up SPI for SD card
    SPI.setFrequency(4000000); // Setting a lower SPI frequency to 4 MHz
    delay(1000); // Wait for a second before SD init

    if (!SD.begin(SD_CS)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialization done.");

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); 

    audio.connecttoFS(SD, "/When The Smoke Clears - Evening Telecast.mp3");
    
    Serial.print("Free Heap before operations: ");
    Serial.println(ESP.getFreeHeap());

    // List files before operation
    Serial.println("Files on SD before operation:");
    listFiles();

    
}

void loop() {
    audio.loop();
    vTaskDelay(1);
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}

void listFiles() {
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
    root.close();
}
