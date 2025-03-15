// Suppress annoying library warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"

#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <chrono> // For timekeeping
#include <SD.h>
#include <SPI.h>

// -------------------------- AMP Pin Setup ------------------------------------------
#define I2S_BCLK GPIO_NUM_5
#define I2S_DOUT GPIO_NUM_6
#define I2S_LRC  GPIO_NUM_14
i2s_chan_handle_t tx_handle;

// -------------------------- SD Pin Setup ------------------------------------------
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_SCK GPIO_NUM_12
#define SD_CS GPIO_NUM_10

#define SAMPLE_RATE 22050 // Deepgram Aura likely uses 22.05kHz, adjust if needed
AudioInfo info(SAMPLE_RATE, 1, 16);

// -------------------------- Wifi Credentials ------------------------------------------
const char *ssid = "";
const char *password = "";

// -------------------------- Deepgram Credentials ------------------------------------------
const char *deepgramApiKey = "";
#define STT_LANGUAGE "en"
#define TIMEOUT_DEEPGRAM 12
#define STT_KEYWORDS "&keywords=Gigolo&keywords=Google"
String optional_param = "?model=nova-2-general";

// -------------------------- Audio Tools Setup ------------------------------------------
URLStream url;
I2SStream i2s;
EncodedAudioStream decoder(&i2s, new MP3DecoderHelix());
StreamCopy copier(decoder);

// -------------------------- Misc Setup ------------------------------------------
unsigned long lastKeepAliveTime = 0;
const unsigned long keepAliveInterval = 15000; // 10 seconds

void SD_setup() {
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(10);
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    SPI.setFrequency(4000000); // 4MHz is reasonable for SD
    delay(100);
    if (!SD.begin(SD_CS)) {
        Serial.println("SD card mount failed!");
        while (1) delay(1000); // Halt if SD fails
    }
    Serial.println("SD card mounted successfully.");
}

void deepgramTTS(const char* textToSpeech) {
    char escapedText[512];
    int j = 0;
    for (int i = 0; textToSpeech[i] != '\0'; i++) {
        if (textToSpeech[i] == '"') {
            escapedText[j++] = '\\';
        }
        escapedText[j++] = textToSpeech[i];
    }
    escapedText[j] = '\0';

    char requestBody[1024];
    JsonDocument doc;
    doc["text"] = escapedText;
    serializeJson(doc, requestBody);

    Serial.println("RequestBody: " + String(requestBody));
    Serial.println("Free Heap before TTS fetch: " + String(ESP.getFreeHeap()));

    // Open file for writing
    if (SD.exists("/recordings/temp.mp3")) {
        SD.remove("/recordings/temp.mp3");
    }
    File mp3File = SD.open("/recordings/temp.mp3", FILE_WRITE);
    if (!mp3File) {
        Serial.println("Failed to open /recordings/temp.mp3 for writing");
        return;
    }

    // HTTP setup
    HTTPClient http;
    http.begin("https://api.deepgram.com/v1/speak?model=aura-luna-en");
    http.addHeader("Authorization", "Token " + String(deepgramApiKey));
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(requestBody);
    Serial.print("HTTP Response Code: ");
    Serial.println(httpCode);

    if (httpCode == HTTP_CODE_OK) {
        AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
        Stream& httpStream = http.getStream();
        const size_t bufferSize = 512;
        uint8_t buffer[bufferSize];
        size_t totalBytes = 0;

        // Setup MultiOutput
        MultiOutput multiOut;
        I2SStream i2s;
        EncodedAudioStream decoder(&i2s, new MP3DecoderHelix());
        multiOut.add(mp3File); // Save to SD
        multiOut.add(decoder); // Play via I2S
        multiOut.begin();

        // Configure I2S
        auto cfg = i2s.defaultConfig(TX_MODE);
        cfg.pin_bck = I2S_BCLK;
        cfg.pin_ws = I2S_LRC;
        cfg.pin_data = I2S_DOUT;
        cfg.i2s_format = I2S_STD_FORMAT;
        cfg.sample_rate = 22050;
        cfg.channels = 1;
        cfg.bits_per_sample = 16;
        i2s.begin(cfg);
        decoder.begin();

        bool dataAvailable = true;
        unsigned long lastDataTime = millis();
        unsigned long firstChunkTime = 0;
        bool firstChunkProcessed = false;

        // Fetch and play simultaneously
        while (dataAvailable) {
            if (httpStream.available()) {
                size_t bytesRead = httpStream.readBytes(buffer, bufferSize);
                if (bytesRead > 0) {
                    if (firstChunkTime == 0) {
                        firstChunkTime = millis(); // Time when first chunk is received
                        Serial.println("First chunk received at: " + String(firstChunkTime) + " ms");
                    }
                    multiOut.write(buffer, bytesRead); // Write to SD and decode/play
                    if (!firstChunkProcessed) {
                        unsigned long playbackStartTime = millis();
                        Serial.println("Playback started at: " + String(playbackStartTime) + " ms");
                        Serial.println("Delay from first chunk to playback: " + String(playbackStartTime - firstChunkTime) + " ms");
                        firstChunkProcessed = true;
                    }
                    totalBytes += bytesRead;
                    // Serial.println("Fetched and played " + String(totalBytes) + " bytes so far");
                    lastDataTime = millis();
                }
            } else {
                if (millis() - lastDataTime > 5000) {
                    Serial.println("No more data received, stopping fetch and playback");
                    dataAvailable = false;
                } else {
                    delay(100);
                }
            }
        }

        // Flush and cleanup
        multiOut.flush();
        multiOut.end();
        decoder.end();
        i2s.end();
        mp3File.close();

        if (totalBytes == 0) {
            Serial.println("No MP3 data fetched");
            SD.remove("/recordings/temp.mp3");
            return;
        }

        // Verify file size
        File verifyFile = SD.open("/wakewords/temp.mp3", FILE_READ);
        if (verifyFile) {
            Serial.println("Verified MP3 file size on SD: " + String(verifyFile.size()) + " bytes");
            verifyFile.close();
        } else {
            Serial.println("Failed to verify MP3 file size on SD");
        }

        Serial.println("MP3 fetched, saved, and played from /wakewords/temp.mp3: " + String(totalBytes) + " bytes, Free heap: " + String(ESP.getFreeHeap()));
        http.end();
        delay(100); 
   
    } else {
        Serial.printf("[HTTP] POST TTS... failed, error: %s\n", http.errorToString(httpCode).c_str());
        Serial.println("Response: " + http.getString());
        mp3File.close();
        SD.remove("/recordings/temp.mp3");
    }

    
}

void setup() {
    Serial.begin(115200);
    delay(10);

    SD_setup();
    #ifdef DEBUG_MODE
    Serial.println("Free Heap after SD setup: " + String(ESP.getFreeHeap()));
    #endif

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("Connected to WiFi");
}

void loop() {
    deepgramTTS("I am truly sorry for the repeated errors. I am still under development and learning, but this is a particularly bad mistake that I need to address. Thank you for your patience and for pointing out my error..");
    delay(1000000);
}
