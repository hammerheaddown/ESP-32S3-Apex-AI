#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Arduino_JSON.h>
#include <WiFiClientSecure.h>
#include <Audio.h>
#include <esp_wifi.h>
#include <UrlEncode.h>
#include "time.h"

// Digital I/O used
#define I2S_BCLK          5
#define I2S_DOUT          6
#define I2S_LRC           4

#define SD_MISO            13
#define SD_MOSI            11
#define SD_SCK             12
#define SD_CS              10

//  edit these:
const char *ssid     = "";
const char *password = "";
const char* TTS_GOOGLE_LANGUAGE = "en";
const char* Gemini_Token = ""; // ***REPLACE WITH YOUR KEY***
const char* Gemini_Max_Tokens = "100";

// ------------------------------------------------------------------------------------------------------------------------------

Audio audio;
WiFiClientSecure client;

String lastQuestion = "";
const int MAX_CHUNK_LENGTH = 87;

// Declare askGemini() *before* it's used
void askGemini(String question = "");

// ------------------------------------------------------------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.print("\nConnected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Free Heap after WiFi connect: " + String(ESP.getFreeHeap()));

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21);
    Serial.println("Free Heap after audio setup: " + String(ESP.getFreeHeap()));

}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "gemini") {
            askGemini();
        } else if (command == "repeat") {
            repeatLastQuestion();
        } else {
            Serial.println("Invalid command. Use 'gemini' or 'repeat'.");
        }
    }
}

void repeatLastQuestion() {
    if (lastQuestion.length() > 0) {
        Serial.print("Repeating last question: ");
        Serial.println(lastQuestion);
        askGemini(lastQuestion);
    } else {
        Serial.println("No question to repeat.");
    }
}

void askGemini(String question) { // Definition - NO default argument here
    if (question.length() == 0) {
        Serial.println("Enter your question:");
        while (!Serial.available()) {
            delay(10);
        }
        question = Serial.readStringUntil('\n');
        question.trim();

        if (question.length() == 0) {
            Serial.println("Question cannot be empty.");
            return;
        }

        lastQuestion = question;
    } else {
        Serial.println("Asking Gemini: " + question);
    }

    String jsonQuestion = "\"" + question + "\"";

    HTTPClient https;

    if (https.begin("https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=" + (String)Gemini_Token)) {
        https.addHeader("Content-Type", "application/json");
        String payload = String("{\"contents\": [{\"parts\":[{\"text\":" + jsonQuestion + "}]}],\"generationConfig\": {\"maxOutputTokens\": " + (String)Gemini_Max_Tokens + "}}");

        int httpCode = https.POST(payload);

        if (httpCode > 0) {
            String payload = https.getString();

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, payload);

            if (error) {
                Serial.print("deserializeJson() failed: ");
                Serial.println(error.f_str());
                Serial.println(payload); // Print the received payload for debugging
                return;
            }

            if (doc["candidates"].is<JsonArray>() && doc["candidates"].size() > 0 &&
                doc["candidates"][0].is<JsonObject>() && doc["candidates"][0]["content"].is<JsonObject>() &&
                doc["candidates"][0]["content"]["parts"].is<JsonArray>() && doc["candidates"][0]["content"]["parts"].size() > 0 &&
                doc["candidates"][0]["content"]["parts"][0].is<JsonObject>() && doc["candidates"][0]["content"]["parts"][0]["text"].is<JsonString>()) {

                String answer = doc["candidates"][0]["content"]["parts"][0]["text"];

                answer.replace("**", "");
                answer.replace("*", "");

                answer.trim();
                String filteredAnswer = "";
                for (size_t i = 0; i < answer.length(); i++) {
                    char c = answer[i];
                    if (isalnum(c) || isspace(c) || ispunct(c)) {
                        filteredAnswer += c;
                    } else {
                        filteredAnswer += ' ';
                    }
                }
                answer = filteredAnswer;

                Serial.println("\nHere is your Answer:");
                Serial.println(answer);

                speakTextInChunks(answer);
            } else {
                Serial.println("Unexpected JSON format or missing data.");
                Serial.println(payload); // For debugging
            }

        } else {
            Serial.printf("[HTTPS] POST... failed, error: %s\n", https.errorToString(httpCode).c_str());
        }

        https.end();
    } else {
        Serial.printf("[HTTPS] Unable to connect\n");
    }
}

void speakTextInChunks(String text) {
    int start = 0;

    while (start < text.length()) {
        int end = start;
        int bestEnd = -1;
        int bestEndPriority = 0;

        while (end < text.length() && end - start < MAX_CHUNK_LENGTH) {
            if (text[end] == '.' || text[end] == '!' || text[end] == '?') {
                if (end > 1 && text[end - 1] == '.') {
                    if (end > 3 && (text.substring(end - 3, end) == "U.S." || text.substring(end - 4, end) == "U.S.A.")) {
                        end++;
                        continue;
                    }
                }
                bestEnd = end;
                bestEndPriority = 2;
                break;
            } else if (text[end] == ',') {
                int prevWordEnd = end - 1;
                while (prevWordEnd > start && text[prevWordEnd] != ' ') {
                    prevWordEnd--;
                }
                String tempWord = text.substring(prevWordEnd + 1, end);
                tempWord.trim();
                String prevWord = tempWord;

                if (!(prevWord.toInt())) {
                    bestEnd = end;
                    bestEndPriority = 1;
                }
            } else if (text[end] == ' ') {
                bestEnd = end;
                bestEndPriority = 3; // Highest priority for spaces
            }
            end++;
        }

        if (bestEnd != -1) {
            end = bestEnd + 1;
        } else {
            end = start + MAX_CHUNK_LENGTH;
            if (end > text.length()) {
                end = text.length();
            }
            int prevSpace = end;
            while (prevSpace > start && text[prevSpace] != ' ') {
                prevSpace--;
            }
            if (prevSpace > start) {
                end = prevSpace + 1;
            }
        }

        if (end > start) {
            int chunkLength = end - start;
            char* chunkChars = new char[chunkLength + 1];

            if (chunkLength > 0) {
                text.substring(start, end).toCharArray(chunkChars, chunkLength + 1);
                chunkChars[chunkLength] = '\0';
            } else {
                chunkChars[0] = '\0';
            }

            String chunk = String(chunkChars);
            chunk.trim();

            delete[] chunkChars;

            //Serial.println("Starting to speak chunk...");
            Serial.println("Free Heap bfore speaking chunk: " + String(ESP.getFreeHeap()));
            Serial.println(chunk);
            audio.connecttospeech(chunk.c_str(), TTS_GOOGLE_LANGUAGE);

            while (audio.isRunning()) {
                audio.loop();
                vTaskDelay(1);
            }

            Serial.println("Free Heap after Finished playing chunk: " + String(ESP.getFreeHeap()));
            delay(1);
        }

        start = end;
    }
}
