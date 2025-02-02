// esp 32 dev kit

#include "driver/i2s.h"
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <math.h> // For M_PI definition
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Micro SD TF Card Adater Reader Module 6Pin SPI Interface Driver card pins
#define SD_CS          5
#define SD_MOSI       23
#define SD_MISO       19
#define SD_SCK        18

//INMP441 Omnidirectional Mic pins
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_LEFT_RIGHT_CLOCK    GPIO_NUM_25
#define I2S_MIC_SERIAL_DATA         GPIO_NUM_32
#define I2S_MIC_SERIAL_CLOCK        GPIO_NUM_33

// MAX98357 I2S Audio Amplifier pins
#define I2S_BCLK GPIO_NUM_14
#define I2S_DOUT GPIO_NUM_13
#define I2S_LRC GPIO_NUM_12

// Edit as you need here
#define VOLUME_SCALE 0.12  // Volume scaling factor (0.0 — silence, 1.0 — original level)
#define DURATION_SEC 10
const int16_t noiseGateThreshold = 1500;  // Anything over 2000 can start to lose human voice

#define LOW_PASS_FREQ        4000 // Hz    6000
#define HIGH_PASS_FREQ       100  // Hz     200
#define SAMPLE_RATE 16000  // Sampling rate (16 kHz)
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT  // 16 bits per sample
#define BUFFER_SIZE 2048  // Buffer size

// NTP Configuration
const char *ssid     = "";
const char *password = "";
long gmtOffset_sec = 0;  // Global variable to store the timezone offset

// Global buffers for recording
int32_t buffer32[BUFFER_SIZE];  // Buffer for 32-bit data from the microphone
int16_t buffer16[BUFFER_SIZE];  // Buffer for 16-bit data to be saved to the file

// High Pass Filter function
float highPassFilter(int16_t sample, float *lastSample, float cutoffFreq, float sampleRate) {
    float RC = 1.0 / (cutoffFreq * 2 * M_PI);
    float dt = 1.0 / sampleRate;
    float alpha = RC / (RC + dt);
    *lastSample = sample * (1.0 - alpha) + (*lastSample) * alpha;
    return sample - *lastSample;
}

// Low Pass Filter function
float lowPassFilter(int16_t sample, float *lastSample, float cutoffFreq, float sampleRate) {
    float RC = 1.0 / (cutoffFreq * 2 * M_PI);
    float dt = 1.0 / sampleRate;
    float alpha = dt / (RC + dt);
    *lastSample = alpha * sample + (1 - alpha) * (*lastSample);
    return *lastSample;
}

// Noise Gate function
bool noiseGate(int16_t sample, int16_t threshold) {
    // Absolute value of the sample to check against the threshold
    return abs(sample) > threshold;
}

void SD_setup() {
 // Initialize SD Card
    Serial.print("Initializing SD card...");
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH); // Ensure SD card is deselected
    delay(10); // Small delay to ensure pin state is set
    
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // Set up SPI for SD card
   
  // Initialize SPI for SD card
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    SPI.setFrequency(4000000); // Setting a lower SPI frequency to 4 MHz
    delay(1000); // Wait for a second before SD init

      if (!SD.begin(SD_CS)) {
        Serial.println("SD card mount failed!");
        Serial.println("Please check your SD card connection or formatting.");
        while (1);
      } else {
        Serial.println("SD card mounted successfully.");
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
          Serial.println("No SD card attached");
          while (1);
        } else {
          Serial.print("SD Card Type: ");
          if (cardType == CARD_MMC) {
            Serial.println("MMC");
          } else if (cardType == CARD_SD) {
            Serial.println("SDSC");
          } else if (cardType == CARD_SDHC) {
            Serial.println("SDHC");
          } else {
            Serial.println("UNKNOWN");
          }
          uint64_t cardSize = SD.cardSize() / (1024 * 1024);
          Serial.printf("SD Card Size: %lluMB\n", cardSize);
        }
      }

      // Ensure the "recordings" directory exists
      if (!SD.exists("/recordings")) {
        SD.mkdir("/recordings");
        Serial.println("Created recordings directory.");
      }

    Serial.print("Free Heap before operations: ");
    Serial.println(ESP.getFreeHeap());

    listFiles();
}

void i2s_setup() {

    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false,

        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // Pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK,
        .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SERIAL_DATA
    };

    if (i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Error installing I2S driver");
        while (1);
    }

    if (i2s_set_pin(I2S_NUM_0, &pin_config) != ESP_OK) {
        Serial.println("Error configuring I2S pins");
        while (1);
    }
  }

// Get Current Time as Filename-Friendly String
String getCurrentTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "UNKNOWN_TIME";
  }
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%02I-%M-%S-%m-%d-%Y", &timeinfo);
  return String(buffer);
}

long customStringToInt(String s) {
  long result = 0;
  int sign = 1;
  if (s.charAt(0) == '-') {
    sign = -1;
    s = s.substring(1);  // Remove the negative sign
  }
  for (int i = 0; i < s.length(); i++) {
    result = result * 10 + (s.charAt(i) - '0');
  }
  return sign * result;
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  
  // Fetch time offset from WorldTimeAPI only once in setup
  HTTPClient http;
  if (http.begin("http://worldtimeapi.org/api/ip")) {
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      int offsetStart = payload.indexOf("\"raw_offset\":") + 13; // Changed to 13 to include the negative sign if present
      int offsetEnd = payload.indexOf(",", offsetStart);
      String rawOffsetStr = payload.substring(offsetStart, offsetEnd); // Removed .trim() here, it was causing the issue

      // Debug print the string before conversion
      //Serial.print("Raw Offset String: ");
      Serial.println(rawOffsetStr);

      // Convert to long, ensuring negative values are preserved
      long offset = 0;
      int sign = 1;
      if (rawOffsetStr.charAt(0) == '-') {
        sign = -1;
        rawOffsetStr = rawOffsetStr.substring(1);  // Remove the negative sign
      }
      for (int i = 0; i < rawOffsetStr.length(); i++) {
        offset = offset * 10 + (rawOffsetStr.charAt(i) - '0');
      }
      gmtOffset_sec = sign * offset;

      // Debug print after conversion
      //Serial.print("GMT Offset (sec): ");
      Serial.println(gmtOffset_sec);

      // Parse client IP and timezone
      int ipStart = payload.indexOf("\"client_ip\":\"") + 13;
      int ipEnd = payload.indexOf("\"", ipStart);
      String clientIP = payload.substring(ipStart, ipEnd);
      
      int tzStart = payload.indexOf("\"timezone\":\"") + 12;
      int tzEnd = payload.indexOf("\"", tzStart);
      String timezone = payload.substring(tzStart, tzEnd);

      Serial.print("Client IP: ");
      Serial.println(clientIP);
      Serial.print("Timezone: ");
      Serial.println(timezone);
    }
    http.end();
  }
}

// Setup NTP Synchronization
void setupNTP() {
  const char *ntpServer = "pool.ntp.org";
  const int daylightOffset_sec = 3600;  // Assuming DST is one hour

  // Use the global gmtOffset_sec here
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
 // Serial.print("Configuring NTP with offset: ");
  Serial.println(gmtOffset_sec);
 // Serial.println("NTP synchronization complete.");
}

void setup() {
    Serial.begin(115200);
        setupWiFi();
        setupNTP();  
        SD_setup();
        i2s_setup();
        start_recording();
}

void loop() {

}

// Record sound 
void start_recording() {

    // Start recording
    Serial.println("Recording Audio...");

    // File name for saving
    String filename = "/recordings/" + getCurrentTimestamp() + ".wav";
    recordAudio(filename.c_str(), DURATION_SEC);
    Serial.println("Recording saved to file: " + filename);

}

// Record sound in WAV format
void recordAudio(const char *path, uint32_t duration) {
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    // WAV file header
    writeWAVHeader(file, SAMPLE_RATE, 1, BITS_PER_SAMPLE);

    uint32_t samples = SAMPLE_RATE * duration;  // Number of samples
    
    // Variables for filters
    float lastSampleHP = 0;
    float lastSampleLP = 0;

    while (samples > 1024) {
        size_t bytesRead = 0;
        // Read data from the I2S microphone
        i2s_read(I2S_NUM_0, buffer32, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);

        // Convert 32-bit data to 16-bit, apply noise gate, then filters
        for (int i = 0; i < bytesRead / sizeof(int32_t); i++) {
            int16_t sample = (int16_t)((buffer32[i] >> 8) * VOLUME_SCALE);
            
            // Apply noise gate
            if (!noiseGate(sample, noiseGateThreshold)) {
                sample = 0; // Mute if below threshold
            } else {


                sample = (int16_t)highPassFilter(sample, &lastSampleHP, HIGH_PASS_FREQ, SAMPLE_RATE);  // High pass
                sample = (int16_t)lowPassFilter(sample, &lastSampleLP, LOW_PASS_FREQ, SAMPLE_RATE);  // Low pass

            }
            buffer16[i] = sample;
        }

        // Write data to the file
        file.write((uint8_t *)buffer16, bytesRead / 2);
        samples -= bytesRead / sizeof(int32_t);
    }

    // Update the WAV file header
    updateWAVHeader(file);
    file.close();
}

// Function to write the WAV header
void writeWAVHeader(File &file, uint32_t sampleRate, uint16_t channels, uint16_t bitsPerSample) {
    uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
    uint16_t blockAlign = channels * (bitsPerSample / 8);

    file.write((const uint8_t *)"RIFF", 4);  // ChunkID
    uint32_t chunkSize = 0; // Size will be updated later
    file.write((const uint8_t *)&chunkSize, 4);  // ChunkSize
    file.write((const uint8_t *)"WAVE", 4);  // Format
    file.write((const uint8_t *)"fmt ", 4);  // Subchunk1ID
    uint32_t subChunk1Size = 16;
    file.write((const uint8_t *)&subChunk1Size, 4);  // Subchunk1Size
    uint16_t audioFormat = 1;  // PCM
    file.write((const uint8_t *)&audioFormat, 2);  // AudioFormat
    file.write((const uint8_t *)&channels, 2);  // NumChannels
    file.write((const uint8_t *)&sampleRate, 4);  // SampleRate
    file.write((const uint8_t *)&byteRate, 4);  // ByteRate
    file.write((const uint8_t *)&blockAlign, 2);  // BlockAlign
    file.write((const uint8_t *)&bitsPerSample, 2);  // BitsPerSample
    file.write((const uint8_t *)"data", 4);  // Subchunk2ID
    uint32_t subChunk2Size = 0; // Size will be updated later
    file.write((const uint8_t *)&subChunk2Size, 4);  // Subchunk2Size
}

// Function to update the WAV header
void updateWAVHeader(File &file) {
    uint32_t fileSize = file.size();
    uint32_t dataSize = fileSize - 44;

    file.seek(4);  // Move to the ChunkSize field
    uint32_t chunkSize = fileSize - 8;
    file.write((const uint8_t *)&chunkSize, 4);

    file.seek(40);  // Move to the Subchunk2Size field
    file.write((const uint8_t *)&dataSize, 4);
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

