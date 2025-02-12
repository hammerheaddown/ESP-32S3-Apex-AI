// ------------------------------------------------------------------------------------------------------------------------------
// ------------------                                                                                          ------------------
// ------------------                            Hardware:                                                     ------------------ 
// ------------------                 INMP441 Omnidirectional Microphone                                       ------------------
// ------------------                 ESP32-S3-DevKit N16R8 Development Board                                  ------------------
// ------------------                 MAX98357 I2S Audio Amplifier Module                                      ------------------
// ------------------                                                                                          ------------------ 
// ------------------             This is the modified code and the origanal code was                          -----------------
// ------------------               from KALOPROJECTS. A huge Shoutout to his amazing work                      -----------------
// ------------------   KALO PROJECTS Github Repo - https://github.com/kaloprojects/KALO-ESP32-Voice-Assistant  -----------------
// ------------------------------------------------------------------------------------------------------------------------------


#include "driver/i2s_std.h"                // Use this instead of deprecated i2s.h
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

#ifndef DEBUG
#  define DEBUG true
#  define DebugPrint(x);      if(DEBUG){Serial.print(x);}
#  define DebugPrintln(x);    if(DEBUG){Serial.println(x);}
#endif

// api 
const char *ssid     = "";                              // ***REPLACE WITH YOUR WIFI SSID ***
const char *password = "";                              // ***REPLACE WITH YOUR WIFI PASSWORD***


const char* deepgramApiKey = "";                        // ***REPLACE WITH YOUR KEY***
#define STT_LANGUAGE      "en"
#define TIMEOUT_DEEPGRAM   12
#define STT_KEYWORDS       "&keywords=Gigolo&keywords=Google"


// --------------------  Google TTS  (not used here but will be in next release   -------------------------------------------

const char* TTS_GOOGLE_LANGUAGE = "en";
const char* Gemini_Token = "";                         // ***REPLACE WITH YOUR KEY***
const char* Gemini_Max_Tokens = "100";

const int MAX_CHUNK_LENGTH = 87;
void askGemini(String question = "");
String lastQuestion = "";

// --------------------------    Mic Pin Setup    -------------------------------------------------------------------------------
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_LEFT_RIGHT_CLOCK      GPIO_NUM_2        // WS (Word Select) - ESP32-S3 GPIO2
#define I2S_MIC_SERIAL_DATA           GPIO_NUM_4        // SD (Serial Data)  - ESP32-S3 GPIO4
#define I2S_MIC_SERIAL_CLOCK          GPIO_NUM_15       // SCK (Serial Clock) - ESP32-S3 GPIO15
i2s_chan_handle_t mic_rx_handle;

// --------------------------   SD Pin Setup      -------------------------------------------------------------------------------
#define SD_MISO         13
#define SD_MOSI         11
#define SD_SCK          12
#define SD_CS           10

// --------------------------   AMP Pin Setup    -------------------------------------------------------------------------------
#define I2S_BCLK GPIO_NUM_14
#define I2S_DOUT GPIO_NUM_13
#define I2S_LRC GPIO_NUM_12

// --------------------------  Recording Setup    -------------------------------------------------------------------------------
bool isRecording = false;
File recordingFile;
float lastSampleHP = 0;
float lastSampleLP = 0;
unsigned long recordingStartTime = 0;
unsigned long countdownStartTime = 0;
#define SAMPLE_RATE 8000                        // Sampling rate (up to 44.1 kHz)
#define BUFFER_SIZE 2048                        // Buffer size
#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_8BIT

// Global buffers for recording
int32_t buffer32[BUFFER_SIZE];                  // Buffer for 32-bit data from the microphone
int16_t buffer16[BUFFER_SIZE];                  // Buffer for 16-bit data to be saved to the file

// Edit as you need here
#define VOLUME_SCALE 0.125                      // Volume scaling factor (0.0 — silence, 1.0 — original level)
#define DURATION_SEC 10
const int16_t noiseGateThreshold = 750;          // Anything over 2000 can start to lose human voice

#define LOW_PASS_FREQ        5000               // Hz    
#define HIGH_PASS_FREQ       200                // Hz     

// --------------------------    NTP Configuration   ---------------------------------------------------------------------------
long gmtOffset_sec = 0;  // Global variable to store the timezone offset

// --------------------------    Timer Set up       ---------------------------------------------------------------------------
int countdown = 10;                             // Initial countdown value
int currentCountdown = countdown;               // Holds the active countdown value
bool timerRunning = false;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;            // 1-second interval
const unsigned long startDelay = 1000;          // 1 second delay before starting countdown
const unsigned long stopDelay = 1000;           // 1 second delay after stopping countdown before stopping recording

// --------------------------    Misc  dependencies   ---------------------------------------------------------------------------
//Audio audio;
WiFiClientSecure client;

// ------------------------------------------------------------------------------------------------------------------------------

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

String json_object(String input, String element) {
  String content = "";
  int pos_start = input.indexOf(element);
  if (pos_start > 0) {
    pos_start += element.length();
    int pos_end = input.indexOf(",\"", pos_start);
    if (pos_end > pos_start) {
      content = input.substring(pos_start, pos_end);
    }
    content.trim();
    if (content.startsWith("\"")) {
      content = content.substring(1, content.length() - 1);
    }
  }
  return (content);
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

}

void i2s_setup() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &mic_rx_handle);

    i2s_std_config_t  std_cfg = { 
          .clk_cfg  = { 
          .sample_rate_hz = SAMPLE_RATE,
          .clk_src = I2S_CLK_SRC_DEFAULT,
          .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg =  { 
          .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,    
          .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO, 
          .slot_mode = I2S_SLOT_MODE_MONO,           
          .slot_mask = I2S_STD_SLOT_LEFT,              
          .ws_width =  I2S_DATA_BIT_WIDTH_16BIT,           
          .ws_pol = false, 
          .bit_shift = true,   
        },
        .gpio_cfg =   { 
          .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_MIC_SERIAL_CLOCK,
            .ws = (gpio_num_t)I2S_MIC_LEFT_RIGHT_CLOCK,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t)I2S_MIC_SERIAL_DATA,
          .invert_flags = 
          { .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
          },
        },
      };

  /*
        i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, 
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_MIC_SERIAL_CLOCK,
            .ws = (gpio_num_t)I2S_MIC_LEFT_RIGHT_CLOCK,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t)I2S_MIC_SERIAL_DATA,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

  */

    if (i2s_channel_init_std_mode(mic_rx_handle, &std_cfg) != ESP_OK) {
        Serial.println("Error installing I2S driver");
        while (1);
    }

    if (i2s_channel_enable(mic_rx_handle) != ESP_OK) {
        Serial.println("Error enabling I2S channel");
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

void setupNTP() {
  const char *ntpServer = "pool.ntp.org";
  const int daylightOffset_sec = 3600;  // Assuming DST is one hour

  // Use the global gmtOffset_sec here
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void setup() {
    Serial.begin(115200);

    setupWiFi();
    delay(10);
    Serial.println("Free Heap after wifi setup: " + String(ESP.getFreeHeap()));

    setupNTP();  
    delay(10);
    Serial.println("Free Heap after NTP setup: " + String(ESP.getFreeHeap()));

    SD_setup();
    delay(10);
    Serial.println("Free Heap after SD setup: " + String(ESP.getFreeHeap()));

    i2s_setup();   
    delay(10);
    Serial.println("Free Heap after mic setup: " + String(ESP.getFreeHeap()));

    Serial.print("Sample rate: ");
    Serial.println(SAMPLE_RATE);
    Serial.print("Bits per sample: ");
    Serial.println(BITS_PER_SAMPLE);
/*
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT); // If you are still using this for audio output
    audio.setVolume(21);

    audio.connecttospeech("Welcome back Chaddoe !! What can i help you with today? ", "en"); // Google TTS          
    while (audio.isRunning()) {audio.loop();} 

*/
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("start")) {
      if (!timerRunning) {
        Serial.println("Starting timer...");
        timerRunning = true;
        currentCountdown = countdown;
        start_recording();
        Serial.println("Recording started...");
        countdownStartTime = millis() + startDelay; // Delay countdown start
      } else {
        Serial.println("Recording is already running.");
      }
    }

    if (command.equalsIgnoreCase("stop")) {
      if (timerRunning) {
        timerRunning = false;
        String filename = recordingFile.name();
        stop_recording();
        finishedRecording(filename);
        //uploadToDeepgram(filename);  // Call this after recording is finished
      } else {
        Serial.println("No recording is running.");
      }
    }

    if (command.equalsIgnoreCase("recordings")) {
      listRecordings();  // List recordings when "recordings" is received
    }
  }

  if (timerRunning) {
    unsigned long currentMillis = millis();
    if (currentMillis >= countdownStartTime && currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      Serial.printf("%d seconds left\n", currentCountdown);
      currentCountdown--;

      if (currentCountdown == 0) {
        Serial.println("Debug: Countdown reached zero, preparing to stop recording...");
        delay(stopDelay); // Wait before stopping recording
        String filename = recordingFile.name();
        stop_recording();
        finishedRecording(filename);
       // uploadToDeepgram(filename);  // Call this after recording is finished
        Serial.println("Recording finished!");
        timerRunning = false;
      }
    }
  }
}

void start_recording() {
  if (!isRecording) {
    Serial.println("Debug: Attempting to start recording...");
    String filename = "/recordings/" + getCurrentTimestamp() + ".wav";
    recordingFile = SD.open(filename, FILE_WRITE);
    if (!recordingFile) {
      Serial.println("Failed to open file for writing.");
      timerRunning = false; 
      return;
    }
    Serial.println("File opened for writing: " + filename);
    writeWAVHeader(recordingFile, SAMPLE_RATE, 1, I2S_DATA_BIT_WIDTH_16BIT);
    isRecording = true;
    recordingStartTime = millis(); 
    xTaskCreate(recordAudioTask, "recordAudioTask", 8192, NULL, 2, NULL); // Higher priority with larger stack size
  } else {
    Serial.println("Already recording!");
  }
}

void stop_recording() {
  if (isRecording) {
    Serial.println("Debug: Stopping recording...");
    isRecording = false;
    if (recordingFile) {
      updateWAVHeader(recordingFile);
      recordingFile.close();
      Serial.println("Recording stopped and file closed.");
    }
  } else {
    Serial.println("No recording to stop!");
  }
}

void recordAudioTask(void *pvParameters) {
  //Serial.println("Debug: Record Audio Task started");
  while (1) { 
    if (!isRecording) {
      Serial.println("Debug: Record Audio Task noticed stop signal");
      vTaskDelete(NULL); // Clean up by deleting the task
    }
    if (!recordAudio(recordingFile.name(), 1)) {
      if (isRecording) {
        Serial.println("Error in recording audio!");
      } else {
        // Serial.println("Recording stopped normally");
      }
      vTaskDelete(NULL); // Stop task
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay if write was successful
  }
}

bool recordAudio(const char *path, uint32_t duration) {
    if (!recordingFile) {
        Serial.println("Failed to open file for writing in record audio");
        return false;
    }

    uint32_t samples = SAMPLE_RATE * duration;
    
    while (samples > 0 && isRecording) {
        size_t bytesRead = 0;
        i2s_channel_read(mic_rx_handle, buffer32, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);

        if (bytesRead == 0) {
            Serial.println("No data read from microphone");
            continue;  // No data read, try again
        }

        for (int i = 0; i < bytesRead / sizeof(int32_t); i++) {
            int16_t sample = (int16_t)((buffer32[i] >> 8) * VOLUME_SCALE);
            
            if (!noiseGate(sample, noiseGateThreshold)) {
                sample = 0;
            } else {
                sample = (int16_t)highPassFilter(sample, &lastSampleHP, HIGH_PASS_FREQ, SAMPLE_RATE);  
                sample = (int16_t)lowPassFilter(sample, &lastSampleLP, LOW_PASS_FREQ, SAMPLE_RATE);  
            }
            buffer16[i] = sample;
        }
  
        if (recordingFile.write((uint8_t *)buffer16, bytesRead / 2) != (bytesRead / 2)) {
            //Serial.println("Failed to write audio data to file. Bytes attempted: " + String(bytesRead / 2));
            // Serial.println("File position before write: " + String(recordingFile.position()));
            //Serial.println("SD Card Free Space: " + String(SD.totalBytes() - SD.usedBytes()));
            return false;
        }
        
        samples -= bytesRead / sizeof(int32_t);
    }

    return true;
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

void finishedRecording(String filename) {
  Serial.print(filename);
  Serial.println(" is done recording and available to listen to.");
  
  String fullPath = "/recordings/" + filename;
  File file = SD.open(fullPath, FILE_READ);
  if (!file) {
    Serial.println("Error: Could not open file to check size.");
    return; // No need to attempt upload if we can't even read the file
  }

  size_t fileSize = file.size();
  file.close();

  Serial.print("File size: ");  // Corrected the print statement for better readability
  Serial.println(fileSize);

  if (fileSize <= 44) {  // 44 is the size of a WAV header without data
    Serial.println("Warning: No audio data recorded, skipping Deepgram upload.");
  } else {
    
    uploadToDeepgram(filename); // Commented out to prevent upload
  }
}

void uploadToDeepgram(String filename) {
  Serial.println("Uploading " + filename + " to Deepgram for transcription...");
  
  if (!client.connected()) {
    DebugPrintln("> Initialize Deepgram Server connection ... ");
    client.setInsecure();
    if (!client.connect("api.deepgram.com", 443)) {
      Serial.println("\nERROR - WifiClientSecure connection to Deepgram Server failed!");
      client.stop(); 
      return;
    }
    DebugPrintln("Done. Connected to Deepgram Server.");
  }

  String fullPath = "/recordings/" + filename;  // Assuming the file is in the recordings directory
  if (!SD.exists(fullPath)) {
    Serial.println("ERROR - File does not exist: " + fullPath);
    return;
  }

  File audioFile = SD.open(fullPath, FILE_READ);
  if (!audioFile) {
    Serial.print("ERROR - Failed to open file for reading at path: ");
    Serial.println(fullPath);
    return;
  }
  
  size_t audio_size = audioFile.size();
  audioFile.close();
  DebugPrintln("> Audio File [" + fullPath + "] found, size: " + (String) audio_size);

  // Clear any existing data from the client
  while (client.available()) { client.read(); }

  // Sending HTTPS request header to Deepgram Server
  String optional_param = "?model=nova-2-general";
 // optional_param += (STT_LANGUAGE != "") ? ("&language=" + (String)STT_LANGUAGE) : ("&detect_language=true");
  optional_param += (strcmp(STT_LANGUAGE, "") == 0) ? "&detect_language=true" : "&language=" + String(STT_LANGUAGE);
  optional_param += "&smart_format=true";
  optional_param += "&numerals=true";
  optional_param += STT_KEYWORDS;

  client.println("POST /v1/listen" + optional_param + " HTTP/1.1"); 
  client.println("Host: api.deepgram.com");
  client.println("Authorization: Token " + String(deepgramApiKey));
  client.println("Content-Type: audio/wav");
  client.println("Content-Length: " + String(audio_size));
  client.println();

  DebugPrintln("> POST Request to Deepgram Server started, sending WAV data now ...");

  File file = SD.open(fullPath, FILE_READ);
  const size_t bufferSize = 1024; 
  uint8_t buffer[bufferSize];
  size_t bytesRead;
  while (file.available()) {
    bytesRead = file.read(buffer, sizeof(buffer));
    if (bytesRead > 0) { client.write(buffer, bytesRead); }
  }
  file.close();
  DebugPrintln("> All bytes sent, waiting for Deepgram transcription");

  String response = "";
  unsigned long timeout = millis() + TIMEOUT_DEEPGRAM * 1000;
  while (response == "" && millis() < timeout) {
    while (client.available()) {
      response += (char)client.read();
    }
    DebugPrint(".");  
    delay(100);
  }
  DebugPrintln();
  
  if (millis() >= timeout) {
    Serial.print("\n*** TIMEOUT ERROR - forced TIMEOUT after " + (String) TIMEOUT_DEEPGRAM + " seconds");
    Serial.println(" (is your Deepgram API Key valid ?) ***\n");
  } else {
    // Parse the response here if needed
    String transcription = json_object(response, "\"transcript\":");
    DebugPrintln("Transcription: " + transcription);
  }

  client.stop();
}

void listRecordings() {
    Serial.println("Listing files in /recordings:");
    File dir = SD.open("/recordings");
    if (!dir) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!dir.isDirectory()) {
        Serial.println("Not a directory");
        dir.close();
        return;
    }

    File file = dir.openNextFile();
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
        file = dir.openNextFile();
    }
    file.close();
    dir.close();
}
