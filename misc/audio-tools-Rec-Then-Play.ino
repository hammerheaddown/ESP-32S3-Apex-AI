#include "driver/i2s_std.h"
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecWAV.h"

// -------------------------- Mic Pin Setup --------------------------------------
#define I2S_MIC_LEFT_RIGHT_CLOCK    GPIO_NUM_42
#define I2S_MIC_SERIAL_DATA         GPIO_NUM_17
#define I2S_MIC_SERIAL_CLOCK        GPIO_NUM_18

// -------------------------- SD Pin Setup --------------------------------------
#define SD_MISO                     GPIO_NUM_13
#define SD_MOSI                     GPIO_NUM_11
#define SD_SCK                      GPIO_NUM_12
#define SD_CS                       GPIO_NUM_10

// -------------------------- AMP Pin Setup --------------------------------------
#define I2S_BCLK                    GPIO_NUM_5
#define I2S_DOUT                    GPIO_NUM_6
#define I2S_LRC                     GPIO_NUM_14

// -------------------------- Button Setup --------------------------------------
#define PUSH_BUTTON_PIN             GPIO_NUM_21
#define MY_DEBOUNCE_DELAY           50

const char* file_name = "/rec.wav";
AudioInfo info(16000, 1, 16);

// -------------------------- Audio Tools Setup --------------------------------------
I2SStream i2s_in;                                                            // Mic input
VolumeStream mic_volume(i2s_in);                                             // Volume control for mic
I2SStream i2s_out;                                                           // Speaker output
File file;                                                                   // SD file for recording
WAVEncoder encoder;                                                          // WAV encoder
EncodedAudioStream out(&file, &encoder);                                     // Encoded output stream
StreamCopy copier(out, mic_volume);                                          // Copier for recording, now from mic_volume

int channels = 1;
int samples_per_second = 16000;

// -------------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Setup started...");
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

  mic_config();
  delay(10);
  #ifdef DEBUG_MODE
    Serial.println("Free Heap after mic config: " + String(ESP.getFreeHeap()));
  #endif

  SD_setup();
  delay(10);
  #ifdef DEBUG_MODE
    Serial.println("Free Heap after SD setup: " + String(ESP.getFreeHeap()));
  #endif

  amp_config();
  delay(10);
  #ifdef DEBUG_MODE
    Serial.println("Free Heap after amp config: " + String(ESP.getFreeHeap()));
  #endif

  Serial.println("Recorder ready. Hold button to record, release to stop, '1' to play.");
}

// -------------------------- Config Functions --------------------------------------
void SD_setup() {
  Serial.println("Starting SD setup...");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(10);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SPI.setFrequency(4000000);
  delay(100);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card mount failed!");
    while (1);
  }
  Serial.println("SD card mounted successfully.");
}

void mic_config() {
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
  auto cfg_in = i2s_in.defaultConfig(RX_MODE);
  cfg_in.copyFrom(info);
  cfg_in.channels = channels;
  cfg_in.sample_rate = samples_per_second;
  cfg_in.use_apll = false;
  cfg_in.i2s_format = I2S_STD_FORMAT;
  cfg_in.is_master = true;
  cfg_in.port_no = 0;
  cfg_in.pin_bck = I2S_MIC_SERIAL_CLOCK;
  cfg_in.pin_ws = I2S_MIC_LEFT_RIGHT_CLOCK;
  cfg_in.pin_data = I2S_MIC_SERIAL_DATA;
  cfg_in.buffer_size = 1024;
  cfg_in.buffer_count = 8;
  i2s_in.begin(cfg_in);

  // Configure mic volume
  auto vcfg = mic_volume.defaultConfig();
  vcfg.channels = 1;
  vcfg.sample_rate = 16000;
  vcfg.bits_per_sample = 16;
  vcfg.allow_boost = true;                                          // Allow mic boost
  mic_volume.begin(vcfg);
  mic_volume.setVolume(18.5);                                       // Boost mic input by 6x
}

void amp_config() {
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
  auto cfg = i2s_out.defaultConfig(TX_MODE);
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_LRC;
  cfg.pin_data = I2S_DOUT;
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.channels = 1;
  cfg.sample_rate = 16;
  cfg.bits_per_sample = 16;
  if (!i2s_out.begin(cfg)) {
    Serial.println("Failed to begin I2S output");
  }
}

// -------------------------- Playback Functions --------------------------------------
void playWavFile(String filename) {
  WAVDecoder wav;
  VolumeStream volume(i2s_out); // Volume control for playback
  EncodedAudioStream decoder(&volume, &wav);
  StreamCopy playCopier;
  File audioFile = SD.open(filename.c_str());
  
  if (!audioFile) {
    Serial.print("Failed to open WAV file: ");
    Serial.println(filename);
    return;
  }
  
  // Configure playback volume
  auto vcfg = volume.defaultConfig();
  vcfg.channels = 1;
  vcfg.sample_rate = 16000;
  vcfg.bits_per_sample = 16;
  vcfg.allow_boost = true;                                                  // Allow playback boost
  volume.begin(vcfg);
  volume.setVolume(2.0);                                                    // Boost playback by 2.0x

  Serial.println("Starting playback...");
  decoder.begin();
  playCopier.begin(decoder, audioFile);
  
  while (playCopier.copy()) {
    // Playback continues until file ends
  }
  
  audioFile.close();
  decoder.end();
  volume.end();
  Serial.println("WAV playback finished.");
}

void playWavTask(void *pvParameters) {
  String filename = *((String*)pvParameters);
  playWavFile(filename);
  delete (String*)pvParameters;
  vTaskDelete(NULL);
}

void startWavPlayback(String filename) {
  String* filenamePtr = new String(filename);
  xTaskCreate(
      playWavTask,
      "WAV Playback",
      4096,
      filenamePtr,
      5,
      NULL
  );
}

// -------------------------- Loop --------------------------------------
void loop() {
  static bool buttonState = HIGH;
  static bool lastButtonState = HIGH;
  int currentButtonState = digitalRead(PUSH_BUTTON_PIN);

  if (currentButtonState != lastButtonState) {
    delay(MY_DEBOUNCE_DELAY);
    currentButtonState = digitalRead(PUSH_BUTTON_PIN);

    if (currentButtonState == LOW && lastButtonState == HIGH) { 
      Serial.println("Button pressed - Starting recording");
      file = SD.open(file_name, FILE_WRITE);
      if (!file) {
        Serial.println("Failed to reopen file for writing!");
        return;
      }
      encoder.begin();
      auto cfg_out = out.defaultConfig();
      cfg_out.bits_per_sample = 16;
      cfg_out.channels = 1;
      cfg_out.sample_rate = 16000;
      out.begin(cfg_out);
      copier.setCheckAvailableForWrite(false);
    } else if (currentButtonState == HIGH && lastButtonState == LOW) { 
      Serial.println("Button released - Stopping recording");
      out.end();
      file.flush();
      Serial.print("File has ");
      Serial.print(file.size());
      Serial.println(" bytes");
      file.close();
    }
    lastButtonState = currentButtonState;
  }

  if (currentButtonState == LOW) {
    copier.copy();                                                            // Record while button is pressed
  }

  if (Serial.available()) {
    char command = Serial.read();
    if (command == '1') {
      startWavPlayback(file_name);
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
