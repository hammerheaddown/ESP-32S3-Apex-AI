#include <SD.h>
#include <SPI.h>
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// --------------------------  SD Pin Setup      --------------------------------------
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_SCK  GPIO_NUM_12
#define SD_CS   GPIO_NUM_10

// --------------------------  AMP Pin Setup     ------------------------------------------
#define I2S_BCLK GPIO_NUM_5
#define I2S_DOUT GPIO_NUM_6
#define I2S_LRC  GPIO_NUM_14

// --------------------------  Push Button Setup ---------------------------------------
#define PUSH_BUTTON_PIN 21                                                  // Replace with your push button pin
#define MY_DEBOUNCE_DELAY 50

I2SStream i2s;
const int chipSelect = SD_CS;

volatile bool interruptRequested = false;                                   // Global interrupt flag
String speech2TxtFilename= "/recordings/speech2txt.wav";
String text2speechFilename = "/recordings/text2speech.mp3";

void playWavFile(String filename);
void playMp3File(String filename);
void audio_tools_i2s_config();
void listRecordings();
void interruptAudioPlayback();
void handleButtonInterrupt();
void playMp3Task(void *pvParameters);
void playWavTask(void *pvParameters);
void startWavPlayback(String filename);
void startMp3Playback(String filename);

void audio_tools_i2s_config() {
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.pin_bck = I2S_BCLK;
  cfg.pin_ws = I2S_LRC;
  cfg.pin_data = I2S_DOUT;
  cfg.i2s_format = I2S_STD_FORMAT;
  i2s.begin(cfg);
  
}

void interruptAudioPlayback() {
  interruptRequested = true;

  delay(100); // Add a 50ms debounce delay
  #ifdef DEBUG_MODE
    Serial.println("Free Heap " + String(ESP.getFreeHeap()));
  #endif
  delay(10);
}

void handleButtonInterrupt() {
  interruptAudioPlayback();                       // Call the same interrupt function
}

void playWavFile(String filename) {
 
  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

  WAVDecoder wav;
  EncodedAudioStream decoder(&i2s, &wav);
  StreamCopy copier;
  File audioFile;

  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

  audioFile = SD.open(filename.c_str());

  if (!audioFile) {
    Serial.print("Failed to open WAV file: ");
    Serial.println(filename);
    return;
  }
  decoder.begin();
  copier.begin(decoder, audioFile);

  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

  while (copier.copy()) {
    if (interruptRequested) {

      #ifdef DEBUG_MODE
        Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
      #endif

      copier.end();
      decoder.end();
      interruptRequested = false;
      Serial.println("WAV playback interrupted.");
      return;
    }
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("interrupt") || command.equalsIgnoreCase("3")) {
        interruptAudioPlayback();
      }
    }
  }
  audioFile.close();
  decoder.end();
  copier.end();


  Serial.println("WAV playback finished.");
  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

}

void playMp3File(String filename) {
  
  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

  MP3DecoderHelix mp3;
  EncodedAudioStream decoder(&i2s, &mp3);
  StreamCopy copier;
  File audioFile;

  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif
  
  audioFile = SD.open(filename.c_str());
  if (!audioFile) {
    Serial.print("Failed to open MP3 file: ");
    Serial.println(filename);
    return;
  }
  decoder.begin();
  copier.begin(decoder, audioFile);

  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

  while (copier.copy()) {
    if (interruptRequested) {

      #ifdef DEBUG_MODE
        Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
      #endif

      copier.end();
      decoder.end();
      interruptRequested = false;
      Serial.println("MP3 playback interrupted.");
      return;
    }
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("interrupt") || command.equalsIgnoreCase("3")) {
        interruptAudioPlayback();
      }
    }
  }
  audioFile.close();
  decoder.end();
  copier.end();
  Serial.println("MP3 playback finished.");

  #ifdef DEBUG_MODE
    Serial.println("Free Heap before b4 we continue " + String(ESP.getFreeHeap()));
  #endif

}

void startMp3Playback(String filename) {
  String* filenamePtr = new String(filename); // Allocate memory for filename
  xTaskCreate(
      playMp3Task,
      "MP3 Playback",
      4096,
      filenamePtr,
      5,
      NULL
  );
}

void startWavPlayback(String filename) {
  String* filenamePtr = new String(filename); // Allocate memory for filename
  xTaskCreate(
      playWavTask,
      "WAV Playback",
      4096,
      filenamePtr,
      5,
      NULL
  );
}

void playMp3Task(void *pvParameters) {
  String filename = *((String*)pvParameters); // Retrieve filename from parameter
  playMp3File(filename);
  delete (String*)pvParameters; // Clean up allocated memory
  vTaskDelete(NULL); // Delete the task
}

void playWavTask(void *pvParameters) {
  String filename = *((String*)pvParameters); // Retrieve filename from parameter
  playWavFile(filename);
  delete (String*)pvParameters; // Clean up allocated memory
  vTaskDelete(NULL); // Delete the task
}

void listRecordings() {
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

void setup() {
  Serial.begin(115200);
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Info);
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);                                                       // Configure button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), handleButtonInterrupt, FALLING);      // Use your desired mode
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  audio_tools_i2s_config();
}

void loop() {
  // --- Push Button Logic with Debouncing ---
  int buttonState = digitalRead(PUSH_BUTTON_PIN);
  static bool lastButtonState = HIGH;

  if (interruptRequested) {
    interruptRequested = false;
  }

  if (buttonState != lastButtonState) {
    delay(MY_DEBOUNCE_DELAY);
    buttonState = digitalRead(PUSH_BUTTON_PIN);

    if (buttonState == LOW && lastButtonState == HIGH) {
      interruptAudioPlayback(); // Interrupt on button press
    }
    lastButtonState = buttonState;
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("interrupt") || command.equalsIgnoreCase("3")) {
      interruptAudioPlayback();
    }

    if (command.equalsIgnoreCase("recordings") || command.equalsIgnoreCase("5")) {
      listRecordings();
    }

    if (command.equalsIgnoreCase("speech") || command.equalsIgnoreCase("6")) {
      startWavPlayback(speech2TxtFilename);
    }

    if (command.equalsIgnoreCase("text") || command.equalsIgnoreCase("7")) {
      startMp3Playback(text2speechFilename);
    }
  }
}
