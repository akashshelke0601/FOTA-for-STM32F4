/*
 * ESP32 OTA Bridge (Stateful Production Version)
 * * Function:
 * - Checks server for firmware updates (version.txt).
 * - Downloads binary based on STM32's current bank.
 * - Transfers firmware to STM32 via SPI.
 * - Manages version tracking internally (ESP32 Preferences).
 * * Wiring:
 * ESP32 GPIO18 (SCK)  --> STM32 PA5 (SPI1_SCK)
 * ESP32 GPIO23 (MOSI) --> STM32 PA7 (SPI1_MOSI)
 * ESP32 GPIO19 (MISO) <-- STM32 PA6 (SPI1_MISO)
 * ESP32 GPIO5  (CS)   --> STM32 PA4 (CS)
 * ESP32 GPIO21 (NRST) --> STM32 NRST
 * ESP32 GND           --> STM32 GND
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <Preferences.h>

/* WiFi Credentials */
const char* WIFI_SSID = "yash123";
const char* WIFI_PASS = "1234567890";

/* OTA Server - Updated based on your ifconfig */
const char* SERVER_URL = "http://10.128.87.108:5000"; /* Change to your PC IP */

#define STM32_NRST_PIN  21   // choose a free ESP32 GPIO

/* SPI Pins */
#define SPI_CS_PIN 5

/* SPI Commands */
#define CMD_PING        0x01
#define CMD_START_OTA   0x10
#define CMD_DATA_CHUNK  0x20
#define CMD_END_OTA     0x30
#define CMD_GET_VERSION 0x40
#define CMD_REBOOT      0x50
#define CMD_GET_BANK_ID 0x60 // New Command

/* SPI Responses */
#define RSP_PONG        0x02
#define RSP_OK          0xAA
#define RSP_ERROR       0xFF

// Use a safe chunk size (max 255 because our length is 1 byte)
#define SAFE_CHUNK_SIZE 64

/* Firmware buffer */
#define CHUNK_SIZE 256
uint8_t firmwareBuffer[CHUNK_SIZE];
Preferences preferences;
int targetVersion = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n=== ESP32 OTA Bridge (Stateful Version) ===");

    /* Init Pins */
    pinMode(STM32_NRST_PIN, OUTPUT);
    digitalWrite(STM32_NRST_PIN, HIGH);
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.begin();

    /* Connect to WiFi */
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("WiFi connection failed!");
    }

    // Ensure STM32 is alive
    if (!pingSTM32()) {
        stm32_hard_reset();
        delay(2000);
    }

    /* 1. Get Local Version from ESP32 Memory */
    preferences.begin("ota_app", true); // Open in Read-Only mode
    int currentVersion = preferences.getInt("version", 0); // Default to 0
    preferences.end();

    Serial.print("[MAIN] Current Firmware Version: ");
    Serial.println(currentVersion);

    /* 2. Check Server for Update */
    // Note: checkForUpdate updates the global 'targetVersion' variable
    if (checkForUpdate(currentVersion)) {
        
        Serial.println("[MAIN] Update available. Preparing STM32...");
        
        // Reset STM32 to ensure it's listening
        stm32_hard_reset();
        delay(1000);

        // 3. Perform the Update
        if (downloadAndInstall()) {
            Serial.println("[MAIN] Update Success!");
            
            // 4. Save New Version to ESP32 Memory
            preferences.begin("ota_app", false); // Open in Read-Write mode
            preferences.putInt("version", targetVersion);
            preferences.end();
            
            Serial.print("[MAIN] Saved new version: ");
            Serial.println(targetVersion);
        } else {
            Serial.println("[MAIN] Update Failed.");
        }
    } else {
        Serial.println("[MAIN] No update needed.");
    }
}


void stm32_hard_reset(void)
{
    Serial.println("[ESP32] Resetting STM32...");

    pinMode(STM32_NRST_PIN, OUTPUT);

    digitalWrite(STM32_NRST_PIN, LOW);   // Assert reset
    delay(50);                           // Hold reset (50 ms)

    digitalWrite(STM32_NRST_PIN, HIGH);  // Release reset
    delay(1000);                          // Allow STM32 to boot

    Serial.println("[ESP32] STM32 reset released");
}


uint8_t getSTM32BankID() {
    uint8_t bank = 0xFF;
    int retries = 0;

    while (retries < 5) {
        Serial.print("[SPI] Getting Bank ID (Attempt ");
        Serial.print(retries + 1);
        Serial.print(")... ");
        
        spiTransfer(CMD_GET_BANK_ID);
        delay(10); 
        bank = spiTransfer(0x00);

        if (bank == 0 || bank == 1) {
            Serial.print("Valid Bank: ");
            Serial.println(bank);
            return bank;
        }

        Serial.print("Invalid response: ");
        Serial.println(bank);
        delay(100); 
        retries++;
    }

    Serial.println("[ERROR] Failed to get valid Bank ID.");
    return 0xFF; // Return Error Code
}

uint8_t spiTransfer(uint8_t data) {
    uint8_t response;
    
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_PIN, LOW);
    delayMicroseconds(50);  // Longer setup time
    
    response = SPI.transfer(data);
    
    delayMicroseconds(50);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    
    delay(5);  // Give STM32 more time to process
    return response;
}

bool pingSTM32() {
    Serial.print("[SPI] Sending PING... ");
    
    uint8_t response = spiTransfer(CMD_PING);
    delay(10);
    response = spiTransfer(0x00);  /* Get actual response */
    
    if (response == RSP_PONG) {
        Serial.println("PONG received! STM32 OK");
        return true;
    } else {
        Serial.print("Unexpected: 0x");
        Serial.println(response, HEX);
        return false;
    }
}

bool checkForUpdate(uint8_t currentVersion) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[HTTP] WiFi not connected!");
        return false;
    }
    
    Serial.println("[HTTP] Checking for update...");
    
    HTTPClient http;
    String url = String(SERVER_URL) + "/version";
    
    http.begin(url);
    int httpCode = http.GET();
    
    if (httpCode == 200) {
        String payload = http.getString();
        int serverVersion = payload.toInt();
    
        targetVersion = serverVersion; // [ADDED] Save this to global variable

        Serial.print("       Server version: ");
        Serial.println(serverVersion);
        Serial.print("       Current version: ");
        Serial.println(currentVersion);
        
        http.end();
        
        if (serverVersion > currentVersion) {
            Serial.println("       Update available!");
            return true;
        } else {
            Serial.println("       Already up to date.");
            return false;
        }
    } else {
        Serial.print("       HTTP error: ");
        Serial.println(httpCode);
        http.end();
        return false;
    }
}

bool downloadAndInstall() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[HTTP] WiFi not connected!");
        return false;
    }

    // 1. Get Bank ID from STM32 (0 or 1)
    uint8_t currentBank = getSTM32BankID();

    if (currentBank > 1) { 
         Serial.println("[OTA] Error: Could not determine valid Bank ID. Aborting.");
         return false; 
    }

    // 2. Build URL with query parameter
    // This tells the server: "I am in Bank X, send me the OTHER firmware"
    String url = String(SERVER_URL) + "/firmware?current_bank=" + String(currentBank);
    
    Serial.println("[OTA] Requesting: " + url);
    Serial.println("[OTA] Starting firmware download...");
    
    HTTPClient http;
    http.begin(url);
    
    // 3. Send GET request
    int httpCode = http.GET();
    
    if (httpCode != 200) {
        Serial.print("[OTA] HTTP error: ");
        Serial.println(httpCode);
        http.end();
        return false;
    }
    
    int firmwareSize = http.getSize();
    Serial.print("[OTA] Firmware size: ");
    Serial.print(firmwareSize);
    Serial.println(" bytes");
    
    if (firmwareSize <= 0) {
        Serial.println("[OTA] Invalid firmware size!");
        http.end();
        return false;
    }
    
    /* 4. Tell STM32 to start OTA */
    Serial.println("[SPI] Sending START_OTA command...");
    spiTransfer(CMD_START_OTA);
    
    // WAIT 3 SECONDS for Mass Erase of the target bank
    delay(3000);  
    
    uint8_t response = spiTransfer(0x00);
    if (response != RSP_OK) {
        Serial.print("[SPI] STM32 rejected OTA: 0x");
        Serial.println(response, HEX);
        http.end();
        return false;
    }
    Serial.println("[SPI] STM32 ready to receive");
    
    /* 5. Stream Data */
    WiFiClient* stream = http.getStreamPtr();
    
    int totalSent = 0;
    int lastPercent = 0;
    
    Serial.println("[OTA] Sending firmware to STM32...");
    
    while (http.connected() && totalSent < firmwareSize) {
        size_t available = stream->available();
        
        if (available) {
            /* Read a block of data from WiFi */
            int bytesToRead = min((int)available, SAFE_CHUNK_SIZE);
            int bytesRead = stream->readBytes(firmwareBuffer, bytesToRead);
            
            /* A. Send Chunk Command */
            spiTransfer(CMD_DATA_CHUNK);
            delayMicroseconds(50);

            /* B. Send Chunk Length */
            spiTransfer((uint8_t)bytesRead);
            delayMicroseconds(50);

            /* C. Send the Data Bytes */
            for (int i = 0; i < bytesRead; i++) {
                uint8_t dataToSend = firmwareBuffer[i];
                spiTransfer(dataToSend);
                delayMicroseconds(30); 
            }
            
            /* D. Wait for STM32 to write this chunk */
            delay(5); 
            
            totalSent += bytesRead;
            
            /* Progress indicator */
            int percent = (totalSent * 100) / firmwareSize;
            if (percent != lastPercent && percent % 10 == 0) {
                Serial.print("[OTA] Progress: ");
                Serial.print(percent);
                Serial.println("%");
                lastPercent = percent;
            }
        }
        // Small yield to keep WiFi stack happy
        delay(1);
    }
    
    http.end();
    
    Serial.print("[OTA] Total sent: ");
    Serial.print(totalSent);
    Serial.println(" bytes");
    
    /* 6. Finish OTA */
    Serial.println("[SPI] Sending END_OTA command...");
    spiTransfer(CMD_END_OTA);
    
    // CRITICAL FIX: Increased to 4000ms
    // STM32 needs time to Erase Sector 11 (Metadata) and write new magic.
    // If we ping too early, we get garbage or break the flow.
    Serial.println("[OTA] Waiting for Metadata update...");
    delay(4000); 
    
    response = spiTransfer(0x00);
    if (response == RSP_OK) {
        Serial.println("[OTA] SUCCESS! Firmware installed.");
        Serial.println("[OTA] Rebooting STM32 automatically...");
        delay(1000); 
        rebootSTM32(); // <--- Auto reboot
        return true;
    } 
    else {
        Serial.print("[OTA] END_OTA failed: 0x");
        Serial.println(response, HEX);
        return false;
    }
}

void rebootSTM32() {
    Serial.println("[SPI] Rebooting STM32...");
    spiTransfer(CMD_REBOOT);
    Serial.println("[SPI] Reboot command sent!");
    
    // CRITICAL FIX: Wait for STM32 to actually reboot and come back up.
    // If we ping too soon, we get 0xFF (255) errors.
    Serial.println("[SPI] Waiting 5 seconds for system restart...");
    delay(5000); 
}


void loop()
{
   
}
