#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include "secrets.h"
#include "FS.h"
#include "ubx.h"
#include "freertos/ringbuf.h"

const char* time_zone = "NZST-12NZDT,M9.5.0,M4.1.0/3";

// This sets Arduino Stack Size - comment this line to use default 8K stack size
SET_LOOP_TASK_STACK_SIZE(16 * 1024);  // 16KB

HardwareSerial gpsSerial(1);

struct SatelliteAssistFileData {
    bool gps = false;
    bool beiDou = false;
    bool glonass = false;
    bool galileo = false;
    bool qzss = false;
    uint16_t startYear = 0;
    uint16_t endYear = 0;
    uint8_t startMonth = 0;
    uint8_t endMonth = 0;
    uint8_t startDay = 0;
    uint8_t endDay = 0;
};

struct AssistNowConfig {
    bool gps = false;
    bool glonass = false;
    bool beiDou = false;
    bool galileo = false;
    bool qzss = false;
    int weeks = 1;
    int interval = 1;
    String token = "";
    String baseUrl = "offline-live1.services.u-blox.com";
};

void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("ESP32 Reset---> ");
    switch (reason) {
        case ESP_RST_UNKNOWN: Serial.println("Unknown Reset"); break;
        case ESP_RST_POWERON: Serial.println("Power-On Reset"); break;
        case ESP_RST_EXT: Serial.println("External Pin Reset"); break;
        case ESP_RST_SW: Serial.println("Software Reboot"); break;
        case ESP_RST_PANIC: Serial.println("Exception/Panic Reset"); break;
        case ESP_RST_INT_WDT: Serial.println("Interrupt Watchdog Reset"); break;
        case ESP_RST_TASK_WDT: Serial.println("Task Watchdog Reset"); break;
        case ESP_RST_WDT: Serial.println("Other Watchdog Reset"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep Sleep Mode Reset"); break;
        case ESP_RST_BROWNOUT: Serial.println("Brownout Reset"); break;
        case ESP_RST_SDIO: Serial.println("SDIO Reset"); break;
        default: Serial.println("Unknown"); break;
    }
}

void handleSatelliteAssist(uint8_t message[], SatelliteAssistFileData* data) {
    // uint8_t type = message[0];
    // uint8_t version = message[1];
    // uint8_t satelliteNumber = message[2];
    // uint8_t constellationNumber = message[3];
    uint16_t year = message[4] + 2000;
    uint8_t month = message[5];
    uint8_t day = message[6];

    if (data->startYear == 0) {
        data->startYear = year;
        data->startMonth = month;
        data->startDay = day;
    }

    if (year > data->endYear || (year == data->endYear && month > data->endMonth) || (year == data->endYear && month == data->endMonth && day > data->endDay)) {
        data->endYear = year;
        data->endMonth = month;
        data->endDay = day;
    }
}

//increasing fileBufferSize helps up to a point, past 8kib it has no use
SatelliteAssistFileData analyzeAssistNowFile(FS* fs, const char* filePath, uint32_t fileBufferSize = 2048) {
    uint16_t messageNumber = 1;
    unsigned long startTime = millis();
    SatelliteAssistFileData data;

    File file = fs->open(filePath, FILE_READ);
    file.setBufferSize(fileBufferSize);

    while (file.available()) {
        uint8_t headBuf[6];  // buffer to store message header bytes
        file.readBytes((char*)headBuf, 6);

        if (headBuf[0] == 0xb5 && headBuf[1] == 0x62) {
            uint8_t msgClass = headBuf[2];
            uint8_t msgid = headBuf[3];

            // Read the Length (2 bytes, Little-Endian unsigned 16-bit integer)
            uint16_t length = (headBuf[4] | (headBuf[5] << 8)) + 2;  // 2 checkbytes not included in length

            uint8_t message[length];
            file.readBytes((char*)message, length);

            switch (msgClass) {
                case 0x13:  //GNSS Assistance Messages
                    switch (msgid) {
                        case 0x00:
                            data.gps = true;
                            break;
                        case 0x02:
                            data.galileo = true;
                            break;
                        case 0x03:
                            data.beiDou = true;
                            break;
                        case 0x05:
                            data.qzss = true;
                            break;
                        case 0x06:
                            data.glonass = true;
                            break;
                        case 0x20:
                            handleSatelliteAssist(message, &data);
                            break;
                        default:
                            Serial.printf("\nunknown id 0x%02x 0x%02x in msg %i ", msgClass, msgid, messageNumber);
                            break;
                    }
                    break;

                default:
                    Serial.printf("\nunknown class 0x%02x in msg %i ", msgClass, messageNumber);
                    break;
            }
        } else {
            Serial.printf("0x%02x ", headBuf[0]);  //byte without correct header (most likely a processing error in this code)
            file.seek(file.position() - 5);        //go back and start processing from where headBuf[1] is
        }
        messageNumber++;
    }
    time_t lastWriteTime = file.getLastWrite();
    struct tm* timeInfo = localtime(&lastWriteTime);

    Serial.printf("File:%s, %3.1fkiB, %04d/%02d/%02d %02d:%02d, ",
                  filePath, (float)file.size() / 1024.0, timeInfo->tm_year + 1900, timeInfo->tm_mon + 1,
                  timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min);

    Serial.printf("GPS:%s, BeiDou:%s, Glonass:%s, Galileo:%s, QZSS:%s, %04d/%02d/%02d->%04d/%02d/%02d",
                  data.gps ? "\x1B[32mYes\x1B[0m" : "No", data.beiDou ? "\x1B[32mYes\x1B[0m" : "No",
                  data.glonass ? "\x1B[32mYes\x1B[0m" : "No", data.galileo ? "\x1B[32mYes\x1B[0m" : "No",
                  data.qzss ? "\x1B[32mYes\x1B[0m" : "No", data.startYear, data.startMonth, data.startDay,
                  data.endYear, data.endMonth, data.endDay);

    Serial.printf(" \x1B[32min %ums\x1B[0m\r\n", millis() - startTime);

    file.close();
    return data;
}

void downloadAssistNowOfflineFile(FS* fs, const char* downloadPath, AssistNowConfig config, uint32_t fileBufferSize = 4096, uint8_t wifiTimeoutSeconds = 30) {
    WiFiClient wifiClient;
    HTTPClient http;

    const char* ntpServer1 = "0.nz.pool.ntp.org";                 // Primary NTP server
    const char* ntpServer2 = "1.nz.pool.ntp.org";                 // Second NTP server (optional)
    const char* ntpServer3 = "pool.ntp.org";                      // Third NTP server (optional)
    configTzTime(time_zone, ntpServer1, ntpServer2, ntpServer3);  // Set timezone and NTP servers

    Serial.print("WiFi-");
    unsigned long startTime = millis();
    WiFi.begin(ssid, password);

    //Wait for connection to WiFi
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("-");
        if (millis() - startTime > wifiTimeoutSeconds * 1000) {
            Serial.printf("\x1B[31mX WiFi Connect Timed out! Restarting in 5s...\x1B[0m\r\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            ESP.restart();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    Serial.printf("> \x1B[32mConnected in %0.1fs\x1B[0m\r\n", (float)(millis() - startTime) / 1000);


    // Wait for ntp time synchronization (max 10 seconds)
    time_t now;
    startTime = millis();
    Serial.print("Time-");
    while (time(&now) < 1704067200) {  // Check if time is synchronized (> 1st Jan 2024)
        Serial.print("-");
        if (millis() - startTime > 10000) {
            Serial.printf("\x1B[31mX ntp time sync Timed out! Restarting in 5s...\x1B[0m\r\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            ESP.restart();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    //Print out time
    struct tm timeInfo;
    getLocalTime(&timeInfo);
    Serial.printf(
        "> \x1B[32mSynced to %d/%02d/%02d %02d:%02d local time in %0.2fs\x1B[0m\r\n",
        timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, (float)(millis() - startTime) / 1000);


    //limit buffer size to stack available
    uint32_t stackAvailable = uxTaskGetStackHighWaterMark(NULL);
    if (stackAvailable < fileBufferSize) {
        fileBufferSize = stackAvailable;
        Serial.printf("\x1B[33mBuffer size reduced to \x1B[31m%0.2f KiB\x1B[33m to stop stack overflow\x1B[0m\r\n", fileBufferSize / 1024.0f);
    }

    // Remove file and open it for Writing
    if (fs->exists(downloadPath) && !fs->remove(downloadPath))
        Serial.printf("Error deleting file: %s\r\n", downloadPath);
    File file = fs->open(downloadPath, FILE_WRITE, true);
    if (!file) Serial.printf("Error opening file: %s\r\n", downloadPath);
    else file.setBufferSize(fileBufferSize);


    String gnss = "gnss=";
    if (config.gps) gnss += "gps,";
    if (config.glonass) gnss += "glo,";
    if (config.beiDou) gnss += "bds,";
    if (config.galileo) gnss += "gal,";
    gnss.remove(gnss.length() - 1);  // remove the extra comma
    gnss += ";";

    String alm = "alm=";
    if (config.gps) alm += "gps,";
    if (config.glonass) alm += "glo,";
    if (config.beiDou) alm += "bds,";
    if (config.galileo) alm += "gal,";
    if (config.qzss) alm += "qzss,";
    alm.remove(alm.length() - 1);  // remove the extra comma
    alm += ";";

    String timeAndResolution = "period=" + String(max(1, min(5, config.weeks))) + ";" + "resolution=" + String(max(1, min(3, config.interval)));

    String url = "http://" + config.baseUrl + "/GetOfflineData.ashx?token=" + config.token + ";" + gnss + alm + timeAndResolution;
    Serial.print("Fetching From " + config.baseUrl);

    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == 200) {
        int32_t fileSize = http.getSize();
        uint8_t buff[fileBufferSize];

        Serial.printf("--->\x1B[32m HTTP 200, Success!\x1B[0m\r\n", (float)(millis() - startTime) / 1000);
        Serial.printf("Download|%0.1fkiB|", fileSize / 1024.0f);
        startTime = millis();

        WiFiClient* stream = http.getStreamPtr();
        unsigned long timeout = millis() + 60 * 1000;  // 60 second timeout
        while (http.connected() && fileSize > 0) {
            if (millis() > timeout) {
                Serial.print("---X \x1B[31m Timeout occurred \x1B[0m");
                break;
            }
            uint16_t buffSize = min((uint32_t)stream->available(), sizeof(buff));
            if (buffSize == 0) {
                vTaskDelay(10 / portTICK_PERIOD_MS);  // wait 10ms for more data to arrive
                continue;
            }
            buffSize = stream->readBytes(buff, buffSize);

            file.write(buff, buffSize);

            // if (file.write(buff, buffSize) != buffSize) {
            //     Serial.print("---X \x1B[31m Error Writing to File \x1B[0m");
            //     break;
            // }
            fileSize -= buffSize;
            Serial.printf("-");
        }

        Serial.printf("> \x1B[32min %0.1fs\x1B[0m\r\n", (float)(millis() - startTime) / 1000);
    } else {
        Serial.printf("---X \x1B[31m HTTPS Error code: %i, %s\x1B[0m\r\n", httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
    wifiClient.stop();
    WiFi.disconnect();

    file.close();
}

void ringBufferDemo() {
    static const char messageToSend[] = "Hello, From the Ring Buffer!\r\n";
    RingbufHandle_t bufferHandle = xRingbufferCreate(1028, RINGBUF_TYPE_BYTEBUF);

    if (bufferHandle == NULL) {
        Serial.println("Failed to create ring buffer!");
        return;
    }

    if (xRingbufferSend(bufferHandle, messageToSend, sizeof(messageToSend), pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("Failed to send message to ring buffer!");
        return;
    }

    size_t receivedMessageSize;
    char* receivedMessage = (char*)xRingbufferReceiveUpTo(bufferHandle, &receivedMessageSize, pdMS_TO_TICKS(1000), sizeof(messageToSend));

    if (receivedMessage != NULL) {
        Serial.write(receivedMessage, receivedMessageSize);
        vRingbufferReturnItem(bufferHandle, (void*)receivedMessage);
    } else {
        Serial.println("Failed to receive message from ring buffer!");
    }
}

void setup() {
    pinMode(OUTPUT_EN, OUTPUT);
    digitalWrite(OUTPUT_EN, HIGH);

    setenv("TZ", time_zone, true);
    tzset();

    Serial.begin();
    while (!Serial.availableForWrite()) { vTaskDelay(100 / portTICK_PERIOD_MS); }
    //Serial.setDebugOutput(true);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printResetReason();

    gpsSerial.setRxBufferSize(512);
    gpsSerial.setTxBufferSize(2048);
    gpsSerial.begin(9600, SERIAL_8N1, JST_UART_RX, JST_UART_TX);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpsSerial.write("$PCAS01,5*19\r\n");  // set to 115200 baud
    gpsSerial.flush();
    gpsSerial.updateBaudRate(115200);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpsSerial.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");  // set to no output
    gpsSerial.write("$PCAS11,1*1C\r\n");                          // Static Model
    while (gpsSerial.available()) { gpsSerial.read(); }           //clear serial buffer

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    AssistNowConfig config;
    config.gps = true;
    config.beiDou = true;
    config.glonass = true;
    config.galileo = true;
    config.qzss = true;
    config.weeks = 5;
    config.interval = 3;
    config.token = ubloxToken;

    downloadAssistNowOfflineFile(&LittleFS, "/mgaoffline.ubx", config);

    //ringBufferDemo();

    analyzeAssistNowFile(&LittleFS, "/mgaoffline.ubx");

    //sendAssistNowDataToGPSFlash("/mgaoffline.ubx", 50);
    sendAssistNowDataToGPSFlash(&LittleFS, "/mgaoffline.ubx", 0);

    //gpsSerial.write("$PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1*02\r\n");  //all outputs on
}

void loop() {
    while (gpsSerial.available()) {
        Serial.print((char)gpsSerial.read());
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}