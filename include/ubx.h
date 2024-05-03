#include <Arduino.h>
#include <LittleFS.h>
#include "FS.h"

extern HardwareSerial gpsSerial;

// Define UbxClass and UbxId
#define MGA_FLASH_CLASS 0x13
#define MGA_FLASH_ID 0x21

struct UbxMessage {
    uint8_t class_ = MGA_FLASH_CLASS;
    uint8_t id_ = MGA_FLASH_ID;
    uint16_t length_;  // payload length
    uint8_t type_;
    uint8_t version_;
    uint8_t *payload_;
};

struct UbxMgaFlashAckMessage : public UbxMessage {
    uint16_t length_ = 6;
    uint8_t type_ = 0x03;
    uint8_t version_ = 0x00;
    uint8_t ack_;
    bool msgReceived_ = false;
    uint16_t blockNumber_;
};

struct UbxMgaFlashDataMessage : public UbxMessage {
    uint8_t type_ = 0x01;
    uint8_t version_ = 0x00;
    uint16_t blockNumber_ = 0;
    uint16_t size_;
};

struct UbxMgaFlashStopMessage : public UbxMessage {
    uint16_t length_ = 2;
    uint8_t type_ = 0x02;
    uint8_t version_ = 0x00;
};

void sendUbxMessage(UbxMessage *msg) {
    uint8_t buffer[2 + 4 + msg->length_ + 2];

    // Preamble (2 bytes)
    buffer[0] = 0xB5;
    buffer[1] = 0x62;

    // Message Class and ID (1 byte each)
    buffer[2] = (uint8_t)msg->class_;
    buffer[3] = (uint8_t)msg->id_;
    // Length (2 bytes, Little-Endian unsigned 16-bit integer)
    uint16_t length = htons(msg->length_);
    memcpy(&buffer[4], &length, 2);

    // Payload
    buffer[6] = msg->type_;     //0
    buffer[7] = msg->version_;  //1

    if (msg->type_ == 0x01 && msg->version_ == 0x00) {
        //UBX-MGA-FLASH-DATA
        UbxMgaFlashDataMessage *data = (UbxMgaFlashDataMessage *)msg;
        memcpy(&buffer[8], &data->blockNumber_, 2);        //2
        memcpy(&buffer[10], &data->size_, 2);              //4
        memcpy(&buffer[12], data->payload_, data->size_);  //6
    } else if (msg->type_ == 0x02 && msg->version_ == 0x00) {
        // UBX-MGA-FLASH-STOP
    }

    // Checksum calculation
    uint8_t checksumBuffer[2];
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (int i = 2; i < msg->length_; i++) {  //checksum not calculated over preamble
        ckA += buffer[i];
        ckB += ckA;
    }

    // CK_A and CK_B fields (1 byte each)
    buffer[msg->length_ + 8] = ckA & 0xFF;
    buffer[msg->length_ + 9] = ckB & 0xFF;

    // Send the message over the GPS serial port
    gpsSerial.write(buffer, msg->length_ + 8);
}

void receiveUbxMessage(UbxMgaFlashAckMessage *msg, uint16_t msToBlock = 0) {
    if (msToBlock > 0) {
        for (uint16_t i = 0; i < msToBlock && 6 > gpsSerial.available(); i++) {  //6 bytes is minimum msg length
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }

    if (gpsSerial.available()) {
        // Read the Preamble (2 bytes)
        uint8_t preamble[2];
        gpsSerial.readBytes(preamble, 2);
        if (preamble[0] != 0xB5 || preamble[1] != 0x62) {
            Serial.printf("invalid preamble (0x%02x 0x%02x) ", preamble[0], preamble[1]);
            while (gpsSerial.available()) {
                Serial.printf("0x%02x ", gpsSerial.read());
            }
        }

        // Read the Message Class and ID (1 byte each)
        msg->class_ = gpsSerial.read();
        msg->id_ = gpsSerial.read();

        // Read the Length (2 bytes, Little-Endian unsigned 16-bit integer)
        uint16_t length;
        gpsSerial.readBytes((uint8_t *)&length, 2);
        msg->length_ = ntohs(length);

        if (msg->class_ == MGA_FLASH_CLASS && msg->id_ == MGA_FLASH_ID) {

            msg->type_ = gpsSerial.read();
            msg->version_ = gpsSerial.read();

            UbxMgaFlashAckMessage *ack;

            if (msg->type_ == ack->type_ && msg->version_ == ack->version_) {
                // Read the UBX-MGA-FLASH-ACK message
                msg->ack_ = gpsSerial.read();
                gpsSerial.read();  //reserved
                msg->blockNumber_ = gpsSerial.read();
                msg->msgReceived_ = true;
                if (msg->ack_ == 0) {
                    Serial.print("UBX-MGA-FLASH-ACK - GPS returned a flash block acknowledged msg ");
                } else {
                    Serial.print("UBX-MGA-FLASH-ACK - GPS returned a flash not acknowledged msg ");
                }

                Serial.print("UBX-MGA-FLASH-ACK - NACK received ");
            } else {
                Serial.print("Invalid Message Type / Version ");
            }
        } else if (msg->class_ == 0x05 && msg->id_ == 0x00) {  //UBX-ACK-NAK Message not acknowledged
            Serial.print("UBX-ACK-NAK - GPS returned a generic not acknowledged msg ");
        }

        // Read the Payload
        uint8_t *payload = new uint8_t[msg->length_];
        gpsSerial.readBytes(payload, msg->length_);
        delete[] payload;

        // Checksum calculation and verification (not implemented in this example)
    } else {
        Serial.print("No Message Received ");
    }
}

void sendAssistNowDataToGPSFlash(FS *fs, const char *filePath, uint16_t msToWaitForAcknowledgeMessage = 0) {
    File file = fs->open(filePath, FILE_READ);
    file.setBufferSize(2048);

    unsigned long startTime = millis();
    Serial.printf("Flash GPS-");

    uint16_t blockNumber = 0;  // start from 0, increment for each block
    while (file.available()) {
        uint16_t blockSize = min(512, file.available());
        uint8_t blockBuffer[blockSize];
        file.read(blockBuffer, blockSize);  //read block into buffer

        UbxMgaFlashDataMessage msg;
        msg.blockNumber_ = blockNumber;
        msg.size_ = blockSize;
        msg.length_ = 6 + blockSize;
        msg.payload_ = blockBuffer;

        sendUbxMessage(&msg);

        if (msToWaitForAcknowledgeMessage > 0) {
            UbxMgaFlashAckMessage ack;
            receiveUbxMessage(&ack, msToWaitForAcknowledgeMessage);
            if (ack.msgReceived_ && ack.ack_ != 0) {  // NACK, re-transmit or abort
                Serial.print("UBX-MGA-FLASH-ACK - NACK received ");
            }
        }

        //Serial.printf(" <- block %i\n", blockNumber);

        if (blockNumber % 5 == 0) Serial.printf("-");  //print a dash on every 5th block
        blockNumber++;
    }

    UbxMgaFlashStopMessage stop;
    sendUbxMessage(&stop);

    // if (msToWaitForAcknowledgeMessage > 0) {
    // UbxMgaFlashAckMessage ack;
    // receiveUbxMessage(&ack, 1000);
    // if (ack.msgReceived_ && ack.ack_ != 0) {  // NACK, re-transmit or abort
    //     Serial.print("UBX-MGA-FLASH-ACK - NACK received ");
    // }
    // }


    //Serial.printf(" <- Flash Stop Command\n");
    Serial.printf("> \x1B[32mDone in %0.1fs\x1B[0m ", (float)(millis() - startTime) / 1024.0f);

    startTime = millis();
    while (gpsSerial.available() && millis() - startTime < 5000) {
        UbxMgaFlashAckMessage ack;
        receiveUbxMessage(&ack, 100);
        // if (ack.msgReceived_ && ack.ack_ != 0) {  // NACK, re-transmit or abort
        //     Serial.print("UBX-MGA-FLASH-ACK - NACK received ");
        // }
    }
}