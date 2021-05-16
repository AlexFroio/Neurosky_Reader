#ifndef Brain_h
#define Brain_h

//#include "Arduino.h"
#include "mbed.h"

#define MAX_PACKET_LENGTH 169
#define EEG_POWER_BANDS 8

struct BrainPacket {
            uint8_t size;
            uint8_t opCode;
            uint8_t data[MAX_PACKET_LENGTH];
            } ;


class Brain {
    public:
        Brain(RawSerial &_brainStream);

        // Run this in the main loop.
        bool update();

        // String with most recent error.
        char* readErrors();

        // Returns comme-delimited string of all available brain data.
        // Sequence is as below.
        char* readCSV();
        char* readEEGPower();

        // Individual pieces of brain data.
        uint8_t readSignalQuality();
        uint8_t readAttention();
        uint8_t readMeditation();
        uint8_t readNotifications();
        uint32_t* readPowerArray();
        uint32_t readDelta();
        uint32_t readTheta();
        uint32_t readLowAlpha();
        uint32_t readHighAlpha();
        uint32_t readLowBeta();
        uint32_t readHighBeta();
        uint32_t readLowGamma();
        uint32_t readMidGamma();
        
        bool newError();
        enum pktType {
            PKT_SIGNAL  = 1, //These are used as shift values for setting and clearing their respective bits.
            PKT_ATTN    = 2,
            PKT_MEDT    = 3,
            PKT_EEG     = 4,
            PKT_RAW     = 5
        };

    private:
        enum error {
            ERR_LONG_PACKET = 1,
            ERR_BAD_CHKSUM,
            ERR_BAD_PARSE
            };
 
        BrainPacket data;
        RawSerial* brainStream;
        struct pktData {
            uint8_t length;
            uint8_t type;
            uint8_t packetData[MAX_PACKET_LENGTH - 1];
            uint8_t cksum;
            };
        uint8_t packetData[MAX_PACKET_LENGTH];
        uint8_t readStep;
        bool freshError;
        bool inPacket;
        uint8_t lb;
        uint8_t lastByte;
        uint8_t packetIndex;
        uint8_t packetLength;
        uint8_t checksum;
        uint8_t checksumAccumulator;
        uint8_t eegPowerLength;
        uint8_t pktNotif;
        bool hasPower;
        void clearPacket();
        void clearEegPower();
        bool parsePacket();

        char* printPacket();
        void init();
        void printCSV(); // maybe should be public?
        void printDebug();

        // With current hardware, at most we would have...
        // 3 x 3 char uint8_t
        // 8 x 10 char uint32_t
        // 10 x 1 char commas
        // 1 x 1 char 0 (string termination)
        // -------------------------
        // 100 characters
        char errorMsg[24];
        char csvBuffer[100];
        char debugBuffer[400];

        // Longest error is
        // 22 x 1 char uint8_ts
        // 1 x 1 char 0 (string termination)
        uint8_t latestError;

        uint8_t signalQuality;
        uint8_t attention;
        uint8_t meditation;

        bool freshPacket;

        // Lighter to just make this public, instead of using the getter?
        uint32_t eegPower[EEG_POWER_BANDS];
};

#endif