//#include "Arduino.h"
#include "Brain.h"
#include "mbed.h"

Brain::Brain(RawSerial &_brainStream) {
    // Attach to our serial stream
    brainStream = &_brainStream;
    //initialize all of the class's local variables
    init();
}

//------------------------------------------------------------------------------
// Function Name: init
// Variables: N/A
// Purpose: Initialization routine
// Returns: N/A
//------------------------------------------------------------------------------
void Brain::init() {
    freshPacket = false;
    inPacket = false;
    freshError = false;
    readStep = 0;
    packetIndex = 0;
    packetLength = 0;
    eegPowerLength = 0;
    hasPower = false;
    checksum = 0;
    checksumAccumulator = 0;
    pktNotif = 0;

    signalQuality = 200;
    attention = 0;
    meditation = 0;

    clearEegPower();
}

//------------------------------------------------------------------------------
// Function Name: update
// Variables: N/A
// Purpose: Process bytes from the serial buffer.
// Returns: 
// False if a packet is not ready to read.
// True if a packet is ready to read out. 
//------------------------------------------------------------------------------
bool Brain::update() {
    lb = brainStream->getc();
        
    switch(readStep){
        case 0x00:
            if (lb == 0xAA)
                readStep = 0x01;
            break;
        case 0x01:
            if (lb == 0xAA)
                readStep = 0x02;
            break;
        case 0x02:
            if (lb == 0xAA)
            {
                //do nothing. We return here.
            }
            else if (lb > 0xAA)
            {
                latestError = ERR_LONG_PACKET;
                freshError = true;
                readStep = 0x00;
            }
            else
            {
                readStep = 0x03;
                packetIndex = 0;
                packetLength = lb;
                checksumAccumulator = 0;
            } 
            break;
        case 0x03:
            packetData[packetIndex++] = lb;
            checksumAccumulator += lb;
            if (packetIndex == packetLength)
                readStep = 0x04;
            break;
        case 0x04:
            checksum = lb;
            checksumAccumulator = 0xFF - checksumAccumulator;
            if(checksum != checksumAccumulator)
            {
                latestError = ERR_BAD_CHKSUM;
                freshError = true;
            }
            else 
            {
                if (parsePacket()) {
                    freshPacket = true;
                }
                else {
                    // Parsing failed, send an error.
                    latestError = ERR_BAD_PARSE;
                    freshError = true;
                    // good place to print the packet if debugging
                }
                readStep = 0x00;
            }
            break;
        default:
            readStep = 0x00;
            break;
        }

    if (freshPacket) {
        freshPacket = false;
        return true;
    }
    else {
        return false;
    }

}

//------------------------------------------------------------------------------
// Function Name: clearPacket
// Variables: N/A
// Purpose: Clears the packetData buffer.
// Returns: N/A
//------------------------------------------------------------------------------
void Brain::clearPacket() {
    memset(&packetData, 0, sizeof(packetData));
}

//------------------------------------------------------------------------------
// Function Name: clearEegPower
// Variables: N/A
// Purpose: Clears the eegPower array
// Returns: N/A
//------------------------------------------------------------------------------
void Brain::clearEegPower() {
    // Zero the power bands.
    memset(&eegPower, 0, sizeof(eegPower));
}

//------------------------------------------------------------------------------
// Function Name: parsePacket
// Variables: N/A
// Purpose: Parses incoming packets from the Brain device.
// Returns: True for succesful parse, False for unsuccesful.
//------------------------------------------------------------------------------
bool Brain::parsePacket() {
    // Loop through the packet, extracting data.
    // Based on mindset_communications_protocol.pdf from the Neurosky Mindset SDK.
    // Returns true if passing succeeds
    hasPower = false;
    bool parseSuccess = true;
    int rawValue = 0;

    clearEegPower();    // clear the eeg power to make sure we're honest about missing values

    for (uint8_t i = 0; i < packetLength; i++) {
        switch (packetData[i]) {
            case 0x2:
                signalQuality = packetData[++i];
                pktNotif |= 1UL << PKT_SIGNAL;
                break;
            case 0x4:
                attention = packetData[++i];
                pktNotif |= 1UL << PKT_ATTN;
                break;
            case 0x5:
                meditation = packetData[++i];
                pktNotif |= 1UL << PKT_MEDT;
                break;
            case 0x83:
                // ASIC_EEG_POWER: eight big-endian 3-uint8_t unsigned integer values representing delta, theta, low-alpha high-alpha, low-beta, high-beta, low-gamma, and mid-gamma EEG band power values
                // The next uint8_t sets the length, usually 24 (Eight 24-bit numbers... big endian?)
                // We dont' use this value so let's skip it and just increment i
                i++;

                // Extract the values
                for (int j = 0; j < EEG_POWER_BANDS; j++) {
                    eegPower[j] = ((uint32_t)packetData[++i] << 16) | ((uint32_t)packetData[++i] << 8) | (uint32_t)packetData[++i];
                }

                hasPower = true;
                pktNotif |= 1UL << PKT_EEG;
                // This seems to happen once during start-up on the force trainer. Strange. Wise to wait a couple of packets before
                // you start reading.
                break;
            case 0x80:
                // We dont' use this value so let's skip it and just increment i
                // uint8_t packetLength = packetData[++i];
                i++;
                rawValue = ((int)packetData[++i] << 8) | packetData[++i];
                pktNotif |= 1UL << PKT_RAW;
                break;
            default:
                // Broken packet 
                parseSuccess = false;
                break;
        }
    }
    return parseSuccess;
}

//------------------------------------------------------------------------------
// Function Name: readErrors
// Variables: N/A
// Purpose: Read out why we were unable to read out a message from Brain device
// Returns: character pointer with error message
//------------------------------------------------------------------------------
char* Brain::readErrors() {
    freshError = false;
    switch (latestError){
        case ERR_LONG_PACKET:
            sprintf(errorMsg, "ERROR: PACKET TOO LONG\r\n");
            break;
        case ERR_BAD_CHKSUM:
            sprintf(errorMsg, "ERROR: BAD CHECKSUM\r\n");
            break;
        case ERR_BAD_PARSE:
            sprintf(errorMsg, "ERROR: BAD PARSE\r\n");
            break;
        }
    return errorMsg;
}

//------------------------------------------------------------------------------
// Function Name: readCSV
// Variables: N/A
// Purpose: Read out all current data in a format exportable to CSV.
// Returns: character pointer with data in string format.
//------------------------------------------------------------------------------
char* Brain::readCSV() {
    if(hasPower) {
        sprintf(csvBuffer,"%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
            signalQuality,
            attention,
            meditation,
            eegPower[0],
            eegPower[1],
            eegPower[2],
            eegPower[3],
            eegPower[4],
            eegPower[5],
            eegPower[6],
            eegPower[7]
        );
        return csvBuffer;
    }
    else {
        sprintf(csvBuffer,"%d,%d,%d\r\n",
            signalQuality,
            attention,
            meditation
        );
        return csvBuffer;
    }
}

//------------------------------------------------------------------------------
// Function Name: printPacket
// Variables: N/A
// Purpose: Print out raw packet
// Returns: char pointer to raw packet data in string format.
//------------------------------------------------------------------------------
char* Brain::printPacket() {
    memset(&debugBuffer, 0, sizeof(debugBuffer));
    sprintf(debugBuffer,"[");
    for (uint8_t i = 0; i < MAX_PACKET_LENGTH; i++) {
        
        sprintf(debugBuffer + strlen(debugBuffer),"%d", packetData[i]);

            if (i < MAX_PACKET_LENGTH - 1) {
                sprintf(debugBuffer + strlen(debugBuffer),", ");
            }
    }
    sprintf(debugBuffer + strlen(debugBuffer),"]");
}

//------------------------------------------------------------------------------
// Function Name: newError
// Variables: N/A
// Purpose: Check for if we have a new error.
// Returns: True if new error, False if no new errors.
//------------------------------------------------------------------------------
bool Brain::newError(){
    return freshError;    
}

//------------------------------------------------------------------------------
// Function Name: readSignalQuality
// Variables: N/A
// Purpose: read out the Signal's Quality
// Returns: uint8 value that returns how good our connection is.
//------------------------------------------------------------------------------
uint8_t Brain::readSignalQuality() {
    return signalQuality;
}

//------------------------------------------------------------------------------
// Function Name: readAttention
// Variables: N/A
// Purpose: read out the subject's attention value
// Returns: uint8 value that goes from 0-100 to tell us a subject's attention
//------------------------------------------------------------------------------
uint8_t Brain::readAttention() {
    return attention;
}

//------------------------------------------------------------------------------
// Function Name: readMeditation
// Variables: N/A
// Purpose: read out the subject's attention value
// Returns: uint8 value that goes from 0-100 to tell us a subject's attention
//------------------------------------------------------------------------------
uint8_t Brain::readMeditation() {
    return meditation;
}

//------------------------------------------------------------------------------
// Function Name: readPowerArray
// Variables: N/A
// Purpose: read out the subject's EEG values.
// Returns: uint32 pointer to the EEGPower array.
//------------------------------------------------------------------------------
uint32_t* Brain::readPowerArray() {
    return eegPower;
}

//------------------------------------------------------------------------------
// Function Name: readDelta
// Variables: N/A
// Purpose: read out the subject's Delta value.
// Returns: uint32 of the Delta wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readDelta() {
    return eegPower[0];
}

//------------------------------------------------------------------------------
// Function Name: readTheta
// Variables: N/A
// Purpose: read out the subject's Theta value.
// Returns: uint32 of the Theta wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readTheta() {
    return eegPower[1];
}

//------------------------------------------------------------------------------
// Function Name: readLowAlpha
// Variables: N/A
// Purpose: read out the subject's low Alpha value.
// Returns: uint32 of the low Alpha wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readLowAlpha() {
    return eegPower[2];
}

//------------------------------------------------------------------------------
// Function Name: readHighAlpha
// Variables: N/A
// Purpose: read out the subject's High Alpha value.
// Returns: uint32 of the High Alpha wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readHighAlpha() {
    return eegPower[3];
}

//------------------------------------------------------------------------------
// Function Name: readLowbeta
// Variables: N/A
// Purpose: read out the subject's low Beta value.
// Returns: uint32 of the Low Beta wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readLowBeta() {
    return eegPower[4];
}

//------------------------------------------------------------------------------
// Function Name: readHighBeta
// Variables: N/A
// Purpose: read out the subject's High Beta value.
// Returns: uint32 of the High beta wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readHighBeta() {
    return eegPower[5];
}

//------------------------------------------------------------------------------
// Function Name: readLowGamma
// Variables: N/A
// Purpose: read out the subject's Low Gamma value.
// Returns: uint32 of the Low Gamma wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readLowGamma() {
    return eegPower[6];
}

//------------------------------------------------------------------------------
// Function Name: readMidGamma
// Variables: N/A
// Purpose: read out the subject's Mid Gamma value.
// Returns: uint32 of the mid Gamma wave values.
//------------------------------------------------------------------------------
uint32_t Brain::readMidGamma() {
    return eegPower[7];
}

//------------------------------------------------------------------------------
// Function Name: readNotifications
// Variables: N/A
// Purpose: read out our current notifications.
// Returns: uint8 of the notification 'register'.
//------------------------------------------------------------------------------
uint8_t Brain::readNotifications() {
    return pktNotif;    
}

//------------------------------------------------------------------------------
// Function Name: readEEGPower
// Variables: N/A
// Purpose: read out the EEG Power array in a CSV friendly format and clear its 
//          notification
// Returns: char pointer to the array values in string format.
//------------------------------------------------------------------------------
char* Brain::readEEGPower() {
    sprintf(csvBuffer,"%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
        signalQuality,
        eegPower[0],
        eegPower[1],
        eegPower[2],
        eegPower[3],
        eegPower[4],
        eegPower[5],
        eegPower[6],
        eegPower[7]
    );
    pktNotif &= ~(1UL << PKT_EEG);
    return csvBuffer;
}