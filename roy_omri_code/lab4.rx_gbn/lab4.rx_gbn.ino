#include "EthernetLab.h"

#define MAX_FRAMES 8  
#define LINE_RATE  10
#define N          3

// this can be replaced with DATA_LEN = {the expected data len}
const char Data[] = "ELAD&RAZIEL"; 
const uint8_t DATA_LEN = sizeof(Data) - 1;

#define HEADER_SIZE  6
#define CRC_SIZE     4
#define FRAME_SIZE   (HEADER_SIZE + DATA_LEN + CRC_SIZE)

uint8_t frame_rx[FRAME_SIZE];
int expected_frame = 0; // keeps the SN of the next expected frame
static int N_counter = 0; // how many frames recived

float total_frames = 0, bad_frames = 0, succ_frame = 0; 
float eff = 0.0;

uint8_t ack_frame[10];  // Global ack

void setup() {
    Serial.begin(115200);
    setAddress(RX, 1);
    Serial.println("Starting in mode: RX");
    Serial.println("GBN RX Started");
    Serial.print("FRAME_SIZE = ");
    Serial.println(FRAME_SIZE);
}

void loop() {
    RX_func();
}

void RX_func() {
    static float Error_prob = 0;
    int flag_receive;

    flag_receive = readPackage(frame_rx, FRAME_SIZE); // 1 if got a package
  
    if (flag_receive == 1) {

        if (N_counter == 0) {
            Serial.println("**** START WINDOW ****");
        }
        N_counter++;

        // get fields from the frame
        uint8_t destination_address = frame_rx[0];
        uint8_t source_address      = frame_rx[1];
        uint8_t frame_type          = frame_rx[2];
        uint8_t length              = frame_rx[3]; 
        int     received_sn         = frame_rx[5];  

        if (length > DATA_LEN) {
            length = DATA_LEN;
        }

        // get the payload out of the frame
        uint8_t data[DATA_LEN] = {0};
        for (uint8_t i = 0; i < length; ++i) {
            data[i] = frame_rx[6 + i];
        }

        int crc_index = HEADER_SIZE + length;  // 6 + DATA_LEN

        // get the crc field from the frame
        unsigned long received_crc = 0;
        received_crc |= (unsigned long)frame_rx[crc_index + 0] << 24;
        received_crc |= (unsigned long)frame_rx[crc_index + 1] << 16;
        received_crc |= (unsigned long)frame_rx[crc_index + 2] << 8;
        received_crc |= (unsigned long)frame_rx[crc_index + 3];

        // calculate crc of the frame
        unsigned long calculated_crc = calculateCRC(frame_rx, crc_index);

        // check for bad crc
        if (calculated_crc != received_crc) {
            Serial.println("CRC mismatch");
            Serial.println();  
            bad_frames++;

            // build ack packet of latest frame
            ack_frame[0] = 0x01; // Destination Address
            ack_frame[1] = 0x11; // Source Address
            ack_frame[2] = 0; // type = ACK
            ack_frame[3] = 0; //  length = {0 for ACK}
            ack_frame[4] = 0; // ACK/DATA = {0 for ACK}
            ack_frame[5] = expected_frame;  // Next Expected SN (stays the same as it was before frame arrives)
            ack_frame[6] = (calculated_crc >> 24) & 0xFF;
            ack_frame[7] = (calculated_crc >> 16) & 0xFF;
            ack_frame[8] = (calculated_crc >> 8)  & 0xFF;
            ack_frame[9] =  calculated_crc        & 0xFF;
        }

        // frame is okay
        else if (received_sn == expected_frame) { // if got the frame SN that was expected
            succ_frame++;

            Serial.print("Payload (SN ");
            Serial.print(received_sn);
            Serial.print("): ");
            for (uint8_t i = 0; i < length; ++i) {
                Serial.print(char(data[i]));
            }
            Serial.println();  
            Serial.println();  

            // updating the next frame
            expected_frame = (expected_frame + 1) % N;

            // ack building
            ack_frame[0] = 0x01; // Destination Address
            ack_frame[1] = 0x11; // Source Address
            ack_frame[2] = 0; // type = ACK
            ack_frame[3] = 0; // length = {0 for ACK}
            ack_frame[4] = 0; // ACK/DATA = {0 for ACK}
            ack_frame[5] = expected_frame; // Next Expected SN - after raising by 1 with modulu
            ack_frame[6] = (calculated_crc >> 24) & 0xFF;
            ack_frame[7] = (calculated_crc >> 16) & 0xFF;
            ack_frame[8] = (calculated_crc >> 8)  & 0xFF;
            ack_frame[9] =  calculated_crc        & 0xFF;
        }

        // if out of order (got SN it wasnt expected)
        else {
            bad_frames++;
            Serial.print("Out of order frame received. Expected SN: ");
            Serial.print(expected_frame);
            Serial.print(", got: ");
            Serial.println(received_sn);

            // ack on the last good frame
            ack_frame[0] = 0x01; // Destination Address
            ack_frame[1] = 0x11; // Source Address
            ack_frame[2] = 0;    // type = ACK
            ack_frame[3] = 0;    // length = {0 for ACK}
            ack_frame[4] = 0;    // ACK/DATA = {0 for ACK}
            ack_frame[5] = expected_frame; // Next Expected SN (without raising ++)
            ack_frame[6] = (calculated_crc >> 24) & 0xFF;
            ack_frame[7] = (calculated_crc >> 16) & 0xFF;
            ack_frame[8] = (calculated_crc >> 8)  & 0xFF;
            ack_frame[9] =  calculated_crc        & 0xFF;
        }

        // sending ack frame 
        Serial.println("Sending ACK frame...");
        int send_result = 0;
        while (send_result != 1) {
            send_result = sendPackage(ack_frame, 10);
        }
        Serial.print("Sent ACK (next expected SN): ");
        Serial.println(ack_frame[5]);

        
        // calculates efficiency and error probabillity
        eff = ((succ_frame * 64.0) / (float(millis()) * LINE_RATE)) * 1000; // Todo - replace 64 with real data length
        total_frames++; 
        Error_prob = (bad_frames / total_frames);

        Serial.print("Efficiency = ");
        Serial.println(eff, 3);
        Serial.print("Error probability = ");
        Serial.println(Error_prob, 3);
        Serial.println();
        if (N_counter == N){
            N_counter  = 0;
        }
    }
}
