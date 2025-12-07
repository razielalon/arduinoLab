#include "EthernetLab.h"

#define BAUD_RATE        115200
#define INITIAL_TIMEOUT  10000   // ms
#define MAX_FRAMES       8

// ----- Payload -----
const char Data[] = "ELAD&RAZIEL";
const uint8_t DATA_LEN = sizeof(Data) - 1; // בלי ה-'\0'

// ----- Frame layout -----
#define HEADER_SIZE  6      // dest, src, type, length, ack/data, SN
#define CRC_SIZE     4
#define FRAME_SIZE   (HEADER_SIZE + DATA_LEN + CRC_SIZE)

uint8_t frame_tx[FRAME_SIZE];
uint8_t ack_tx[10];

enum State { BUILD_FRAME, SEND, WAIT_FOR_ACK };
State state_tx = BUILD_FRAME;

int current_frame  = 0;  // SN: 0/1
int frame_counter  = 0;  // number of RTT samples - incremented upon successful ACK

unsigned long last_sent_time = 0;
unsigned long start_rtt      = 0;
float timeout                = INITIAL_TIMEOUT;

void build_frame() {
    // Header
    frame_tx[0] = 0x11;          // Destination (0x10 + 1)
    frame_tx[1] = 0x01;          // Source      (0x00 + 1)
    frame_tx[2] = 0;             // Type (0 = data, 1 = ACK)
    frame_tx[3] = DATA_LEN;      // Length (payload size)

    frame_tx[4] = frame_tx[3];   // ACK/DATA field --> DATA_LEN for data frames, 0 for ACK frames
    frame_tx[5] = current_frame; // SN

    // Payload
    for (int i = 0; i < DATA_LEN; i++) {
        frame_tx[6 + i] = Data[i];
    }

    // CRC index is right after payload
    int crc_index = 6 + DATA_LEN;
    unsigned long CRC = calculateCRC(frame_tx, crc_index);

    frame_tx[crc_index + 0] = (CRC >> 24) & 0xFF;
    frame_tx[crc_index + 1] = (CRC >> 16) & 0xFF;
    frame_tx[crc_index + 2] = (CRC >> 8)  & 0xFF;
    frame_tx[crc_index + 3] =  CRC        & 0xFF;

    Serial.println("\n=== Building frame ===");
    Serial.print("SN = ");
    Serial.println(current_frame);
    Serial.print("Length (DATA_LEN) = ");
    Serial.println(DATA_LEN);
    Serial.print("Data sent: ");
    for (int i = 0; i < DATA_LEN; i++) {
        Serial.print((char)frame_tx[6 + i]);
    }
    Serial.println();

}

void setup() {
    //Serial.begin(BAUD_RATE);
    //while (!Serial) { ; } // wait for serial port to connect.

    setAddress(TX, 1);

    Serial.println("Starting in mode: TX (Stop & Wait)");
}

void loop() {
    TX_func();
}

void TX_func() {
    unsigned long current_time = millis();
    static int flag_send = 0;

    // ========== STATE 0: בניית פריים ==========
    if (state_tx == BUILD_FRAME) {

        build_frame();

        start_rtt = millis();
        state_tx  = SEND;
    }

    // ========== STATE 1: שליחת פריים ==========
    else if (state_tx == SEND) {

        flag_send = sendPackage(frame_tx, FRAME_SIZE);

        if (flag_send == 1) {
            last_sent_time = current_time;

            Serial.print("Frame sent successfully. SN: ");
            Serial.println(current_frame);

            state_tx = WAIT_FOR_ACK;  // מחכים ל-ACK
        }
    }

    // ========== STATE 2: המתנה ל-ACK ==========
    else if (state_tx == WAIT_FOR_ACK) {

        // אם הגיע ACK
        if (readPackage(ack_tx, 10) == 1) {
            int received_sn = ack_tx[5];

            // ב-S&W שלנו ה-ACK שולח SN = 1 - current_frame
            if (received_sn == 1 - current_frame) {

                unsigned long finish_rtt = millis() - start_rtt;

                frame_counter++;

                // עדכון timeout לפי RTT ממוצע
                timeout =
                    ((float)(frame_counter - 1) / (float)frame_counter) * timeout +
                    (1.0f / (float)frame_counter) * (float)finish_rtt;

                Serial.print("ACK received. SN: ");
                Serial.println(received_sn);
                Serial.print("RTT = ");
                Serial.print(finish_rtt);
                Serial.println(" ms");
                Serial.print("New timeout = ");
                Serial.println(timeout);
                Serial.println();

                // הכנה לפריים הבא
                current_frame = 1 - current_frame;
                state_tx      = BUILD_FRAME;
            }
        }

        // אם עבר זמן גדול מה-timeout ואין ACK → משדרים שוב
        if (millis() - last_sent_time > (unsigned long)timeout) {
            Serial.println("Timeout waiting for ACK. Retransmitting...");
            state_tx = SEND;  // לשלוח שוב את אותו פריים
        }
    }
}
