#include "EthernetLab.h"

#define FRAME_SIZE       18
#define INITIAL_TIMEOUT  10000   // ms
#define MAX_FRAMES       8

uint8_t frame_tx[FRAME_SIZE];
uint8_t ack_tx[10];

const char Data[] = "ROY%OMRI";

enum State { BUILD_FRAME, SEND, WAIT_FOR_ACK };
State state_tx = BUILD_FRAME;
int current_frame  = 0;  // SN: 0/1
int frame_counter  = 0;  // how many RTT samples we already took

unsigned long last_sent_time = 0;
unsigned long start_rtt      = 0;
float timeout                = INITIAL_TIMEOUT;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }  // אופציונלי בלוחות מסוימים

    setAddress(TX, 9);     // מגדיר כתובות + Ethernet

    Serial.println("Starting in mode: TX");
}

void loop() {
    TX_func();
}

void TX_func() {
    unsigned long current_time = millis();
    static int flag_send = 0;

    // ========== STATE 0: בניית פריים ==========
    if (state_tx == BUILD_FRAME) {

        // Header
        frame_tx[0] = 0x19;          // Destination (0x10 + 9)
        frame_tx[1] = 0x09;          // Source      (0x00 + 9)
        frame_tx[2] = 0;             // Type
        frame_tx[3] = 8;             // Length = גודל "ROY%OMRI"

        frame_tx[4] = frame_tx[3];   // ACK/DATA field = length (data)
        frame_tx[5] = current_frame; // SN

        // Payload
        for (int i = 0; i < frame_tx[3]; i++) {
            frame_tx[6 + i] = Data[i];
        }

        // CRC על 14 הבייטים הראשונים
        unsigned long CRC = calculateCRC(frame_tx, 14);
        frame_tx[14] = (CRC >> 24) & 0xFF;
        frame_tx[15] = (CRC >> 16) & 0xFF;
        frame_tx[16] = (CRC >> 8)  & 0xFF;
        frame_tx[17] =  CRC        & 0xFF;

        Serial.println("\n=== Building frame ===");
        Serial.print("SN = ");
        Serial.println(current_frame);

        start_rtt = millis();
        state_tx  = SEND;  // לעבור לשלב השידור
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

            // אם ה-ACK מתאים (1 - current_frame)
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
                state_tx      = BUILD_FRAME;  // חוזרים לבניית פריים חדש
            }
        }

        // אם עבר זמן גדול מה-timeout ואין ACK → משדרים שוב
        if (millis() - last_sent_time > (unsigned long)timeout) {
            Serial.println("Timeout waiting for ACK. Retransmitting...");
            state_tx = SEND;  // לשלוח שוב את אותו פריים
        }
    }
}
