#include "EthernetLab.h"

#define INITIAL_TIMEOUT  10000   // ms
#define N                3       // window size == SN space (0,1,2)

// Data (נשתמש רק ב-8 תווים ראשונים)
const char Data[] = "ELAD&RAZIEL";
const uint8_t DATA_LEN = sizeof(Data) - 1;

// ----- Frame layout -----
#define HEADER_SIZE  6
#define CRC_SIZE     4
#define FRAME_SIZE   (HEADER_SIZE + DATA_LEN + CRC_SIZE)

// Frame buffers
uint8_t frame_tx[FRAME_SIZE];
uint8_t ack_rx[10];

// GBN state
int base_sn      = 0;   // oldest unACKed
int next_sn      = 0;   // next SN to send
int rtt_samples  = 0;   // how many RTT samples used for timeout

// timers
unsigned long timer_start = 0;
bool  timer_running       = false;
float timeout             = INITIAL_TIMEOUT;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    setAddress(TX, 1);

    Serial.print("Starting in mode: TX\nGBN TX Started N = ");
    Serial.println(N);
}

void loop() {
    TX_GBN_func();
}

int sn_distance(int from, int to) {
    int d = to - from;
    if (d < 0) d += N;
    return d;
}

void build_frame_for_sn(uint8_t *frame, int sn) {
    // Header
    frame[0] = 0x11;       // dest
    frame[1] = 0x01;       // src
    frame[2] = 0;          // type = data
    frame[3] = DATA_LEN;   // length = 8

    frame[4] = frame[3];   // ACK/DATA = length
    frame[5] = sn;         // SN

    // Payload – רק 8 בייט
    for (int i = 0; i < DATA_LEN; i++) {
        frame[6 + i] = Data[i];
    }

    // CRC על 14 בייט (6 header + 8 data)
    int crc_index = HEADER_SIZE + DATA_LEN; // 14
    unsigned long CRC = calculateCRC(frame, crc_index);

    frame[crc_index + 0] = (CRC >> 24) & 0xFF;
    frame[crc_index + 1] = (CRC >> 16) & 0xFF;
    frame[crc_index + 2] = (CRC >> 8)  & 0xFF;
    frame[crc_index + 3] =  CRC        & 0xFF;
}

void stop_timer_if_needed() {
    timer_running = false;
}

void TX_GBN_func() {
    unsigned long now = millis();

    // 1) שליחת פריימים חדשים כל עוד יש מקום בחלון
    if (sn_distance(base_sn, next_sn) < N) {
        build_frame_for_sn(frame_tx, next_sn);

        int sent = sendPackage(frame_tx, FRAME_SIZE);
        if (sent == 1) {
            Serial.print("Sent frame with SN = ");
            Serial.println(next_sn);

            if (base_sn == next_sn) {
                timer_start   = now;
                timer_running = true;
            }

            next_sn = (next_sn + 1) % N;
        }
    }

    // 2) קליטת ACK-ים
    if (readPackage(ack_rx, 10) == 1) {
        int ack_sn = ack_rx[5];   // next expected SN בצד RX

        int dist_base_ack  = sn_distance(base_sn, ack_sn);
        int dist_base_next = sn_distance(base_sn, next_sn);

        // ack_sn באמת מקדם את החלון ונמצא בתוך התחום [base_sn+1 .. next_sn]
        if (dist_base_ack > 0 && dist_base_ack <= dist_base_next) {

            unsigned long sample_rtt = millis() - timer_start;
            rtt_samples++;

            timeout =
                ((float)(rtt_samples - 1) / (float)rtt_samples) * timeout +
                (1.0f / (float)rtt_samples) * (float)sample_rtt;

            Serial.print("ACK received. next expected SN = ");
            Serial.println(ack_sn);
            Serial.print("RTT sample = ");
            Serial.print(sample_rtt);
            Serial.println(" ms");
            Serial.print("New timeout = ");
            Serial.println(timeout);
            Serial.println();

            base_sn = ack_sn;

            if (base_sn == next_sn) {
                stop_timer_if_needed();
            } else {
                timer_start   = millis();
                timer_running = true;
            }
        }
    }

    // 3) Timeout – משדרים מחדש את כל הפריימים הלא מאושרים
    if (timer_running && (millis() - timer_start > (unsigned long)timeout)) {
        Serial.println("Timeout! Retransmitting window...");

        int sn = base_sn;
        while (sn != next_sn) {
            build_frame_for_sn(frame_tx, sn);
            int sent = sendPackage(frame_tx, FRAME_SIZE);
            if (sent == 1) {
                Serial.print("Retransmitted SN = ");
                Serial.println(sn);
            }
            sn = (sn + 1) % N;
        }

        timer_start   = millis();
        timer_running = true;
    }
}
