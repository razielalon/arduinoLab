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

    // Payload
    for (int i = 0; i < DATA_LEN; i++) {
        frame[6 + i] = Data[i];
    }

    // CRC (header + data)
    int crc_index = HEADER_SIZE + DATA_LEN; // 14
    unsigned long CRC = calculateCRC(frame, crc_index);

    frame[crc_index + 0] = (CRC >> 24) & 0xFF;
    frame[crc_index + 1] = (CRC >> 16) & 0xFF;
    frame[crc_index + 2] = (CRC >> 8)  & 0xFF;
    frame[crc_index + 3] =  CRC        & 0xFF;
}


void TX_GBN_func() {
    unsigned long now = millis();

    // Sending frames as long as there's room for more
    // sn_distance(base_sn, next_sn) < N  <=>  next_sn is in the window
    if (sn_distance(base_sn, next_sn) < N) { // base_sn is the oldest unACKed frame(start of window), next_sn is the next SN to send
        build_frame_for_sn(frame_tx, next_sn); // build frame for next_sn

        int sent = sendPackage(frame_tx, FRAME_SIZE);
        if (sent == 1) { // successfully sent
            Serial.print("Sent frame with SN = ");
            Serial.println(next_sn);

            if (base_sn == next_sn) { // if we are sending the first frame in the window, start the timer, timer will reset on ACK reception
                timer_start   = now;
                timer_running = true;
            }

            next_sn = (next_sn + 1) % N; // advance next_sn (modulo N so the window size will be always bigger than SN space)
        }
    }

    // receiving Acks
    if (readPackage(ack_rx, 10) == 1) {// successfully received an ACK with size 10
        int ack_sn = ack_rx[5];   // next expected SN in RX side

        int dist_base_ack  = sn_distance(base_sn, ack_sn); // distance from base_sn to ack_sn
        int dist_base_next = sn_distance(base_sn, next_sn); // distance from base_sn to next_sn

        // is ack_sn is useful or garbage?
        if (dist_base_ack > 0 && dist_base_ack <= dist_base_next) { // second term is extra safety
        // if statement explanation:
        // dist_base_ack > 0  <=>  ack_sn is ahead of base_sn (ack to some package that is in the window)
        // dist_base_ack <= dist_base_next  <=>  ack_sn is not beyond next_sn (ack to some package that is not yet sent)

            unsigned long sample_rtt = millis() - timer_start;
            rtt_samples++;

            timeout = ((float)(rtt_samples - 1) / (float)rtt_samples) * timeout +
                      (1.0f / (float)rtt_samples) * (float)sample_rtt;

            Serial.print("ACK received. next expected SN = ");
            Serial.println(ack_sn);
            Serial.print("RTT sample = ");
            Serial.print(sample_rtt);
            Serial.println(" ms");
            Serial.print("New timeout = ");
            Serial.println(timeout);
            Serial.println();

            base_sn = ack_sn; // advance base_sn to ack_sn(because ack_sn is the next package that the RX side expects)

            if (base_sn == next_sn) { // all outstanding frames are ACKed
                timer_running = false; 
            } else { // there are still outstanding frames that need ACKs
                // restart timer
                timer_start   = millis();
                timer_running = true;
            }
        }
    }

    // timeout maker
    if (timer_running && (millis() - timer_start > (unsigned long)timeout)) { // check if timeout occurred
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
