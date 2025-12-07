#include "EthernetLab.h"

#define INITIAL_TIMEOUT  10000   // ms
#define N                3       // window size == SN space (0,1,2)

// Data
const char Data[] = "ELAD&RAZIEL";
const uint8_t DATA_LEN = sizeof(Data) - 1;  // בלי ה-'\0'

// ----- Frame layout -----
#define HEADER_SIZE  6      // dest, src, type, length, ack/data, SN
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
bool timer_running        = false;
float timeout             = INITIAL_TIMEOUT;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    setAddress(TX, 9);

    Serial.println("GBN TX Started (N = 3)");
}

void loop() {
    TX_GBN_func();
}

int sn_distance(int from, int to) {
    // distance in modulo-N (SN in [0..N-1])
    int d = to - from;
    if (d < 0) d += N;
    return d;
}

void build_frame_for_sn(uint8_t *frame, int sn) {
    // Header
    frame[0] = 0x19;         // dest
    frame[1] = 0x09;         // src
    frame[2] = 0;            // type = data
    frame[3] = DATA_LEN;     // length = מספר הבייטים בפיילוד

    frame[4] = frame[3];     // ACK/DATA = length (data)
    frame[5] = sn;           // SN

    // Payload
    for (int i = 0; i < DATA_LEN; i++) {
        frame[6 + i] = Data[i];
    }

    // CRC – מחושב על header+payload, כלומר על HEADER_SIZE + DATA_LEN בייטים
    int crc_index = HEADER_SIZE + DATA_LEN;   // 6 + DATA_LEN
    unsigned long CRC = calculateCRC(frame, crc_index);

    frame[crc_index + 0] = (CRC >> 24) & 0xFF;
    frame[crc_index + 1] = (CRC >> 16) & 0xFF;
    frame[crc_index + 2] = (CRC >> 8)  & 0xFF;
    frame[crc_index + 3] =  CRC        & 0xFF;
}

void start_timer_if_needed() {
    if (!timer_running) {
        timer_start   = millis();
        timer_running = true;
    }
}

void stop_timer_if_needed() {
    timer_running = false;
}

void TX_GBN_func() {
    unsigned long now = millis();

    // 1) שליחת פריימים חדשים כל עוד יש מקום בחלון
    // החלון מלא כאשר distance(base_sn, next_sn) == N
    if (sn_distance(base_sn, next_sn) < N) {
        // אפשר לשלוח פריים חדש עם SN = next_sn
        build_frame_for_sn(frame_tx, next_sn);

        int sent = sendPackage(frame_tx, FRAME_SIZE);
        if (sent == 1) {
            Serial.print("Sent frame with SN = ");
            Serial.println(next_sn);

            // אם זה הפריים הראשון בחלון – מפעילים טיימר
            if (base_sn == next_sn) {
                timer_start   = now;
                timer_running = true;
            }

            next_sn = (next_sn + 1) % N;  // SN במודולו N
        }
    }

    // 2) קליטת ACK-ים (לא בודקים CRC לפי ההנחיה במטלה)
    if (readPackage(ack_rx, 10) == 1) {
        int ack_sn = ack_rx[5];   // next expected SN בצד RX

        // בדוק האם ack_sn באמת מקדם את base_sn
        if (sn_distance(base_sn, ack_sn) > 0 &&
            sn_distance(base_sn, ack_sn) <= N) {

            // יש התקדמות ב-ACK → מודדים RTT
            unsigned long sample_rtt = millis() - timer_start;
            rtt_samples++;

            // עדכון timeout לפי ממוצע
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

            // מקדמים את בסיס החלון
            base_sn = ack_sn;

            // אם כל הפריימים אושרו – נעצור טיימר
            if (base_sn == next_sn) {
                stop_timer_if_needed();
            } else {
                // אחרת – הטיימר מתייחס לפריים הראשון הלא מאושר
                timer_start = millis();
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

        // מאפסים טיימר לחלון החדש
        timer_start   = millis();
        timer_running = true;
    }
}
