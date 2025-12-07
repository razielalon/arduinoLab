#include "EthernetLab.h"
#define frame_size 18
#define INITIAL_TIMEOUT  10000
#define MAX_FRAMES 8
#define N 3

uint8_t frame_tx[frame_size];
uint8_t ack_tx[10];
const char* Data = "ROY%OMRI";
unsigned long CRC;
static int frame_counter = 0;
int state_tx = 0;
int N_counter = 0;
static unsigned long last_sent_time = 0;    
int total_frames = 0, bad_frames = 0, current_frame = 0, next_frame = 0, ack_received = 0;  
static float timeout = INITIAL_TIMEOUT;    


void setup() {
  Serial.begin(115200);
    void TX_func();
    void receiveACK();
    CRC = calculateCRC(Data, 8);
    setAddress(TX, 9);
}

void loop() {
    TX_func();
}

void TX_func() {
  static int received_sn = 0;
    unsigned long current_time = millis();        // Current time
    static int flag_send = 0;                     // Flag for frame send success
    static float last_sent_time = 0;      // Time the last frame was sent
    static float start_rtt = 0;
    static float prev_finish_rtt = 0;                            // Start round-trip time
    

    // Frame preparation state
    if (state_tx == 0) {
        frame_tx[0] = 0x19; // destination_address
        frame_tx[1] = 0x09; // source_address
        frame_tx[2] = 0;    // frame_type
        frame_tx[3] = 8;    // length (size of Data)

        frame_tx[4] = frame_tx[3];             // Length confirmation
        frame_tx[5] = current_frame;          // Sequence number

        // Copy Data into the frame
        for (int i = 0; i < frame_tx[3]; i++) {
            frame_tx[6 + i] = Data[i];
        }

        // Calculate CRC
        unsigned long CRC = calculateCRC(frame_tx, 14);
        frame_tx[14] = (CRC >> 24) & 0xFF;
        frame_tx[15] = (CRC >> 16) & 0xFF;
        frame_tx[16] = (CRC >> 8) & 0xFF;
        frame_tx[17] = CRC & 0xFF;

        state_tx = 1; // Transition to the next state
        start_rtt = millis(); // start transmition
    }
    // Frame transmission state
    else if (state_tx == 1) {
        flag_send = sendPackage(frame_tx, frame_size);

        if (flag_send == 1) { // Frame sent successfully
            N_counter ++;
            Serial.print("Frame sent successfully. SN: ");
            Serial.println(current_frame);
            if (N_counter == N){
                state_tx = 2; // Transition to ACK waiting state
                N_counter = 0;
            }
            else {
              current_frame = (current_frame + 1) % N;
              frame_tx[5] = current_frame;
            }
            
        } 
    }
    // ACK waiting state
    else if (state_tx == 2) {
      current_time = millis();
        if (readPackage(ack_tx, 10)) {
            received_sn = ack_tx[5];  // Get the sequence number of the received ACK
            float finish_rtt = millis() - start_rtt; // Calculate RTT
            frame_counter++; // Increment total frame counter
            Serial.print("frame = ");
            Serial.println(frame_counter);

          //Update timeout using exponential moving average
            timeout = ((float(frame_counter - 1.0) / (float)frame_counter) * timeout) +
                      ((1.0 / frame_counter) * finish_rtt);

            Serial.print("ACK received. SN: ");
            Serial.println(received_sn);
            Serial.print("New Timeout: ");
            Serial.println(timeout);
            Serial.println();

            // Prepare for next frame
            current_frame = received_sn;
            state_tx = 0; // Return to frame preparation
            prev_finish_rtt = finish_rtt;
        
        } else if (current_time - start_rtt > timeout) { // Timeout occurred
            Serial.println("Timeout waiting for ACK. Retrying...");
            state_tx = 1; // Retry sending frame
            current_frame = received_sn;
        }
    }
}





