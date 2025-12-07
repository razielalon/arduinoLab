# Arduino Network Protocol Implementation: Stop-and-Wait vs Go-Back-N

This document provides a detailed explanation of two network protocol implementations for Arduino devices: **Stop-and-Wait (SAW)** and **Go-Back-N (GBN)**.

---

## Overview

Both protocols implement reliable data transmission over Ethernet between two Arduino boards:
- **TX (Transmitter)**: Sends data frames
- **RX (Receiver)**: Receives frames and sends acknowledgments (ACKs)

The key difference is how they handle multiple frames:
- **Stop-and-Wait**: Sends ONE frame at a time, waits for ACK before sending the next
- **Go-Back-N**: Sends multiple frames in a window, retransmits all unacknowledged frames on timeout

---

## Frame Structure

Both protocols use 18-byte frames:

```
Byte 0     : Destination Address (0x10 + device_id)
Byte 1     : Source Address      (0x00 + device_id)
Byte 2     : Frame Type          (0 = data)
Byte 3     : Length              (payload size, 8 bytes for "ROY%OMRI")
Byte 4     : ACK/DATA field      (length for data, 0 for ACK)
Byte 5     : Sequence Number (SN) (0 or 1 for SAW, 0-2 for GBN)
Bytes 6-13 : Payload data        (8 bytes: "ROY%OMRI")
Bytes 14-17: CRC-32 checksum     (4 bytes)
```

---

# Part 1: STOP-AND-WAIT (SAW) Protocol

## Overview
A simple protocol where:
1. Sender transmits ONE frame
2. Sender waits for ACK (with timeout)
3. If ACK received → send next frame
4. If timeout → retransmit the same frame

**Sequence Numbers**: Only 0 and 1 (alternating)

---

## TRANSMITTER CODE (`lab4.tx.ino`)

### Global Variables

```cpp
#define FRAME_SIZE       18          // Fixed frame size
#define INITIAL_TIMEOUT  10000       // 10 seconds timeout
#define MAX_FRAMES       8           // Not really used in SAW

uint8_t frame_tx[FRAME_SIZE];        // Current frame being sent
uint8_t ack_tx[10];                  // Buffer for incoming ACKs

const char Data[] = "ROY%OMRI";      // 8-byte payload

int state_tx       = 0;              // State machine: 0=build, 1=send, 2=wait
int current_frame  = 0;              // SN: alternates between 0 and 1
int frame_counter  = 0;              // Number of successful RTT samples
float timeout      = INITIAL_TIMEOUT; // Adaptive timeout
unsigned long last_sent_time = 0;    // When was frame last sent
unsigned long start_rtt      = 0;    // When did frame go out (for RTT calc)
```

### State Machine (3 States)

#### STATE 0: Build Frame
```
Constructs the frame with:
- Destination: 0x19 (0x10 + 9)
- Source: 0x09 (0x00 + 9)
- Type: 0 (data frame)
- Length: 8 (payload size)
- ACK/DATA field: 8 (data, not ACK)
- SN: current_frame (0 or 1)
- Payload: "ROY%OMRI"
- CRC: Computed on first 14 bytes
```

**Line-by-line breakdown:**
```cpp
frame_tx[0] = 0x19;          // 0x10 + 9 = dest address
frame_tx[1] = 0x09;          // 0x00 + 9 = source address  
frame_tx[2] = 0;             // Type: data (not ACK)
frame_tx[3] = 8;             // Length: "ROY%OMRI" = 8 chars
frame_tx[4] = frame_tx[3];   // ACK/DATA = 8 (indicates data)
frame_tx[5] = current_frame; // SN: 0 or 1

// Copy payload
for (int i = 0; i < 8; i++) {
    frame_tx[6 + i] = Data[i];  // Bytes 6-13 = payload
}

// Calculate 32-bit CRC on bytes 0-13
unsigned long CRC = calculateCRC(frame_tx, 14);
// Store CRC in bytes 14-17 (big-endian)
frame_tx[14] = (CRC >> 24) & 0xFF;  // Most significant byte
frame_tx[15] = (CRC >> 16) & 0xFF;
frame_tx[16] = (CRC >> 8)  & 0xFF;
frame_tx[17] =  CRC        & 0xFF;  // Least significant byte
```

**State transition**: STATE 0 → STATE 1

---

#### STATE 1: Send Frame
```cpp
flag_send = sendPackage(frame_tx, FRAME_SIZE);

if (flag_send == 1) {  // Success
    last_sent_time = current_time;  // Record send time
    Serial.print("Frame sent successfully. SN: ");
    Serial.println(current_frame);
    state_tx = 2;  // Move to waiting for ACK
}
```

**What happens:**
- Tries to send the frame via Ethernet
- If successful (returns 1), record the time and wait for ACK
- If unsuccessful, stays in STATE 1 and retries

**State transition**: STATE 1 → STATE 2

---

#### STATE 2: Wait for ACK with Timeout

```cpp
if (readPackage(ack_tx, 10) == 1) {  // ACK arrived?
    int received_sn = ack_tx[5];      // Extract SN from ACK
    
    // Check if ACK is for the NEXT frame we expect to send
    // (SAW uses opposite SN: if we sent SN=0, ACK should have SN=1)
    if (received_sn == 1 - current_frame) {
        
        unsigned long finish_rtt = millis() - start_rtt;
        frame_counter++;
        
        // ===== ADAPTIVE TIMEOUT CALCULATION =====
        // Weighted average: older samples have less weight
        timeout = 
            ((float)(frame_counter - 1) / (float)frame_counter) * timeout +
            (1.0f / (float)frame_counter) * (float)finish_rtt;
        
        // Example: frame_counter=1: timeout = 0 + 1.0 * RTT = RTT
        //         frame_counter=2: timeout = 0.5 * old_timeout + 0.5 * new_RTT
        //         frame_counter=3: timeout = 0.667 * old_timeout + 0.333 * new_RTT
        
        Serial.print("ACK received. SN: ");
        Serial.println(received_sn);
        Serial.print("RTT = ");
        Serial.print(finish_rtt);
        Serial.println(" ms");
        
        // ===== PREPARE FOR NEXT FRAME =====
        current_frame = 1 - current_frame;  // Toggle SN: 0→1 or 1→0
        state_tx = 0;  // Return to building next frame
    }
}

// ===== TIMEOUT HANDLING =====
if (millis() - last_sent_time > (unsigned long)timeout) {
    Serial.println("Timeout waiting for ACK. Retransmitting...");
    state_tx = 1;  // Resend the SAME frame (no SN change)
}
```

**Key points:**
- **Cumulative ACK**: The ACK's SN field indicates the next frame the receiver expects
- **SAW ACK logic**: If TX sent SN=0, RX replies with SN=1 (next expected = 1)
- **Adaptive timeout**: Gets longer if network is slow, shorter if it's fast
- **Timeout trigger**: Retransmits the same frame without changing SN

**State transition**: 
- Success → STATE 0 (next frame)
- Timeout → STATE 1 (retransmit)

---

### Flow Diagram: TX State Machine

```
┌─────────────────┐
│  STATE 0: BUILD │
│ Create frame    │
│ with SN=0 or 1  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  STATE 1: SEND  │
│ Transmit frame  │
│ Record time     │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│   STATE 2: WAIT FOR ACK             │
│   with adaptive timeout              │
└────────┬──────────────┬────────┘
         │              │
    ACK OK?         Timeout?
         │              │
         ▼              ▼
    Next SN  ─────► Resend (same SN)
    STATE 0         STATE 1
```

---

## RECEIVER CODE (`lab4.rx.ino`)

### Global Variables

```cpp
uint8_t frame_rx[18];        // Received frame buffer
int expected_frame = 0;       // Next SN we expect (0 or 1)
float total_frames = 0;       // Statistics
float bad_frames = 0;
float succ_frame = 0;
float eff = 0.0;              // Efficiency metric
```

### Reception Logic

```cpp
void RX_func() {
    int flag_receive = readPackage(frame_rx, 18);  // Try to receive
    
    if (flag_receive == 1) {  // Frame received
        
        // ===== PARSE FRAME HEADER =====
        uint8_t destination_address = frame_rx[0];
        uint8_t source_address      = frame_rx[1];
        uint8_t length              = frame_rx[3];
        int received_sn             = frame_rx[5];
        
        // ===== EXTRACT PAYLOAD =====
        uint8_t data[8] = {0};
        for (uint8_t i = 0; i < length; ++i) {
            data[i] = frame_rx[6 + i];
        }
        
        // ===== VERIFY CRC =====
        unsigned long received_crc = 0;
        received_crc |= (unsigned long)frame_rx[14] << 24;  // Byte 14 → bits 24-31
        received_crc |= (unsigned long)frame_rx[15] << 16;  // Byte 15 → bits 16-23
        received_crc |= (unsigned long)frame_rx[16] << 8;   // Byte 16 → bits 8-15
        received_crc |= (unsigned long)frame_rx[17];         // Byte 17 → bits 0-7
        
        unsigned long calculated_crc = calculateCRC(frame_rx, 14);
        
        // ===== DECISION TREE =====
        
        // CASE 1: CRC ERROR
        if (calculated_crc != received_crc) {
            Serial.println("CRC mismatch");
            bad_frames++;
            // Don't send ACK (sender will timeout and retransmit)
        }
        
        // CASE 2: CORRECT FRAME, IN ORDER, NO PRIOR ERRORS
        else if (received_sn == expected_frame) {
            succ_frame++;
            expected_frame = 1 - expected_frame;  // Update for next frame
            
            Serial.print("Payload: ");
            for (int i = 0; i < length; ++i) {
                Serial.print(char(data[i]));
            }
            Serial.println();
            
            // ===== SEND ACK =====
            uint8_t ack_frame[10];
            ack_frame[0] = 0x09;  // Swap source/dest
            ack_frame[1] = 0x19;
            ack_frame[2] = 0;     // Type
            ack_frame[3] = 0;     // Length (ACK has no payload)
            ack_frame[4] = 0;     // ACK/DATA field = 0 (ACK, not data)
            ack_frame[5] = 1 - received_sn;  // SN of NEXT expected frame
            
            // CRC of ACK
            ack_frame[6] = (calculated_crc >> 24) & 0xFF;
            ack_frame[7] = (calculated_crc >> 16) & 0xFF;
            ack_frame[8] = (calculated_crc >> 8)  & 0xFF;
            ack_frame[9] =  calculated_crc        & 0xFF;
            
            // Keep sending until successful
            int send_result = 0;
            while (send_result != 1) {
                send_result = sendPackage(ack_frame, 10);
            }
            
            Serial.print("Sent ACK (SN field): ");
            Serial.println(ack_frame[5]);
        }
        
        // CASE 3: OUT-OF-ORDER FRAME
        else {
            bad_frames++;
            Serial.print("Out of order frame. Expected SN: ");
            Serial.println(expected_frame);
            // Don't send ACK
        }
        
        // ===== STATISTICS =====
        eff = ((succ_frame * 64.0) / (float(millis()) * 10)) * 1000;
        total_frames++;
        float Error_prob = (bad_frames / total_frames);
        
        Serial.print("Efficiency = ");
        Serial.println(eff, 3);
        Serial.print("Error probability = ");
        Serial.println(Error_prob, 3);
    }
}
```

### Key RX Logic Points

**Expected Frame Field in ACK:**
- If TX sent SN=0 and it's correct: RX sends ACK with SN=1
- This tells TX: "I've received frame 0, I'm now expecting frame 1"
- This is cumulative acknowledgment

**Error Handling:**
- **CRC mismatch**: Discard frame, don't send ACK (sender retransmits on timeout)
- **Out of order**: Frame arrives with wrong SN, discard it
- **Duplicates**: Not checked explicitly, handled by SN matching

---

# Part 2: GO-BACK-N (GBN) Protocol

## Overview
An optimized protocol where:
1. Sender maintains a **window** of N frames (N=3 in this code)
2. Can send multiple frames without waiting for ACKs
3. On timeout → retransmit ALL unacknowledged frames
4. Cumulative ACKs advance the window

**Sequence Numbers**: 0, 1, 2 (3 possible values, window size N=3)

**Advantages over SAW:**
- Higher throughput (multiple frames in flight)
- Efficient use of network bandwidth

---

## TRANSMITTER CODE (`lab4.tx_gbn.ino`)

### Global Variables

```cpp
#define N                3   // Window size

// Frame window management
int base_sn      = 0;   // Oldest unacknowledged frame
int next_sn      = 0;   // Next SN to send
int rtt_samples  = 0;   // Counter for timeout adaptation

// Timing
unsigned long timer_start = 0;
bool timer_running        = false;
float timeout             = INITIAL_TIMEOUT;
```

**Window concept:**
- **base_sn**: Points to the oldest unacknowledged frame
- **next_sn**: Where we'll place the next NEW frame
- **Window size**: distance(base_sn, next_sn) must be < N
- **Full window**: when distance(base_sn, next_sn) == N

Example with N=3:
```
Frames sent so far: SN=0, SN=1, SN=2
base_sn = 0 (waiting for ACK of frame 0)
next_sn = 0 (no more room, window full)

Then ACK arrives for SN=0,1:
base_sn = 2 (now waiting for frame 2)
next_sn = 2 (can send new frame with SN=2)
```

---

### Helper Functions

#### `sn_distance(int from, int to)`
Calculates the circular distance between two sequence numbers (modulo N):

```cpp
int sn_distance(int from, int to) {
    int d = to - from;
    if (d < 0) d += N;  // Wrap around (e.g., 2 to 0 = 0-2+3 = 1)
    return d;
}

// Examples with N=3:
// distance(0, 0) = 0 (same position)
// distance(0, 1) = 1 (1 ahead)
// distance(0, 2) = 2 (2 ahead)
// distance(1, 0) = 2 (wrap around: 0-1+3 = 2)
// distance(2, 1) = 2 (wrap around: 1-2+3 = 2)
```

#### `build_frame_for_sn(uint8_t *frame, int sn)`
Creates a frame with a specific sequence number:

```cpp
void build_frame_for_sn(uint8_t *frame, int sn) {
    frame[0] = 0x19;         // Destination
    frame[1] = 0x09;         // Source
    frame[2] = 0;            // Type
    frame[3] = 8;            // Length
    frame[4] = frame[3];     // ACK/DATA = 8 (data)
    frame[5] = sn;           // Sequence number
    
    for (int i = 0; i < 8; i++) {
        frame[6 + i] = Data[i];
    }
    
    unsigned long CRC = calculateCRC(frame, 14);
    frame[14] = (CRC >> 24) & 0xFF;
    frame[15] = (CRC >> 16) & 0xFF;
    frame[16] = (CRC >> 8)  & 0xFF;
    frame[17] =  CRC        & 0xFF;
}
```

---

### Main Loop: `TX_GBN_func()`

The transmitter does three things in each loop iteration:

#### PHASE 1: Send New Frames (if window not full)

```cpp
if (sn_distance(base_sn, next_sn) < N) {
    // Window has room for more frames
    
    build_frame_for_sn(frame_tx, next_sn);
    int sent = sendPackage(frame_tx, FRAME_SIZE);
    
    if (sent == 1) {
        Serial.print("Sent frame with SN = ");
        Serial.println(next_sn);
        
        // Start timer if this is the first frame in the window
        if (base_sn == next_sn) {
            timer_start = now;
            timer_running = true;
        }
        
        next_sn = (next_sn + 1) % N;  // Circular increment
    }
}
```

**Key point**: Timer starts when we send the FIRST frame in a window. All frames in the window share the same timer.

Example timeline:
```
Time 0: Send SN=0, start timer
Time 5: Send SN=1 (timer still running for SN=0)
Time 10: Send SN=2 (timer still running for SN=0)
Time 15: ACK for SN=0,1 arrives
         - base_sn becomes 2
         - restart timer for SN=2
Time 20: ACK for SN=2 arrives
         - base_sn becomes 0 (all sent frames ACKed)
         - stop timer (no more unACKed frames)
```

---

#### PHASE 2: Handle Incoming ACKs

```cpp
if (readPackage(ack_rx, 10) == 1) {
    int ack_sn = ack_rx[5];   // Next expected SN in the window
    
    // Validate: ACK must be "newer" than base_sn but not beyond next_sn
    if (sn_distance(base_sn, ack_sn) > 0 &&
        sn_distance(base_sn, ack_sn) <= N) {
        
        // Valid ACK - it advances the window
        
        // Measure RTT only when base_sn advances
        unsigned long sample_rtt = millis() - timer_start;
        rtt_samples++;
        
        // Update timeout adaptively (same formula as SAW)
        timeout =
            ((float)(rtt_samples - 1) / (float)rtt_samples) * timeout +
            (1.0f / (float)rtt_samples) * (float)sample_rtt;
        
        Serial.print("ACK received. next expected SN = ");
        Serial.println(ack_sn);
        Serial.print("RTT sample = ");
        Serial.print(sample_rtt);
        Serial.println(" ms");
        
        // Move base forward - all frames from old base_sn to ack_sn are ACKed
        base_sn = ack_sn;
        
        // Restart timer for remaining frames, or stop if all are ACKed
        if (base_sn == next_sn) {
            // No more unACKed frames
            stop_timer_if_needed();
        } else {
            // Still have unACKed frames, restart timer
            timer_start = millis();
            timer_running = true;
        }
    }
}
```

**Cumulative ACK example with N=3:**
```
Sent: SN=0, SN=1, SN=2
base_sn = 0

ACK arrives with SN=2:
- Means: frames 0,1,2 are all ACKed
- Distance from 0 to 2 = 2 (valid, < N)
- Move base_sn to 2
- Now can send new frames with SN=0,1,2 again
```

---

#### PHASE 3: Handle Timeout (Retransmit All Unacked Frames)

```cpp
if (timer_running && (millis() - timer_start > (unsigned long)timeout)) {
    Serial.println("Timeout! Retransmitting window...");
    
    // Retransmit all frames from base_sn to next_sn
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
    
    // Reset timer for the retransmitted window
    timer_start = millis();
    timer_running = true;
}
```

**Important**: On timeout, GBN retransmits **ALL** frames in the window (from `base_sn` to `next_sn`), not just one frame.

---

### GBN TX Flow Diagram

```
┌──────────────────────────────────────┐
│  PHASE 1: Send new frames            │
│  if window not full                  │
│  (distance(base,next) < N)           │
│  Start timer on first frame          │
└────────┬─────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────┐
│  PHASE 2: Receive & process ACKs     │
│  Move base_sn forward                │
│  Restart/stop timer                  │
│  Update timeout adaptively           │
└────────┬─────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────┐
│  PHASE 3: Timeout check              │
│  If timeout → retransmit all frames  │
│  from base_sn to next_sn             │
└──────────────────────────────────────┘
         │
         └──────────┬──────────┘
                    │
             Back to loop() →
              continuous
```

---

## RECEIVER CODE (`lab4.rx_gbn.ino`)

### Global Variables

```cpp
#define N 3                // Window size (same as TX)

uint8_t frame_rx[18];      // Received frame
int expected_frame = 0;    // Next SN we expect to receive
static int error_flag = 0; // Set to 1 if we received an out-of-order frame
static int N_counter = 0;  // Count how many frames in current window

uint8_t ack_frame[10];     // Single ACK sent after receiving N frames
```

**Key concept**: RX sends ONE ACK after receiving N frames (the entire window), not after each frame.

---

### Reception Logic: `RX_func()`

The receiver processes frames in a window and sends a single cumulative ACK after receiving N frames.

```cpp
void RX_func() {
    int flag_receive = readPackage(frame_rx, frame_size);
    
    if (flag_receive == 1) {
        
        // Start of new window
        if (N_counter == 0) {
            Serial.println("**** START WINDOW ****");
        }
        N_counter++;  // Increment window counter
        
        // Parse frame
        int received_sn = frame_rx[5];
        uint8_t length  = frame_rx[3];
        uint8_t data[8] = {0};
        
        for (int i = 0; i < length; ++i) {
            data[i] = frame_rx[6 + i];
        }
        
        // Extract and verify CRC
        unsigned long received_crc = 0;
        received_crc |= (unsigned long)frame_rx[14] << 24;
        received_crc |= (unsigned long)frame_rx[15] << 16;
        received_crc |= (unsigned long)frame_rx[16] << 8;
        received_crc |= (unsigned long)frame_rx[17];
        
        unsigned long calculated_crc = calculateCRC(frame_rx, 14);
```

#### CASE 1: CRC Error

```cpp
if (calculated_crc != received_crc) {
    Serial.println("CRC mismatch");
    bad_frames++;
    error_flag = 1;  // Mark that we had an error
    
    // Prepare ACK for the LAST GOOD frame we received
    if (error_flag == 1) {  // Only do this once per error
        ack_frame[0] = 0x09;
        ack_frame[1] = 0x19;
        ack_frame[2] = 0;
        ack_frame[3] = 0;
        ack_frame[4] = 0;
        ack_frame[5] = expected_frame;  // SN of next GOOD frame needed
        // ... copy CRC ...
    }
    // Don't update expected_frame (still waiting for the correct frame)
}
```

**Strategy**: When a frame fails CRC, set `error_flag=1` and prepare to ACK the last good SN. Don't update `expected_frame`.

---

#### CASE 2: Correct Frame, In Order, No Prior Errors

```cpp
else if (received_sn == expected_frame && error_flag == 0) {
    succ_frame++;
    
    Serial.print("Payload (SN ");
    Serial.print(received_sn);
    Serial.print("): ");
    for (int i = 0; i < length; ++i) {
        Serial.print(char(data[i]));
        Serial.print(" ");
    }
    Serial.println();
    
    // Move to next expected frame
    expected_frame = (expected_frame + 1) % N;
    
    // Build cumulative ACK
    ack_frame[0] = 0x09;
    ack_frame[1] = 0x19;
    ack_frame[2] = 0;
    ack_frame[3] = 0;
    ack_frame[4] = 0;
    ack_frame[5] = expected_frame;  // Cumulative ACK: "I expect SN = ..."
    // ... copy CRC ...
}
```

**Good path**: Process frame, update expected_frame, prepare ACK with new SN.

---

#### CASE 3: Out-of-Order Frame

```cpp
else {
    bad_frames++;
    Serial.print("Out of order frame received. Expected SN: ");
    Serial.print(expected_frame);
    Serial.print(", got: ");
    Serial.println(received_sn);
    
    error_flag = 1;
    
    // Prepare ACK for the last GOOD SN (not this one)
    if (error_flag == 1) {
        ack_frame[0] = 0x09;
        ack_frame[1] = 0x19;
        ack_frame[2] = 0;
        ack_frame[3] = 0;
        ack_frame[4] = 0;
        ack_frame[5] = expected_frame;  // Still waiting for the correct SN
        // ... copy CRC ...
    }
}
```

**Bad path**: Frame out of order, set `error_flag=1`, keep ACKing the last good SN.

---

#### Window Complete: Send Cumulative ACK

```cpp
if (N_counter == N) {  // Received all N frames in window
    Serial.println("Sending ACK frame...");
    
    int send_result = 0;
    while (send_result != 1) {
        send_result = sendPackage(ack_frame, 10);
    }
    
    Serial.print("Sent ACK (next expected SN): ");
    Serial.println(ack_frame[5]);
    
    // Reset for next window
    error_flag = 0;
    N_counter = 0;
    
    // Update statistics
    eff = ((succ_frame * 64.0) / (float(millis()) * 10)) * 1000;
    total_frames++;
    Error_prob = (bad_frames / total_frames);
    
    Serial.print("Efficiency = ");
    Serial.println(eff, 3);
    Serial.print("Error probability = ");
    Serial.println(Error_prob, 3);
}
```

**Key difference from SAW**: ACK is sent **once per window** (after N frames), not after each frame.

---

### GBN RX Flow Diagram

```
Window starts: N_counter = 0

┌─────────────────────────────┐
│  Receive frame              │
│  N_counter++                │
└────────┬────────────────────┘
         │
         ▼
    ┌─────────────────────┐
    │ 3-way decision:     │
    │ 1. CRC error?       │
    │ 2. In order?        │
    │ 3. After error?     │
    └────────┬────────────┘
             │
        ┌────┴────┬────────┬──────────┐
        ▼         ▼        ▼          ▼
    ERROR    IN-ORDER  OUT-OF-ORDER  (others)
    │         │        │             │
    │         │        │             │
    └────┬────┴────┬───┴─────────────┘
         │         │
    Set       Keep
    error     ACK
    _flag     ready
    
         │
         ▼
    N_counter == N?
    (Received all N frames)
         │
    YES  │  NO
    ─────┼─────
         │
         ▼
    SEND CUMULATIVE ACK
    Reset N_counter = 0
    Reset error_flag = 0
```

---

# Comparison: SAW vs GBN

| Feature | Stop-and-Wait | Go-Back-N |
|---------|---------------|-----------|
| **Frames in flight** | 1 | N (up to 3) |
| **Sequence numbers** | 2 (0, 1) | N (0, 1, ..., N-1) |
| **ACK frequency** | After each frame | After N frames |
| **Timeout behavior** | Retransmit 1 frame | Retransmit all N frames |
| **Throughput** | Lower | Higher |
| **Complexity** | Simple | Moderate |
| **Timer management** | One per frame | One per window |
| **Window concept** | N/A | Central to protocol |

---

# Frame Format Details

### Data Frame (18 bytes)
```
+-----+-----+-----+-----+-----+-----+------+------+---+---+---+---+---+---+---+---+
| Dst | Src | Typ | Len | A/D | SN  | Pay0 | Pay1 |...|   | CRC0| CRC1| CRC2| CRC3|
+-----+-----+-----+-----+-----+-----+------+------+---+---+---+---+---+---+---+---+
  0     1     2     3     4     5     6-13              14    15    16    17
```

### ACK Frame (10 bytes)
```
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
| Dst | Src | Typ | Len | A/D | SN  | CRC0| CRC1| CRC2| CRC3|
+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
  0     1     2     3     4     5     6     7     8     9
```

---

# Critical Concepts Summary

## Sequence Numbers (SN)
- **Purpose**: Identify frames for ordering and ACKing
- **SAW**: Alternates 0↔1 (only 2 values needed)
- **GBN**: Cycles 0→1→2→0 (N=3 values)
- **Why modulo-N?**: Allows window to advance correctly

## Cumulative ACKs
- **Meaning**: "I have received all frames up to SN-1, expecting SN next"
- **Example**: ACK with SN=2 means "Got frames 0,1 already, send me 2 next"
- **GBN advantage**: One ACK can acknowledge multiple frames

## Adaptive Timeout
Used in both protocols:
```
timeout = (old_weight * old_timeout) + (new_weight * new_RTT)
```
- Gets longer if network is slow
- Gets shorter if network is fast
- Prevents unnecessary retransmissions

## Error Recovery

**SAW:**
- Single ACK per frame
- Lost ACK → timeout → retransmit that frame
- Lost data frame → timeout → retransmit that frame

**GBN:**
- Single ACK per window
- Lost data frame → sender times out → retransmits entire window
- Lost ACK → RX keeps sending same ACK after each frame

## Window Management (GBN)

```
Frame sent: base_sn........next_sn
            |            |
            oldest        first frame
            unACKed       not sent

Window available = N - distance(base_sn, next_sn)
Window full when distance = N
```

---

# Ethernet Header Structure (from `EthernetLab.h`)

```cpp
frame[0] = 0x19;      // Destination (0x10 + device_id)
                      // 0x09 = RX device 9
                      // 0x19 = TX device 9 (0x10 + 9)

frame[1] = 0x09;      // Source (0x00 + device_id)
                      // 0x09 = TX is device 9
                      // 0x19 = RX is device 9 (when swapped in ACK)

frame[2] = 0;         // Frame type (0=data, other=control)

frame[3] = length;    // Payload length (8 for "ROY%OMRI")

frame[4] = value;     // ACK/DATA field
                      // = length for data frames
                      // = 0 for ACK frames

frame[5] = sn;        // Sequence number (mod N)
```

---

This explanation covers the complete flow, state machines, data structures, and protocol mechanics for both SAW and GBN implementations.
