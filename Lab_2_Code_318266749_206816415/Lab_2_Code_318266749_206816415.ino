// C++ code
//
#define BAUD_RATE 9600
#define RX_pin 2
#define TX_pin 3
#define BIT_TIME 20
#define SAMP_NUM 3
#define DATA_BITS 8

enum StateType {
  IDLE, START, DATA, PARITY, STOP, ERROR
};

enum Layer2TxState {
  L2_TX_IDLE,
  L2_TX_SEND_LOW,
  L2_TX_WAIT_LOW,
  L2_TX_SEND_HIGH,
  L2_TX_WAIT_HIGH
};

enum Layer2RxState {
  L2_RX_WAIT_LOW,
  L2_RX_WAIT_HIGH
};

// --- Layer 1 Globals ---
int tx_bit_idx, rx_bit, samp_idx, rx_bit_idx; 
int del, i, bit, ones, t_ones, parity, r_parity, majority;
unsigned long punish_time;
unsigned char t_letter;
byte samp_reg;
unsigned char char_reg;
unsigned long curr, curr_rx, start_time, start_punish_time, prev_samp_time;
StateType TxState, RxState;

// --- Layer 1 Interface Flags ---
bool layer_2_req_tx = false;      
byte layer_2_tx_data = 0;         
bool layer_1_tx_busy = false;     
bool layer_1_rx_ready = false;    
byte layer_1_rx_data = 0;         

// --- Layer 2 Globals ---
Layer2TxState l2_tx_state = L2_TX_IDLE;
Layer2RxState l2_rx_state = L2_RX_WAIT_LOW;
char l2_tx_buffer;                // Stores the char being sent
byte l2_rx_temp_nibble;           // Stores the first received nibble

// --- CRC4 Globals ---
byte crc_poly = 0b00010011;

void setup()
{
  Serial.begin(BAUD_RATE);
  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);
  digitalWrite(TX_pin, 1); 

  start_time = millis();
  curr = millis();
  TxState = IDLE;
  RxState = IDLE;
  del = BIT_TIME / (SAMP_NUM + 2);

  tx_bit_idx = 0;
  t_ones = 0;
  ones = 0;
  r_parity = 0;
  majority = 0;
}

// ==========================================
//          HAMMING CODE HELPERS
// ==========================================

// flipping bits function for testing errors
byte flip_bits(byte packet, int n, int b1 = -1, int b2 = -1, int b3 = -1) {
  // n = number of bits to flip
  // b1,b2,b3 = bit positions (0 = LSB ... 7 = MSB)

  if (n >= 1 && b1 >= 0) packet ^= (1 << b1);
  if (n >= 2 && b2 >= 0) packet ^= (1 << b2);
  if (n >= 3 && b3 >= 0) packet ^= (1 << b3);

  return packet;
}

// Hamming (7,4) Encoding
// Input: 4 bits of data (in the lower 4 bits of the byte)
// Output: 7 bits of Hamming code
byte hamming_encode(byte data) {
  // Data bits: d1=bit0, d2=bit1, d3=bit2, d4=bit3
  byte d1 = (data >> 0) & 0x01;
  byte d2 = (data >> 1) & 0x01;
  byte d3 = (data >> 2) & 0x01;
  byte d4 = (data >> 3) & 0x01;

  // Calculate Parity bits
  // p1 covers 1, 3, 5, 7 (d1, d2, d4)
  byte p1 = d1 ^ d2 ^ d4;
  
  // p2 covers 2, 3, 6, 7 (d1, d3, d4)
  byte p2 = d1 ^ d3 ^ d4;
  
  // p4 covers 4, 5, 6, 7 (d2, d3, d4)
  byte p4 = d2 ^ d3 ^ d4;

  // Construct packet: (MSB) 0 d4 d3 d2 p4 d1 p2 p1 (LSB)
  // Bit positions:          7  6  5  4  3  2  1
  byte packet = (d4 << 6) | (d3 << 5) | (d2 << 4) | (p4 << 3) | (d1 << 2) | (p2 << 1) | p1;
  
  // flipping to check errors (pakcet, number of bits we want to flip, index1, index2, index3), default 0, -1, -1, -1
  packet = flip_bits(packet, int 0, int -1, int -1, int -1);

  return packet;
}

// Hamming (7,4) Decoding and Correction
// Input: 7 bit packet
// Output: 4 bits of corrected data
byte hamming_decode(byte packet) {
  // Extract bits
  byte p1 = (packet >> 0) & 0x01;
  byte p2 = (packet >> 1) & 0x01;
  byte d1 = (packet >> 2) & 0x01;
  byte p4 = (packet >> 3) & 0x01;
  byte d2 = (packet >> 4) & 0x01;
  byte d3 = (packet >> 5) & 0x01;
  byte d4 = (packet >> 6) & 0x01;

  // Calculate Syndrome
  byte s1 = p1 ^ d1 ^ d2 ^ d4; // Check pos 1,3,5,7
  byte s2 = p2 ^ d1 ^ d3 ^ d4; // Check pos 2,3,6,7
  byte s4 = p4 ^ d2 ^ d3 ^ d4; // Check pos 4,5,6,7

  byte error_pos = (s4 << 2) | (s2 << 1) | s1;

  if (error_pos != 0) {
    Serial.print("L2: Correction! Error at bit: ");
    Serial.println(error_pos);
    
    // Flip the bit at error_pos (converting 1-based index to 0-based shift)
    // Note: Our packet layout matches the syndrome math directly
    packet ^= (1 << (error_pos - 1));
    
    // Re-extract bits after correction
    d1 = (packet >> 2) & 0x01;
    d2 = (packet >> 4) & 0x01;
    d3 = (packet >> 5) & 0x01;
    d4 = (packet >> 6) & 0x01;
  }

  return (d4 << 3) | (d3 << 2) | (d2 << 1) | d1;
}

// ==========================================
//          CRC4 HELPERS
// ==========================================

byte CRC4_tx() {
  byte input_data = layer_2_tx_data;  // 8-bit character
  byte crc = 0x0;                     // 4-bit CRC register
  byte fb;                            // feedback bit

  // ---- process bit 7 ----
  fb = ((input_data >> 7) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 6 ----
  fb = ((input_data >> 6) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 5 ----
  fb = ((input_data >> 5) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 4 ----
  fb = ((input_data >> 4) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 3 ----
  fb = ((input_data >> 3) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 2 ----
  fb = ((input_data >> 2) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 1 ----
  fb = ((input_data >> 1) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // ---- process bit 0 ----
  fb = ((input_data >> 0) & 1) ^ ((crc >> 3) & 1);
  crc = ((crc << 1) & 0x0F) ^ (fb ? (crc_poly & 0x0F) : 0);

  // keep only 4 bits -> 0000CCCC
  crc &= 0x0F;

  return crc; // equals 0000CCCC
}

void CRC4_rx(byte rx_data, byte rx_crc)
{
  // We temporarily load the received data into the global used by CRC4_tx()
  layer_2_tx_data = rx_data;

  // Recompute the CRC using the *same* function
  byte computed_crc = CRC4_tx();   // only 4-bit result

  // Compare
  bool ok = (computed_crc == (rx_crc & 0x0F));

  // Print result
  Serial.print("RX char: '");
  Serial.print((char)rx_data);
  Serial.print("'  CRC: ");
  Serial.println(ok ? "SUCCESS" : "FAIL");
}

// ==========================================
//          LAYER 2 FUNCTIONS
// ==========================================

void Layer2_TX(char c) {
  // If IDLE, accept new char and start process
  if (l2_tx_state == L2_TX_IDLE) {
    l2_tx_buffer = c;
    l2_tx_state = L2_TX_SEND_LOW;
  }
}

// SEND_LOW/HIGH - makes data for 4 right/left bits [layer_2_tx_data] and switch on request for sending [layer_2_req_tx = true] then state go to waiting to finish [l2_tx_state = L2_TX_WAIT_LOW]
// WAIT_LOW - only waits for [layer_2_req_tx == false && layer_1_tx_busy == false] - happens in layer 1 in STOP, before moving forward in state, and then go to tranfer the high
// WAIT_HIGH - same as above, but change state [l2_tx_state = L2_TX_IDLE] and then prints approvment we finished sending a full letter
void Layer2_TX_Loop() {
  switch (l2_tx_state) {
    case L2_TX_IDLE:
      // Do nothing, waiting for call to Layer2_TX(char)
      break;

    case L2_TX_SEND_LOW:
      if (!layer_1_tx_busy) { // if layer1 tx is available for another send
        byte low_nibble = l2_tx_buffer & 0x0F;
        layer_2_tx_data = hamming_encode(low_nibble);
        layer_2_req_tx = true;
        l2_tx_state = L2_TX_WAIT_LOW;
      }
      break;

    case L2_TX_WAIT_LOW:
      // Wait for L1 to raise busy (ack) then drop busy (done)
      // Simplification: just wait for busy to be false (assuming it went true instantly or we check logic)
      // Better logic: Check if L1 accepted request (req set to false by L1)
      if (layer_2_req_tx == false && layer_1_tx_busy == false) {
         l2_tx_state = L2_TX_SEND_HIGH;
      }
      break;

    case L2_TX_SEND_HIGH:
      if (!layer_1_tx_busy) {
        byte high_nibble = (l2_tx_buffer >> 4) & 0x0F;
        layer_2_tx_data = hamming_encode(high_nibble);
        layer_2_req_tx = true;
        l2_tx_state = L2_TX_WAIT_HIGH;
      }
      break;

    case L2_TX_WAIT_HIGH:
      if (layer_2_req_tx == false && layer_1_tx_busy == false) {
         l2_tx_state = L2_TX_IDLE;
         Serial.println("L2 TX: Sent Full Char");
      }
      break;
  }
}

// works only if [layer_1_rx_ready], if so <<< when is this changed
// in the beginning we take the global info from rx1 [byte raw_packet = layer_1_rx_data] and then say layer1 'was used' [layer_1_rx_ready = false]
// WAIT_LOW is ON => we take the data [l2_rx_temp_nibble = decoded_data] and move to wait high
// WAIT_HIGH => taking the two nibbles into one word [final_char], printing it and going back to state wait low
void Layer2_RX_Loop() {
  if (layer_1_rx_ready) {
    byte raw_packet = layer_1_rx_data;
    layer_1_rx_ready = false; // Acknowledge L1

    byte decoded_data = hamming_decode(raw_packet);

    if (l2_rx_state == L2_RX_WAIT_LOW) {
      l2_rx_temp_nibble = decoded_data; // Store lower nibble
      l2_rx_state = L2_RX_WAIT_HIGH;
    } 
    else if (l2_rx_state == L2_RX_WAIT_HIGH) {
      char final_char = (decoded_data << 4) | l2_rx_temp_nibble;
      Serial.print("L2 RX Final: ");
      Serial.println(final_char);
      l2_rx_state = L2_RX_WAIT_LOW; // Reset for next char
    }
  }
}

// ==========================================
//          LAYER 1 (PHYSICAL)
// ==========================================

// In the beginning: we insert a hamm'ed-half-letter [t_letter = layer_2_tx_data]
// layer 1 busy now [layer_1_tx_busy = true] and we stop asking new info from layer 2 [layer_2_req_tx = false] <<< when to switch this
//In the end: when we get to stop we say we are not busy in LAYER 1 [layer_1_tx_busy = false]
void Uart_TX() {
  curr = millis();
  if (TxState == IDLE) {
    if (layer_2_req_tx == true) {
      layer_1_tx_busy = true;
      t_letter = layer_2_tx_data; 
      layer_2_req_tx = false; 
      start_time = millis();
      TxState = START;
      tx_bit_idx = 0;
      digitalWrite(TX_pin, 0); 
    }
  }
  else {
    if ((curr - start_time) >= BIT_TIME) {
      start_time = curr;
      if (TxState == START) {
        TxState = DATA;
        bit = bitRead(t_letter, tx_bit_idx);
        digitalWrite(TX_pin, bit);
      }
      else if (TxState == DATA) {
        tx_bit_idx++;
        if (tx_bit_idx < DATA_BITS) {
          bit = bitRead(t_letter, tx_bit_idx);
          digitalWrite(TX_pin, bit);
        }
        else {
          TxState = PARITY;
          ones = 0;
          for (i = 0; i < DATA_BITS; i++) {
            if (bitRead(t_letter, i)) ones++;
          }
          parity = (ones % 2 == 0) ? 1 : 0; 
          digitalWrite(TX_pin, parity);
        }
      }
      else if (TxState == PARITY) {
        TxState = STOP;
        digitalWrite(TX_pin, 1); 
      }
      else if (TxState == STOP) {
        TxState = IDLE;
        digitalWrite(TX_pin, 1);
        layer_1_tx_busy = false; 
      }
    }
  }
}

// we collect the bits to a data [layer_1_rx_data]
// when at STOP we finished and we say layer1 is ready to give layer 2 the info [layer_1_rx_ready = true]
void Uart_RX() {
  curr_rx = millis();
  rx_bit = digitalRead(RX_pin);

  if ((RxState == IDLE) && (rx_bit == 0)) { 
    RxState = START;
    prev_samp_time = curr_rx;
    samp_idx = 0;
  }
  else { 
    if ((RxState != IDLE) && (RxState != ERROR) && (curr_rx - prev_samp_time >= del)) {
      bitWrite(samp_reg, samp_idx, digitalRead(RX_pin)); 
      prev_samp_time = curr_rx;
      samp_idx++;

      if (samp_idx == SAMP_NUM + 2) { 
        if (RxState == START) { 
          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          int start_bit = (mid_sum >= 2) ? 1 : 0;
          if (start_bit == 1) { 
            RxState = ERROR;
            start_punish_time = curr_rx;
            punish_time = BIT_TIME * (1 + DATA_BITS + 1 + 1);
          }
          else { 
            RxState = DATA;
            rx_bit_idx = 0;
            t_ones = 0;
          }
          samp_idx = 0;
        }
        else if (RxState == DATA) { 
          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          majority = (mid_sum >= 2) ? 1 : 0;
          if (rx_bit_idx < DATA_BITS) { 
            bitWrite(char_reg, rx_bit_idx, majority); 
            if (majority == 1) t_ones++;
            rx_bit_idx++; 
          }
          else { 
            r_parity = majority;
            if ((t_ones % 2) != r_parity) RxState = STOP;
            else { 
              RxState = ERROR;
              start_punish_time = curr;
              punish_time = BIT_TIME * (DATA_BITS + 3); 
            }
          }
          samp_idx = 0;
        }
        else if (RxState == STOP) { 
          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          majority = (mid_sum >= 2) ? 1 : 0;
          samp_idx = 0;
          if (majority == 1) { 
            layer_1_rx_data = char_reg; 
            layer_1_rx_ready = true;    
            RxState = IDLE; 
          }
          else { 
            RxState = ERROR;
            start_punish_time = curr_rx;
            punish_time = BIT_TIME * (DATA_BITS + 3);
          }
        }
      }
    }  
  }    
  if ((RxState == ERROR) && (curr_rx - start_punish_time >= punish_time)) {
    RxState = IDLE;
  }
}

// ==========================================
//               MAIN LOOP
// ==========================================

char data2send[] = "Raziel & Elad";
int msg_idx = 0;

void loop()
{
  // 1. Physical Layer
  Uart_TX();
  Uart_RX();

  // 2. Data Link Layer
  Layer2_TX_Loop();
  Layer2_RX_Loop();

  // 3. Application Layer (send next char immediately when possible)
  if (l2_tx_state == L2_TX_IDLE) {
    char ch = data2send[msg_idx];

<<<<<<< HEAD
    Serial.print("App Sending: ");
    Serial.println(ch);

    Layer2_TX(ch);     // Give to Layer 2

    msg_idx++;
    if (msg_idx >= strlen(data2send)) {
      msg_idx = 0;     // restart the message
    }
  }
}
=======
      msg_idx++;
      if (msg_idx >= strlen(msg)) msg_idx = 0; // if finished with message, start over
      last_app_action = millis();
    }
  }
  // end layer 3 simulation
}
>>>>>>> 38b9e5bef20e7b004668ad9798808bb51b69b60f
