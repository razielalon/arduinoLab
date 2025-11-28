// C++ code
//
#define BAUD_RATE 9600
#define RX_pin 2
#define TX_pin 3
#define BIT_TIME 30
#define SAMP_NUM 3
#define DATA_BITS 8

// making errors on purpose
#define NUM_ERR_BITS  0
#define ERR_BIT_1     -1
#define ERR_BIT_2     -1
#define ERR_BIT_3     -1

char data2send[] = "Elad\n";

enum StateType {
  IDLE,
  START,
  DATA,
  PARITY,
  STOP,
  ERROR
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

enum L2_Check_Mode {
  CHECK_IDLE,
  CHECK_HAMMING,
  CHECK_CRC
};

// Layer 1 global variables
int tx_bit_idx, tx_letter_idx, rx_bit, samp_idx, rx_bit_idx; 
int del, i, bit, ones, t_ones, parity, r_parity, majority;
int delay1;
unsigned long punish_time;
unsigned char t_letter;
byte samp_reg;
unsigned char char_reg;
unsigned long curr, curr_rx, start_time, delay_time, start_punish_time, prev_samp_time;
StateType TxState, RxState;

// ========================================
// ====== Layer 2 states & variables ======
// ========================================
Layer2TxState L2TxState;
Layer2RxState L2RxState;
L2_Check_Mode L2Mode = CHECK_CRC;

int l2_data_idx = 0;

byte tx_code_low, tx_code_high;
byte rx_code_low, rx_code_high;

// =========================================================
// ====== Layer 1 / Layer 2 handshake flags & buffers ======
// =========================================================
volatile bool layer_1_tx_busy    = false;
volatile bool layer_1_rx_busy    = false;
volatile bool layer_1_rx_got_char = false;
volatile bool layer_2_tx_request = false;

volatile byte l1_tx_buffer = 0;
volatile byte l1_rx_buffer = 0;

// --- CRC4 Globals ---
byte crc_poly = 0b00010011;
byte crc_rx_data = 0;
byte crc_rx_crc  = 0;

void setup()
{
  Serial.begin(BAUD_RATE);

  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);

  start_time = millis();
  curr = millis();

  TxState = IDLE;
  RxState = IDLE;
  L2TxState = L2_TX_IDLE;
  L2RxState = L2_RX_WAIT_LOW;
  L2Mode    = CHECK_CRC;

  randomSeed(analogRead(A0));

  del = BIT_TIME / (SAMP_NUM + 2);
  delay1 = 0;

  tx_letter_idx = 0;
  tx_bit_idx = 0;

  t_ones = 0;
  ones = 0;

  r_parity = 0;
  majority = 0;

}

// ==========================================
//             HELPING HELPERS
// ==========================================
byte flip_bits(byte packet, int n, int b1 = -1, int b2 = -1, int b3 = -1) {
  // n = number of bits to flip
  // b1,b2,b3 = bit positions (0 = LSB ... 7 = MSB)

  if (n >= 1 && b1 >= 0) packet ^= (1 << b1);
  if (n >= 2 && b2 >= 0) packet ^= (1 << b2);
  if (n >= 3 && b3 >= 0) packet ^= (1 << b3);

  return packet;
}

// ==========================================
//          HAMMING CODE HELPERS
// ==========================================
byte hamming_decode(byte packet) {
  Serial.print("hamming_decode IN: ");
  for (int i = 6; i >= 0; --i) Serial.print((packet >> i) & 1);
  Serial.println();

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
    Serial.print("  Correction! Error at bit: ");
    Serial.println(error_pos);

    // Flip the bit at error_pos (1-based index)
    packet ^= (1 << (error_pos - 1));

    Serial.print("  Corrected code: ");
    for (int i = 6; i >= 0; --i) Serial.print((packet >> i) & 1);
    Serial.println();

    // Re-extract bits after correction
    p1 = (packet >> 0) & 0x01;
    p2 = (packet >> 1) & 0x01;
    d1 = (packet >> 2) & 0x01;
    p4 = (packet >> 3) & 0x01;
    d2 = (packet >> 4) & 0x01;
    d3 = (packet >> 5) & 0x01;
    d4 = (packet >> 6) & 0x01;
  }

  byte nibble = (d4 << 3) | (d3 << 2) | (d2 << 1) | d1;

  Serial.print("  OUT nibble = ");
  for (int i = 3; i >= 0; --i) Serial.print((nibble >> i) & 1);
  Serial.println();

  return nibble;
}

// קידוד Hamming (7,4,3)
// nibble = 4 ביט (d4 d3 d2 d1) = ביטים 3..0
// קידוד למבנה: [p1 p2 d1 p4 d2 d3 d4] = ביטים 0..6
byte hamming_encode(byte nibble) {
  byte d1 = (nibble >> 0) & 0x01;
  byte d2 = (nibble >> 1) & 0x01;
  byte d3 = (nibble >> 2) & 0x01;
  byte d4 = (nibble >> 3) & 0x01;

  byte p1 = d1 ^ d2 ^ d4;   // מכסה עמדות 1,3,5,7
  byte p2 = d1 ^ d3 ^ d4;   // מכסה עמדות 2,3,6,7
  byte p4 = d2 ^ d3 ^ d4;   // מכסה עמדות 4,5,6,7

  byte code = 0;
  code |= (p1 << 0);  // bit 0
  code |= (p2 << 1);  // bit 1
  code |= (d1 << 2);  // bit 2
  code |= (p4 << 3);  // bit 3
  code |= (d2 << 4);  // bit 4
  code |= (d3 << 5);  // bit 5
  code |= (d4 << 6);  // bit 6

  return code; // 7 ביט בשימוש (0..6)
}

void Hamming47_tx(char c) {
  byte low  = c & 0x0F;
  byte high = (c >> 4) & 0x0F;

  byte code_low  = hamming_encode(low);
  byte code_high = hamming_encode(high);

  Serial.println("===== Hamming47_tx =====");
  Serial.print("Original char: '");
  Serial.print(c);
  Serial.println("'");

  // 1) הדפסת המילים לפני הכנסת טעויות
  Serial.print("TX low  (orig): ");
  for (int i = 6; i >= 0; --i) Serial.print((code_low >> i) & 1);
  Serial.println();

  Serial.print("TX high (orig): ");
  for (int i = 6; i >= 0; --i) Serial.print((code_high >> i) & 1);
  Serial.println();

  // 2) הכנסת טעויות לפי NUM_ERR_BITS ו-ERR_BIT_1..3
  byte noisy_low  = flip_bits(code_low,  NUM_ERR_BITS, ERR_BIT_1, ERR_BIT_2, ERR_BIT_3);
  byte noisy_high = flip_bits(code_high, NUM_ERR_BITS, ERR_BIT_1, ERR_BIT_2, ERR_BIT_3);

  Serial.print("TX low  (noisy): ");
  for (int i = 6; i >= 0; --i) Serial.print((noisy_low >> i) & 1);
  Serial.println();

  Serial.print("TX high (noisy): ");
  for (int i = 6; i >= 0; --i) Serial.print((noisy_high >> i) & 1);
  Serial.println();

  // בשלב הזה, ב"עולם האמיתי", היית שולח noisy_low ואז noisy_high דרך שכבה 1.
  // כרגע, בשביל בדיקות לוגיות, נקרא ישירות ל-RX:
  Hamming47_rx(noisy_low, noisy_high);
}

void Hamming47_rx(byte rx_code_low, byte rx_code_high) {
  Serial.println("===== Hamming47_rx =====");

  // הדפסה של מה שקיבלנו (7 ביט כל אחד)
  Serial.print("RX low  (raw): ");
  for (int i = 6; i >= 0; --i) Serial.print((rx_code_low >> i) & 1);
  Serial.println();

  Serial.print("RX high (raw): ");
  for (int i = 6; i >= 0; --i) Serial.print((rx_code_high >> i) & 1);
  Serial.println();

  // שימוש ב-hamming_decode לכל ניבל
  Serial.println("Decoding low nibble:");
  byte low_nibble  = hamming_decode(rx_code_low);

  Serial.println("Decoding high nibble:");
  byte high_nibble = hamming_decode(rx_code_high);

  // הרכבת האות מחדש
  char reconstructed = (char)((high_nibble << 4) | (low_nibble & 0x0F));

  Serial.print("Reconstructed char: '");
  Serial.print(reconstructed);
  Serial.println("'");
  Serial.println();
}

// ==========================================
//          CRC4 HELPERS
// ==========================================
byte CRC4_compute(byte input_data) {
  byte crc = 0x0;  // 4-bit CRC register
  byte fb;         // feedback bit

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

  return crc & 0x0F;  // 4-bit CRC
}

byte CRC4_tx(char c) {
  byte data = (byte)c;
  byte crc  = CRC4_compute(data);

  // Print the 12-bit frame (data + crc) as the assignment asks
  uint16_t frame = (((uint16_t)data) << 4) | crc;
  Serial.print("CRC4 TX frame for '");
  Serial.print(c);
  Serial.print("': ");
  for (int i = 11; i >= 0; --i) {
    Serial.print((frame >> i) & 1);
  }
  Serial.println();

  return crc; // lower 4 bits are the CRC
}

void CRC4_rx(byte rx_data, byte rx_crc) {
  byte computed_crc = CRC4_compute(rx_data);

  bool ok = (computed_crc == (rx_crc & 0x0F));

  Serial.print("RX char: '");
  Serial.print((char)rx_data);
  Serial.print("'  CRC: ");
  Serial.println(ok ? "SUCCESS" : "FAIL");
}

// ==========================================
//          LAYER 2 FUNCTIONS
// ==========================================
void layer2_tx() {
  // static so it will be saved between loops
  static unsigned long next_char_time = 0; 
  static bool prev_l1_busy = false;

  unsigned long now = millis();
  bool curr_l1_busy = layer_1_tx_busy; // in each iteration, get the current busy status

  // detect falling edge of layer 1 busy
  bool l1_busy_falling_edge = (prev_l1_busy && !curr_l1_busy); // was busy last iteration, now not busy
  prev_l1_busy = curr_l1_busy; // afeter checking and setting l1_busy_falling_edge, update prev_l1_busy for next iteration

  // ---- Hamming state ----
  if (L2Mode == CHECK_HAMMING) {

    switch (L2TxState) {

      case L2_TX_IDLE:
        if (now < next_char_time) return; // not time yet to send next char (some random delay between chars)

        if (!layer_1_tx_busy && !layer_2_tx_request) { // layer 1 is free to send new char, and no pending request from layer 2
          char c = data2send[l2_data_idx]; // load next char to send
          if (c == '\0') {       // if last char reached, restart from beginning
            l2_data_idx = 0;
            c = data2send[0];
          }

          byte low  = c & 0x0F; // extract low nibble
          byte high = (c >> 4) & 0x0F; // extract high nibble

          // encode both nibbles using Hamming (7,4)
          tx_code_low  = hamming_encode(low);
          tx_code_high = hamming_encode(high);

          // inject errors according to NUM_ERR_BITS and ERR_BIT_1..3
          tx_code_low  = flip_bits(tx_code_low,  NUM_ERR_BITS, ERR_BIT_1, ERR_BIT_2, ERR_BIT_3);
          tx_code_high = flip_bits(tx_code_high, NUM_ERR_BITS, ERR_BIT_1, ERR_BIT_2, ERR_BIT_3);

          // start sending low nibble first
          l1_tx_buffer       = tx_code_low; // put data in layer 1 tx buffer
          layer_2_tx_request = true; // request layer 1 to send

          L2TxState = L2_TX_WAIT_LOW; // state L2_TX_IDLE -> L2_TX_WAIT_LOW
        }
        break;

      case L2_TX_WAIT_LOW:
        if (l1_busy_falling_edge) { // layer 1 finished sending low nibble in this iteration exactly
          if (!layer_1_tx_busy && !layer_2_tx_request) { // layer 1 is free to send new char, and no pending request from layer 2
            l1_tx_buffer       = tx_code_high; // put high nibble data in layer 1 tx buffer
            layer_2_tx_request = true; // request layer 1 to send
            L2TxState          = L2_TX_WAIT_HIGH; // state L2_TX_WAIT_LOW -> L2_TX_WAIT_HIGH
          }
        }
        break;

      case L2_TX_WAIT_HIGH:
        if (l1_busy_falling_edge) { // layer 1 finished sending high nibble in this iteration exactly
          l2_data_idx++; // move to next char to send
          if (data2send[l2_data_idx] == '\0') { // if last char reached, restart from beginning
            l2_data_idx = 0;
          }
          next_char_time = now + random(500, 2000); // set random delay before sending next char
          L2TxState      = L2_TX_IDLE; // state L2_TX_WAIT_HIGH -> L2_TX_IDLE
        }
        break;
    }
    return;  // finished Hamming branch
  }

  // ---- CRC-4 state ----
  if (L2Mode == CHECK_CRC) {
    // Small FSM: send data first, then crc
    static bool sending_crc = false;
    static char current_char = 0;
    static byte current_crc  = 0;

    // If just finished sending previous byte
    if (l1_busy_falling_edge) {
      if (sending_crc) {
        // Finished crc -> move to next char with delay
        sending_crc   = false;
        l2_data_idx++;
        if (data2send[l2_data_idx] == '\0') {
          l2_data_idx = 0;
        }
        next_char_time = now + random(500, 2000);
      }
    }

    // If not time yet to send next char
    if (now < next_char_time) {
      return;
    }

    // Layer 1 is free to send new char?
    if (!layer_1_tx_busy && !layer_2_tx_request) {

      if (!sending_crc) {
        // Step 1: Send the data byte
        current_char = data2send[l2_data_idx];
        if (current_char == '\0') {
          l2_data_idx = 0;
          current_char = data2send[0];
        }

        current_crc = CRC4_compute((byte)current_char);

        // אפשר (לא חובה) להדפיס frame כמו ב-CRC4_tx:
        Serial.print("CRC4 TX frame for '");
        Serial.print(current_char);
        Serial.print("': ");
        uint16_t frame = (((uint16_t)current_char) << 4) | (current_crc & 0x0F);
        for (int i = 11; i >= 0; --i) {
          Serial.print((frame >> i) & 1);
        }
        Serial.println();

        // כאן אפשר להזריק שגיאות על ה-data בלבד (לבינתיים):
        byte noisy_data = flip_bits((byte)current_char,
                                    NUM_ERR_BITS, ERR_BIT_1, ERR_BIT_2, ERR_BIT_3);

        l1_tx_buffer       = noisy_data;
        layer_2_tx_request = true;
        sending_crc        = true;    // הבא שנשלח יהיה CRC
      }
      else {
        // שלב 2: שולחים את ה-CRC עצמו
        byte noisy_crc = current_crc; // אם תרצה, תוסיף גם כאן flip_bits נפרד
        l1_tx_buffer       = noisy_crc;
        layer_2_tx_request = true;
        // sending_crc ייהפך ל-false אחרי נפילת busy למעלה
      }
    }
  }
}

void layer2_rx() {
  static bool prev_l1_rx_busy = false;

  bool curr_l1_rx_busy = layer_1_rx_busy;
  bool falling_edge    = (prev_l1_rx_busy && !curr_l1_rx_busy);
  prev_l1_rx_busy      = curr_l1_rx_busy;

  if (!falling_edge) return;

  if (!layer_1_rx_got_char) {
    // מעבר מ-ERROR ל-IDLE, לא תו אמיתי
    return;
  }

  layer_1_rx_got_char = false;
  byte code = l1_rx_buffer;

  // ---- Hamming ----
  if (L2Mode == CHECK_HAMMING) {
    switch (L2RxState) {
      case L2_RX_WAIT_LOW:
        rx_code_low = code;
        L2RxState   = L2_RX_WAIT_HIGH;
        break;

      case L2_RX_WAIT_HIGH:
        rx_code_high = code;
        Hamming47_rx(rx_code_low, rx_code_high);
        L2RxState = L2_RX_WAIT_LOW;
        break;
    }
    return;
  }

  // ---- CRC-4 ----
  if (L2Mode == CHECK_CRC) {
    static bool waiting_crc = false;
    static byte rx_data_tmp = 0;

    if (!waiting_crc) {
      // זה ה-data byte
      rx_data_tmp  = code;
      waiting_crc  = true;
    } else {
      // זה ה-CRC byte
      byte rx_crc = code;
      CRC4_rx(rx_data_tmp, rx_crc);
      waiting_crc = false;
    }
  }
}

// ==========================================
//          LAYER 1 (PHYSICAL)
// ==========================================
void layer1_tx()
{
  curr = millis();

  if (TxState == IDLE) {
    if (layer_2_tx_request) {
      if (!layer_1_tx_busy){
        layer_1_tx_busy    = true;
        layer_2_tx_request = false;

        t_letter   = l1_tx_buffer;
        tx_bit_idx = 0;

        start_time = curr;
        TxState    = START;
        digitalWrite(TX_pin, 0);        // start bit
      }
    } else {
      return;
    }
  }

  // הגענו לכאן אם כבר התחלנו שידור (START / DATA / PARITY / STOP)
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
          if (bitRead(t_letter, i)) {
            ones++;
          }
        }

        parity = (ones % 2 == 0) ? 1 : 0;
        digitalWrite(TX_pin, parity);
      }
    }

    else if (TxState == PARITY) {
      TxState = STOP;
      digitalWrite(TX_pin, 1);  // stop bit
    }

    else if (TxState == STOP) {
      TxState = IDLE;
      digitalWrite(TX_pin, 1);  // קו חופשי

      layer_1_tx_busy = false;  // לשכבה 2 מותר לבקש תו חדש
    }
  }
}

void layer1_rx()
{
  curr_rx = millis();
  rx_bit = digitalRead(RX_pin);

  if (RxState == IDLE) { // got 0, enter START state
    if (rx_bit == 0){
      RxState = START;
      layer_1_rx_busy = true;
      layer_1_rx_got_char = false;
      prev_samp_time = curr_rx;
      samp_idx = 0;
    }
  }

  else { // not on IDLE, need to sample

    if ((RxState != IDLE) && (RxState != ERROR) && (curr_rx - prev_samp_time >= del)) {
		
      bitWrite(samp_reg, samp_idx, digitalRead(RX_pin)); // write to the samp register(holds 5 samples)

      prev_samp_time = curr_rx;
      samp_idx++;

      if (samp_idx == SAMP_NUM + 2) { // if finished 5 samples

        if (RxState == START) { // handling START state samples (reading 0 bit)
			
          //calculate majority (hold on start_bit var):
          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          int start_bit = (mid_sum >= 2) ? 1 : 0;

          if (start_bit == 1) { // start bit is expected to be 0!
            Serial.println("RX: ERROR - bad START bit");
            RxState = ERROR;
            start_punish_time = curr_rx;
            punish_time = BIT_TIME * (1 + DATA_BITS + 1 + 1);
          }
          else { // good start bit (read 0)
            RxState = DATA;
            rx_bit_idx = 0;
            t_ones = 0;
          }

          samp_idx = 0;
        }

        
        else if (RxState == DATA) { // handling DATA state sampling (reading a letter -one bit at a time)

          //calculating majority for each bit in idx
          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          majority = (mid_sum >= 2) ? 1 : 0;

          if (rx_bit_idx < DATA_BITS) { // didnt got to the final bit in a letter
            bitWrite(char_reg, rx_bit_idx, majority); //writing the majority chosen bit to the corresponding index on the char

            if (majority == 1) { // adding 1 to t_ones in order to calculate parity
              t_ones++;
            }
            rx_bit_idx++; 
          }

          else { // ended cahracter, getting parity
            r_parity = majority;
            if ((t_ones % 2) != r_parity) { // if parity good
              RxState = STOP;
            }
            else { // if parity bad
              Serial.println("RX: ERROR - bad parity");
              RxState = ERROR;
              start_punish_time = curr;
              punish_time = BIT_TIME * (DATA_BITS + 3); // time of (char + start + stop + parity) bits
            }
          }

          samp_idx = 0;
        }

    
        else if (RxState == STOP) { // handling STOP state

          int mid_sum = bitRead(samp_reg,1) + bitRead(samp_reg,2) + bitRead(samp_reg,3);
          majority = (mid_sum >= 2) ? 1 : 0;

          samp_idx = 0;
          
          if (majority == 1) { // stop bit is 1     
            l1_rx_buffer = char_reg;

            RxState = IDLE;
            layer_1_rx_busy = false;
            layer_1_rx_got_char = true;
          }
          else { // stop bit isnt 1
            Serial.println("RX: ERROR - bad STOP bit");
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
    layer_1_rx_busy = false;
    layer_1_rx_got_char = false;
    Serial.println("RX: leaving ERROR, back to IDLE");
  }
}

void loop()
{
  samp_idx = 0;
  samp_reg = 0;
  layer2_tx();
  layer2_rx();
  layer1_tx();
  layer1_rx();
}