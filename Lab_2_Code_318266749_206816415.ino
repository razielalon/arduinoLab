// C++ code
//
#define BAUD_RATE 9600
#define RX_pin 2
#define TX_pin 3
#define BIT_TIME 10
#define SAMP_NUM 3
#define DATA_BITS 8

enum StateType {
  IDLE,
  START,
  DATA,
  PARITY,
  STOP,
  ERROR
};

enum l2_tx_state {
  IDLE,
  SEND_MSB,
  SEND_LSB
};

int tx_bit_idx, tx_letter_idx, rx_bit, samp_idx, rx_bit_idx;
int del, i, bit, ones, t_ones, parity, r_parity, majority;
int delay1;
unsigned long punish_time;
unsigned char t_letter;
unsigned char l2_output;
bool l1_tx_busy, l2_tx_busy;
char name[] = "ELAD\n";
byte samp_reg;
unsigned char char_reg;
unsigned long curr, curr_rx, start_time, delay_time, start_punish_time, prev_samp_time;
StateType TxState, RxState;
l2_tx_state l2_tx;

void setup()
{
  Serial.begin(BAUD_RATE);

  pinMode(TX_pin, OUTPUT);
  pinMode(RX_pin, INPUT);

  start_time = millis();
  curr = millis();

  TxState = IDLE;
  RxState = IDLE;
  l2_tx = IDLE;

  del = BIT_TIME / (SAMP_NUM + 2);
  delay1 = 0;

  tx_letter_idx = 0;
  tx_bit_idx = 0;

  t_ones = 0;
  ones = 0;

  r_parity = 0;
  majority = 0;
  Serial.println("sending");
  char a = Hamming47_tx('a', 0);
  Serial.println("reciving");
  a = 0b01110110;
  Hamming47_rx(a);
}

void layer1_tx()
{
  curr = millis();

  if ((delay1 == 1) && (curr - start_time >= delay_time)) {
    delay1 = 0;
  }

  if (delay1 == 0) {

    if (TxState == IDLE) {
      start_time = millis();
      TxState = START;

      tx_bit_idx = 0;
      digitalWrite(TX_pin, 0);
    }

    if ((curr - start_time) >= BIT_TIME) {
      start_time = curr;

      if (TxState == START) {
        TxState = DATA;
        //
        //t_letter = name[tx_letter_idx];
        //

        // if can send
        t_letter = l2_output;
        /// l1_tx_busy = 1;

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
        digitalWrite(TX_pin, 1);

      }

      else if (TxState == STOP) {

        TxState = IDLE;
        digitalWrite(TX_pin, 1);

        delay1 = 1;
        start_time = curr;
        //delay_time = random(500, 2000);
        /// l1_tx_busy = 0;
        /// l2_tx_busy = 0;

        tx_letter_idx++;
        if (tx_letter_idx >= strlen(name)) {

          tx_letter_idx = 0;
        }
      }
    }
  }
}

void layer1_rx()
{
  curr_rx = millis();
  rx_bit = digitalRead(RX_pin);

  if ((RxState == IDLE) && (rx_bit == 0)) { // got 0, enter START state


    RxState = START;
    prev_samp_time = curr_rx;
    samp_idx = 0;
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

            Serial.print((char)char_reg); // print the recived char

            RxState = IDLE;
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
    Serial.println("RX: leaving ERROR, back to IDLE");
  }
}

void layer2_tx(unsigned char letter){
  switch (l2_tx){
    case IDLE :
      char msb = Hamming47_tx(letter, 0);
      char lsb = Hamming47_tx(letter, 1);
      if(Tx_State == IDLE){
        l2_output = msb;
        l2_tx = SEND_MSB;
      }
      break;
    case SEND_MSB:
      if(Tx_State == IDLE){
        l2_output = lsb;
        l2_tx = SEND_LSB;
      }
      break;
    case SEND_LSB:
      if(Tx_State == IDLE){
        l2_output = lsb;
        l2_tx = SEND_LSB;
      }
      break;
  }
  char msb = Hamming47_tx(letter, 0);
  char lsb = Hamming47_tx(letter, 1);

  // check if can send

  // if yes
  if((l1_tx_busy == 0) && (l2_tx_busy == 0)){
    l2_output = msb;
    l2_tx_busy = 1;
  }

  // if sent
  if((l1_tx_busy == 0) && (l2_tx_busy == 0)){
    l2_output = lsb;
    l2_tx_busy = 1;
  }
}

void layer2_rx(){
  // getting the first 8 bit

}

void printByteBits(unsigned char b) {
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(b, i));
  }
}

unsigned char Hamming47_tx(unsigned char eight_bit_char, unsigned char is_lsb){
  Serial.print("Input:  ");
  printByteBits(eight_bit_char);
  Serial.println();

  unsigned char half_word, output;
  unsigned char d1, d2, d3, d4;
  unsigned char p1, p2, p3;

  if (is_lsb) {
    half_word = eight_bit_char & 0x0F;
  } else {
    half_word = (eight_bit_char >> 4) & 0x0F;
  }
  Serial.print("half_word:  ");
  printByteBits(half_word);
  Serial.println();


  d1 = bitRead(half_word, 3);
  d2 = bitRead(half_word, 2);
  d3 = bitRead(half_word, 1);
  d4 = bitRead(half_word, 0);

  p1 = d1 ^ d2 ^ d4;
  p2 = d1 ^ d3 ^ d4;
  p3 = d2 ^ d3 ^ d4;

  output = 0;
  bitWrite(output, 6, p1);
  bitWrite(output, 5, p2);
  bitWrite(output, 4, d1);
  bitWrite(output, 3, p3);
  bitWrite(output, 2, d2);
  bitWrite(output, 1, d3);
  bitWrite(output, 0, d4);
  bitWrite(output, 7, 0);

  Serial.print("Output: ");
  printByteBits(output);
  Serial.println();

  return output;
}

void Hamming47_rx(unsigned char eight_bit_char_rx){
  Serial.println("recived char: ");
  printByteBits(eight_bit_char_rx);
  unsigned char output;
  unsigned char d1, d2, d3, d4;
  unsigned char p1, p2, p3;
  unsigned char s1, s2, s3;

  d1 = bitRead(eight_bit_char_rx, 2);
  d2 = bitRead(eight_bit_char_rx, 4);
  d3 = bitRead(eight_bit_char_rx, 5);
  d4 = bitRead(eight_bit_char_rx, 6);

  p1 = bitRead(eight_bit_char_rx, 0);
  p2 = bitRead(eight_bit_char_rx, 1);
  p3 = bitRead(eight_bit_char_rx, 3);

  s1 = p1 ^ d1 ^ d2 ^ d4;
  s2 = p2 ^ d1 ^ d3 ^ d4;
  s3 = p3 ^ d2 ^ d3 ^ d4;

  unsigned char syndrome = (s3 << 2) | (s2 << 1) | s1;

  Serial.print("Syndrome = ");
  Serial.println(syndrome);

  switch (syndrome) {
    case 0: Serial.println("No error"); break;
    case 1: Serial.println("Error in bit 1"); break;
    case 2: Serial.println("Error in bit 2"); break;
    case 3: Serial.println("Error in bit 3"); break;
    case 4: Serial.println("Error in bit 4"); break;
    case 5: Serial.println("Error in bit 5"); break;
    case 6: Serial.println("Error in bit 6"); break;
    case 7: Serial.println("Error in bit 7"); break;
  }

  if (syndrome != 0) {
    unsigned char bit_index = syndrome - 1;
    eight_bit_char_rx ^= (1 << bit_index);
  }

  Serial.println("fixed char:");
  printByteBits(eight_bit_char_rx);
}

void loop(){

  //layer2_tx(); // Calls Hamming47_tx() or CRC4_tx()
  //layer2_rx(); // Calls the Rx version of the above
  //layer1_tx(); // Either uart_tx() or usart_tx() from Labs 1/2
  //layer1_rx(); // Rx version of the above
}
