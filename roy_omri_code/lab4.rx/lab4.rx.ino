#include "EthernetLab.h"
#define frame_size 18
#define MAX_FRAMES 8  
#define LINE_RATE 10
#define BAUD_RATE        115200


uint8_t frame_rx[frame_size];
int expected_frame = 0;        
float total_frames = 0, bad_frames = 0, succ_frame = 0; 
float eff = 0.0;

void setup() {
    Serial.begin(BAUD_RATE);   // תואם ל-setAddress ולמה שצריך במעבדה
    while (!Serial) { ; }   // אופציונלי

    setAddress(RX, 9);

    Serial.println("Starting in mode: RX");
}

void loop() {
    RX_func();
}

void RX_func() {
  static float Error_prob = 0;
  int flag_receive;

  flag_receive = readPackage(frame_rx, frame_size);
  
  if (flag_receive == 1) {  

      uint8_t destination_address = frame_rx[0];
      uint8_t source_address      = frame_rx[1];
      uint8_t frame_type          = frame_rx[2];
      uint8_t length              = frame_rx[3];
      uint8_t ack_data_field      = frame_rx[4];
      int received_sn             = frame_rx[5];

      Serial.println("\n==== Received Frame ====");
      Serial.print("Destination: 0x"); Serial.println(destination_address, HEX);
      Serial.print("Source:      0x"); Serial.println(source_address, HEX);
      Serial.print("Type:        ");   Serial.println(frame_type);
      Serial.print("Length:      ");   Serial.println(length);
      Serial.print("ACK/DATA:    ");   Serial.println(ack_data_field);
      Serial.print("SN:          ");   Serial.println(received_sn);

      uint8_t data[8] = {0};
      for (uint8_t i = 0; i < length; ++i) {
          data[i] = frame_rx[6 + i];
      }
      
      // CRC (4 bytes אחרונים)
      unsigned long received_crc = 0;
      received_crc |= (unsigned long)frame_rx[14] << 24;
      received_crc |= (unsigned long)frame_rx[15] << 16;
      received_crc |= (unsigned long)frame_rx[16] << 8;
      received_crc |= (unsigned long)frame_rx[17];
      
      unsigned long calculated_crc = calculateCRC(frame_rx, 14);

      Serial.print("CRC received:  0x"); Serial.println(received_crc, HEX);
      Serial.print("CRC expected:  0x"); Serial.println(calculated_crc, HEX);
      
      if (calculated_crc != received_crc) {
          Serial.println("CRC mismatch");
          bad_frames++;
      }
      else if (received_sn == expected_frame) {
          expected_frame = 1 - expected_frame;
          
          Serial.print("Payload: ");
          for (uint8_t i = 0; i < length; ++i) {
              Serial.print(char(data[i]));
          }
          Serial.println();  

          // ACK frame
          uint8_t ack_frame[10];  
          ack_frame[0] = 0x09;  // destination_address
          ack_frame[1] = 0x19;  // source_address
          ack_frame[2] = 0;     // frame_type (ACK)
          ack_frame[3] = 0;     // length
          ack_frame[4] = 0;     // ACK/DATA = 0
          ack_frame[5] = 1 - received_sn;  // שולחים SN של הפריים הבא הצפוי

          ack_frame[6] = (calculated_crc >> 24) & 0xFF;
          ack_frame[7] = (calculated_crc >> 16) & 0xFF;
          ack_frame[8] = (calculated_crc >> 8)  & 0xFF;
          ack_frame[9] =  calculated_crc        & 0xFF; 
          
          Serial.println("Sending ACK frame...");

          int send_result = 0;
          while (send_result != 1) {
            send_result = sendPackage(ack_frame, 10);
          }

          Serial.print("Sent ACK (SN field): ");
          Serial.println(ack_frame[5]);
          succ_frame ++;
          
      } else {
          bad_frames++;
          Serial.print("Out of order frame. Expected SN: ");
          Serial.println(expected_frame);
      }

      eff = ((succ_frame * 64.0) / (float(millis()) * LINE_RATE)) * 1000;
      Serial.print("Efficiency = ");
      Serial.println(eff, 3);

      total_frames++; 
      Error_prob = (bad_frames / total_frames);
      Serial.print("Error probability = ");
      Serial.println(Error_prob, 3);
      Serial.println();
  }
}
