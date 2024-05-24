#include <stdint.h>
#include <SPI.h>
#include <CRC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/Org_01.h>
#include <MD_KeySwitch.h>
#include <EEPROM.h>
#include <RS-FEC.h>
#include <Ticker.h>
#include "sx126x.h"
#include "sx126x_regs.h"
#include "sx126x_hal.h"
#include "sx126x_long_pkt.h"

#undef LED_BUILTIN
#define LED_BUILTIN 2

//DISP_DIN=23, DISP_CLK=18
const int RADIO_SCLK_PIN = 18, RADIO_MISO_PIN = 19, RADIO_MOSI_PIN = 23, RADIO_NSS_PIN = 5, RADIO_BUSY_PIN = 4, RADIO_RST_PIN = 15, RADIO_DIO1_PIN = 17,
          LED = 32, BUZZER = 25, DISP_DC = 3, DISP_RST = 22, DISP_CS = 21, BUTTON_SEL = 27, BUTTON_UP = 16,
          PACKET_LENGTH = 312, WIDTH = 84, SERIAL_LENGTH = 8;

SPISettings spiSettings = SPISettings(2E6L, MSBFIRST, SPI_MODE0);
struct sx126x_long_pkt_rx_state pktRxState;
uint32_t freq = 403000;
uint8_t buf[PACKET_LENGTH];
RS::ReedSolomon<99 + (PACKET_LENGTH - 48) / 2, 24> rs;
int nBytesRead = 0;
Adafruit_PCD8544 disp = Adafruit_PCD8544(DISP_DC, DISP_CS, DISP_RST);
MD_KeySwitch buttonSel(BUTTON_SEL, LOW), buttonUp(BUTTON_UP, LOW);
Ticker tickBuzzOff, tickSaveContrast;
char serial[SERIAL_LENGTH + 1] = "????????";
int frame = 0, rssi;
float lat = 0, lng = 0, alt = 0;
uint8_t contrast = 50;
bool encrypted = false;
// clang-format off
const uint8_t flipByte[] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
  },
  whitening[] = { 
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26, 0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1, 
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1, 0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C, 
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61, 0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23, 
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1, 0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98 
  };
// clang-format on
sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
  int i;

  digitalWrite(RADIO_NSS_PIN, LOW);
  while (digitalRead(RADIO_BUSY_PIN) == HIGH)
    ;
  SPI.beginTransaction(spiSettings);
  for (i = 0; i < command_length; i++)
    SPI.transfer(command[i]);
  for (i = 0; i < data_length; i++)
    SPI.transfer(data[i]);
  SPI.endTransaction();
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
  int i;

  digitalWrite(RADIO_NSS_PIN, LOW);
  while (digitalRead(RADIO_BUSY_PIN) == HIGH)
    ;
  SPI.beginTransaction(spiSettings);
  for (i = 0; i < command_length; i++)
    SPI.transfer(command[i]);
  for (i = 0; i < data_length; i++)
    data[i] = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
  digitalWrite(RADIO_RST_PIN, LOW);
  delayMicroseconds(120);
  digitalWrite(RADIO_RST_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
  Serial.println("WAKEUP");
  digitalWrite(RADIO_NSS_PIN, LOW);
  delay(1);
  digitalWrite(RADIO_NSS_PIN, HIGH);
  return SX126X_HAL_STATUS_OK;
}

void bip(int duration, int freq) {
  analogWriteFrequency(freq);
  analogWrite(BUZZER, 128);
  tickBuzzOff.once_ms(duration, []() {
    analogWrite(BUZZER, 0);
  });
}

void initDisplay() {
  disp.begin();
  disp.setContrast(contrast);
}

void clearDisplay() {
  int16_t x1, y1;
  uint16_t w, h;
  char sFreq[] = "400.000";

  dtostrf(freq / 1000.0, 6, 3, sFreq);
  disp.setFont(&FreeSansBold9pt7b);
  disp.getTextBounds(sFreq, 0, 0, &x1, &y1, &w, &h);
  disp.clearDisplay();
  disp.setCursor((WIDTH - w) / 2, 18);
  disp.print(sFreq);
  disp.setCursor(4, 44);
  disp.print("RS41rssi");

  disp.display();
}

void updateDisplay(uint8_t val, int frame, const char* serial, bool encrypted, float lat, float lng, float alt) {
  int16_t x1, y1;
  uint16_t w, h;
  static char s[10];

  disp.clearDisplay();
  w = constrain(map(val, 0, 256, 0, WIDTH), 0, WIDTH);
  disp.drawRect(0, 0, WIDTH - 1, 6, BLACK);
  disp.fillRect(0, 0, w, 6, BLACK);

  disp.setFont(&FreeSansBold9pt7b);
  dtostrf(-val / 2.0, 3, 1, s);
  disp.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  disp.setCursor((WIDTH - w) / 2, 22);
  disp.print(s);

  disp.setFont(NULL);
  disp.setCursor(0, 40);
  disp.print(serial);

  disp.setFont(&Org_01);

  if (frame > 0) {
    itoa(frame, s, 10);
    disp.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
    disp.setCursor(25 - w, 35);
    disp.print(s);
  }

  if (encrypted) {  //lock
    disp.fillCircle(73, 36, 5, BLACK);
    disp.fillCircle(73, 36, 3, WHITE);
    disp.fillRect(67, 38, 13, 12, BLACK);
  } else {
    disp.setCursor(40, 32);
    dtostrf(lat, 8, 5, s);
    disp.print(s);
    disp.setCursor(40, 38);
    dtostrf(lng, 8, 5, s);
    disp.print(s);
    itoa((int)alt, s, 10);
    disp.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
    disp.setCursor(83 - w, 44);
    disp.print(s);
  }

  disp.display();
}

void editFreq() {
  const int left = 10, top = 40, w = 10, h = 20;
  int pos = 2;
  char sFreq[] = "400.000";

  dtostrf(freq / 1000.0, 6, 3, sFreq);

  disp.clearDisplay();

  disp.setFont(NULL);
  disp.setCursor(0, 0);
  disp.print("Nuova freq:");

  disp.setFont(&FreeSansBold9pt7b);
  disp.setCursor(left, top);
  disp.print(sFreq);
  disp.drawRect(left + w * 2, top - 16, w, h, BLACK);
  disp.display();

  while (true) {
    switch (buttonSel.read()) {
      case MD_KeySwitch::KS_PRESS:
        disp.fillRect(left + w * pos - (pos > 2 ? 6 : 0), top - 16, w, h, WHITE);
        if (++pos == 3) pos = 4;
        if (pos == 7) {
          pos = 2;
          freq = (int)roundf(1000 * atof(sFreq));
          Serial.printf("nuova frequenza: %d\n", freq);
          clearDisplay();
          sx126x_long_pkt_rx_complete(NULL);
          sx126x_set_rf_freq(NULL, freq * 1000UL);
          sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
          EEPROM.writeUInt(0, freq);
          EEPROM.commit();
          return;
        }
        disp.setCursor(left, top);
        disp.print(sFreq);
        disp.drawRect(left + w * pos - (pos > 2 ? 6 : 0), top - 16, w, h, BLACK);
        disp.display();
        break;
    }
    switch (buttonUp.read()) {
      case MD_KeySwitch::KS_PRESS:
      case MD_KeySwitch::KS_RPTPRESS:
        if (++sFreq[pos] > '9' || pos == 2 && sFreq[2] > '5') sFreq[pos] = '0';
        disp.fillRect(left + w * pos - (pos > 2 ? 6 : 0), top - 16, w, h, WHITE);
        disp.setCursor(left, top);
        disp.print(sFreq);
        disp.drawRect(left + w * pos - (pos > 2 ? 6 : 0), top - 16, w, h, BLACK);
        disp.display();
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(sizeof freq + sizeof contrast);
  freq = EEPROM.readUInt(0);
  if (freq < 400000UL || freq >= 406000UL)
    freq = 403000UL;
  contrast = EEPROM.read(4);
  if (contrast == 0xFF)
    contrast = 50;

  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON_SEL, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  buttonSel.enableRepeat(false);
  buttonSel.enableLongPress(false);
  buttonUp.enableRepeat(false);
  buttonUp.enableLongPress(true);

  initDisplay();
  clearDisplay();

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_NSS_PIN);

  pinMode(LED, OUTPUT);
  pinMode(RADIO_NSS_PIN, OUTPUT);
  pinMode(RADIO_RST_PIN, OUTPUT);
  pinMode(RADIO_BUSY_PIN, INPUT);
  pinMode(RADIO_DIO1_PIN, INPUT);

  digitalWrite(LED, HIGH);
  analogWriteFrequency(1000);
  analogWrite(BUZZER, 128);
  delay(200);
  analogWriteFrequency(2000);
  delay(500);
  analogWrite(BUZZER, 0);
  digitalWrite(LED, LOW);

  sx126x_mod_params_gfsk_t modParams = {
    .br_in_bps = 4800,
    .fdev_in_hz = 3600,                          //?
    .pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF,  //?
    .bw_dsb_param = SX126X_GFSK_BW_9700          //?
  };
  sx126x_pkt_params_gfsk_t pktParams = {
    .preamble_len_in_bits = 320,
    .preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS,
    .sync_word_len_in_bits = 64,
    .address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type = SX126X_GFSK_PKT_FIX_LEN,
    .pld_len_in_bytes = 255,
    .crc_type = SX126X_GFSK_CRC_OFF,
    .dc_free = SX126X_GFSK_DC_FREE_OFF
  };
  sx126x_status_t res = sx126x_reset(NULL);
  res = sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
  Serial.printf("sx126x_set_standby %d\n", res);
  res = sx126x_set_dio3_as_tcxo_ctrl(NULL, SX126X_TCXO_CTRL_2_7V, 128);  //2ms
  delay(1);
  res = sx126x_set_pkt_type(NULL, SX126X_PKT_TYPE_GFSK);
  Serial.printf("sx126x_set_pkt_type %d\n", res);
  res = sx126x_set_rf_freq(NULL, freq * 1000UL);
  Serial.printf("sx126x_set_rf_freq %d\n", res);
  res = sx126x_set_gfsk_mod_params(NULL, &modParams);
  Serial.printf("sx126x_set_gfsk_mod_params %d\n", res);
  res = sx126x_long_pkt_rx_set_gfsk_pkt_params(NULL, &pktParams);
  Serial.printf("sx126x_long_pkt_rx_set_gfsk_pkt_params %d\n", res);
  res = sx126x_set_dio_irq_params(NULL, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  Serial.printf("sx126x_set_dio_irq_params %d\n", res);
  uint8_t syncWord[] = { 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 };
  for (int i = 0; i < sizeof syncWord; i++)
    syncWord[i] = flipByte[syncWord[i]];
  res = sx126x_set_gfsk_sync_word(NULL, syncWord, sizeof syncWord);
  Serial.printf("sx126x_set_gfsk_sync_word %d\n", res);

  res = sx126x_cal_img(NULL, 0x6B, 0x6F);
  // uint8_t val = 0x96;
  // res = sx126x_write_register(NULL, SX126X_REG_RXGAIN, &val, 1);

  res = sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
  Serial.printf("sx126x_long_pkt_set_rx %d\n", res);
  res = sx126x_clear_device_errors(NULL);
  char s[17] = "";
  res = sx126x_read_register(NULL, 0x320, (uint8_t*)s, 16);
  Serial.printf("device ID: %s\n", s);
}

void dump(uint8_t buf[], int size) {
  for (int i = 0; i < size; i++)
    Serial.printf("%02X%c", buf[i], i % 16 == 7 ? '-' : i % 16 == 15 ? '\n'
                                                                     : ' ');
  if (size % 16 != 0) Serial.println();
}

bool correctErrors(uint8_t data[]) {
  static uint8_t buf[256], dec[256];
  int i;

  //prima parte
  memset(buf, 0, 256);
  for (i = 0; i < (PACKET_LENGTH - 48) / 2; i++)
    buf[99 + i] = data[PACKET_LENGTH - 1 - 2 * i];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[24 + i];

  if (0 != rs.Decode(buf, dec)) return false;

  for (i = 0; i < (PACKET_LENGTH - 48) / 2; i++)
    data[311 - 2 * i] = dec[99 + i];

  //seconda parte
  memset(buf, 0, 256);
  for (i = 0; i < (PACKET_LENGTH - 48) / 2; i++)
    buf[99 + i] = data[PACKET_LENGTH - 1 - 2 * i - 1];
  for (i = 0; i < 24; i++)
    buf[254 - i] = data[i];

  if (0 != rs.Decode(buf, dec)) return false;

  for (i = 0; i < (PACKET_LENGTH - 48) / 2; i++)
    data[PACKET_LENGTH - 1 - 2 * i - 1] = dec[99 + i];

  return true;
}

//https://gis.stackexchange.com/questions/265909/converting-from-ecef-to-geodetic-coordinates
void ecef2wgs84(float x, float y, float z, float& lat, float& lng, float& height) {
  // WGS84 constants
  float a = 6378137.0,
        f = 1.0 / 298.257223563;
  // derived constants
  float b = a - f * a,
        e = sqrt(pow(a, 2.0) - pow(b, 2.0)) / a,
        clambda = atan2(y, x),
        p = sqrt(pow(x, 2.0) + pow(y, 2)),
        h_old = 0.0;
  //first guess with h=0 meters
  float theta = atan2(z, p * (1.0 - pow(e, 2.0))),
        cs = cos(theta),
        sn = sin(theta),
        N = pow(a, 2.0) / sqrt(pow(a * cs, 2.0) + pow(b * sn, 2.0)),
        h = p / cs - N;
  int nMaxLoops = 100;
  while (abs(h - h_old) > 1.0e-6 && nMaxLoops-- > 0) {
    h_old = h;
    theta = atan2(z, p * (1.0 - pow(e, 2.0) * N / (N + h)));
    cs = cos(theta);
    sn = sin(theta);
    N = pow(a, 2.0) / sqrt(pow(a * cs, 2.0) + pow(b * sn, 2.0));
    h = p / cs - N;
  }
  lng = clambda / M_PI * 180;
  lat = theta / M_PI * 180;
  height = h;
}

void processPacket(uint8_t buf[], int rssi) {
  float x, y, z;
  int n = 48 + 1;

  frame = 0;
  strcpy(serial, "????????");
  encrypted = false;

  Serial.printf("RSSI: %d\n", rssi);
  if (correctErrors(buf) && buf[48] == 0x0F) {
    while (n < PACKET_LENGTH) {
      int blockType = buf[n];
      int blockLength = buf[n + 1];
      uint16_t crc = calcCRC16(buf + n + 2, blockLength, CRC16_CCITT_FALSE_POLYNOME, CRC16_CCITT_FALSE_INITIAL, CRC16_CCITT_FALSE_XOR_OUT, CRC16_CCITT_FALSE_REV_IN, CRC16_CCITT_FALSE_REV_OUT);

      //Serial.printf("Blocco 0x%02X, lunghezza %d, CRC: %02X%02X/%02X%02X\n", blockType, blockLength, buf[n + blockLength + 3], buf[n + blockLength + 2], crc >> 8, crc & 0xFF);
      if ((crc & 0xFF) == buf[n + blockLength + 2] && (crc >> 8) == buf[n + blockLength + 3]) {  //CRC OK
        switch (blockType) {
          case 0x79:  //status
            frame = buf[n + 2] + (buf[n + 3] << 8);
            Serial.printf("\tframe: %d [%.8s]\n", frame, buf + n + 4);
            strncpy(serial, (char*)buf + n + 4, sizeof serial - 1);
            serial[sizeof serial - 1] = 0;
            break;
          case 0x7B:  //GPSPOS
            x = buf[n + 2] + 256 * (buf[n + 3] + 256 * (buf[n + 4] + 256 * buf[n + 5])) / 100.0;
            y = buf[n + 6] + 256 * (buf[n + 7] + 256 * (buf[n + 8] + 256 * buf[n + 9])) / 100.0;
            z = buf[n + 10] + 256 * (buf[n + 11] + 256 * (buf[n + 12] + 256 * buf[n + 13])) / 100.0;
            ecef2wgs84(x, y, z, lat, lng, alt);
            Serial.printf("\tlat:%f lon:%f h:%f\n", lat, lng, alt);
            break;
          case 0x80:  //CRYPTO
            encrypted = true;
            break;
        }
      }
      n += blockLength + 4;
    }
  }
  updateDisplay(rssi, frame, serial, encrypted, lat, lng, alt);
  bip(200, constrain(map(rssi, 255, 0, 150, 9000), 150, 9000));
}

void loop() {
  static uint64_t tLastRead = 0, tLastPacket = 0;
  sx126x_status_t res;
  sx126x_pkt_status_gfsk_t pktStatus;
  sx126x_rx_buffer_status_t bufStatus;

  switch (buttonUp.read()) {
    case MD_KeySwitch::KS_LONGPRESS:
      updateDisplay(rssi, frame, serial, encrypted, lat, lng, alt);
      delay(3000);
      clearDisplay();
      break;
    case MD_KeySwitch::KS_PRESS:
      contrast += 5;
      disp.setContrast(contrast);
      if (tickSaveContrast.active())
        tickSaveContrast.detach();
      tickSaveContrast.once_ms(3000, []() {
        EEPROM.begin(sizeof freq + sizeof contrast);
        EEPROM.write(4, contrast);
        EEPROM.commit();
      });
      break;
  }

  if (buttonSel.read() == MD_KeySwitch::KS_PRESS)
    editFreq();
  if (tLastPacket != 0 && millis() - tLastPacket > 3000) {
    clearDisplay();
    tLastPacket = 0;
  }

  if (digitalRead(RADIO_DIO1_PIN) == HIGH) {
    //Serial.println("SYNC");
    digitalWrite(LED, HIGH);
    bip(50, 2000);
    tLastPacket = tLastRead = millis();
    nBytesRead = 0;
    res = sx126x_clear_irq_status(NULL, SX126X_IRQ_SYNC_WORD_VALID);
  }
  if (tLastRead != 0 && millis() - tLastRead > 1000) {
    res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, 0);
    tLastRead = 0;
    nBytesRead = 0;
  }
  if (tLastRead != 0 && millis() - tLastRead > 300) {
    tLastRead = millis();
    uint8_t read;
    res = sx126x_long_pkt_rx_get_partial_payload(NULL, &pktRxState, buf + nBytesRead, sizeof buf - nBytesRead, &read);
    if (read == 0) {
      tLastRead = 0;
      nBytesRead = 0;
      return;
    }
    //Serial.printf("READ %d\n", read);
    nBytesRead += read;
    if (sizeof buf - nBytesRead <= 255) {
      res = sx126x_long_pkt_rx_prepare_for_last(NULL, &pktRxState, sizeof buf - nBytesRead);
      digitalWrite(LED, LOW);
    }
    if (nBytesRead == sizeof buf) {
      sx126x_long_pkt_rx_complete(NULL);
      sx126x_long_pkt_set_rx_with_timeout_in_rtc_step(NULL, &pktRxState, SX126X_RX_CONTINUOUS);
      tLastRead = 0;
      nBytesRead = 0;
      for (int i = 0; i < sizeof buf; i++)
        buf[i] = whitening[i % sizeof whitening] ^ flipByte[buf[i]];
      //dump(buf, sizeof buf);

      sx126x_get_gfsk_pkt_status(NULL, &pktStatus);
      rssi = pktStatus.rssi_sync;
      processPacket(buf, rssi);
    }
  }
}
