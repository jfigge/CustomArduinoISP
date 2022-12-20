#include "Arduino.h"
#include "SPI.h"

#undef SERIAL
#define SERIAL Serial

#define ON   true
#define OFF false

#define PROG_FLICKER true
#define SPI_CLOCK 		(1000000/6)

// Programmer
#define PIN_SCK   SCK  // 13 Sync clock
#define PIN_MISO  MISO // 12 Master in, slave out
#define PIN_MOSI  MOSI // 11 Master out, slave in
#define RESET     10 // Use pin 10 to reset the target rather than SS
#define LED_HB    6  // Heartbeat LED
#define LED_ERR   7  // Error LED
#define LED_PMODE 8  // Programming LED
#define LED_POWER 9  // VCC Power LED

// Fuse writer
#define DATAIN  13    // 11 Connect to Serial Data Output (SDO) Pin 7
#define INSTOUT 12    // 10 Connect to Serial Instruction Input (SII) Pin 6
#define DATAOUT 11    //  9 Connect to Serial Data Input (SDI) Pin 5 
#define VCC      5    //  8 Connect to VCC Pin 8
#define CLKOUT   4    // 12 Connect to Serial Clock Input (SCI) Pin 2
#define RST_12V  3    // 13 Output to level shifter for !RESET from transistor to Pin 1
#define SWITCH   2    // Function switch 

#define  LFUSE  0xE2   // E2 = 8Mhz, 62 = 1Mhz   

// Configure the baud rate:
#define BAUDRATE	19200

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

boolean fuseWriterMode = false;

void setup() {
  // Initialize output pins as needed
  pinMode(RST_12V, OUTPUT);
  power12V(OFF);

  // Enable function switch and interrupt handler
  pinMode(SWITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH), ISR_switchMode, CHANGE);

  pinMode(CLKOUT, OUTPUT);
  digitalWrite(CLKOUT, LOW);

  SERIAL.begin(BAUDRATE);

  if (digitalRead(SWITCH) == HIGH) {
    fuse_setup();
  } else {
    clearLog();
    Serial.println("Programming mode...");
  }
  
  // Pulse status LEDs
  pinMode(LED_POWER, OUTPUT);
  pulse(LED_POWER, 2);
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
  pinMode(VCC, OUTPUT);

}

void(* resetFunc) (void) = 0;//declare reset function at address 0

void ISR_switchMode() {
  power(OFF);
  power12V(OFF);
  digitalWrite(LED_PMODE, LOW);
  digitalWrite(LED_ERR,   LOW);
  digitalWrite(LED_HB,    LOW);
  resetFunc();
}

int error = 0;
int pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  uint8_t flashpoll;
  uint16_t eeprompoll;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;
}
parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)
    return;
  last_time = now;
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
}

static bool rst_active_high;

void reset_target(bool reset) {
  digitalWrite(RESET, ((reset && rst_active_high) || (!reset && !rst_active_high)) ? LOW : HIGH);
}

void loop() {
  // is pmode active?
  if (pmode) {
    digitalWrite(LED_PMODE, HIGH);
  } else {
    digitalWrite(LED_PMODE, LOW);
  }
  // is there an error?
  if (error) {
    digitalWrite(LED_ERR, HIGH);
  } else {
    digitalWrite(LED_ERR, LOW);
  }

  // light the heartbeat LED
  heartbeat();

  if (SERIAL.available()) {
    if (fuseWriterMode) {
      fuse_loop();
    } else {
      avrisp();
    }
  }
}

uint8_t getch() {
  while (!SERIAL.available());
  return SERIAL.read();
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

#define PTIME 20
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER) {
    digitalWrite(LED_PMODE, state);
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  SPI.transfer(a);
  SPI.transfer(b);
  SPI.transfer(c);
  return SPI.transfer(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    SERIAL.print((char)STK_INSYNC);
    SERIAL.print((char)STK_OK);
  } else {
    error++;
    SERIAL.print((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    SERIAL.print((char)STK_INSYNC);
    SERIAL.print((char)b);
    SERIAL.print((char)STK_OK);
  } else {
    error++;
    SERIAL.print((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
    case 0x80:
      breply(HWVER);
      break;
    case 0x81:
      breply(SWMAJ);
      break;
    case 0x82:
      breply(SWMIN);
      break;
    case 0x93:
      breply('S'); // serial programmer
      break;
    default:
      breply(0);
  }
}

void set_parameters() {
  // call this after reading parameter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
                    + buff[17] * 0x00010000
                    + buff[18] * 0x00000100
                    + buff[19];

  // AVR devices have active low reset, AT89Sx are active high
  rst_active_high = (param.devicecode >= 0xe0);
}

void power(boolean on) {
  if (on) {
    digitalWrite(LED_POWER, HIGH);
    digitalWrite(VCC,       HIGH);
  } else {
    digitalWrite(LED_POWER, LOW);
    digitalWrite(VCC,       LOW);
  }
}

void power12V(boolean on) {
  if (on && fuseWriterMode) {
    digitalWrite(LED_POWER, HIGH);
    digitalWrite(RST_12V,   HIGH);
  } else {
    digitalWrite(LED_POWER, LOW);
    digitalWrite(RST_12V,   LOW);
  }
}

void start_pmode() {
  // Turn on VCC
  power(ON);

  // Reset target before driving PIN_SCK or PIN_MOSI

  // SPI.begin() will configure SS as output, so SPI master mode is selected.
  // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
  // So we have to configure RESET as output here,
  // (reset_target() first sets the correct level)
  reset_target(true);
  pinMode(RESET, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

  // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

  // Pulse RESET after PIN_SCK is low:
  digitalWrite(PIN_SCK, LOW);
  delay(20); // discharge PIN_SCK, value arbitrarily chosen
  reset_target(false);
  // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
  // speeds above 20 KHz
  delayMicroseconds(100);
  reset_target(true);

  // Send the enable programming command:
  delay(50); // datasheet: must be > 20 msec
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  SPI.end();
  // We're about to take the target out of reset so configure SPI pins as input
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_SCK, INPUT);
  reset_target(false);
  pinMode(RESET, INPUT);
  pmode = 0;

  // Turn off VCC
  power(OFF);
}

void universal() {
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo,
                  addr >> 8 & 0xFF,
                  addr & 0xFF,
                  data);
}
void commit(unsigned int addr) {
  if (PROG_FLICKER) {
    prog_lamp(LOW);
  }
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(PTIME);
    prog_lamp(HIGH);
  }
}

unsigned int current_page() {
  if (param.pagesize == 32) {
    return here & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return here & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return here & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return here & 0xFFFFFF80;
  }
  return here;
}


void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    SERIAL.print((char) STK_INSYNC);
    SERIAL.print((char) write_flash_pages(length));
  } else {
    error++;
    SERIAL.print((char) STK_NOSYNC);
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length) {
  // here is a word address, get the byte address
  unsigned int start = here * 2;
  unsigned int remaining = length;
  if (length > param.eepromsize) {
    error++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH);
  return STK_OK;
}

void program_page() {
  char result = (char) STK_FAILED;
  unsigned int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      SERIAL.print((char) STK_INSYNC);
      SERIAL.print(result);
    } else {
      error++;
      SERIAL.print((char) STK_NOSYNC);
    }
    return;
  }
  SERIAL.print((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8,
                         (addr >> 8) & 0xFF,
                         addr & 0xFF,
                         0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    SERIAL.print((char) low);
    uint8_t high = flash_read(HIGH, here);
    SERIAL.print((char) high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    SERIAL.print((char) ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    error++;
    SERIAL.print((char) STK_NOSYNC);
    return;
  }
  SERIAL.print((char) STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  SERIAL.print(result);
}

void read_signature() {
  if (CRC_EOP != getch()) {
    error++;
    SERIAL.print((char) STK_NOSYNC);
    return;
  }
  SERIAL.print((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  SERIAL.print((char) high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  SERIAL.print((char) middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  SERIAL.print((char) low);
  SERIAL.print((char) STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
void avrisp() {
  uint8_t ch = getch();
  switch (ch) {
    case '0': // signon
      error = 0;
      empty_reply();
      break;
    case '1':
      if (getch() == CRC_EOP) {
        SERIAL.print((char) STK_INSYNC);
        SERIAL.print("AVR ISP");
        SERIAL.print((char) STK_OK);
      }
      else {
        error++;
        SERIAL.print((char) STK_NOSYNC);
      }
      break;
    case 'A':
      get_version(getch());
      break;
    case 'B':
      fill(20);
      set_parameters();
      empty_reply();
      break;
    case 'E': // extended parameters - ignore for now
      fill(5);
      empty_reply();
      break;
    case 'P':
      if (!pmode)
        start_pmode();
      empty_reply();
      break;
    case 'U': // set address (word)
      here = getch();
      here += 256 * getch();
      empty_reply();
      break;

    case 0x60: //STK_PROG_FLASH
      getch(); // low addr
      getch(); // high addr
      empty_reply();
      break;
    case 0x61: //STK_PROG_DATA
      getch(); // data
      empty_reply();
      break;

    case 0x64: //STK_PROG_PAGE
      program_page();
      break;

    case 0x74: //STK_READ_PAGE 't'
      read_page();
      break;

    case 'V': //0x56
      universal();
      break;
    case 'Q': //0x51
      error = 0;
      end_pmode();
      empty_reply();
      break;

    case 0x75: //STK_READ_SIGN 'u'
      read_signature();
      break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
    case CRC_EOP:
      error++;
      SERIAL.print((char) STK_NOSYNC);
      break;

    // anything else we will return STK_UNKNOWN
    default:
      error++;
      if (CRC_EOP == getch())
        SERIAL.print((char)STK_UNKNOWN);
      else
        SERIAL.print((char)STK_NOSYNC);
  }
}

// ########## Fuse writer #####################################################

int inByte = 0;     // incoming serial byte Computer
int inData = 0;     // incoming serial byte AVR
int HFUSE = 0xD7;   // 57 = Disable reset, D7 = Enable reset

void fuse_setup()
{
  // Set up control lines for HV parallel programming
  pinMode(DATAOUT, OUTPUT);
  pinMode(INSTOUT, OUTPUT);
  pinMode(DATAIN,  OUTPUT);  // configured as input when in programming mode
  digitalWrite(RESET, LOW);

  fuseWriterMode = true;
  
  clearLog();
  establishContact();  // send a byte to establish contact until receiver responds
}

int state = 0;
void fuse_loop()
{
  // get incoming byte:
  inByte = Serial.read();
  if (inByte == (byte)'1') {
    HFUSE = 0xD7;   // 57 = Disable reset, D7 = Enable reset
    state = 1;
  } else if (inByte == (byte)'2') {
    HFUSE = 0x57;   // 57 = Disable reset, D7 = Enable reset
    state = 2;
  } else if (inByte == (byte)'3') {
    state = 3;
  } else {
    establishContact();
  }

  if (state != 0) {
    clearLog();

    // Consume remaining input
    while (Serial.available() > 0) {
      Serial.read();
    }

    if (state < 3) {
      digitalWrite(LED_PMODE, HIGH);
    } else {
      Serial.println("Checking fuses\n");
    }

    // Initialize pins to enter programming mode
    pinMode(DATAIN, OUTPUT);  //Temporary
    digitalWrite(DATAOUT, LOW);
    digitalWrite(INSTOUT, LOW);
    digitalWrite(DATAIN, LOW);
    power12V(OFF);  // Level shifter is inverting, this shuts off 12V

    // Enter High-voltage Serial programming mode
    power(ON);  // Apply VCC to start programming process
    delayMicroseconds(20);
    power12V(ON);   //Turn on 12v
    delayMicroseconds(10);
    pinMode(DATAIN, INPUT);   //Release DATAIN
    delayMicroseconds(300);

    //Programming mode

    readFuses();

    if (state < 3) {
      //Write hfuse
      Serial.println("Writing hfuse");
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, HFUSE, 0x2C);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x74);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7C);

      //Write lfuse
      Serial.println("Writing lfuse\n");
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, LFUSE, 0x2C);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x64);
      shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);

      readFuses();
      digitalWrite(LED_PMODE, LOW);
    }

    digitalWrite(CLKOUT, LOW);
    power(OFF);
    power12V(OFF);   //Turn off 12v
  }
  establishContact();
}

void establishContact() {
  Serial.println("http://www.engbedded.com/fusecalc/\n");
  Serial.println("Menu");
  Serial.println("1)..... Enable reset (write D7)");
  Serial.println("2).... Disable reset (write 57)");
  Serial.println("3)...... Check fuses");
  Serial.println("Enter choice: ");
  state = 0;
}

int shiftOut2(uint8_t dataPin, uint8_t dataPin1, uint8_t clockPin, uint8_t bitOrder, byte val, byte val1)
{
  int i;
  int inBits = 0;
  //Wait until DATAIN goes high
  while (!digitalRead(DATAIN));

  //Start bit
  digitalWrite(DATAOUT, LOW);
  digitalWrite(INSTOUT, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  for (i = 0; i < 8; i++)  {

    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(val & (1 << i)));
      digitalWrite(dataPin1, !!(val1 & (1 << i)));
    }
    else {
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));
      digitalWrite(dataPin1, !!(val1 & (1 << (7 - i))));
    }
    inBits <<= 1;
    inBits |= digitalRead(DATAIN);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);

  }


  //End bits
  digitalWrite(DATAOUT, LOW);
  digitalWrite(INSTOUT, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  return inBits;
}

void readFuses() {
  //Read lfuse
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x68);
  inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);
  Serial.print("lfuse reads as ");
  Serial.println(inData, HEX);

  //Read hfuse
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7A);
  inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7E);
  Serial.print("hfuse reads as ");
  Serial.println(inData, HEX);

  //Read efuse
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6A);
  inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6E);
  Serial.print("efuse reads as ");
  Serial.println(inData, HEX);
  Serial.println();
}

void clearLog() {
  for (int i = 0; i < 100; i++) {
    Serial.println();
  }
}
