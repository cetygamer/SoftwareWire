
#ifndef SoftwareWire_h
#define SoftwareWire_h

#include <Arduino.h>
#include <Wire.h>


// Transmission status error, the return value of endTransmission()
#define SOFTWAREWIRE_NO_ERROR       0
#define SOFTWAREWIRE_BUFFER_FULL    1
#define SOFTWAREWIRE_ADDRESS_NACK   2
#define SOFTWAREWIRE_DATA_NACK      3
#define SOFTWAREWIRE_OTHER          4

#define SOFTWAREWIRE_BUFSIZE 32        // same as buffer size of Arduino Wire library


class SoftwareWire : public TwoWire
{
public:
  SoftwareWire();
  SoftwareWire(uint8_t sdaPin, uint8_t sclPin, boolean pullups = true, boolean detectClockStretch = true);
  ~SoftwareWire();
  void end();

  void begin();

  // Generate compile error when slave mode begin(address) is used
  void __attribute__ ((error("I2C/TWI Slave mode is not supported by the SoftwareWire library"))) begin(uint8_t addr);
  void __attribute__ ((error("I2C/TWI Slave mode is not supported by the SoftwareWire library"))) begin(int addr);

  void setClock(uint32_t clock);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  uint8_t endTransmission(boolean sendStop = true);
  uint8_t requestFrom(uint8_t address, uint8_t size, boolean sendStop = true);
  uint8_t requestFrom(int address, int size, boolean sendStop = true);
  size_t write(uint8_t data) override;
  size_t write(const uint8_t *data, size_t quantity) override;
  int available(void) override;
  int read(void) override;
  int readBytes(uint8_t* buf, uint8_t size);
  int readBytes(char * buf, uint8_t size);
  int readBytes(char * buf, int size);
  int peek(void) override;
  void setTimeout(long timeout);  // timeout to wait for the I2C bus
  void printStatus(Print& Ser);   // print information using specified object class


public:
  // per object data

  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _sdaBitMask;
  uint8_t _sclBitMask;

  volatile uint8_t *_sdaPortReg;
  volatile uint8_t *_sclPortReg;
  volatile uint8_t *_sdaDirReg;
  volatile uint8_t *_sclDirReg;
  volatile uint8_t *_sdaPinReg;
  volatile uint8_t *_sclPinReg;

  uint8_t _transmission;      // transmission status, returned by endTransmission(). 0 is no error.
  uint16_t _i2cdelay;         // delay in micro seconds for sda and scl bits.
  boolean _pullups;           // using the internal pullups or not
  boolean _stretch;           // should code handle clock stretching by the slave or not.
  unsigned long _timeout;     // timeout in ms. When waiting for a clock pulse stretch. 2017, Fix issue #6

  uint8_t rxBuf[SOFTWAREWIRE_BUFSIZE];   // buffer inside this class, a buffer per SoftwareWire.
  uint8_t rxBufPut;           // index to rxBuf, just after the last valid byte.
  uint8_t rxBufGet;           // index to rxBuf, the first new to be read byte.

  // private methods

  void i2c_writebit( uint8_t c );
  uint8_t i2c_readbit(void);
  void i2c_init(void);
  boolean i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  uint8_t i2c_write(uint8_t c);
  uint8_t i2c_read(boolean ack);
};

#include "USBAPI.h"

extern Serial_ Serial;

#define FORCE_INLINE __attribute__((always_inline)) inline

class SoftIIC : public SoftwareWire
{
public:
  SoftIIC(uint8_t sdaPin, uint8_t sclPin, boolean pullups = true, boolean detectClockStretch = true) :
    SoftwareWire(sdaPin, sclPin, pullups, detectClockStretch)
  {
    _busBitMask = (_sclBitMask | _sdaBitMask);

    // Verify if port is the same
    uint8_t sdaPort = digitalPinToPort(_sdaPin);
    uint8_t sclPort = digitalPinToPort(_sclPin);

    if(sdaPort != sclPort)
      Serial.println("!!! Ports are NOT the same !!!");
  }
  ~SoftIIC();

protected:
  uint8_t _busBitMask;

  // IIC slave mode timing related functions/variables
  FORCE_INLINE uint8_t spin_until_start();
  FORCE_INLINE uint8_t spin_until_clock_rises();
  FORCE_INLINE uint8_t spin_until_clock_falls();

  // IIC slave mode lower level functions
  FORCE_INLINE uint8_t get_byte(uint8_t *value, uint8_t (*my_ack_function)(uint8_t rxbyte));
  FORCE_INLINE uint8_t set_byte(uint8_t (*my_get_byte_function)(uint8_t *txbyte));

  // IIC state functions
  uint8_t IIC_STATE;
  uint8_t IIC_STATE_PREVIOUS;

  FORCE_INLINE uint8_t StateIdle();
  FORCE_INLINE uint8_t StateStart();
  FORCE_INLINE uint8_t StateStop();
  FORCE_INLINE uint8_t StateData();
  FORCE_INLINE uint8_t StateClockFell();
  FORCE_INLINE uint8_t StateDataBit();
  FORCE_INLINE uint8_t StateClockLow();
  FORCE_INLINE uint8_t StateClockHigh();

  FORCE_INLINE void bus_read_same_port();
  FORCE_INLINE void bus_read();

  uint8_t i2c_read(boolean ack, unsigned long stretchOnDataRecv);

public:
  uint8_t SlaveHandleTransaction(
    uint8_t (*fp_respond_to_address)(uint8_t chip_address),
    uint8_t (*fp_respond_to_data)(uint8_t value),
    uint8_t (*fp_generate_byte)(uint8_t *value)
  );

  uint8_t requestFrom(uint8_t address, uint8_t size, unsigned long stretchOnAddrAck,
    unsigned long stretchOnDataRecv, boolean sendStop = true);

  void set_sda_hi();
  void set_sda_lo();
};

void i2c_isr();

typedef struct i2c_s {
  // Signal level
  int      state;   // Status of I2C state machine (see I2CSTATE_XXX)
  int      pulse;   // The number of SCL pulses seen so far
  uint32_t qus;     // Transaction start, later length, in us
  int      sdacap;  // State of SDA captured on rising SCL
  int      data;    // Bits of ADDR or DATA
  int      bitcnt;  // Bit count 0..7 of bits in data
  int      logbits; // Should individual bits be logged
  // Segment level
  int      addr;    // Last slave address on the line
  int      bytecnt; // Byte count in segment
} i2c_t;

class I2CTool : public SoftwareWire
{
public:
  I2CTool(uint8_t sdaPin, uint8_t sclPin, boolean pullups = true, boolean detectClockStretch = true);
  ~I2CTool();

  friend void i2c_isr();

protected:
  // Signal level
  int      state;   // Status of I2C state machine (see I2CSTATE_XXX)
  int      pulse;   // The number of SCL pulses seen so far
  uint32_t qus;     // Transaction start, later length, in us
  int      sdacap;  // State of SDA captured on rising SCL
  int      data;    // Bits of ADDR or DATA
  int      bitcnt;  // Bit count 0..7 of bits in data
  int      logbits; // Should individual bits be logged
  // Segment level
  int      addr;    // Last slave address on the line
  int      bytecnt; // Byte count in segment
};

extern I2CTool i2c;

// Extended function for microsecond sleep
static FORCE_INLINE void udelay(unsigned long us)
{
  if(us > 1000)
  {
    unsigned long ms_part = us / 1000;
    unsigned long us_part = us % 1000;
    if(ms_part)
      delay(ms_part);
    if(us_part)
      delayMicroseconds(us_part);
  }
  else
  {
    delayMicroseconds(us);
  }
}

#endif // SoftwareWire_h
