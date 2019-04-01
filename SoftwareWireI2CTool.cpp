#include "SoftwareWire.h"

#define i2c_sda_lo()              \
  *(i2c._sdaPortReg) &= ~i2c._sdaBitMask;   \
  *(i2c._sdaDirReg)  |=  i2c._sdaBitMask;

#define i2c_scl_lo()              \
  *(i2c._sclPortReg) &= ~i2c._sclBitMask;   \
  *(i2c._sclDirReg)  |=  i2c._sclBitMask;

#define i2c_sda_hi()              \
  *(i2c._sdaDirReg) &= ~i2c._sdaBitMask;    \
  if(i2c._pullups) { *(i2c._sdaPortReg) |= i2c._sdaBitMask; }

#define i2c_scl_hi()              \
  *(i2c._sclDirReg) &= ~i2c._sclBitMask;    \
  if(i2c._pullups) { *(i2c._sclPortReg) |= i2c._sclBitMask; }

#define i2c_sda_read()   ((uint8_t) (*(i2c._sdaPinReg) & i2c._sdaBitMask) ? 1 : 0)
#define i2c_scl_read()   ((uint8_t) (*(i2c._sclPinReg) & i2c._sclBitMask) ? 1 : 0)

#define SLAVE_ADDR                  0x8 // I2C slave address
#define SLAVE_REGCOUNT              32   // Number of bytes exposed as registers over I2C
#define SLAVE_REG_ENABLE()          (slave_regs[0] * 256 + slave_regs[1]) // Register ENABLE: number of transaction the clock stretch enabled
#define SLAVE_REG_PULSE()           (slave_regs[2] * 256 + slave_regs[3]) // Register PULSE: which pulse to stretch
#define SLAVE_REG_US()              (slave_regs[4] * 256 + slave_regs[5]) // Register US: how many us to stretch
#define SLAVE_REG_QPULSE()          (slave_regs[6] * 256 + slave_regs[7]) // Register QPULSE: number of pulses in last transaction
#define SLAVE_REG_QUS()             (slave_regs[8] * 256UL * 256 * 256 + slave_regs[9] * 256UL * 256 + slave_regs[10] * 256UL + slave_regs[11]) // Register QUS: duration in transaction in us
#define SLAVE_REG_RSVD()            (slave_regs[12] * 256UL * 256 * 256 + slave_regs[13] * 256UL * 256 + slave_regs[14] * 256UL + slave_regs[15]) // Register RSVD: reserved
#define SLAVE_REG_SET_ENABLE(c)     do { slave_regs[0] = (c) >> 8; slave_regs[1] = (c); } while(0)
#define SLAVE_REG_SET_QPULSE(c)     do { slave_regs[6] = (c) >> 8; slave_regs[7] = (c); } while(0)
#define SLAVE_REG_SET_QUS(us)       do { slave_regs[8] = (us) >> 24; slave_regs[9] = (us) >> 16; slave_regs[10] = (us) >> 8; slave_regs[11] = (us); } while(0)
uint8_t slave_regs[SLAVE_REGCOUNT]; // The backing memory
uint8_t slave_cra;                  // The CRA or Current Register Address

#define I2CSTATE_UNKNOWN       1
#define I2CSTATE_ERROR         2
#define I2CSTATE_IDLE          3
#define I2CSTATE_STARTED       4
#define I2CSTATE_ADDRBIT_SCLLO 5
#define I2CSTATE_ADDRBIT_SCLHI 6
#define I2CSTATE_ADDRACK_SCLLO 7
#define I2CSTATE_ADDRACK_SCLHI 8
#define I2CSTATE_DATABIT_SCLLO 9
#define I2CSTATE_DATABIT_SCLHI 10
#define I2CSTATE_DATAACK_SCLLO 11
#define I2CSTATE_DATAACK_SCLHI 12

#define TIME_COUNT (1000UL * 1000UL)

uint32_t time_lpms;

void time_init()
{
  uint32_t t0 = micros();
  for(volatile uint32_t i = 0; i < TIME_COUNT; i++) ;
  uint32_t t1 = micros();
  uint32_t dt = t1 - t0;
  time_lpms = 1000 * TIME_COUNT / dt;
}

void time_wait_us(int us)
{
  uint32_t count = time_lpms * us / 1000;
  for(volatile uint32_t i = 0; i < count; i++) ;
}

#pragma GCC optimize ("O3")

void i2c_isr()
{
  uint8_t scl = i2c_scl_read();
  uint8_t sda = i2c_sda_read();

  // START/STOP detector
  if(i2c.state==I2CSTATE_ADDRBIT_SCLHI || i2c.state==I2CSTATE_ADDRACK_SCLHI
    || i2c.state==I2CSTATE_DATABIT_SCLHI || i2c.state==I2CSTATE_DATAACK_SCLHI)
  {
    // SCL was high, is it still? Did SDA change?
    if(scl == 1 && sda != i2c.sdacap)
    {
      // SDA changed while CLK was high
      if(sda == 0)
      {
        // SDA went low during CLK high: (repeated) START
        i2c.state = I2CSTATE_STARTED;
        // log_char('s');
      }
      else
      {
        // SDA went high during CLK high: STOP
        if(SLAVE_REG_ENABLE() > 0 && SLAVE_REG_ENABLE() < 0xFFFF)
          SLAVE_REG_SET_ENABLE(SLAVE_REG_ENABLE() - 1);
        SLAVE_REG_SET_QPULSE(i2c.pulse);
        SLAVE_REG_SET_QUS(micros() - i2c.qus);
        i2c.state = I2CSTATE_IDLE;
        // log_char('p');
        // log_char(']');
      }
    }
  }

  // Normal processing
  switch(i2c.state)
  {
  case I2CSTATE_UNKNOWN:
    if(scl == 1 && sda == 1)
    {
      i2c.state = I2CSTATE_IDLE;
    }
    else
    {
      // log_char('!'); // Log error state
      i2c.state= I2CSTATE_ERROR;
    }
    break;

  case I2CSTATE_ERROR:
    if(scl == 1 && sda == 1)
    {
      i2c.state= I2CSTATE_IDLE;
    }
    break;

  case I2CSTATE_IDLE:
    // SCL and SDA both high ("idle state")
    if(scl == 1)
    {
      if(sda == 1)
      {
        // SCL=1, SDA=1: Stay in IDLE
      }
      else
      {
        // SDA went low, while SCL stays high
        // log_char('['); // Absolute start (not repeated)
        // log_char('s');
        i2c.pulse = 0;
        i2c.qus = micros();
        i2c.addr = 0;
        i2c.state = I2CSTATE_STARTED;
      }
    }
    else
    {
      // log_char('!'); // Log error state
      i2c.state = I2CSTATE_ERROR;
    }
    break;

  case I2CSTATE_STARTED:
    // SCL was high, SDA was low; but SCL must go down and SDA must be first bit of addr
    if(scl == 0)
    {
      i2c.pulse++;
      if(i2c.pulse == SLAVE_REG_PULSE() && SLAVE_REG_ENABLE() > 0)
      {
        i2c_scl_lo();
        // log_char('_');
        time_wait_us(SLAVE_REG_US());
        i2c_scl_hi();
      }
      i2c.data = 0;
      i2c.bitcnt = 0;
      i2c.logbits = (i2c.pulse < SLAVE_REG_PULSE())
        && (SLAVE_REG_PULSE() < i2c.pulse + 8)
        && (SLAVE_REG_ENABLE() > 0);
      i2c.state = I2CSTATE_ADDRBIT_SCLLO;
    }
    break;

  case I2CSTATE_ADDRBIT_SCLLO:
    // Reading address bits. SCL is low.
    if(scl == 1)
    {
      // SCL went high, so SDA has data (if it stays like this the full CLK period)
      i2c.sdacap = sda;
      i2c.state = I2CSTATE_ADDRBIT_SCLHI;
    }
    break;

  case I2CSTATE_ADDRBIT_SCLHI:
    // Reading address bits. SCL is high.
    if(scl == 0)
    {
      // SCL went low, so we have a complete address bit
      i2c.pulse++;
      // if(i2c.logbits)
      //   log_char(i2c.sdacap + '0');
      if(i2c.sdacap)
        i2c.data |= 1 << (7 - i2c.bitcnt);
      i2c.bitcnt++;
      if(i2c.bitcnt < 8)
      {
        i2c.state= I2CSTATE_ADDRBIT_SCLLO;
      }
      else
      {
        // Received a complete address byte (but not yet the ack)
        i2c.addr = i2c.data;
        if(i2c.addr == SLAVE_ADDR || i2c.addr == SLAVE_ADDR + 1)
        {
          // Pull down to ACK the ADDR
          i2c_sda_lo();
        }
        // if(i2c.logbits)
        //   log_char('/');
        // log_byte(i2c.data);
        i2c.state = I2CSTATE_ADDRACK_SCLLO;
      }
      if(i2c.pulse == SLAVE_REG_PULSE() && SLAVE_REG_ENABLE() > 0)
      {
        i2c_scl_lo();
        // log_char('_');
        time_wait_us(SLAVE_REG_US());
        i2c_scl_hi();
      }
    }
    break;

  case I2CSTATE_ADDRACK_SCLLO:
    // Reading address ack bit. SCL is low.
    if(scl == 1)
    {
      // SCL went high, so SDA has data (if it stays like this the full CLK period)
      i2c.sdacap = sda;
      i2c.state = I2CSTATE_ADDRACK_SCLHI;
    }
    break;

  case I2CSTATE_ADDRACK_SCLHI:
    // Reading address ack bit. SCL is high.
    if(scl == 0)
    {
      // SCL went low, so we have a complete address ack bit
      i2c.pulse++;
      if(i2c.addr == SLAVE_ADDR)
      {
        // Release ACK of ADDR
        i2c_sda_hi();
      }
      else if(i2c.addr == SLAVE_ADDR + 1)
      {
        // We need to release ACK of ADDR, but also push out the first data bit
        uint8_t val = (0 <= slave_cra && slave_cra < SLAVE_REGCOUNT) ? slave_regs[slave_cra] : 0x55;
        int mask = 1 << 7;
        if(val & mask)
        {
          i2c_sda_hi();
        }
        else
        {
          i2c_sda_lo();
        }
      }
      // if(i2c.sdacap)
      //   log_char('n');
      // else
      //   log_char('a');
      // log_char(' '); // Space after address
      if(i2c.pulse == SLAVE_REG_PULSE() && SLAVE_REG_ENABLE() > 0)
      {
        i2c_scl_lo();
        // log_char('_');
        time_wait_us(SLAVE_REG_US());
        i2c_scl_hi();
      }
      i2c.bitcnt = 0;
      i2c.logbits = (i2c.pulse < SLAVE_REG_PULSE())
        && (SLAVE_REG_PULSE() < i2c.pulse + 8)
        && (SLAVE_REG_ENABLE() > 0);
      i2c.data = 0;
      i2c.bytecnt = 0;
      i2c.state = I2CSTATE_DATABIT_SCLLO;
    }
    break;

  case I2CSTATE_DATABIT_SCLLO:
    // Reading data bits. SCL is low.
    if(scl == 1)
    {
      // SCL went high, so SDA has data (if it stays like this the full CLK period)
      i2c.sdacap = sda;
      i2c.state = I2CSTATE_DATABIT_SCLHI;
    }
    break;

  case I2CSTATE_DATABIT_SCLHI:
    // Reading data bits. SCL is high.
    if(scl == 0)
    {
      // SCL went low, so we have a complete data bit
      i2c.pulse++;
      if(i2c.sdacap)
        i2c.data |= 1 << (7 - i2c.bitcnt);
      // if(i2c.logbits)
      //   log_char(i2c.sdacap + '0');
      i2c.bitcnt++;
      if(i2c.bitcnt < 8)
      {
        // Push next bit out
        if(i2c.addr == SLAVE_ADDR + 1)
        {
          // Get bits from register pointed to by CRA
          uint8_t val= (0 <= slave_cra && slave_cra < SLAVE_REGCOUNT) ? slave_regs[slave_cra] : 0x55;
          int mask= 1 << (7 - i2c.bitcnt);
          if(val & mask)
          {
            i2c_sda_hi();
          }
          else
          {
            i2c_sda_lo();
          }
        }
        i2c.state = I2CSTATE_DATABIT_SCLLO;
      }
      else
      {
        // Received a complete data byte (but not yet the ack)
        if(i2c.addr == SLAVE_ADDR)
        {
          // Pull down to ACK the ADDR
          i2c_sda_lo();
          if(i2c.bytecnt == 0)
          {
            // First data byte goes to CRA ...
            slave_cra = i2c.data;
          }
          else
          {
            // ... all other data bytes go to the register the CRA points to and the CRA is stepped ("auto increment")
            if(0 <= slave_cra && slave_cra < SLAVE_REGCOUNT)
              slave_regs[slave_cra] = i2c.data;
            slave_cra++;
          }
        }
        else if(i2c.addr == SLAVE_ADDR + 1)
        {
          // Release a (potential) pull down for the "previous" data bit
          i2c_sda_hi();
        }
        // if(i2c.logbits)
        //   log_char('/');
        // log_byte(i2c.data);
        i2c.state = I2CSTATE_DATAACK_SCLLO;
      }
      if(i2c.pulse == SLAVE_REG_PULSE() && SLAVE_REG_ENABLE() > 0)
      {
        i2c_scl_lo();
        // log_char('_');
        time_wait_us(SLAVE_REG_US());
        i2c_scl_hi();
      }
    }
    break;

  case I2CSTATE_DATAACK_SCLLO:
    // Reading data ack bit. SCL is low.
    if(scl == 1)
    {
      // SCL went high, so SDA has data (if it stays like this the full CLK period)
      i2c.sdacap = sda;
      i2c.state = I2CSTATE_DATAACK_SCLHI;
    }
    break;

  case I2CSTATE_DATAACK_SCLHI:
    // Reading data ack bit. SCL is high.
    if(scl == 0)
    {
      // SCL went low, so we have a complete data ack bit
      i2c.pulse++;
      if(i2c.addr == SLAVE_ADDR)
      {
        // Master is writing to us, we just acked the received data byte
        // Release ACK of DATA
        i2c_sda_hi();
      }
      else if(i2c.addr == SLAVE_ADDR + 1)
      {
        // Master is reading from us, the master flagged with the ack if more data bytes are needed
        // One complete byte read, move to next
        slave_cra++;
        if(i2c.sdacap)
        {
          // Master set ack bit to 1 (nack), no more bytes needed
          // Release ACK of DATA (and do not push out first bit of next data byte)
          i2c_sda_hi();
        }
        else if(i2c.addr == SLAVE_ADDR + 1)
        {
          // We need to release ACK of DATA, but also push out the first data bit of the next byte
          uint8_t val = (0 <= slave_cra && slave_cra < SLAVE_REGCOUNT) ? slave_regs[slave_cra] : 0x55;
          int mask = 1 << 7;
          if(val & mask)
          {
            i2c_sda_hi();
          }
          else
          {
            i2c_sda_lo();
          }
        }
      }
      // if(i2c.sdacap)
      //   log_char('n');
      // else
      //   log_char('a');
      // log_char(' '); // Space after data byte
      if(i2c.pulse == SLAVE_REG_PULSE() && SLAVE_REG_ENABLE() > 0)
      {
        i2c_scl_lo();
        // log_char('_');
        time_wait_us(SLAVE_REG_US());
        i2c_scl_hi();
      }
      i2c.bitcnt = 0;
      i2c.logbits = (i2c.pulse < SLAVE_REG_PULSE())
        && (SLAVE_REG_PULSE() < i2c.pulse + 8)
        && (SLAVE_REG_ENABLE() > 0);
      i2c.data = 0;
      i2c.bytecnt++;
      i2c.state = I2CSTATE_DATABIT_SCLLO;
    }
    break;

  default:
    // log_byte(i2c.state);
    // log_char('x'); // Unknown state
    break;
  }
}

I2CTool::I2CTool(uint8_t sdaPin, uint8_t sclPin, boolean pullups, boolean detectClockStretch) :
    SoftwareWire(sdaPin, sclPin, pullups, detectClockStretch)
{
  time_init();

  attachInterrupt(digitalPinToInterrupt(_sdaPin), i2c_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_sclPin), i2c_isr, CHANGE);

  if(i2c_scl_read() == 1 && i2c_sda_read() == 1)
    state = I2CSTATE_IDLE;
  else
    state = I2CSTATE_ERROR;
}
