#include "SoftwareWire.h"

#define i2c_sda_lo()              \
  *_sdaPortReg &= ~_sdaBitMask;   \
  *_sdaDirReg  |=  _sdaBitMask;

#define i2c_scl_lo()              \
  *_sclPortReg &= ~_sclBitMask;   \
  *_sclDirReg  |=  _sclBitMask;

#define i2c_sda_hi()              \
  *_sdaDirReg &= ~_sdaBitMask;    \
  if(_pullups) { *_sdaPortReg |= _sdaBitMask; }

#define i2c_scl_hi()              \
  *_sclDirReg &= ~_sclBitMask;    \
  if(_pullups) { *_sclPortReg |= _sclBitMask; }

#define i2c_sda_read()   ((uint8_t) (*_sdaPinReg & _sdaBitMask) ? 1 : 0)
#define i2c_scl_read()   ((uint8_t) (*_sclPinReg & _sclBitMask) ? 1 : 0)

static const uint8_t IIC_ZERO      = 0x00;  // Rising edge of clock with data = 0
static const uint8_t IIC_START     = 0x10;  // Data going low while clock remains high
static const uint8_t IIC_STOP      = 0x20;  // Data going high while clock remains high
static const uint8_t IIC_TIMEOUT   = 0xFF;  // ANY STATIC CONDITION THAT EXCEEDS THE TIMEOUT TIME

static const uint8_t IIC_RWMASK    = 0x01;  //Byte to AND with address for read/write bit
static const uint8_t IIC_READ      = 0x01;  //Byte to OR with address for read start and read restart
static const uint8_t IIC_WRITE     = 0x00;  //Byte to OR with address for write start and write restart

#define RETVAL_SUCCESS          IIC_ZERO     // The function has exited successfully.
#define RETVAL_UNEXPECTED_START IIC_START    // During operation, the bus encountered an unexpected start condition.
#define RETVAL_UNEXPECTED_STOP  IIC_STOP     // During operation, the bus encountered an unexpected stop condition.
#define RETVAL_TIMEOUT          IIC_TIMEOUT  // The bus timed out within the function.

#pragma GCC optimize ("O3")

SoftIIC::~SoftIIC()
{
  end();
}

uint8_t SoftIIC::StateIdle()
{
  return ((IIC_STATE_PREVIOUS == _busBitMask) & (IIC_STATE == _busBitMask));
}

uint8_t SoftIIC::StateStart()
{
  return ((IIC_STATE_PREVIOUS == _busBitMask) & (IIC_STATE == _sclBitMask));
}

uint8_t SoftIIC::StateStop()
{
  return ((IIC_STATE_PREVIOUS == _sclBitMask) & (IIC_STATE == _busBitMask));
}

uint8_t SoftIIC::StateData()
{
  return (((~IIC_STATE_PREVIOUS) & IIC_STATE) & _sclBitMask);
}

uint8_t SoftIIC::StateClockFell()
{
  return (((~IIC_STATE) & IIC_STATE_PREVIOUS) & _sclBitMask);
}

uint8_t SoftIIC::StateDataBit()
{
  return (IIC_STATE & (_sdaBitMask));
}

uint8_t SoftIIC::StateClockLow()
{
  return (~(IIC_STATE & _sclBitMask));
}

uint8_t SoftIIC::StateClockHigh()
{
  return ((IIC_STATE & _sclBitMask));
}

// TODO: verify that pins are on the same port
void SoftIIC::bus_read()
{
  SoftIIC::bus_read_same_port();
}

void SoftIIC::bus_read_same_port()
{
  IIC_STATE_PREVIOUS = IIC_STATE;
  IIC_STATE = *_sclPinReg;
  IIC_STATE &= _busBitMask;
}

uint8_t SoftIIC::spin_until_start(){
  uint8_t number_overflows = 255;
  unsigned long prevMillis = millis();

  while(number_overflows > 0)
  {
    SoftIIC::bus_read();
    if(SoftIIC::StateStart())
      return RETVAL_SUCCESS;

    if(millis() - prevMillis >= _timeout)
    {
      prevMillis = millis();
      number_overflows--;
    }
  }

  return IIC_TIMEOUT;
}

uint8_t SoftIIC::spin_until_clock_rises()
{
  unsigned long prevMillis = millis();

  while(millis() - prevMillis < _timeout)
  {
    SoftIIC::bus_read();
    if(SoftIIC::StateClockHigh())
      return RETVAL_SUCCESS;
    SoftIIC::bus_read();
    if(SoftIIC::StateClockHigh())
      return RETVAL_SUCCESS;
  }

  return IIC_TIMEOUT;
}

uint8_t SoftIIC::spin_until_clock_falls()
{
  unsigned long prevMillis = millis();

  while(millis() - prevMillis < _timeout)
  {
    if(i2c_scl_read() == 0)
      return RETVAL_SUCCESS;
    if(i2c_scl_read() == 0)
      return RETVAL_SUCCESS;
  }

  return IIC_TIMEOUT;
}

uint8_t SoftIIC::get_byte(uint8_t *value, uint8_t (*my_ack_function)(uint8_t rxbyte))
{
  uint8_t my_bit = 7;
  unsigned long prevMillis = millis();

  if(SoftIIC::spin_until_clock_falls())
    goto get_byte_failure;

  i2c_sda_hi();
  IIC_STATE = 0x00;

getting_a_bit:
    SoftIIC::bus_read();
    if(SoftIIC::StateData())
    {
      *value |= ((SoftIIC::StateDataBit()) > 0);
      if(my_bit == 0x00)
        goto getbyte_handle_ack;
      *value = *value << 1;
      my_bit--;
      goto getting_a_bit; // If successful, skip the safety checks and get the next bit.
    }

    if(SoftIIC::StateStart())
      goto get_byte_unexpected_start;
    if(SoftIIC::StateStop())
      goto get_byte_unexpected_stop;
    if(millis() - prevMillis >= _timeout)
      goto get_byte_failure;

  goto getting_a_bit;

getbyte_handle_ack:
  if(SoftIIC::spin_until_clock_falls())
    goto get_byte_failure;
  i2c_scl_lo();
  if(my_ack_function(*value) == 0)
  {
    i2c_sda_hi();
    i2c_scl_hi();
    goto get_byte_unexpected_stop;
  }
  i2c_sda_lo();
  i2c_scl_hi();
  if(SoftIIC::spin_until_clock_rises())
    goto get_byte_failure;

  return RETVAL_SUCCESS;

get_byte_failure:
  return RETVAL_TIMEOUT;

get_byte_unexpected_start:
  return RETVAL_UNEXPECTED_START;

get_byte_unexpected_stop:
  return RETVAL_UNEXPECTED_STOP;
}

uint8_t SoftIIC::set_byte(uint8_t (*my_get_byte_function)(uint8_t *txbyte))
{
  uint8_t my_bit = 0b10000000;
  uint8_t value = 0;
  unsigned long prevMillis = millis();

setting_a_bit:
  SoftIIC::bus_read();
  if(SoftIIC::StateClockFell())
  {
    if(my_bit == 0b10000000) {
      i2c_scl_lo();
      if(my_get_byte_function(&value) > 0) {
        i2c_scl_hi();
        goto set_byte_failure;
      }
    }
    if(value & my_bit) {
      i2c_sda_hi();
    } else {
      i2c_sda_lo();
    }
    if(my_bit == 0b10000000) {
      i2c_scl_hi();
    }
    my_bit = my_bit >> 1;
    if(my_bit == 0x00)
      goto getting_an_ack;
    goto setting_a_bit;
  }
  // These checks unfortunately have to be bypassed for performance reasons.
  // if(SoftIIC::StateStart()) goto set_byte_failure;
  // if(SoftIIC::StateStop()) goto set_byte_failure;
  if(millis() - prevMillis >= _timeout)
    goto set_byte_failure;
  goto setting_a_bit;

getting_an_ack:
  if(SoftIIC::spin_until_clock_rises())
    goto set_byte_failure;
  if(SoftIIC::spin_until_clock_falls())
    goto set_byte_failure;
  i2c_sda_hi();
  if(SoftIIC::spin_until_clock_rises())
    goto set_byte_failure;
  return (SoftIIC::StateDataBit());

set_byte_failure:
    SoftIIC::spin_until_clock_falls();
    i2c_sda_hi();
    return IIC_TIMEOUT;
}

uint8_t SoftIIC::SlaveHandleTransaction(
  uint8_t (*fp_respond_to_address)(uint8_t chip_address),
  uint8_t (*fp_respond_to_data)(uint8_t value),
  uint8_t (*fp_generate_byte)(uint8_t *value)
)
{
  uint8_t tretval;
  uint8_t chipddr_has_been_set = 0;
  uint8_t chip_address = 0x00;
  uint8_t rwbit = 0;
  uint8_t value = 0x00;
  uint8_t (*my_ack_function)(uint8_t) = fp_respond_to_address;

beginning:
  SoftIIC::bus_read();
  SoftIIC::bus_read();

waiting_for_start:
  tretval = SoftIIC::spin_until_start();
  if(tretval == RETVAL_TIMEOUT)
    goto done_with_iic_transaction;
  if(tretval != RETVAL_SUCCESS)
    goto waiting_for_start;

getting_a_byte:
  tretval = SoftIIC::get_byte(&value, my_ack_function);
  switch(tretval)
  {
    case RETVAL_SUCCESS:
      if(chipddr_has_been_set == 0)
      {
        chip_address = value >> 1;
        rwbit = value & IIC_RWMASK;
        chipddr_has_been_set = 1;
        my_ack_function = fp_respond_to_data;
        if(rwbit == IIC_READ)
          goto sending_a_byte;
        else
          goto getting_a_byte;
      }
      else
      {
        goto getting_a_byte;
      }
      break;
    case RETVAL_UNEXPECTED_START:
      goto slave_unexpected_start;
      break;
    case RETVAL_UNEXPECTED_STOP:
      goto slave_unexpected_stop;
      break;
    case RETVAL_TIMEOUT:
      goto done_with_iic_transaction;
      break;
    default:
      goto done_with_iic_transaction;
      break;
  }

sending_a_byte:
  tretval = SoftIIC::set_byte(fp_generate_byte);
  switch(tretval)
  {
    case RETVAL_SUCCESS:
      goto sending_a_byte;
      break;
    case RETVAL_UNEXPECTED_START:
      goto slave_unexpected_start;
      break;
    case RETVAL_UNEXPECTED_STOP:
      goto slave_unexpected_stop;
      break;
    case RETVAL_TIMEOUT:
      goto done_with_iic_transaction;
      break;
    default:
      goto done_with_iic_transaction;
      break;
  }

slave_unexpected_start:
  chipddr_has_been_set = 0;
  my_ack_function = fp_respond_to_address;
  goto getting_a_byte;

slave_unexpected_stop:
  goto slave_unexpected_start;

done_with_iic_transaction:
  i2c_scl_hi();
  i2c_sda_hi();

  return tretval;
}
