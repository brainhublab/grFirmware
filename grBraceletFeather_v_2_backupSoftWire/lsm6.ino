// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
/*
bool lsm6timeoutOccurred()
{
  bool tmp = lsm6did_timeout;
  lsm6did_timeout = false;
  return tmp;
}

void lsm6setTimeout(uint16_t timeout)
{
  lsm6io_timeout = timeout;
}

uint16_t lsm6getTimeout()
{
  return lsm6io_timeout;
}

bool lsm6init(lsm6deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == lsm6device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == lsm6device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && lsm6testReg(DS33_SA0_HIGH_ADDRESS, LSM6_WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == lsm6device_auto) {
          device = device_DS33;
        }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && lsm6testReg(DS33_SA0_LOW_ADDRESS, LSM6_WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == lsm6device_auto) {
          device = device_DS33;
        }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == lsm6device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _lsm6device = device;

  switch (device)
  {
    case device_DS33:
      lsm6address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
  }

  return true;
}
*/
/*
  Enables the LSM6's accelerometer and gyro. Also:
  - Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
  - Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
  - Enables automatic increment of register address during multiple byte access
  Note that this function will also reset other settings controlled by
  the registers it writes to.
*/
/*
void lsm6enableDefault(void)
{
  if (_lsm6device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    lsm6writeReg(CTRL1_XL, 0x80);

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    lsm6writeReg(CTRL2_G, 0x80);

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    lsm6writeReg(CTRL3_C, 0x04);
  }
}

void lsm6writeReg(uint8_t reg, uint8_t value)
{
  I2c.beginTransmission(lsm6address);
  I2c.send(reg);
  I2c.send(value);
  lsm6last_status = I2c.endTransmission();
}

uint8_t lsm6readReg(uint8_t reg)
{
  uint8_t value;

  I2c.beginTransmission(lsm6address);
  I2c.send(reg);
  lsm6last_status = I2c.endTransmission();
  I2c.requestFrom(lsm6address, (uint8_t)1);
  value = I2c.receive();
  I2c.endTransmission();

  return value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void lsm6readAcc(void)
{
  I2c.beginTransmission(lsm6address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  I2c.write(OUTX_L_XL);
  I2c.endTransmission();
  I2c.requestFrom(lsm6address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (I2c.available() < 6) {
    if (lsm6io_timeout > 0 && ((uint16_t)millis() - millis_start) > lsm6io_timeout)
    {
      lsm6did_timeout = true;
      return;
    }
  }

  uint8_t xla = I2c.receive();
  uint8_t xha = I2c.receive();
  uint8_t yla = I2c.receive();
  uint8_t yha = I2c.receive();
  uint8_t zla = I2c.receive();
  uint8_t zha = I2c.receive();

  // combine high and low bytes
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 gyro channels and stores them in vector g
void lsm6readGyro(void)
{
  I2c.beginTransmission(lsm6address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  I2c.write(OUTX_L_G);
  I2c.endTransmission();
  I2c.requestFrom(lsm6address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (I2c.available() < 6) {
    if (lsm6io_timeout > 0 && ((uint16_t)millis() - millis_start) > lsm6io_timeout)
    {
      lsm6did_timeout = true;
      return;
    }
  }

  uint8_t xlg = I2c.receive();
  uint8_t xhg = I2c.receive();
  uint8_t ylg = I2c.receive();
  uint8_t yhg = I2c.receive();
  uint8_t zlg = I2c.receive();
  uint8_t zhg = I2c.receive();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void lsm6read(void)
{
  lsm6readAcc();
  lsm6readGyro();
}


// Private Methods //////////////////////////////////////////////////////////////

int16_t lsm6testReg(uint8_t address, lsm6regAddr reg)
{
  I2c.beginTransmission(address);
  I2c.write((uint8_t)reg);
  if (I2c.endTransmission() != 0)
  {
    return LSM6_TEST_REG_ERROR;
  }

  I2c.requestFrom(address, (uint8_t)1);
  if (I2c.available())
  {
    return I2c.receive();
  }
  else
  {
    return LSM6_TEST_REG_ERROR;
  }
}
*/
