

// Constructors ////////////////////////////////////////////////////////////////
/*
  LIS3MDL(void)
  {
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
  }
*/
// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool lis3mdltimeoutOccurred()
{
  bool tmp = lis3mdldid_timeout;
  lis3mdldid_timeout = false;
  return tmp;
}

void lis3mdlsetTimeout(uint16_t timeout)
{
  lis3mdlio_timeout = timeout;
}

uint16_t lis3mdlgetTimeout()
{
  return lis3mdlio_timeout;
}

bool lis3mdlinit(lis3mdldeviceType device, sa1State sa1)
{
  // perform auto-detection unless device type and SA1 state were both specified
  if (device == lis3mdldevice_auto || sa1 == sa1_auto)
  {
    // check for LIS3MDL if device is unidentified or was specified to be this type
    if (device == lis3mdldevice_auto || device == device_LIS3MDL)
    {
      // check SA1 high address unless SA1 was specified to be low
      if (sa1 != sa1_low && lis3mdltestReg(LIS3MDL_SA1_HIGH_ADDRESS, LIS3MDL_WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_high;
        if (device == lis3mdldevice_auto) {
          device = device_LIS3MDL;
        }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && lis3mdltestReg(LIS3MDL_SA1_LOW_ADDRESS, LIS3MDL_WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == lis3mdldevice_auto) {
          device = device_LIS3MDL;
        }
      }
    }

    // make sure device and SA1 were successfully detected; otherwise, indicate failure
    if (device == lis3mdldevice_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }

  _lis3mdldevice = device;

  switch (device)
  {
    case device_LIS3MDL:
      lis3mdaddress = (sa1 == sa1_high) ? LIS3MDL_SA1_HIGH_ADDRESS : LIS3MDL_SA1_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
  Enables the LIS3MDL's magnetometer. Also:
  - Selects ultra-high-performance mode for all axes
  - Sets ODR (output data rate) to default power-on value of 10 Hz
  - Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
  - Enables continuous conversion mode
  Note that this function will also reset other settings controlled by
  the registers it writes to.
*/
void lis3mdlenableDefault(void)
{
  if (_lis3mdldevice == device_LIS3MDL)
  {
    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    lis3mdlwriteReg(CTRL_REG1, 0x70);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    lis3mdlwriteReg(CTRL_REG2, 0x00);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    lis3mdlwriteReg(CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    lis3mdlwriteReg(CTRL_REG4, 0x0C);
  }
}

// Writes a mag register
void lis3mdlwriteReg(uint8_t reg, uint8_t value)
{
//  I2c.beginTransmission(lis3mdaddress);
  I2c.write(lis3mdaddress, reg);
  I2c.write(lis3mdaddress, value);
 // lis3mdllast_status = I2c.endTransmission();
}

// Reads a mag register
uint8_t lis3mdlreadReg(uint8_t reg)
{
  uint8_t value;

//  I2c.beginTransmission(lis3mdaddress);
  I2c.write(lis3mdaddress, reg);
//  lis3mdllast_status = I2c.endTransmission();
//  I2c.requestFrom(lis3mdaddress, (uint8_t)1);
  value = I2c.receive();
//  I2c.endTransmission();

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void lis3mdlread()
{
//  I2c.beginTransmission(lis3mdaddress);
  Serial.println("begintransmission in read");

  // assert MSB to enable subaddress updating
  I2c.write(lis3mdaddress, OUT_X_L | 0x80);
    Serial.println("write1 in read");

//  I2c.endTransmission();
    Serial.println("endtransmission in read");

  //I2c.requestFrom(lis3mdaddress, (uint8_t)6);
    Serial.println("request in read");


  uint16_t millis_start = millis();
  while (I2c.available() < 6)
  {
    if (lis3mdlio_timeout > 0 && ((uint16_t)millis() - millis_start) > lis3mdlio_timeout)
    {
      lis3mdldid_timeout = true;
      return;
    }
    Serial.println("I2cavailable()");
  }
  Serial.println("avalible in read");

  uint8_t xlm = I2c.receive();
  uint8_t xhm = I2c.receive();
  uint8_t ylm = I2c.receive();
  uint8_t yhm = I2c.receive();
  uint8_t zlm = I2c.receive();
  uint8_t zhm = I2c.receive();

  // combine high and low bytes
  lis3mdlm.x = (int16_t)(xhm << 8 | xlm);
  lis3mdlm.y = (int16_t)(yhm << 8 | ylm);
  lis3mdlm.z = (int16_t)(zhm << 8 | zlm);
}



// Private Methods //////////////////////////////////////////////////////////////

int16_t lis3mdltestReg(uint8_t address, lis3mdlregAddr reg)
{
  //I2c.beginTransmission(address);
  if(I2c.write(address, (uint8_t)reg) != 0)
  {
  //if (I2c.endTransmission() != 0)
  //{
    return LIS3MDL_TEST_REG_ERROR;
  }

  I2c.read(address, (uint8_t)1);
  if (I2c.available())
  {
    return I2c.receive();
  }
  else
  {
    return LIS3MDL_TEST_REG_ERROR;
  }
}
