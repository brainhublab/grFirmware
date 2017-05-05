#include <LSM6.h>
#include <LIS3MDL.h>

struct sens
{
  uint8_t gyroUUID[16];
  uint8_t gyroId;
  uint8_t accUUID[16];
  uint8_t accId;
  uint8_t magUUID[16];
  uint8_t magId;

  LIS3MDL compass; //TODO need to be mag
  LSM6 gyro;
};

typedef sens sensor;

