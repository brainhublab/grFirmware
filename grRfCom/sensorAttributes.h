#include <LSM6.h>
#include <LIS3MDL.h>

struct sens
{
  LIS3MDL compass; //TODO need to be mag
  LSM6 gyro;
};

typedef sens sensor;

