#include <Arduino.h>
#include <Wire.h>
#include "hmc5883l.h"


static float _hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

void hmc5883l::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

byte hmc5883l::read8(byte address,byte reg)
{
    byte value;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(address,(byte)1);
    value=Wire.read();
    Wire.endTransmission();
    return value;
}

magnetic_t hmc5883l:: getReading()
{
    magnetic_t result;
    Wire.beginTransmission((byte)HMC5883_ADDRESS_MAG);
    Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
    Wire.endTransmission();
    Wire.requestFrom((byte)HMC5883_ADDRESS_MAG,(byte)6);

    while (Wire.available() < 6);

    uint8_t xhi=Wire.read();
    uint8_t xlo=Wire.read();
    uint8_t zhi=Wire.read();
    uint8_t zlo=Wire.read();
    uint8_t yhi=Wire.read();
    uint8_t ylo=Wire.read();

    result.x = (int16_t)(xlo | ((int16_t)xhi << 8));
    result.y = (int16_t)(ylo | ((int16_t)yhi << 8));
    result.z = (int16_t)(zlo | ((int16_t)zhi << 8));

    return result;

}


bool hmc5883l::begin()
{
  // Enable I2C
  Wire.begin(32,33);

  // Enable the magnetometer
  write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

  // Set the gain to a known level
  setMagGain(HMC5883_MAGGAIN_1_3);

  return true;
}

void hmc5883l::setMagGain(hmc5883MagGain gain) {
  write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte)gain);

  _magGain = gain;

  switch (gain) {
  case HMC5883_MAGGAIN_1_3:
    _hmc5883_Gauss_LSB_XY = 1100;
    _hmc5883_Gauss_LSB_Z = 980;
    break;
  case HMC5883_MAGGAIN_1_9:
    _hmc5883_Gauss_LSB_XY = 855;
    _hmc5883_Gauss_LSB_Z = 760;
    break;
  case HMC5883_MAGGAIN_2_5:
    _hmc5883_Gauss_LSB_XY = 670;
    _hmc5883_Gauss_LSB_Z = 600;
    break;
  case HMC5883_MAGGAIN_4_0:
    _hmc5883_Gauss_LSB_XY = 450;
    _hmc5883_Gauss_LSB_Z = 400;
    break;
  case HMC5883_MAGGAIN_4_7:
    _hmc5883_Gauss_LSB_XY = 400;
    _hmc5883_Gauss_LSB_Z = 255;
    break;
  case HMC5883_MAGGAIN_5_6:
    _hmc5883_Gauss_LSB_XY = 330;
    _hmc5883_Gauss_LSB_Z = 295;
    break;
  case HMC5883_MAGGAIN_8_1:
    _hmc5883_Gauss_LSB_XY = 230;
    _hmc5883_Gauss_LSB_Z = 205;
    break;
  }
}