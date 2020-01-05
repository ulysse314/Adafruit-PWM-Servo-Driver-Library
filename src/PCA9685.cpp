/*!
 *  @file PCA9685.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "PCA9685.h"

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
PCA9685::PCA9685(uint8_t addr, TwoWire *i2c) :
    _i2caddr(addr),
    _i2c(i2c) {
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
bool PCA9685::begin(uint8_t prescale) {
  if (!reset()) {
    return false;
  }
  bool result = true;
  if (prescale) {
    setExtClk(prescale);
  } else {
    // set a default frequency
    result = setPWMFreq(1000);
  }
  // set the default internal frequency
  setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  return result;
}
/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
bool PCA9685::reset() {
  if (!write8(PCA9685_MODE1, MODE1_RESTART)) {
    return false;
  }
  return true;
}

/*!
 *  @brief  Puts board into sleep mode
 */
bool PCA9685::sleep() {
  uint8_t awake = 0;
  if (!read8(PCA9685_MODE1, &awake)) {
    return false;
  }
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  bool result = write8(PCA9685_MODE1, sleep);
  delay(5); // wait until cycle ends for sleep to be active
  return result;
}

/*!
 *  @brief  Wakes board from sleep
 */
bool PCA9685::wakeup() {
  uint8_t sleep = 0;
  if (!read8(PCA9685_MODE1, &sleep)) {
    return false;
  }
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  return write8(PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
bool PCA9685::setExtClk(uint8_t prescale) {
  uint8_t oldmode = 0;
  if (!read8(PCA9685_MODE1, &oldmode)) {
    return false;
  }
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  if (!write8(PCA9685_MODE1, newmode)) { // go to sleep, turn off internal oscillator
    return false;
  }

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  if (!write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK))) {
    return false;
  }

  if (!write8(PCA9685_PRESCALE, prescale)) { // set the prescaler
    return false;
  }

  delay(5);
  // clear the SLEEP bit to start
  if (!write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI)) {
    return false;
  }

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  uint8_t value = 0;
  read8(PCA9685_MODE1, &value);
  Serial.println(value, HEX);
#endif
  return true;
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
bool PCA9685::setPWMFreq(float freq) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif
  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: ");
  Serial.println(prescale);
#endif

  uint8_t oldmode = 0;
  if (!read8(PCA9685_MODE1, &oldmode)) {
    return false;
  }
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  if (!write8(PCA9685_MODE1, newmode)) {                             // go to sleep
    return false;
  }
  if (!write8(PCA9685_PRESCALE, prescale)) { // set the prescaler
    return false;
  }
  if (!write8(PCA9685_MODE1, oldmode)) {
    return false;
  }
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  if (!write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI)) {
    return false;
  }

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  uint8_t value = 0;
  read8(PCA9685_MODE1, &value);
  Serial.println(value, HEX);
#endif
  return true;
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
bool PCA9685::setOutputMode(bool totempole) {
  uint8_t oldmode = 0;
  if (!read8(PCA9685_MODE2, &oldmode)) {
    return false;
  }
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
  return write8(PCA9685_MODE2, newmode);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
bool PCA9685::readPrescale(uint8_t *prescale) {
  return read8(PCA9685_PRESCALE, prescale);
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
bool PCA9685::getPWM(uint8_t num, uint8_t *pwm) {
  int8_t read = _i2c->requestFrom((int)_i2caddr, PCA9685_LED0_ON_L + 4 * num, (int)4);
  if (read != 1) {
    return false;
  }
  if (pwm) {
    *pwm = _i2c->read();
  }
  return true;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
bool PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif

  _i2c->beginTransmission(_i2caddr);
  _i2c->write(PCA9685_LED0_ON_L + 4 * num);
  _i2c->write(on);
  _i2c->write(on >> 8);
  _i2c->write(off);
  _i2c->write(off >> 8);
  return _i2c->endTransmission() == 0;
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
bool PCA9685::setPin(uint8_t num, uint16_t val, bool invert) {
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  uint16_t on = 0;
  uint16_t off = 0;
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      on = 4096;
      off = 0;
    } else if (val == 4095) {
      // Special value for signal fully off.
      on = 0;
      off = 4096;
    } else {
      on = 0;
      off = 4095 - val;
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
      on = 4096;
      off = 0;
    } else if (val == 0) {
      // Special value for signal fully off.
      on = 0;
      off = 4096;
    } else {
      on = 0;
      off = val;
    }
  }
  return setPWM(num, on, off);
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
bool PCA9685::writeMicroseconds(uint8_t num,
                                                uint16_t Microseconds) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM Via Microseconds on output");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(Microseconds);
  Serial.println("->");
#endif

  double pulse = Microseconds;
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second

  // Read prescale
  uint8_t prescaleValue = 0;
  if (!readPrescale(&prescaleValue)) {
    return false;
  }
  uint16_t prescale = prescaleValue;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(prescale);
  Serial.println(" PCA9685 chip prescale");
#endif

  // Calculate the pulse for PWM based on Equation 1 from the datasheet section
  // 7.3.5
  prescale += 1;
  pulselength *= prescale;
  pulselength /= _oscillator_freq;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulselength);
  Serial.println(" us per bit");
#endif

  pulse /= pulselength;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulse);
  Serial.println(" pulse for PWM");
#endif

  return setPWM(num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t PCA9685::getOscillatorFrequency() {
  return _oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void PCA9685::setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}

/******************* Low level I2C interface */
bool PCA9685::read8(uint8_t addr, uint8_t *returnedValue) const {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  if (_i2c->endTransmission(false) != 0) {
    return false;
  }

  uint8_t read = _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  if (read != 1) {
    return false;
  }
  uint8_t readValue = _i2c->read();
  if (returnedValue) {
    *returnedValue = readValue;
  }
  return true;
}

bool PCA9685::write8(uint8_t addr, uint8_t d) const {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->write(d);
  return _i2c->endTransmission() == 0;
}
