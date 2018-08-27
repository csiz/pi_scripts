"""Control servos (PCA9685 on Servo PWM Pi Zero board).


"""

from typing import Tuple
import logging
from asyncio import sleep

from smbus2_asyncio import SMBus2Asyncio


class Servo:
  """Control up to 16 servos using the Servo PWM Pi Zero shield based on the PCA9685 chip.

  Product info: Info https://www.abelectronics.co.uk/p/72/servo-pwm-pi-zero
  Chip manual/datasheet: https://www.abelectronics.co.uk/docs/pdf/pca9685.pdf
  I2C address selection by soldering the pins, range is [0x40, 0x7F]: https://www.abelectronics.co.uk/docs/pdf/datasheet-servopizero.pdf

  Examples:
    Start the servo chip and drive servo 1 to some position.
    >>> servo = Servo(async_smbus)
    >>> await servo.setup()
    >>> await servo.drive(n=1, position=0.2)
  """

  # Base address of the chip.
  ADDRESS = 0x40

  MODE1 = 0x00
  MODE2 = 0x01
  SUBADR1 = 0x02
  SUBADR2 = 0x03
  SUBADR3 = 0x04
  ALLCALLADR = 0x05

  # For each LED n, there are 4 registers LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H.
  LED_BEGIN= 0x06
  LED_COUNT = 16

  # The logic to control the lEDs from the manual.
  # > There will be two 12-bit registers per LED output. These registers will be programmed by
  # > the user. Both registers will hold a value from 0 to 4095. One 12-bit register will hold a
  # > value for the ON time and the other 12-bit register will hold the value for the OFF time. The
  # > ON and OFF times are compared with the value of a 12-bit counter that will be running
  # > continuously from 0000h to 0FFFh (0 to 4095 decimal).
  # >
  # > Update on ACK requires all 4 PWM channel registers to be loaded before outputs will
  # > change on the last ACK.

  # Further bit 13 turns each LED either always on or always off.
  # > The LEDn_ON_H output control bit 4, when set to logic 1, causes the output to be always
  # > ON. The turning ON of the LED is delayed by the amount in the LEDn_ON registers.
  # > LEDn_OFF[11:0] are ignored. When this bit = 0, then the LEDn_ON and LEDn_OFF
  # > registers are used according to their normal definition.
  # >
  # > The LEDn_OFF_H output control bit 4, when set to logic 1, causes the output to be
  # > always OFF. In this case the values in the LEDn_ON registers are ignored.

  ALL_LED_ON_L = 0xFA
  ALL_LED_ON_H = 0xFB
  ALL_LED_OFF_L = 0xFC
  ALL_LED_OFF_H = 0xFD

  PRE_SCALE = 0xFE


  INTERNAL_CLK_RATE = 25_000_000

  TICKS_PER_CYCLE = 0x1000


  SLEEP_BIT = 0b_0001_0000
  AUTO_INCREMENT_BIT = 0b_0010_0000
  RESTART_BIT = 0b_1000_0000

  def __init__(
      self,
      smbus: SMBus2Asyncio,
      address_selection: Tuple[bool] = [False, False, False, False, False],
    ):
    """Initialize the Servo controller; the servo itself must be `setup` before using it!

    Args:
      smbus: Opened async smbus.
      address_selection: Which adress bits are set. The sensor addres range is 0x40-0x7F.

    Members:
      low_limit: Time in miliseconds for the signal to set a servo to the low position.
      high_limit: Miliseconds for high position.
    """

    # Set the I2C bus and address.
    assert isinstance(smbus, SMBus2Asyncio)
    self._bus = smbus
    self.address = Servo.ADDRESS
    for i in range(5):
      self.address += bool(address_selection[i]) << i

    # Modulation frequency used to drive the servos.
    self.modulation_rate = None

    # Servo position limits (miliseconds).
    self.low_limit = 1.0
    self.high_limit = 2.0

  async def _read(self, ptr):
    return await self._bus.read_byte_data(self.address, ptr)

  async def _write(self, ptr, value):
    return await self._bus.write_byte_data(self.address, ptr, value)

  async def _read_block(self, ptr, size):
    return await self._bus.read_i2c_block_data(self.address, ptr, size)

  async def _write_block(self, ptr, data):
    # Split data into 32 byte chunks; because that's what I2C wants.
    for i in range(0, len(data), 32):
      await self._bus.write_i2c_block_data(self.address, ptr+i, data[i:i+32])




  async def setup(self, modulation_rate=50):
    """Setup the servo.

    1. Set the modulation frequency.
    2. Set the auto increment bit; this will allow us to update register in a single i2c call.
    """

    if not 40 <= modulation_rate <= 1000:
      raise ValueError("Servo.setup: modulation_rate must be between 40Hz and 1000Hz.")

    # > The hardware forces a minimum value that can be loaded into the PRE_SCALE register
    # > at ‘3’. The PRE_SCALE register defines the frequency at which the outputs modulate. The
    # > prescale value is determined with the formula shown in Equation 1:
    # >
    # > prescale value = round(osc_clock / 4096 * update_rate) - 1
    # >
    # > where the update rate is the output modulation frequency required.

    scale = int(round(Servo.INTERNAL_CLK_RATE / (Servo.TICKS_PER_CYCLE * modulation_rate)) - 1)
    scaled_rate = Servo.INTERNAL_CLK_RATE / ((scale + 1) * Servo.TICKS_PER_CYCLE)

    # Get the state of mode1, but ignore the restart bit and sleep.
    mode1 = (await self._read(Servo.MODE1)) & ~(Servo.RESTART_BIT | Servo.SLEEP_BIT)
    # Put servo to sleep.
    await self._write(Servo.MODE1, mode1 | Servo.SLEEP_BIT)
    # Now we can write to the pre scale register.
    await self._write(Servo.PRE_SCALE, scale)
    # Restore mode1 without sleep active.
    await self._write(Servo.MODE1, mode1)

    # Restart the PWM channels, need to wait for 5us of active time.
    await sleep(0.005)

    await self._write(Servo.MODE1, mode1 | Servo.RESTART_BIT)

    self.modulation_rate = scaled_rate



    # Enable auto incrementing so we can set all LED registers with a single i2c call.
    await self._write(Servo.MODE1, mode1 | Servo.AUTO_INCREMENT_BIT)



  def _compute_pwm_bytes(self, position):
    if not 0.0 <= position <= 1.0:
      raise ValueError("Servo.drive: position must be between 0 and 1.")

    ms_signal = position * (self.high_limit - self.low_limit) + self.low_limit
    pwm_value = ms_signal * 1e-3 * self.modulation_rate

    # Turn the pwm [0,1) fraction into the 12bit offset needed by the chip.
    pwm_offset = min(max(int(pwm_value * Servo.TICKS_PER_CYCLE), 0x0000), Servo.TICKS_PER_CYCLE-1)

    # Using an ON time of 0, the OFF time is just pwm_offset; compute and
    # return the the the ON_L, ON_H, OFF_L, OFF_H bytes.
    return [0x00, 0x00, pwm_offset & 0xFF, (pwm_offset >> 8) & 0xFF]


  async def drive(self, n, position):
    """Drive servo `n` (1 based indexing as printed on the hat) to `position`."""
    if not 1 <= n <= Servo.LED_COUNT or not isinstance(n, int):
      raise ValueError(f"Servo.drive: n must be the servo index from 1 to {Servo.LED_COUNT} inclusive.")

    await self._write_block(Servo.LED_BEGIN + 4*(n-1), self._compute_pwm_bytes(position))

  async def drive_multiple(self, positions):
    """Drive multiple sensors with a single bus write."""
    if len(positions) > Servo.LED_COUNT:
      raise ValueError(f"Servo.drive_multiple: the number of positions must be at most {Servo.LED_COUNT}.")

    data = []
    for position in positions:
      data += self._compute_pwm_bytes(position)

    await self._write_block(Servo.LED_BEGIN, data)


  async def release(self, n):
    """Release control of servo `n` (1 based indexing)."""
    if not 1 <= n <= Servo.LED_COUNT or not isinstance(n, int):
      raise ValueError(f"Servo.drive: n must be the servo index from 1 to {Servo.LED_COUNT} inclusive.")

    await self._write_block(Servo.LED_BEGIN + 4*(n-1), [0, 0, 0, 0])

  async def release_all(self):
    """Release control of all servos."""
    await self._write_block(Servo.LED_BEGIN, [0]*4*Servo.LED_COUNT)


# Example
# -------

# Continuously oscillate a servo to and fro.

if __name__ == "__main__":
  from . loop_runner import run_tasks

  async def oscillate_servo():
    smbus = SMBus2Asyncio(1)
    await smbus.open()

    servo = Servo(smbus)
    await servo.setup(modulation_rate=50)

    # Yay, cheap servo has a weird range.
    servo.low_limit = 0.7
    servo.high_limit = 2.6

    while True:
      print("Driving to: 0.0", end=", ", flush=True)
      await servo.drive(1, position=0.0)
      await sleep(1.0)
      print("1.0", end=", ", flush=True)
      await servo.drive(1, position=1.0)
      await sleep(1.0)
      print("0.5", end="; ", flush=True)
      await servo.drive(1, position=0.5)
      await sleep(1.0)

      print("pause 3s...")
      await sleep(3.0)

  run_tasks(oscillate_servo())
