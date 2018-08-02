"""Control a gyro and accelerometer (MPU-6050 on GY-521 breakout board).

In a guy's example he does (https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino):
1. Read WHO_AM_I to confirm connection.
2. Does a self test on accel and gyro (see MPU6050SelfTest).
3. Takes a few miliseconds of measurements to calibrate it (assumes gravity is fully on z axis (meh)).

A German guy does (https://tutorials-raspberrypi.com/measuring-rotation-and-acceleration-raspberry-pi/):
1. Take out of sleep mode.
2. Start reading data. (short and simple, so I picked the other guy's way...)

`SMBUS(1)` stands for revision 1.
"""

from time import monotonic
from asyncio import sleep
from math import pi
from dataclasses import dataclass
from typing import Tuple
import logging

from smbus2_asyncio import SMBus2Asyncio

class Gyro:
  """Control the MPU-6050 Gyroscope and Accelerometer (on top of the GY-521 breakout board).

  See manual at https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  """

  # Base address of the chip.
  ADDRESS = 0x68

  # Register addresses
  # ------------------

  SMPLRT_DIV = 0x19
  CONFIG = 0x1A
  GYRO_CONFIG = 0x1B
  ACCEL_CONFIG = 0x1C
  FIFO_EN = 0x23
  INT_ENABLE = 0x38

  INT_STATUS = 0x3A
  ACCEL_XOUT_H = 0x3B
  ACCEL_XOUT_L = 0x3C
  ACCEL_YOUT_H = 0x3D
  ACCEL_YOUT_L = 0x3E
  ACCEL_ZOUT_H = 0x3F
  ACCEL_ZOUT_L = 0x40
  TEMP_OUT_H = 0x41
  TEMP_OUT_L = 0x42
  GYRO_XOUT_H = 0x43
  GYRO_XOUT_L = 0x44
  GYRO_YOUT_H = 0x45
  GYRO_YOUT_L = 0x46
  GYRO_ZOUT_H = 0x47
  GYRO_ZOUT_L = 0x48

  USER_CTRL = 0x6A
  PWR_MGMT_1 = 0x6B
  FIFO_COUNTH = 0x72
  FIFO_COUNTL = 0x73
  FIFO_R_W = 0x74
  WHO_AM_I = 0x75


  # Bit Constants
  # -------------

  # PWR_MGMT_1
  SLEEP = 0b_0100_0000
  DEVICE_RESET = 0b_1000_0000
  CLKSEL_GYRO_X = 0b_001

  # CONFIG
  DLPF_CFG_188Hz = 0b_001

  # INT_ENABLE
  FIFO_OFLOW_EN = 0b_0001_0000
  DATA_RDY_EN = 0b_0000_0001

  # INT_STATUS
  FIFO_OFLOW_INT = 0b_0001_0000
  DATA_RDY_INT = 0b_0000_0001

  # USER_CTRL
  FIFO_EN_BIT = 0b_0100_0000
  FIFO_RESET = 0b_0000_0100

  # FIFO_EN
  TEMP_FIFO_EN  = 0b_1000_0000
  XG_FIFO_EN    = 0b_0100_0000
  YG_FIFO_EN    = 0b_0010_0000
  ZG_FIFO_EN    = 0b_0001_0000
  ACCEL_FIFO_EN = 0b_0000_1000


  # Standard Units Ratios
  # ---------------------

  # Gravitational constant they use (from some SO thread, others mention just 9.81).
  g = 9.80665

  # Range of the accelerometer in g per second.
  accel_g_range = [2, 4, 8, 16]

  # Degrees to radians.
  rad = pi / 180

  # Range of the gyro in degrees per second.
  gyro_deg_range = [250, 500, 1000, 2000]


  @dataclass
  class Measure:
    """Return struct for gyroscope measures."""
    count: int
    time: float
    duration: float
    acceleration: Tuple[float, float, float]
    rotation: Tuple[float, float, float]


  class Malfunction(Exception):
    pass

  class MemoryOverflow(Exception):
    pass

  def __init__(
      self,
      smbus: SMBus2Asyncio,
      AD0: bool = False,
    ):
    """Initialize the Gyro controller; the gyro itself must be `setup` before measuring!

    Examples:
      Manually start and close the gyroscope and read a set of measures.
      >>> gyro = Gyro(async_smbus)
      >>> await gyro.setup()
      >>> await gyro.get_measures()
      >>> await gyro.close()

      Use `async with` to handle closing.
      >>> async with Gyro(async_smbus) as gyro:
      >>>   async for m in gyro:
      >>>     print(m.time, m.acceleration, m.rotation)

    Args:
      smbus: Opened async smbus.
      AD0: Whether the address bit is set (connected to 3V3). The sensor addres is 0x68+AD0.
    """

    # Set the I2C bus and address.
    assert isinstance(smbus, SMBus2Asyncio)
    self._bus = smbus
    self.address = Gyro.ADDRESS + AD0

    # How often the gyro samples its sensors and makes them available.
    self.sample_rate = None

    # The sample_rate will be set to `1000Hz / sample_rate_div`.
    self.sample_rate_div = 4

    # Measurement range index (for both acceleration and gyro).
    self.sensors_range = None

    # Keep track of how many measures we got from the sensor.
    self.count = 0


  async def _read(self, ptr):
    return await self._bus.read_byte_data(self.address, ptr)

  async def _write(self, ptr, value):
    return await self._bus.write_byte_data(self.address, ptr, value)

  async def _read_block(self, ptr, size):
    return await self._bus.read_i2c_block_data(self.address, ptr, size)

  async def setup(self):
    """Setup the gyroscope.

    Take control of the sensor, reset it to known state, then configure it:
      1. Check that it's connected and reset to initial state.
      2. Wake it for configuration and measurements.
      3. Set the sampling rate.
      4. Allow interrupts so we can detect overflows and data ready.
      5. Set gyroscope and accelerometer ranges.
      6. Enable FIFO memory so we can keep track of all outputs.
      7. Start measuring.
    """

    # Check
    # -----

    # Confirm connection by reading WHO_AM_I (which, by the way, ignores AD0).
    if await self._read(Gyro.WHO_AM_I) != Gyro.ADDRESS:
      raise Gyro.Malfunction("Gyro does not appear to be connected!")

    await self.reset()

    # Wakey
    # -----

    # Turn off sleep mode and switch the clock to gyro x (says in the manual
    # it's better than internal clock.)
    await self._write(Gyro.PWR_MGMT_1, Gyro.CLKSEL_GYRO_X)

    # Set rate
    # --------

    # From the manual.
    # > The sensor register output, FIFO output, and DMP sampling are all based on the Sample Rate.
    # > The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:
    # > Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    # > where Gyroscope Output Rate = 8kHz when the DLPF is disabled(DLPF_CFG=0 or 7), and 1kHz
    # > when the DLPF is enabled(see Register 26).
    # > Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz,
    # > the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than
    # > once.

    # Aim for the below sampling rate (Hz)
    self.sample_rate = 1000.0 / self.sample_rate_div

    # Lower the `sample rate = gyro rate / (1 + SMPLRT_DIV)` since it's much
    # faster than the gyro bandwith (1kHz vs 256Hz). The bandwith is how many
    # times the sensor actually makes a measurement.
    await self._write(Gyro.SMPLRT_DIV, int(self.sample_rate_div - 1))
    # Set the DLPF_CFG low pass filter so the accelerometer band gyro sampling
    # both go to 1kHz so they're a bit more in sync.
    await self._write(Gyro.CONFIG, Gyro.DLPF_CFG_188Hz)


    # Allow interrupts
    # ----------------

    # Allow interrupts from data ready and FIFO overflow. We need to know if it
    # overflowed since the data won't be alligned anymore and oops.
    await self._write(Gyro.INT_ENABLE, Gyro.FIFO_OFLOW_EN | Gyro.DATA_RDY_EN)
    # Read it once to make sure it's clear.
    await self._read(Gyro.INT_STATUS)

    # Set sensor options
    #-------------------

    await self.set_sensors_range(0)

    # TODO: do the test sequence thingy

    # Enable FIFO
    # -----------

    # Enable the FIFO storage for gyro x, y, z and accel x, y, z measurements.
    await self._write(Gyro.FIFO_EN,
      Gyro.XG_FIFO_EN | Gyro.YG_FIFO_EN | Gyro.ZG_FIFO_EN | Gyro.ACCEL_FIFO_EN)
    # Enable FIFO.
    await self._write(Gyro.USER_CTRL, Gyro.FIFO_EN_BIT)

    # Start
    # -----

    # End the starting sequence and return the start time.
    return monotonic()


  async def get_instant_measure(self, wait_until_ready=True):
    """Get instant sensor measure, ignoring internal memory.

    To read data we should:
      0. Use DATA_RDY_EN to enable interrupts on data.
      1. Read INT_STATUS and check DATA_RDY_INT is set.
      2. Read all values from sensor registers in a signle block.
    """

    for _ in range(10):
      time = monotonic()

      # Read all measures at once (it's faster to read status in the same block):
      # + 1 byte status
      # + 3 * 2 byte accelleration
      # + 2 byte temperature
      # + 3 * 2 byte gyro.

      data = await self._read_block(Gyro.INT_STATUS, 15)

      # Check if data was ready, if not try again very soon.
      if wait_until_ready and (not data[0] & Gyro.DATA_RDY_INT):
        await sleep(0.2 / self.sample_rate)
        continue

      ax = make_float16(data[1:3])
      ay = make_float16(data[3:5])
      az = make_float16(data[5:7])
      # Ignore temperature bytes.
      gx = make_float16(data[9:11])
      gy = make_float16(data[11:13])
      gz = make_float16(data[13:15])

      return self._create_measure(time, 1.0 / self.sample_rate, ax, ay, az, gx, gy, gz)

    else:
      raise Gyro.Malfunction("Gyro didn't have measurements for 2 times its sample rate.")

  async def get_measures(self):
    """Read measures from the internal memory.

    To read FIFO probably simplest way would be to read FIFO_COUNT make sure
    it's >= 12, then read 12 bytes at a time, then double check FIFO_OFLOW_INT
    wasn't set. If it was, then use FIFO_RESET to discard partial data.
    Otherwise, if all good, parse the 6 sensor measures and push to queue.
    Then keep going while tehre's still something in the FIFO.

    Note:
      FIFO max size is 1024 bytes. Which at 0.5 kHz sample rate and using 6
      measures of 2 bytes each, means we have to 170ms worth of memory.
    """
    fifo_count = make_uint16(await self._read_block(Gyro.FIFO_COUNTH, 2))

    # Read packs of the 6, 2 byte measurements.
    packs_in_fifo = fifo_count // 12

    duration = 1.0 / self.sample_rate

    # At this time the measures were already taken and available for reading.
    time = monotonic()


    while packs_in_fifo > 0:
      # The block read can only do 32 bytes at a time, limit to 2 packs of
      # measurements (maximum of 24 bytes at a time).
      packs_to_read = min(2, packs_in_fifo)

      # Read as much at once to minize syscalls.
      data = await self._read_block(Gyro.FIFO_R_W, packs_to_read * 12)

      # Check that the FIFO hasn't overflowed (FIFO_OFLOW_INT is 0).
      if await self._read(Gyro.INT_STATUS) & Gyro.FIFO_OFLOW_INT:
        await self._reset_fifo()
        raise Gyro.MemoryOverflow("Internal memory was full, couldn't read measures.")

      # FIFO is good, parse measures.

      for i in range(packs_to_read):
        ax = make_float16(data[12*i +  0 : 12*i +  2])
        ay = make_float16(data[12*i +  2 : 12*i +  4])
        az = make_float16(data[12*i +  4 : 12*i +  6])
        gx = make_float16(data[12*i +  6 : 12*i +  8])
        gy = make_float16(data[12*i +  8 : 12*i + 10])
        gz = make_float16(data[12*i + 10 : 12*i + 12])

        # If we have more measures in fifo, then we need adjust the time each
        # measure was actually take by how fast the gyro samples its sensors.
        pack_time = time - duration * (packs_in_fifo - i - 1)

        yield self._create_measure(pack_time, duration, ax, ay, az, gx, gy, gz)

      # Compute how many packs are remaining to be processed.
      packs_in_fifo -= packs_to_read


  def _create_measure(self, time, duration, ax, ay, az, gx, gy, gz):
    """Scale the sensor values to SI units and emit the measure event."""
    accel_scale = Gyro.accel_g_range[self.sensors_range] * Gyro.g
    gyro_scale = Gyro.gyro_deg_range[self.sensors_range] * Gyro.rad

    # Increment the total measures count.
    self.count += 1

    return Gyro.Measure(
      count=self.count,
      time=time,
      duration=duration,
      acceleration=(accel_scale * ax, accel_scale * ay, accel_scale * az),
      rotation=(gyro_scale * gx, gyro_scale * gy, gyro_scale * gz),
    )


  async def set_sensors_range(self, sensors_range):
    if type(sensors_range) is not int or sensors_range not in (0, 1, 2, 3):
      raise ValueError("Sensor range must be an integer within [0, 3].")

    # Set the range of the gyro and accelerometer.
    await self._write(Gyro.GYRO_CONFIG, sensors_range << 3)
    await self._write(Gyro.ACCEL_CONFIG, sensors_range << 3)
    # Use the new range from now.
    self.sensors_range = sensors_range

    await self._reset_fifo()


  async def reset(self):
    """Reset the device to initial state."""

    # Reset the device to all 0s and wait for confirmation.
    # After a reset, all registers should be 0 except SLEEP and WHO_AM_I.
    await self._write(Gyro.PWR_MGMT_1, Gyro.DEVICE_RESET)

    # Wait for the device to reset. It only mentions to wait for 100ms
    # for the SPI interface but seems like it's true for both.
    await sleep(0.1)


    # Cofirm the device reset is complete.
    for _ in range(10):
      if await self._read(Gyro.PWR_MGMT_1) & Gyro.DEVICE_RESET:
        # Reset bit still high, try again in a bit (hehe).
        await sleep(0.1)
      else:
        # The reset bit was cleared, we're good.
        break
    else:
      raise Gyro.Malfunction("Gyro didn't reset within a second!")


  async def _reset_fifo(self):
    """Drop all data from the internal memory."""

    # Disable FIFO_EN.
    await self._write(Gyro.USER_CTRL, 0)

    # Trigger FIFO_RESET.
    await self._write(Gyro.USER_CTRL, Gyro.FIFO_RESET)
    for _ in range(100):
      if await self._read(Gyro.USER_CTRL) & Gyro.FIFO_RESET:
        # Still resetting, but I think FIFO resetting should be fast, do the minimal sleep.
        await sleep(0.0)
      else:
        # The reset bit was cleared, we're good.
        break

    # Re-enable FIFO_EN.
    await self._write(Gyro.USER_CTRL, Gyro.FIFO_EN_BIT)


  async def self_test(self):
    """Run the internal test procedure for the gyro and report results."""
    raise NotImplementedError


  async def close(self):
    """Reset the sensor and close the connection."""
    await self.reset()


  async def __aiter__(self):
    while True:
      try:
        async for m in self.get_measures():
          yield m
      except Gyro.MemoryOverflow:
        logging.warning("Gyro FIFO overflow occurred.")

  async def __aenter__(self):
    await self.setup()
    return self

  async def __aexit__(self, *_):
    await self.close()


# Misc functions
# --------------

def make_uint16(high_low_bytes):
  return (high_low_bytes[0] << 8) + high_low_bytes[1]

def to_signed_int16(uint16):
  return uint16 if uint16 < 0x8000 else -((0xFFFF - uint16) + 1)

def make_float16(high_low_bytes):
  return to_signed_int16(make_uint16(high_low_bytes)) / 0x8000


# Example
# -------

# Continuously print gyro acceleration.

if __name__ == "__main__":
  from . loop_runner import run_tasks

  async def read_gyro():
    smbus = SMBus2Asyncio(1)
    await smbus.open()

    async with Gyro(smbus) as gyro:
      while True:
        try:
          measure = None
          async for measure in gyro.get_measures():
            pass
        except Gyro.MemoryOverflow:
          print("Internal FIFO queue overflow.")

        if measure:
          print("ax: {:6.2f}, ay: {:6.2f}, az: {:6.2f}".format(*measure.acceleration))
        else:
          print("no measures yet")

        await sleep(0.05)


  run_tasks(read_gyro())
