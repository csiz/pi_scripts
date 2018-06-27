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

from smbus import SMBus # pylint: disable=import-error
from threading import Thread
from time import sleep, monotonic
import asyncio
from math import pi


from oberserver import EventRouter

class Gyro(EventRouter):
  ADDRESS = 0x68

  # Registers we use:
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

  # Gravitational constant they use (from some SO thread, others mention just 9.81).
  g = 9.80665

  # Range of the accelerometer in g per second.
  accel_g_range = [2, 4, 8, 16]

  # Degrees to radians.
  rad = pi / 180

  # Range of the gyro in degrees per second.
  gyro_deg_range = [250, 500, 1000, 2000]

  class Measure:
    __slots__ = ("time", "duration", "acceleration", "rotation")

    def __init__(self, time, duration, acceleration, rotation):
      self.time = time
      self.duration = duration
      self.acceleration = acceleration
      self.rotation = rotation

    def __repr__(self):
      return "Gyro.Measure(time={}, duration={}, acceleration={}, rotation={})".format(
        self.time,
        self.duration,
        self.acceleration,
        self.rotation,
      )

  class StopWorker(Exception):
    pass

  class Malfunction(Exception):
    pass

  def __init__(
      self,
      AD0: bool = False,
    ):
    """Control the MPU-6050 Gyroscope and Accelerometer (on top of the GY-521 breakout board).

    See manual at https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

    Args:
      AD0: Whether the address bit is set (connected to 3V3). The sensor addres is 0x68+AD0.

    Events:
      measure: Receives the latest Gyro.Measure from the gyro.
      exception: Exception that occured while reading the gyro.
      test_result: Test results from a gyro self test.
      start: Time when the sensor starts (or restarts after reset).
      discarded: Some measures were discared, because sensor overflow or range change.

    Commands:
      stop: Stop the worker thread.
      reset: Reset the devie and re-setup.
      sensor_range: Set the sensor range to whatever.
    """

    # Connect to the I2C serial bus version 1.
    self.bus = SMBus(1)
    self.address = Gyro.ADDRESS + AD0

    # How often the gyro samples its sensors and makes them available.
    self.sample_rate = None

    # The sample_rate will be set to `1000Hz / sample_rate_div`.
    self.sample_rate_div = 4

    # Measurement range index (for both acceleration and gyro).
    self.sensors_range = None

    # We'll use a worker thread to manage the device as we need to keep getting
    # data from it fairly often. The worker will report measures through self as
    # it's an EventRouter. We need another EventRouter to send commands to it.
    super().__init__(maxsize=2048)
    self.commands = EventRouter()

    self.commands.observe("reset", self._setup)
    self.commands.observe("sensors_range", self._set_sensors_range)
    self.commands.observe("stop", self._stop_worker)

    # And we'll start by ensuring it resets.
    self.commands.trigger("reset")

    self.worker = Thread(target=self._worker_loop)
    self.worker.daemon = True
    self.worker.start()

  def _worker_loop(self):
    """Run the commands tick and measures in a loop."""
    exception_count = 0
    while exception_count < 1:
      try:
        self.commands.tick()
        self._get_measures()
      except Gyro.StopWorker:
        break
      except Exception as e:
        self.trigger("exception", e)
        exception_count += 1
      else:
        exception_count = 0


  def _stop_worker(self):
    raise Gyro.StopWorker

  def _setup(self):
    # Check
    # -----

    # Confirm connection by reading WHO_AM_I (which, by the way, ignores AD0).
    if self.bus.read_byte_data(self.address, Gyro.WHO_AM_I) != Gyro.ADDRESS:
      raise Gyro.Malfunction("Gyro does not appear to be connected!")

    # Reset the device to all 0s and wait for confirmation.
    # After a reset, all registers should be 0 except SLEEP and WHO_AM_I.
    self.bus.write_byte_data(self.address, Gyro.PWR_MGMT_1, 0b10000000)
    # Wait for the device to reset. It only mentions to wait for 100ms
    # for the SPI interface but seems like it's true for both.
    sleep(0.1)
    for _ in range(10):
      if self.bus.read_byte_data(self.address, Gyro.PWR_MGMT_1) & 0b10000000 == 0:
        # The reset bit was clearer, we're good.
        break
      else:
        # Try again in a bit.
        sleep(0.1)
    else:
      raise Gyro.Malfunction("Gyro didn't reset within a second!")

    # Wakey
    # -----

    # Turn off sleep mode and switch the clock to gyro x (says in the manual
    # it's better than internal clock.)
    self.bus.write_byte_data(self.address, Gyro.PWR_MGMT_1, 0b00000001)

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

    # Halve the `sample rate = gyro rate / (1 + SMPLRT_DIV)` since it's much
    # faster than the gyro bandwith (1kHz vs 256Hz).
    self.bus.write_byte_data(self.address, Gyro.SMPLRT_DIV, int(self.sample_rate_div - 1))
    # Set the DLPF_CFG low pass filter so the accelerometer band gyro sampling
    # both go to 1kHz so they're a bit more in sync.
    self.bus.write_byte_data(self.address, Gyro.CONFIG, 0b001)


    # Allow interrupts
    # ----------------

    # Allow interrupts from data ready and FIFO overflow. We need to know if it
    # overflowed since the data won't be alligned anymore and oops.
    self.bus.write_byte_data(self.address, Gyro.INT_ENABLE, 0b00010001)
    # Read it once to make sure it's clear.
    self.bus.read_byte_data(self.address, Gyro.INT_STATUS)

    # Set sensor options
    #-------------------

    self._set_sensors_range(0, read_fifo=False)

    # TODO: do the test sequence thingy

    # Enable FIFO
    # -----------

    # Enable the FIFO storage for gyro x, y, z and accel x, y, z measurements.
    self.bus.write_byte_data(self.address, Gyro.FIFO_EN, 0b01111000)
    # Enable FIFO.
    self.bus.write_byte_data(self.address, Gyro.USER_CTRL, 0b01000000)

    # Start
    # -----

    # End the starting sequence and mention the start time.
    self.trigger("start", monotonic())

  def _get_instant_measures(self, wait_until_ready=True):
    # To read data we should:
    #     0. Use DATA_RDY_EN to enable interrupts on data.
    #     1. Read INT_STATUS and check DATA_RDY_INT is set.
    #     2. Read all values from sensor registers in a signle block.

    for _ in range(10):
      time = monotonic()

      # Read all measures at once (it's faster to read status in the same block):
      # + 1 byte status
      # + 3 * 2 byte accelleration
      # + 2 byte temperature
      # + 3 * 2 byte gyro.

      data = self.bus.read_i2c_block_data(self.address, Gyro.INT_STATUS, 15)

      # Check if data was ready, if not try again very soon.
      if wait_until_ready and (not data[0] & 0b00000001):
        sleep(0.2 / self.sample_rate)
        continue

      ax = make_float16(data[1:3])
      ay = make_float16(data[3:5])
      az = make_float16(data[5:7])
      # Ignore temperature bytes.
      gx = make_float16(data[9:11])
      gy = make_float16(data[11:13])
      gz = make_float16(data[13:15])

      self._emit_measure(time, ax, ay, az, gx, gy, gz)

      break
    else:
      raise Gyro.Malfunction("Gyro didn't have measurements for 2 times its sample rate.")


  def _disable_fifo(self):
    self.bus.write_byte_data

  def _set_sensors_range(self, sensors_range, read_fifo=True):
    assert type(sensors_range) is int and sensors_range in (0, 1, 2, 3)
    if read_fifo:
      # Get all measures so we don't waste anything.
      self._get_measures()

    # Set the range of the gyro and accelerometer.
    self.bus.write_byte_data(self.address, Gyro.GYRO_CONFIG, sensors_range << 3)
    self.bus.write_byte_data(self.address, Gyro.ACCEL_CONFIG, sensors_range << 3)
    # Use the new range from now.
    self.sensors_range = sensors_range

    if read_fifo:
      self._reset_fifo()

  def _reset_fifo(self):
    # Disable FIFO_EN.
    self.bus.write_byte_data(self.address, Gyro.USER_CTRL, 0b00000000)

    # Trigger FIFO_RESET.
    self.bus.write_byte_data(self.address, Gyro.USER_CTRL, 0b00000100)
    for _ in range(100):
      if self.bus.read_byte_data(self.address, Gyro.USER_CTRL) & 0b00000100 == 0:
        # The reset bit was clearer, we're good.
        break
      # I think FIFO resetting should be fast, do the minimal sleep.
      sleep(0.0)

    # Re-enable FIFO_EN.
    self.bus.write_byte_data(self.address, Gyro.USER_CTRL, 0b01000000)

    self.trigger("discarded")

  def _emit_measure(self, time, ax, ay, az, gx, gy, gz):
    accel_scale = Gyro.accel_g_range[self.sensors_range] * Gyro.g
    gyro_scale = Gyro.gyro_deg_range[self.sensors_range] * Gyro.rad

    measure = Gyro.Measure(
      time=time,
      duration=1.0 / self.sample_rate,
      acceleration=(accel_scale * ax, accel_scale * ay, accel_scale * az),
      rotation=(gyro_scale * gx, gyro_scale * gy, gyro_scale * gz),
    )

    self.trigger("measure", measure)

  def _get_measures(self):
    # FIFO max size is 1024 bytes. Which at 0.5 kHz sample rate and using 6
    # measures of 2 bytes each, means we have to 170ms worth of memory.

    # To read FIFO probably simplest way would be to read FIFO_COUNT make sure
    # it's >= 12, then read 12 bytes at a time, then double check FIFO_OFLOW_INT
    # wasn't set. If it was, then use FIFO_RESET to discard partial data.
    # Otherwise, if all good, parse the 6 sensor measures and push to queue.
    # Then keep going while still something in the FIFO.
    fifo_count = make_uint16(self.bus.read_i2c_block_data(self.address, Gyro.FIFO_COUNTH, 2))

    # Read packs of the 6, 2 byte measurements.
    packs_in_fifo = fifo_count // 12

    while packs_in_fifo > 0:
      # The block read can only do 32 bytes at a time, limit to 2 packs of
      # measurements (maximum of 24 bytes at a time).
      packs_to_read = min(2, packs_in_fifo)

      time = monotonic()

      # Read as much at once to minize syscalls.
      data = self.bus.read_i2c_block_data(self.address, Gyro.FIFO_R_W, packs_to_read * 12)
      packs_in_fifo -= packs_to_read

      # Check that the FIFO hasn't overflowed (FIFO_OFLOW_INT is 0).
      if self.bus.read_byte_data(self.address, Gyro.INT_STATUS) & 0b00010000:
        self._reset_fifo()
        return
      # FIFO is good, parse measures.

      for i in range(packs_to_read):
        ax = make_float16(data[12*i +  0 : 12*i +  2])
        ay = make_float16(data[12*i +  2 : 12*i +  4])
        az = make_float16(data[12*i +  4 : 12*i +  6])
        gx = make_float16(data[12*i +  6 : 12*i +  8])
        gy = make_float16(data[12*i +  8 : 12*i + 10])
        gz = make_float16(data[12*i + 10 : 12*i + 12])

        self._emit_measure(time, ax, ay, az, gx, gy, gz)


  def _self_test(self):
    raise NotImplementedError

    # TODO: pump test results into self.trigger("test_result")

  def __del__(self):
    # Tell the worker thread to stop.
    self.commands.trigger("stop")

    # Let it finish.
    self.worker.join()

    self.bus.close()


def make_uint16(high_low_bytes):
  return (high_low_bytes[0] << 8) + high_low_bytes[1]

def to_signed_int16(uint16):
  return uint16 if uint16 < 0x8000 else -((0xFFFF - uint16) + 1)

def make_float16(high_low_bytes):
  return to_signed_int16(make_uint16(high_low_bytes)) / 0x8000
