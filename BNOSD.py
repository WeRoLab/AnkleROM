import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR,BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,BNO_REPORT_ACCELEROMETER,BNO_REPORT_GAME_ROTATION_VECTOR
import time
import sdcardio
import storage
import os
import digitalio


# LED on Feather designed to flash on SD card write cycles
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Configure I2C connections to the BNO085 IMUs
i2c = busio.I2C(board.SCL, board.SDA)
bno_1 = BNO08X_I2C(i2c, address=0x4B)
bno_2 = BNO08X_I2C(i2c, address=0x4A)

# Enable Rotation&Accelerometer reporting
bno_1.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno_2.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno_1.enable_feature(BNO_REPORT_ACCELEROMETER)
bno_2.enable_feature(BNO_REPORT_ACCELEROMETER)

# Attempted Geomagnetic reports as well but standard rotation
# vector seem to fit our usecase better. See project doc for more analysis.
# bno_1.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
# bno_2.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

time_init = time.monotonic()
cycle_count = 0
cycle_time = 100  # ms

# Previous board code accounting for incorrect wiring, should be corrected now.
# spi = busio.SPI(board.SCK, board.MISO, board.MOSI)
# sd = sdcardio.SDCard(spi, board.A5)
# Detect SD Card
spi = busio.SPI(board.SCK, board.MISO, board.MOSI)
sd = sdcardio.SDCard(spi, board.A5)

# Mount to /sd
vfs = storage.VfsFat(sd)
storage.mount(vfs, '/sd')

# find unused file name as current file.
# eg. IMU1.TXT -> IMU2.TXT -> IMU3.TXT ...
existing_files = os.listdir('/sd')
i = 1
while True:
    file_name = 'IMU' + str(i) + '.TXT'
    if file_name not in existing_files:
        break
    i += 1
file_name = '/sd/' + file_name

# Calibration sequence that didn't work very well.
# led.value = True
# bno_1.begin_calibration()
# bno_2.begin_calibration()
# while bno_1.calibration_status != 3 or bno_2.calibration_status !=3:
#     bno_1.quaternion
#     bno_2.quaternion
#     print('calibration status: {}, {}'.format(bno_1.calibration_status, bno_2.calibration_status))
#     time.sleep(1)

# Main Loop Write Cycle
with open(file_name, 'a') as f:
    while True:
        # Pad cycle times to cycle_time (currently 100ms), ensuring even recording of datapoints.
        while time.monotonic() - time_init < (cycle_time / 1000)*cycle_count:
            time.sleep(0.001)
        timer = time.monotonic()

        # Collect Quaternion and Accelerations
        qr, qi, qj, qk = bno_1.quaternion
        pr, pi, pj, pk = bno_2.quaternion
        qa, qb, qc = bno_1.acceleration
        pa, pb, pc = bno_2.acceleration
        output = '{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
            cycle_count, timer-time_init,
            qr, qi, qj, qk,
            pr, pi, pj, pk,
            qa, qb, qc,
            pa, pb, pc
        )
        f.write(output)
        # Flush ensures it is physically written to SD card in case of power loss or program interrupt.
        f.flush()
        cycle_count += 1
        # print('writing to file: {}'.format(file_name))
        # print('cycle time: {} ms'.format((time.monotonic() - timer) * 1000))
        # print('elapsed time: {} s'.format(time.monotonic() - time_init))
        print(output)
        # blink when normal, when lagging behind remain red
        if cycle_count % 2 == 0 or time.monotonic()-time_init>cycle_time/1000*cycle_count:
            led.value = True
        else:
            led.value = False
