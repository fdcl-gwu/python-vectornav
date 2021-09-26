from vectornav import Vectornav

import time
from datetime import datetime


if __name__ == '__main__':
    print('Program started')

    t0 = datetime.now()

    port = '/dev/cu.usbserial-AK05WMX3'
    baud = 115200

    thread_imu = Vectornav(1, port, baud, t0)
    thread_imu.start()

    while True:
        try:
            # Get and print data from IMU thread at every 0.5 seconds.
            time.sleep(0.5)
            imu_data = thread_imu.output_data()
            print(imu_data.ypr.T)

        except KeyboardInterrupt:
            print('Exit command received')
            break
    
    thread_imu.end_thread()

    print('Program closed')
