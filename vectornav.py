import numpy as np
import serial
import struct
import threading
import time

from array import array
from datetime import datetime



class ImuData:
    def __init__(self, t=0.0, freq=0, ypr=np.zeros(3), a=np.zeros(3), \
            W=np.zeros(3)):
        self.t = t
        self.freq = freq
        self.ypr = ypr
        self.a = a
        self.W = W



class Vectornav(threading.Thread):

    def __init__(self, thread_id, port, baud, t0):
        '''Instantiate the IMU thread.

        Args:
        thread_id: (int) - Thread ID
        port: (string) - Port name of the IMU
        baud: (int) - Baud rate of the IMU
        t0: (datetime object) - Epoch
        '''
        threading.Thread.__init__(self)
        self.thread_id = thread_id

        self._lock = threading.Lock()

        self._on = True
        self._t0 = t0
        self._port = port
        self._baud = baud

        self._t = (datetime.now() - t0).total_seconds()

        self._ypr = np.zeros(3)
        self._a = np.zeros(3)
        self._W = np.zeros(3)

        # This is specific message has 41 bytes. You should update this to match
        # your configuration.
        self._len_payload = 41

        print('IMU: initialized')


    def run(self):
        '''Start the thread.
        '''

        print('IMU: reading from {} at {}'.format(self._port, self._baud))

        # In case the port is not properly closed,
        try:
            temp = serial.Serial(self._port, self._baud)
            temp.close()
        except:
            print('\033[91m' + 'Unable to open IMU port at ' + self._port
                  + ':' + str(self._baud) + '\033[0m')
            return

        # Open the serial port and start reading.
        with serial.Serial(self._port, self._baud, timeout=1) as s:
            
            # Clear the buffer first.
            print('IMU: clearing buffer')
            num_bytes = s.in_waiting
            s.read(num_bytes)

            print('IMU: starting main loop')
            while self._on:
                imu_sync_detected = False

                # Check if there are bytes waiting in the buffer.
                num_bytes = s.in_waiting
                if num_bytes == 0:
                    # Reduce/delete this sleep time if you are reading data at
                    # a faster rate.
                    time.sleep(0.01)
                    continue
                
                # IMU sends 0xFA (int 250) as the first byte. This marks the 
                # begining of the message.
                imu_sync_detected = self.check_sync_byte(s)
                if not imu_sync_detected:
                    continue
                
                # If the sync byte us detected, read the rest of the message.
                success = self.read_imu_data(s)
                if not success:
                    continue
                
        print('IMU: thread closed')

    
    def check_sync_byte(self, s):
        '''Check if the sync byte is detected.
        
        IMU sends 0xFA (int 250) as the first byte. This marks the begining of 
        the message. 

        Args:
        s: (serial object) - Already open serial port of the IMU.

        Return:
        bool - True if the sync byte is detected in the current buffer.
        '''

        # Iterate over all the bytes in the current buffer.
        for _ in range(s.in_waiting):
            byte_in = s.read(1)

            # Check if the sync byte 0xFA (int 250) is detected.
            int_in = int.from_bytes(byte_in, 'little')
            if int_in == 250:
                return True
        
        return False


    def read_imu_data(self, s):
        '''Read and parse the payload of the IMU message.

        Args:
        s: (serial object) - Already open serial port of the IMU.

        Return:
        bool - True if the operation is succesfull
        '''

        # Read data.
        N = self._len_payload
        data = s.read(N)

        # Check if there are unexpected errors in the message.
        # Last two bytes of the payload is the checksum bytes.
        checksum_array = array('B', [data[N-1], data[N-2]])
        checksum = struct.unpack('H', checksum_array)[0]

        # Compare the received checksum value against the calculated checksum.
        crc = self.calculate_imu_crc(data[:N-2])
        if not crc == checksum:
            print('IMU CRC error')
            return False

        # If the checksum is valid, parse the data.
        return self.parse_data(data)


    def parse_data(self, data):
        '''Parse the bytes of the sensor measurements
        
        Args:
        data: (byte array) - data read from the serial port

        Return:
        bool - True if the operation is succesfull
        '''

        try:
            with self._lock:
                self._ypr[0] = struct.unpack('f', data[3:7])[0]
                self._ypr[1] = struct.unpack('f', data[7:11])[0]
                self._ypr[2] = struct.unpack('f', data[11:15])[0]

                self._a[0] = struct.unpack('f', data[15:19])[0]
                self._a[1] = struct.unpack('f', data[19:23])[0]
                self._a[2] = struct.unpack('f', data[23:27])[0]

                self._W[0] = struct.unpack('f', data[27:31])[0]
                self._W[1] = struct.unpack('f', data[31:35])[0]
                self._W[2] = struct.unpack('f', data[35:39])[0]

        except:
            print('IMU: error parsing data')
            return False
        
        return True

    
    def calculate_imu_crc(self, data):
        '''Calculate the 16-bit CRC for the given message.

        Args:
        data: (byte array) - data read from the serial port

        Return:
        unsigned short - CRC checksum value
        '''
        data = bytearray(data)

        crc = np.array([0], dtype=np.ushort)
        for i in range(len(data)):
            crc[0] = (crc[0] >> 8) | (crc[0] << 8)
            crc[0] ^= data[i]
            crc[0] ^= (crc[0] & 0xff) >> 4
            crc[0] ^= crc[0] << 12
            crc[0] ^= (crc[0] & 0x00ff) << 5
        
        return crc[0]


    def output_data(self):
        '''Output the current measurements.
        
        Return:
        ImuData - current IMU data
        '''

        with self._lock:
            data = ImuData(
                self._t,
                self._ypr,
                self._a,
                self._W
            )
        return data


    def end_thread(self):
        '''Call to end the IMU thread.'''

        self._on = False
        print('IMU: thread close signal received')
