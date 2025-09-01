"""Prints the readings of a Bota Systems EtherCAT sensor.

Usage: python bota_minimal_example.py <adapter>

This example expects a physical slave layout according to
_expected_slave_layout, see below.
"""

import sys
import struct
import time
import collections
import os

# For non-blocking keyboard input on Windows
import msvcrt

import pysoem
import ctypes
import struct


class MinimalExample:

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256
    # The time step is set according to the sinc filter size
    time_step = 1.0;

    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

    def clear_screen(self):
        """Clear the terminal screen"""
        os.system('cls' if os.name == 'nt' else 'clear')

    def display_dashboard(self, data, last_mean_f=None):
        """Display sensor data in a formatted dashboard"""
        self.clear_screen()

        print("=" * 80)
        print("                    BOTA SENSOR DASHBOARD")
        print("=" * 80)
        print(f"Status: {data['status']:<10} | Warnings/Errors/Fatals: {data['warningsErrorsFatals']}")
        print(f"Temperature: {data['temperature']:.2f}°C")
        print("-" * 80)
        print("FORCE & TORQUE SENSORS")
        print("-" * 80)
        print(f"Fx: {data['Fx']:>10.3f} N    | Fy: {data['Fy']:>10.3f} N    | Fz: {data['Fz']:>10.3f} N")
        print(f"Mx: {data['Mx']:>10.3f} Nm   | My: {data['My']:>10.3f} Nm   | Mz: {data['Mz']:>10.3f} Nm")
        print(f"Force-Torque Saturated: {data['forceTorqueSaturated']}")
        print("-" * 80)
        print("ACCELEROMETER")
        print("-" * 80)
        print(f"Ax: {data['Ax']:>10.3f} m/s² | Ay: {data['Ay']:>10.3f} m/s² | Az: {data['Az']:>10.3f} m/s²")
        print(f"Acceleration Saturated: {data['accelerationSaturated']}")
        print("-" * 80)
        print("GYROSCOPE")
        print("-" * 80)
        print(f"Rx: {data['Rx']:>10.3f} rad/s | Ry: {data['Ry']:>10.3f} rad/s | Rz: {data['Rz']:>10.3f} rad/s")
        print(f"Angular Rate Saturated: {data['angularRateSaturated']}")
        print("-" * 80)
        # Dedicated block for last mean Fz value
        print("LAST 1s MEAN Fz RECORDING")
        print("-" * 80)
        if last_mean_f is not None:
            print(f"Fz = {last_mean_f:.4f} N")
        else:
            print("Fz = N/A")
        print("-" * 80)
        print(f"Update Rate: {1.0/self.time_step:.1f} Hz | Press Ctrl+C to stop")
        print("=" * 80)

    def bota_sensor_setup(self, slave_pos):
        print("bota_sensor_setup")
        slave = self._master.slaves[slave_pos]

        ## Set sensor configuration
        # calibration matrix active
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        # temperature compensation
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        # IMU active
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))

        ## Set force torque filter
        # FIR disable
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        # FAST enable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        # CHOP enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        # Sinc filter size
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))

        # Set sampling rate to 100 Hz
        try:
            slave.sdo_write(0x8011, 0, struct.pack('h', 100))
            print("Sampling rate set to 100 Hz")
        except Exception as e:
            print(f"Failed to set sampling rate: {e}")

        ## Get sampling rate
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        print("Sampling rate {}".format(sampling_rate))
        if sampling_rate > 0:
            self.time_step = 1.0/float(sampling_rate)

        print("time step {}".format(self.time_step))

    def run(self):

        self._master.open(self._ifname)

        # config_init returns the number of slaves found
        if self._master.config_init() > 0:

            print("{} slaves found and configured".format(
                len(self._master.slaves)))

            for i, slave in enumerate(self._master.slaves):
                assert(slave.man == self.BOTA_VENDOR_ID)
                assert(
                    slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func

            # PREOP_STATE to SAFEOP_STATE request - each slave's config_func is called
            self._master.config_map()

            # wait 50 ms for all slaves to reach SAFE_OP state
            if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.SAFEOP_STATE:
                        print('{} did not reach SAFEOP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached SAFEOP state')

            self._master.state = pysoem.OP_STATE
            self._master.write_state()

            self._master.state_check(pysoem.OP_STATE, 50000)
            if self._master.state != pysoem.OP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.OP_STATE:
                        print('{} did not reach OP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                raise Exception('not all slaves reached OP state')

            try:

                # Zero offsets for force/torque
                zero_offsets = {
                    'Fx': 0.0, 'Fy': 0.0, 'Fz': 0.0,
                    'Mx': 0.0, 'My': 0.0, 'Mz': 0.0
                }

                print("Press 'r' to reset/recalibrate force/torque to zero.")
                print("Press SPACE to record Fz for 1s and report mean Fz.")

                recording_fz = False
                fz_samples = []
                record_start_time = None
                last_mean_fz = None

                while 1:
                    # free run cycle
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)

                    sensor_input_as_bytes = self._master.slaves[0].input
                    status = struct.unpack_from('B', sensor_input_as_bytes, 0)[0]
                    warningsErrorsFatals = struct.unpack_from('I', sensor_input_as_bytes, 1)[0]
                    Fx = struct.unpack_from('f', sensor_input_as_bytes, 5)[0]
                    Fy = struct.unpack_from('f', sensor_input_as_bytes, 9)[0]
                    Fz = struct.unpack_from('f', sensor_input_as_bytes, 13)[0]
                    Mx = struct.unpack_from('f', sensor_input_as_bytes, 17)[0]
                    My = struct.unpack_from('f', sensor_input_as_bytes, 21)[0]
                    Mz = struct.unpack_from('f', sensor_input_as_bytes, 25)[0]
                    forceTorqueSaturated = struct.unpack_from('H', sensor_input_as_bytes, 29)[0]
                    Ax = struct.unpack_from('f', sensor_input_as_bytes, 31)[0]
                    Ay = struct.unpack_from('f', sensor_input_as_bytes, 35)[0]
                    Az = struct.unpack_from('f', sensor_input_as_bytes, 39)[0]
                    accelerationSaturated = struct.unpack_from('B', sensor_input_as_bytes, 43)[0]
                    Rx = struct.unpack_from('f', sensor_input_as_bytes, 44)[0]
                    Ry = struct.unpack_from('f', sensor_input_as_bytes, 48)[0]
                    Rz = struct.unpack_from('f', sensor_input_as_bytes, 52)[0]
                    angularRateSaturated = struct.unpack_from('B', sensor_input_as_bytes, 56)[0]
                    temperature = struct.unpack_from('f', sensor_input_as_bytes, 57)[0]

                    # Check for key press (non-blocking)
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        if key in [b'r', b'R']:
                            zero_offsets['Fx'] = Fx
                            zero_offsets['Fy'] = Fy
                            zero_offsets['Fz'] = Fz
                            zero_offsets['Mx'] = Mx
                            zero_offsets['My'] = My
                            zero_offsets['Mz'] = Mz
                            print("\nForce/Torque recalibrated to zero.")
                        elif key == b' ' and not recording_fz:
                            recording_fz = True
                            fz_samples = []
                            record_start_time = time.time()
                            print("\nStarted 1s Fz recording...")

                    # Apply zero offsets
                    Fx_disp = Fx - zero_offsets['Fx']
                    Fy_disp = Fy - zero_offsets['Fy']
                    Fz_disp = Fz - zero_offsets['Fz']
                    Mx_disp = Mx - zero_offsets['Mx']
                    My_disp = My - zero_offsets['My']
                    Mz_disp = Mz - zero_offsets['Mz']

                    # If recording, collect Fz_disp samples for 1s
                    if recording_fz:
                        fz_samples.append(Fz_disp)
                        if (time.time() - record_start_time) >= 1.0:
                            mean_fz = sum(fz_samples) / len(fz_samples) if fz_samples else 0.0
                            last_mean_fz = mean_fz
                            print(f"\nRecording finished. Mean Fz over 1s: {mean_fz:.4f} N")
                            recording_fz = False

                    # Create data dictionary for dashboard
                    data = {
                        'status': status,
                        'warningsErrorsFatals': warningsErrorsFatals,
                        'Fx': Fx_disp, 'Fy': Fy_disp, 'Fz': Fz_disp,
                        'Mx': Mx_disp, 'My': My_disp, 'Mz': Mz_disp,
                        'forceTorqueSaturated': forceTorqueSaturated,
                        'Ax': Ax, 'Ay': Ay, 'Az': Az,
                        'accelerationSaturated': accelerationSaturated,
                        'Rx': Rx, 'Ry': Ry, 'Rz': Rz,
                        'angularRateSaturated': angularRateSaturated,
                        'temperature': temperature
                    }

                    # Display dashboard with last_mean_fz
                    self.display_dashboard(data, last_mean_f=last_mean_fz)

                    time.sleep(self.time_step)

            except KeyboardInterrupt:
                # ctrl-C abort handling
                print('\nstopped')

            self._master.state = pysoem.INIT_STATE
            # request INIT state for all slaves
            self._master.write_state()
            self._master.close()
        else:
            print('slaves not found')


if __name__ == '__main__':

    print('minimal_example')

    if len(sys.argv) > 1:
        try:
            MinimalExample(sys.argv[1]).run()
        except Exception as expt:
            print(expt)
            sys.exit(1)
    else:
        print('usage: minimal_example ifname')
        sys.exit(1)
