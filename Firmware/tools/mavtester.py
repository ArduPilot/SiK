#!/usr/bin/env python

'''
test MAVLink performance between two radios
'''

import sys, time, os

from optparse import OptionParser
parser = OptionParser("mavtester.py [options]")

parser.add_option("--baudrate", type='int',
                  help="connection baud rate", default=57600)
parser.add_option("--port1", default=None, help="serial port 1")
parser.add_option("--port2", default=None, help="serial port 2")
parser.add_option("--rate", default=4, type='float', help="initial stream rate")
parser.add_option("--override-rate", default=1, type='float', help="RC_OVERRIDE rate")
parser.add_option("--show", action='store_true', default=False, help="show messages")
parser.add_option("--rtscts", action='store_true', default=False, help="enable RTSCTS hardware flow control")

(opts, args) = parser.parse_args()

from pymavlink import mavutil

if opts.port1 is None or opts.port2 is None:
    print("You must specify two serial ports")
    sys.exit(1)

# create GCS connection
gcs = mavutil.mavlink_connection(opts.port1, baud=opts.baudrate, input=True)
gcs.setup_logfile('gcs.tlog')
vehicle = mavutil.mavlink_connection(opts.port2, baud=opts.baudrate, input=False)
vehicle.setup_logfile('vehicle.tlog')

if opts.rtscts:
    gcs.set_rtscts(True)
    vehicle.set_rtscts(True)

start_time = time.time()
last_vehicle_send = time.time()
last_gcs_send = time.time()
last_override_send = time.time()
vehicle_lat = 0
gcs_lat = 0

def send_telemetry():
    '''
    send telemetry packets from the vehicle to
    the GCS. This emulates the typical pattern of telemetry in
    ArduPlane 2.75 in AUTO mode
    '''    
    global last_vehicle_send, vehicle_lat
    now = time.time()
    # send at rate specified by user. This doesn't do rate adjustment yet (APM does adjustment
    # based on RADIO packets)
    if now - last_vehicle_send < 1.0/opts.rate:
        return
    last_vehicle_send = now
    time_usec = int((now - start_time) * 1.0e6)
    time_ms = time_usec // 1000

    vehicle.mav.heartbeat_send(1, 3, 217, 10, 4, 3)
    vehicle.mav.global_position_int_send(time_ms, vehicle_lat, 1491642131, 737900, 140830, 2008, -433, 224, 35616)
    vehicle.mav.rc_channels_scaled_send(time_boot_ms=time_ms, port=0, chan1_scaled=280, chan2_scaled=3278, chan3_scaled=-3023, chan4_scaled=0, chan5_scaled=0, chan6_scaled=0, chan7_scaled=0, chan8_scaled=0, rssi=0)
    vehicle.mav.servo_output_raw_send(time_usec=time_usec, port=0, servo1_raw=1470, servo2_raw=1628, servo3_raw=1479, servo4_raw=1506, servo5_raw=1500, servo6_raw=1556, servo7_raw=1500, servo8_raw=1500)
    vehicle.mav.rc_channels_raw_send(time_boot_ms=time_ms, port=0, chan1_raw=1470, chan2_raw=1618, chan3_raw=1440, chan4_raw=1509, chan5_raw=1168, chan6_raw=1556, chan7_raw=1224, chan8_raw=994, rssi=0)
    vehicle.mav.raw_imu_send(time_usec, 562, 382, -3917, -3330, 3445, 35, -24, 226, -523)
    vehicle.mav.scaled_pressure_send(time_boot_ms=time_ms, press_abs=950.770019531, press_diff=-0.0989062488079, temperature=463)
    vehicle.mav.sensor_offsets_send(mag_ofs_x=-68, mag_ofs_y=-143, mag_ofs_z=-34, mag_declination=0.206146687269, raw_press=95077, raw_temp=463, gyro_cal_x=-0.063114002347, gyro_cal_y=0.0479440018535, gyro_cal_z=0.0190890002996, accel_cal_x=0.418922990561, accel_cal_y=0.284875005484, accel_cal_z=-0.436598002911)
    vehicle.mav.sys_status_send(onboard_control_sensors_present=64559, onboard_control_sensors_enabled=64559, onboard_control_sensors_health=64559, load=82, voltage_battery=11877, current_battery=0, battery_remaining=100, drop_rate_comm=0, errors_comm=0, errors_count1=0, errors_count2=0, errors_count3=0, errors_count4=0)
    vehicle.mav.mission_current_send(seq=1)
    vehicle.mav.gps_raw_int_send(time_usec=time_usec, fix_type=3, lat=-353637616, lon=1491642012, alt=737900, eph=169, epv=65535, vel=2055, cog=34782, satellites_visible=9)
    vehicle.mav.nav_controller_output_send(nav_roll=0.0, nav_pitch=0.319999992847, nav_bearing=-18, target_bearing=343, wp_dist=383, alt_error=-37.0900001526, aspd_error=404.800537109, xtrack_error=1.52732038498)
    vehicle.mav.attitude_send(time_boot_ms=time_ms, roll=0.00283912196755, pitch=-0.0538846850395, yaw=-0.0708072632551, rollspeed=0.226980209351, pitchspeed=-0.00743395090103, yawspeed=-0.154820173979)
    vehicle.mav.vfr_hud_send(airspeed=21.9519939423, groundspeed=20.5499992371, heading=355, throttle=35, alt=737.900024414, climb=-0.784280121326)
    vehicle.mav.ahrs_send(omegaIx=0.000540865410585, omegaIy=-0.00631708558649, omegaIz=0.00380697473884, accel_weight=0.0, renorm_val=0.0, error_rp=0.094664350152, error_yaw=0.0121578350663)
    vehicle.mav.hwstatus_send(Vcc=0, I2Cerr=0)
    vehicle.mav.wind_send(direction=27.729429245, speed=5.35723495483, speed_z=-1.92264056206)

    vehicle_lat += 1

def send_GCS():
    '''
    send GCS heartbeat messages
    '''
    global last_gcs_send
    now = time.time()
    if now - last_gcs_send < 1.0:
        return
    gcs.mav.heartbeat_send(1, 6, 0, 0, 0, 0)
    last_gcs_send = now

def send_override():
    '''
    send RC_CHANNELS_OVERRIDE messages from GCS
    '''
    global last_override_send
    now = time.time()
    if opts.override_rate == 0:
        return
    if now - last_override_send < 1.0/opts.override_rate:
        return
    time_ms = int((now - start_time) * 1.0e3)
    time_ms_low  = time_ms % 65536
    time_ms_high = (time_ms - time_ms_low) // 65536
    gcs.mav.rc_channels_override_send(1, 2, time_ms_low, time_ms_high, 0, 0, 0, 0, 0, 0)
    last_override_send = now

def process_override(m):
    '''
    process an incoming RC_CHANNELS_OVERRIDE message, measuring latency
    '''
    now = time.time()
    time_ms_sent = m.chan2_raw*65536 + m.chan1_raw
    time_ms = int((now - start_time) * 1.0e3)
    latency = time_ms - time_ms_sent

    stats.latency_count += 1
    stats.latency_total += latency
    if stats.latency_min == 0 or latency < stats.latency_min:
        stats.latency_min = latency
    if latency > stats.latency_max:
        stats.latency_max = latency
    
def recv_vehicle():
    '''
    receive packets in the vehicle
    '''
    m = vehicle.recv_match(blocking=False)
    if not m:
        # timeout
        return False
    if m.get_type() == 'BAD_DATA':
        stats.vehicle_bad_data += 1
        return True
    if opts.show:
        print(m)
    stats.vehicle_received += 1
    if m.get_type() in ['RADIO','RADIO_STATUS']:
        stats.vehicle_radio_received += 1
        stats.vehicle_txbuf = m.txbuf
    if m.get_type() == 'RC_CHANNELS_OVERRIDE':
        process_override(m)
    return True


def recv_GCS():
    '''
    receive packets in the GCS
    '''
    m = gcs.recv_match(blocking=False)
    if not m:
        # timeout
        return False   
    if m.get_type() == 'BAD_DATA':
        stats.gcs_bad_data += 1
        return True
    if m.get_type() == 'GLOBAL_POSITION_INT':
        global gcs_lat
        if gcs_lat != m.lat:
            print("Lost %u GLOBAL_POSITION_INT messages" % (m.lat - gcs_lat))
            gcs_lat = m.lat
        gcs_lat += 1
    if opts.show:
        print(m)
    stats.gcs_received += 1        
    if m.get_type() in ['RADIO','RADIO_STATUS']:
        stats.gcs_radio_received += 1            
        stats.gcs_txbuf = m.txbuf
    return True
    


class PacketStats(object):
    '''
    class to hold statistics on the link
    '''
    def __init__(self):
        self.gcs_sent = 0
        self.vehicle_sent = 0
        self.gcs_received = 0
        self.vehicle_received = 0
        self.gcs_radio_received = 0
        self.vehicle_radio_received = 0
        self.gcs_last_bytes_sent = 0
        self.vehicle_last_bytes_sent = 0
        self.latency_count = 0
        self.latency_total = 0
        self.latency_min = 0
        self.latency_max = 0
        self.vehicle_bad_data = 0
        self.gcs_bad_data = 0
        self.last_gcs_radio = None
        self.last_vehicle_radio = None
        self.vehicle_txbuf = 100
        self.gcs_txbuf = 100

    def __str__(self):
        gcs_bytes_sent = gcs.mav.total_bytes_sent - self.gcs_last_bytes_sent
        vehicle_bytes_sent = vehicle.mav.total_bytes_sent - self.vehicle_last_bytes_sent
        self.gcs_last_bytes_sent = gcs.mav.total_bytes_sent
        self.vehicle_last_bytes_sent = vehicle.mav.total_bytes_sent

        avg_latency = 0
        if stats.latency_count != 0:
            avg_latency = stats.latency_total / stats.latency_count
        
        return "Veh:%u/%u/%u  GCS:%u/%u/%u  pend:%u rates:%u/%u lat:%u/%u/%u bad:%u/%u txbuf:%u/%u loss:%u:%u%%/%u:%u%%" % (
            self.vehicle_sent,
            self.vehicle_received,
            self.vehicle_received - self.vehicle_radio_received,
            self.gcs_sent,
            self.gcs_received,
            self.gcs_received - self.gcs_radio_received,
            self.vehicle_sent - (self.gcs_received - self.gcs_radio_received),
            gcs_bytes_sent,
            vehicle_bytes_sent,
            stats.latency_min,
            stats.latency_max,
            avg_latency,
            self.vehicle_bad_data,
            self.gcs_bad_data,
            self.vehicle_txbuf,
            self.gcs_txbuf,
            gcs.mav_loss,
            gcs.packet_loss(),
            vehicle.mav_loss,
            vehicle.packet_loss())
                                 
    
'''
main code
'''
last_report = time.time()
stats = PacketStats()

while True:

    send_telemetry()
    stats.vehicle_sent = vehicle.mav.total_packets_sent

    send_GCS()
    send_override()
    stats.gcs_sent = gcs.mav.total_packets_sent

    if not recv_vehicle() and not recv_GCS():
        time.sleep(0.01)

    if time.time() - last_report >= 1.0:
        print(stats)
        last_report = time.time()
