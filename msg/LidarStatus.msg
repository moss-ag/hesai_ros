# Temperature
# 0: Bottom circuit board T1
# 1: Bottom circuit board T2
# 2: Laser emitting board RT_L1
# 3: Laser emitting board RT_L2
# 4: Laser Receiving board RT_R
# 5: Laser Receiving board RT2
# 6: Top circuit RT3
# 7: Top circuit RT4


uint32 system_uptime
uint16 motor_speed # rpm
float32[8] temperature # celsius
uint8 gps_pps_lock
uint8 gps_gprmc_status
uint32 startup_times
uint32 total_operation_time
uint8 ptp_status
uint8 ptp_offset
