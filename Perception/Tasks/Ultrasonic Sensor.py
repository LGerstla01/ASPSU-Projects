# Shortest possible measurement frequency calculation for time-multiplexed ultrasonic sensors

def calc_shortest_measurement_frequency(
    num_sensors, 
    max_range_m=5.5, 
    speed_of_sound_m_s=343.0,
    pulse_width_s=300e-6,         # 300 µs
    fade_away_time_s=700e-6       # 700 µs
):
    """
    Calculate the shortest possible measurement frequency for time-multiplexed ultrasonic sensors.
    Args:
        num_sensors (int): Number of sensors in the setup.
        max_range_m (float): Maximum range of the sensor in meters.
        speed_of_sound_m_s (float): Speed of sound in m/s.
        pulse_width_s (float): Effective width of emitted pulse in seconds.
        fade_away_time_s (float): Time for signal fade away in seconds.
    Returns:
        float: Shortest possible measurement frequency in Hz.
        float: Time between measurements for each sensor in seconds.
    """
    # Time for sound to travel to max range and back (round trip)
    t_echo = 2 * max_range_m / speed_of_sound_m_s  # seconds
    # Add pulse width and fade away time as additional dead time per sensor
    t_sensor = t_echo + pulse_width_s + fade_away_time_s
    # Each sensor must wait for the echo and dead time before the next can transmit
    t_cycle = num_sensors * t_sensor  # total time for all sensors to complete one measurement
    freq = 1.0 / t_cycle  # Hz
    return freq, t_sensor

if __name__ == "__main__":
    sensor_setups = [1, 2, 4, 6]
    print("Number of sensors | Shortest frequency (Hz) | Time per sensor (ms)")
    for n in sensor_setups:
        freq, t_sensor = calc_shortest_measurement_frequency(n)
        print(f"{n:17d} | {freq:21.2f} | {t_sensor*1000:18.2f}")
