from airis.payload_hw import PHASE_ASCENT, PHASE_FLOAT, ThermalWatchdog, heater_policy


def test_heater_policy():
    assert heater_policy(-20.0, PHASE_FLOAT).on
    assert not heater_policy(5.0, PHASE_FLOAT).on
    assert not heater_policy(None, PHASE_FLOAT).on             # dead RTD at float: off
    assert heater_policy(None, PHASE_ASCENT, duty_tick=0).on   # dead RTD in ascent: duty-cycled on
    assert not heater_policy(None, PHASE_ASCENT, duty_tick=1).on


def test_watchdog_shuts_down_above_limit():
    calls = []
    heater = []
    wd = ThermalWatchdog(read_zones=lambda: {"cpu": 70.0, "gpu": 89.0}, read_rtd=lambda: (-30.0, 10.0),
                         read_camera=lambda: 41.0, set_heater=heater.append, shutdown=lambda: calls.append("off"),
                         flight_phase=lambda: PHASE_FLOAT, limit_c=88.0)
    s = wd.tick()
    assert s.jetson_peak_c == 89.0 and s.heater_on and heater == [True]
    assert calls == ["off"]


def test_watchdog_survives_rtd_errors():
    def boom():
        raise RuntimeError("MAX31865 fault")
    wd = ThermalWatchdog(read_zones=lambda: {}, read_rtd=boom, flight_phase=lambda: PHASE_ASCENT)
    s = wd.tick()
    assert s.sensor_c is None and s.jetson_peak_c is None and "ascent" in s.reason
