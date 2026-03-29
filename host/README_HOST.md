Host (Raspberry Pi) control scripts
----------------------------------

Requirements:
  pip install pyserial

Files:
  direct_sine_test_host.py  - sim_run (local) and hw_run (sends START/STOP to ESP32)
  run_machine_host.py       - interactive CLI to send simple commands to ESP32

Usage:
  python3 direct_sine_test_host.py --mode sim --freq 1.0 --amp 5.0 --duration 6.0
  python3 direct_sine_test_host.py --mode hw --freq 1.0 --amp 5.0 --duration 20.0 --serial /dev/ttyUSB0

Interactive:
  python3 run_machine_host.py
