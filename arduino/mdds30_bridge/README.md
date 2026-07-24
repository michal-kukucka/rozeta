# mdds30_bridge

Arduino UNO firmware that turns USB-serial commands into PWM/DIR signals for a
Cytron MDDS30 (SmartDriveDuo-30). This is the default drive path for a Rozeta
robot — **the board must be flashed with this sketch once before the robot can
move.**

Upload with the Arduino IDE (Tools → Board → Arduino UNO, then Upload) or:

```bash
arduino-cli compile --fqbn arduino:avr:uno arduino/mdds30_bridge
arduino-cli upload  -p /dev/cu.usbmodem14101 --fqbn arduino:avr:uno arduino/mdds30_bridge
```

Full procedure, wiring table, MDDS30 DIP-switch positions, serial protocol and
troubleshooting: [`docs/arduino_mdds30_bridge.md`](../../docs/arduino_mdds30_bridge.md).

Protocol summary — 115200 baud, 8N1, `\n`-terminated ASCII lines:
`M L=<-100..100> R=<-100..100>`, `STOP`, `PING`, `STATUS`, `TIMEOUT=<ms>`.
A 300 ms communication watchdog stops both motors if commands stop arriving.
