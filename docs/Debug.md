# Linux debug utilities

## CAN interface

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate  250000
sudo ip link set up can0


candump can0 -xct z -n 20
ip -details link show can0
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate  250000
```

```bash
cansend can0  001#1122334455667788
```

## System log
```bash
tail -f /var/log/syslog
dmesg | tail
```

## Odrive check
```bash
odrivetool
dump_errors(odrv0)
```

## I2C

```bash
i2cdetect -l
i2cdump i2cbus chip-address
i2cget i2cbus chip-address data-address
i2cset i2cbus chip-address data-address value
```