import smbus2

bus = smbus2.SMBus(1) # Create an SMBus instance
AK8963_ADDRESS = 0x0C
WHO_AM_I = 0x00

who_am_i = bus.read_byte_data(AK8963_ADDRESS, WHO_AM_I)
print(hex(who_am_i)) # It should print: 0x48