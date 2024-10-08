from time import sleep
import pigpio

pi = pigpio.pi('localhost', 8888, True)     # displays error when connection fails
print(pi)                                   # print handle

telemetryHdl = pi.i2c_open(1, 4, 0)         # open I2C for telemetry subsystem (addr 4)
if telemetryHdl < 0:
	print("> Failed to init telemetry subsystem!\n")
else:
	print("> telemetry subsystem \t\tINIT OK\n")

def main():
    print("Requesting data")
    data = pi.i2c_read_device(telemetryHdl, 9) # read telemetry data. stored as tuple: (count, data)
    print(type(data))
    byteArr = data[1]
    lat = byteArr[0:4]
    lon = byteArr[4:8]
    rpm = byteArr[8]
    print(int.from_bytes(lat, 'little'))
    print(int.from_bytes(lon, 'little'))
    print(rpm)
    sleep(1)
    

while True:
    try:
        if __name__ == "__main__":
            main()
    except KeyboardInterrupt:
        print("\nCTRL+C pressed. Cleaning up...")
        pi.i2c_close(telemetryHdl)
        pi.stop()
        break
        
