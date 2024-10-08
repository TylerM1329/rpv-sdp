from time import sleep
import pigpio
import xmpp
import math

disableXMPP = False # Debug mode: disable XMPP functionality

# PIGPIO INITIALIZATION
pi = pigpio.pi('localhost', 8888, True)         # displays error when connection fails
print(pi)                                       # print handle

telemetryHdl = pi.i2c_open(1, 4, 0)             # open I2C for telemetry subsystem (addr 4)
if telemetryHdl < 0:
	print("> Failed to init telemetry subsystem!\n")
else:
	print("> telemetry subsystem \t\tINIT OK\n")

# XMPP VARIABLES
username = 'rpv'
passwd = '022223'
rsu = 'roza@0.0.0.0'
traffic = 'traffic@0.0.0.0'
msg = "ACCI#5231.453#1323.658#10#33.33#200#400" # placeholder message / 8 9

# XMPP INITIALIZATION
if disableXMPP != True:
    client = xmpp.Client('0.0.0.0')             # define client object
    client.connect(server=('10.8.0.4', 5222))   # connect to XMPP server (VPN IP)
    client.auth(username, passwd)               # authenticate
    client.sendInitPresence()                   # send roster request and initial presence


def main():
    # GET LAT, LON, RPM FROM TELEMETRY VIA I2C
    print("Requesting data")
    data = pi.i2c_read_device(telemetryHdl, 10)  # read telemetry data. stored as tuple: (count, data)
    # print(data)                               # print tuple
    byteArr = data[1]                           # extract bytearray from tuple (index 1)
    latByte = byteArr[0:4]                      # extract first bytearray (bytes 0~3) for latitude
    lonByte = byteArr[4:8]                      # extract second bytearray (bytes 4~7) for longitude
    rpmByte = byteArr[8]                        # extract last byte (byte 8) for rpm
    ctrlByte = byteArr[9]
    lat = int.from_bytes(latByte, 'little')     # convert bytearray to integer using little-endian
    lon = int.from_bytes(lonByte, 'little')     # 
    rpm = rpmByte
    ctrl = ctrlByte
    # check control byte to correct coordinates
    if ctrl == 1:
        lon = lon * -1                          # negate longitude 
    elif ctrl == 2:
        lat = lat * -1                          # negate latitude
    elif ctrl == 3:
        lon = lon * -1                          # negate longitude
        lat = lat * -1                          # negate latitude
    
    lat = lat / 1E6                             # scale back to normal by dividing by 1E6
    lon = lon / 1E6                             # scale back to normal by dividing by 1E6

    # # convert to decimal degrees to NMEA for latitude
    # latInt = int(lat) * 100
    # latDecimal = (lat % 1) * 60
    # latNMEA = latInt + latDecimal

    # # convert to decimal degrees to NMEA for longitude
    # lonInt = int(lon) * 100
    # lonDecimal = (lon % 1) * 60
    # lonNMEA = lonInt + lonDecimal

    print(f"DECIMAL DEGREES: lat: {lat}, lon: {lon}, rpm: {rpm}, ctrl: {ctrl}")
    # print(f"NMEA: lat: {latNMEA}, lon: {lonNMEA}, rpm: {rpm}, ctrl: {ctrl}")

    # CONSTRUCT MESSAGE
    #msg = f'ACCI#{latNMEA}#{lonNMEA}#0#{rpm}#200#400'   # vehicle status, lat, lon, alt, speed, length, width ! force decimal places later.
    msg = f'ACCI#{lat}#{lon}#10#{rpm}#200#400'
    print(msg)                                  # print message

    # SEND XMPP MESSAGE
    if disableXMPP != True:
        message = xmpp.Message(rsu, msg)         # construct message object using target user and message
        message.setAttr('type', 'chat')         # set message object as chat message
        client.send(message)                    # send message
        message = xmpp.Message(traffic, msg)         # construct message object using target user and message
        message.setAttr('type', 'chat')         # set message object as chat message
        client.send(message)                    # send message



    sleep(5) # 7.5
    

while True: # MAIN LOOP
    try:
        if __name__ == "__main__":
            main()
    except KeyboardInterrupt:
        print("\nCTRL+C pressed. Cleaning up...")
        pi.i2c_close(telemetryHdl)
        pi.stop()
        break
        
