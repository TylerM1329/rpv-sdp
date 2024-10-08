message = "0#-114.072869#37.442312#1637.4#8.2"

data = message.split("#")
print(type(data))
print("Original message:", message)
print("\n")
print("Status:", int(data[0]))
print("Latitude:", float(data[1]))
print("Longitude:", float(data[2]))
print("Altitude:", float(data[3]))
print("Speed:", float(data[4]))
