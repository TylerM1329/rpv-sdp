number = 999999999
bytes = number.to_bytes((number.bit_length() + 7) // 8, 'big')
print(type(bytes))
print(int.from_bytes(bytes, 'big'))