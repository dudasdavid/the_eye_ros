

def calcCheckSum(input_byte_array):

    return input_byte_array + bytes([sum(input_byte_array) & 0xFF])

def padded_hex(s):
    return '0x' + s[2:].zfill(8)

def readSteps(motor_id):
    packet = bytearray()

    if motor_id == "zoom":
        packet.append(0xe0)
    elif motor_id == "focus":
        packet.append(0xe1)
    else:
        raise TypeError("Wrong motor ID")
    
    packet.append(0x33)

    packet = calcCheckSum(packet)

    return packet

def processSteps(motor_id, input_byte_array):
    #print(motor_id, input_byte_array)

    input_bytes = b''
    for i in range(1,5):
        input_bytes += input_byte_array[i]

    steps = int.from_bytes(input_bytes, byteorder='big', signed=True)
    #print(int.from_bytes(input_bytes, byteorder='big', signed=True))

    return steps

def setSpeed(motor_id, dir, speed):

    if speed > 127:speed = 127
    if speed < 1: speed = 1

    packet = bytearray()

    if motor_id == "zoom":
        packet.append(0xe0)
    elif motor_id == "focus":
        packet.append(0xe1)
    else:
        raise TypeError("Wrong motor ID")
    
    # set speed packet header    
    packet.append(0xf6)

    if dir == "cw":
        packet.append(128 | speed)

    elif dir == "ccw":
        packet.append(0 | speed) 

    packet = calcCheckSum(packet)

    return packet 

def stopMotor(motor_id):
    packet = bytearray()

    if motor_id == "zoom":
        packet.append(0xe0)
    elif motor_id == "focus":
        packet.append(0xe1)
    else:
        raise TypeError("Wrong motor ID")
    
    # stop motor packet header    
    packet.append(0xf7)

    packet = calcCheckSum(packet)

    return packet 

def enableMotor(motor_id, ena):
    packet = bytearray()

    if motor_id == "zoom":
        packet.append(0xe0)
    elif motor_id == "focus":
        packet.append(0xe1)
    else:
        raise TypeError("Wrong motor ID")
    
    # enable motor packet header    
    packet.append(0xf3)
    
    if ena == 1:
        packet.append(0x01)
    else:
        packet.append(0x00)

    packet = calcCheckSum(packet)

    return packet 

def moveToAngle(motor_id, dir, angle, speed):
    packet = bytearray()

    if motor_id == "zoom":
        packet.append(0xe0)
    elif motor_id == "focus":
        packet.append(0xe1)
    else:
        raise TypeError("Wrong motor ID")
    
    
    #packet.append(0xe1)
    packet.append(0xfd)

    if dir == "cw":
        packet.append(128 | speed)

    elif dir == "ccw":
        packet.append(0 | speed)

    hex_str = padded_hex(hex(angle))
    packet.append(int("0x"+hex_str[2:4],0))
    packet.append(int("0x"+hex_str[4:6],0))
    packet.append(int("0x"+hex_str[6:8],0))
    packet.append(int("0x"+hex_str[8:10],0))

    packet = calcCheckSum(packet)

    return packet
    