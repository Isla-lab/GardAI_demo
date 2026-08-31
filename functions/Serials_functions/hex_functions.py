import struct

# Decoders for u-blox UBX binary field types (U=unsigned, I=signed, X=bitfield,
# trailing digit = size in bytes), following u-blox's own protocol naming.
# Used by classes.classes_externals.DoppiaAntenna to parse the UBX_NAV_RELPOSNED
# message from the dual-antenna GPS. hex_to_i2/x1/x2 are kept for other UBX
# messages even though the current message doesn't need them.
def hex_to_u1(hex_str):
    """Convert hexadecimal string to U1 (unsigned 8-bit integer)."""
    return int(hex_str, 16)

def hex_to_i1(hex_str):
    """Convert hexadecimal string to I1 (signed 8-bit integer)."""
    value = int(hex_str, 16)
    if value > 0x7F:
        value -= 0x100
    return value

def hex_to_x1(hex_str):
    """Convert hexadecimal string to X1 (8-bit bitfield)."""
    return int(hex_str, 16)

def hex_to_u2(hex_str):
    """Convert hexadecimal string to U2 (unsigned 16-bit integer, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<H', value)[0]

def hex_to_i2(hex_str):
    """Convert hexadecimal string to I2 (signed 16-bit integer, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<h', value)[0]

def hex_to_x2(hex_str):
    """Convert hexadecimal string to X2 (16-bit bitfield, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<H', value)[0]

def hex_to_u4(hex_str):
    """Convert hexadecimal string to U4 (unsigned 32-bit integer, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<I', value)[0]

def hex_to_i4(hex_str):
    """Convert hexadecimal string to I4 (signed 32-bit integer, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<i', value)[0]

def hex_to_x4(hex_str):
    """Convert hexadecimal string to X4 (32-bit bitfield, little-endian)."""
    value = bytes.fromhex(hex_str)
    return struct.unpack('<I', value)[0]
