import struct


packet = struct.pack(
    "<fff",
    12.4,
    56.1,
    100.0,
)

print(packet)
print(struct.unpack("<fff", packet))