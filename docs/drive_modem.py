#!/usr/bin/env python3
# drive_modem.py — feed a few real OVP frames to opv-modem, then go quiet so the
# session runs its full lifecycle:  preamble -> real frames -> dummy (hang) -> postamble.
#
#   Terminal 1:  ./bin/opv-modem -t -v -p 57372 | ./bin/opv-demod -s -c
#   Terminal 2:  python3 drive_modem.py
import socket, time

# 12-byte OVP header (W5NYV / token 0xBBAADD), matching make test-server-send
HDR = bytes([0,0,3, 0x74,0x26,0x97, 0xBB,0xAA,0xDD, 0,0,0])

def frame(n):
    payload = bytes((n + i) & 0xFF for i in range(122))   # walking ramp, 122 bytes
    return HDR + payload                                   # 134 bytes total

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for n in range(4):                      # 4 real frames, ~40 ms apart
    s.sendto(frame(n), ('127.0.0.1', 57372))
    print(f"sent real frame {n}")
    time.sleep(0.040)

print("going quiet — hang timer (~25 frames ≈ 1 s) now fires dummies, then the postamble")
time.sleep(2.0)                         # outlast the hang so dummies + postamble appear
