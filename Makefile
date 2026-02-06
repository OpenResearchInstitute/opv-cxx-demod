CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall
BINDIR = bin

all: $(BINDIR)/opv-mod $(BINDIR)/opv-demod $(BINDIR)/opv-modem

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/opv-mod: src/opv-mod.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

$(BINDIR)/opv-demod: src/opv-demod.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

$(BINDIR)/opv-modem: src/opv-modem.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

clean:
	rm -rf $(BINDIR)

# Basic loopback test (mod → demod)
test: all
	@echo "=== Pipe Loopback Test ==="
	@$(BINDIR)/opv-mod -S W5NYV -B 5 | $(BINDIR)/opv-demod -s 2>&1 | grep -E "Station|Token|Summary"

# Raw mode loopback test
test-raw: all
	@echo "=== Raw Mode Loopback Test ==="
	@python3 -c "import sys; [sys.stdout.buffer.write(bytes([0,0,3,0x74,0x26,0x97,0xBB,0xAA,0xDD]+[0]*3+[(i+j)&0xFF for j in range(122)])) for i in range(3)]" | \
		$(BINDIR)/opv-mod -R | $(BINDIR)/opv-demod -s -r > /tmp/test_out.bin 2>/dev/null
	@python3 -c "import sys; [sys.stdout.buffer.write(bytes([0,0,3,0x74,0x26,0x97,0xBB,0xAA,0xDD]+[0]*3+[(i+j)&0xFF for j in range(122)])) for i in range(3)]" > /tmp/test_in.bin
	@diff /tmp/test_in.bin /tmp/test_out.bin && echo "✓ Raw mode: 3/3 frames match"

# Server loopback test (requires opv-modem running)
test-server: all
	@echo "=== Server Loopback Test ==="
	@echo "Start server with: $(BINDIR)/opv-modem -l"
	@echo "Then run: make test-server-send"

test-server-send:
	@python3 -c "\
import socket; \
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); \
sock.bind(('127.0.0.1', 0)); \
sock.settimeout(2.0); \
frame = bytes([0,0,3,0x74,0x26,0x97,0xBB,0xAA,0xDD]+[0]*3+list(range(122))); \
sock.sendto(frame, ('127.0.0.1', 57372)); \
data, _ = sock.recvfrom(256); \
print('✓ Server loopback: MATCH' if data == frame else '✗ MISMATCH')"

# Test RX mode (mod → modem -R → UDP)
test-rx: all
	@echo "=== RX Mode Test ==="
	@python3 -c "\
import socket, subprocess, sys, threading, time; \
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); \
sock.bind(('127.0.0.1', 57399)); \
sock.settimeout(3.0); \
count = [0]; \
def recv(): \
    while count[0] < 3: \
        try: \
            data, _ = sock.recvfrom(256); \
            if len(data) == 134: count[0] += 1 \
        except: break \
t = threading.Thread(target=recv); t.start(); \
time.sleep(0.2); \
mod = subprocess.Popen(['$(BINDIR)/opv-mod', '-S', 'TEST', '-B', '3'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL); \
modem = subprocess.Popen(['$(BINDIR)/opv-modem', '-R', '-r', '57399', '-q'], stdin=mod.stdout, stderr=subprocess.DEVNULL); \
mod.stdout.close(); modem.wait(); t.join(); \
print('✓ RX mode: {}/3 frames received via UDP'.format(count[0]))"

.PHONY: all clean test test-raw test-server test-server-send test-rx
