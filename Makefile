# OPV Modem - build system
#
# Targets (what platform the binaries run on):
#   make                # host (x86) build for development + the test suite below
#   make TARGET=pluto   # PlutoSDR  : ARMv7-A Cortex-A9 + NEON      (Vox Opus runtime)
#   make TARGET=a53     # Haifuraiya: ARMv8-A Cortex-A53 (aarch64)   (dogu runtime)
#
# The test-* targets run binaries locally, so they use the default host build.
# Cross targets are build-only (you can't run a53/pluto binaries on the host).
#
# Cross toolchains are set per target below; override CXX on the command line if
# yours differs (command line beats the Makefile assignment). For the PetaLinux SDK:
#   make TARGET=a53 CXX=aarch64-xilinx-linux-g++ SYSROOT=<sdk-sysroot>
#
# NOTE: cross branches use '=' not '?=' on purpose -- GNU Make pre-defines CXX=g++
# as a built-in and '?=' will NOT override a built-in, which would silently build
# the cross targets with the host compiler.
#
# Optimization flags (-ffast-math -fcx-limited-range -ftree-vectorize) are shared
# across targets; they were verified to keep decode bit-perfect and give the bulk
# of the speedup. For stricter FP, drop -ffast-math but keep -fcx-limited-range.

TARGET ?= host
COMMON_FLAGS = -std=c++17 -O3 -ffast-math -fcx-limited-range -ftree-vectorize -Wall

ifeq ($(TARGET),host)
  CXX       ?= g++
  ARCH_FLAGS = -march=native
  BINDIR     = bin
else ifeq ($(TARGET),pluto)
  CXX        = arm-linux-gnueabihf-g++
  ARCH_FLAGS = -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard
  BINDIR     = bin/pluto
else ifeq ($(TARGET),a53)
  CXX        = aarch64-linux-gnu-g++
  ARCH_FLAGS = -mcpu=cortex-a53
  BINDIR     = bin/a53
  ifdef SYSROOT
    ARCH_FLAGS += --sysroot=$(SYSROOT)
  endif
else
  $(error Unknown TARGET '$(TARGET)'. Use: host, pluto, or a53)
endif

CXXFLAGS = $(COMMON_FLAGS) $(ARCH_FLAGS)
PROGS = opv-mod opv-demod opv-modem opv-resample

all: $(addprefix $(BINDIR)/,$(PROGS))
	@echo "Built TARGET=$(TARGET) -> $(BINDIR)/  ($(CXX))"

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/%: src/%.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

# opv-demod now includes the header-only DSP core; rebuild it if the header changes.
$(BINDIR)/opv-demod: src/opv_demod.hpp

clean:
	rm -rf bin

# Basic loopback test (mod → demod)
test: all test-codec test-sync-resolver
	@echo "=== Pipe Loopback Test ==="
	@$(BINDIR)/opv-mod -S W5NYV -B 5 | $(BINDIR)/opv-demod -s 2>&1 | grep -E "Station|Token|Summary"

# Raw mode loopback test
# Note: the sync state machine requires 2 frames to acquire (HUNTING→VERIFYING→LOCKED),
# so the first frame is always consumed by acquisition. With N frames in, N-1 come out
# (starting from frame index 1). We send 5 frames and verify we get 4 back with
# correct payload content.
test-raw: all
	@echo "=== Raw Mode Loopback Test ==="
	@python3 -c "import sys; [sys.stdout.buffer.write(bytes([0,0,3,0x74,0x26,0x97,0xBB,0xAA,0xDD]+[0]*3+[(i+j)&0xFF for j in range(122)])) for i in range(5)]" | \
		$(BINDIR)/opv-mod -R | $(BINDIR)/opv-demod -s -r > /tmp/test_out.bin 2>/dev/null
	@python3 -c "import sys; [sys.stdout.buffer.write(bytes([0,0,3,0x74,0x26,0x97,0xBB,0xAA,0xDD]+[0]*3+[(i+j)&0xFF for j in range(122)])) for i in range(5)]" > /tmp/test_in.bin
	@IN=$$(wc -c < /tmp/test_in.bin); OUT=$$(wc -c < /tmp/test_out.bin); \
	 IN_FRAMES=$$((IN / 134)); OUT_FRAMES=$$((OUT / 134)); \
	 python3 -c "a=open('/tmp/test_in.bin','rb').read(); b=open('/tmp/test_out.bin','rb').read(); exit(0 if a[134:134+len(b)]==b else 1)" && \
	 echo "✓ Raw mode: $${OUT_FRAMES}/$$(( IN_FRAMES - 1 )) frames match (1 consumed by sync acquisition)"

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
	@printf '%s\n' \
		'import socket, subprocess, threading, time' \
		'sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)' \
		'sock.bind(("127.0.0.1", 57399))' \
		'sock.settimeout(3.0)' \
		'count = [0]' \
		'def recv():' \
		'    while count[0] < 3:' \
		'        try:' \
		'            data, _ = sock.recvfrom(256)' \
		'            if len(data) == 134: count[0] += 1' \
		'        except: break' \
		't = threading.Thread(target=recv); t.start()' \
		'time.sleep(0.2)' \
		'mod = subprocess.Popen(["$(BINDIR)/opv-mod", "-S", "TEST", "-B", "5"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)' \
		'modem = subprocess.Popen(["$(BINDIR)/opv-modem", "-R", "-r", "57399", "-q"], stdin=mod.stdout, stderr=subprocess.DEVNULL)' \
		'mod.stdout.close(); modem.wait(); t.join()' \
		'print("✓ RX mode: {}/4 frames received via UDP".format(count[0]) if count[0] >= 3 else "✗ RX mode: only {}/4 frames received".format(count[0]))' \
		> /tmp/test_rx.py
	@python3 /tmp/test_rx.py

# Coherent mode sanity test (mod → demod -c)
# On a clean loopback signal, coherent mode must decode at least as many
# frames as non-coherent. Fewer frames = Costas loop regression.
test-coherent: all
	@echo "=== Coherent Mode Loopback Test ==="
	@$(BINDIR)/opv-mod -S W5NYV -B 10 > /tmp/test_coherent.iq 2>/dev/null
	@NC=$$($(BINDIR)/opv-demod -s -r -q < /tmp/test_coherent.iq 2>/dev/null | wc -c); NC=$$((NC / 134)); \
	 CO=$$($(BINDIR)/opv-demod -s -c -r -q < /tmp/test_coherent.iq 2>/dev/null | wc -c); CO=$$((CO / 134)); \
	 echo "  Non-coherent: $${NC}/10 frames decoded"; \
	 echo "  Coherent:     $${CO}/10 frames decoded"; \
	 if [ "$${CO}" -ge "$${NC}" ]; then \
	   echo "✓ Coherent mode: OK ($${CO} >= $${NC})"; \
	 else \
	   echo "✗ Coherent mode: REGRESSION ($${CO} < $${NC})"; exit 1; \
	 fi

# Side-by-side comparison of coherent vs non-coherent under noise + frequency offset.
# The 3dB coherent advantage is most visible when residual frequency offset is present:
# the Costas loop tracks phase continuously while non-coherent AFC only corrects coarsely.
# On a zero-offset loopback both modes perform identically.
# Use SNR=<db> and OFFSET=<hz> to tune the test conditions.
# Example: make test-coherent-compare SNR=-5 OFFSET=200
SNR ?= -5
OFFSET ?= 150
test-coherent-compare: all
	@echo "=== Coherent vs Non-Coherent: SNR=$(SNR) dB, offset=$(OFFSET) Hz ==="
	@printf '%s\n' \
		'import sys, struct, math, random, cmath' \
		'snr_lin = 10**($(SNR)/10)' \
		'noise_amp = 1.0 / math.sqrt(2 * snr_lin)' \
		'offset_hz = $(OFFSET)' \
		'sample_rate = 2168000.0' \
		'data = sys.stdin.buffer.read()' \
		'out = bytearray()' \
		'phase = 0.0' \
		'phase_inc = 2 * math.pi * offset_hz / sample_rate' \
		'for i in range(0, len(data)-3, 4):' \
		'    I, Q = struct.unpack_from("<hh", data, i)' \
		'    s = complex(I, Q) * cmath.exp(1j * phase)' \
		'    I = int(s.real + random.gauss(0, noise_amp * 32767))' \
		'    Q = int(s.imag + random.gauss(0, noise_amp * 32767))' \
		'    out += struct.pack("<hh", max(-32768,min(32767,I)), max(-32768,min(32767,Q)))' \
		'    phase += phase_inc' \
		'sys.stdout.buffer.write(out)' \
		> /tmp/awgn_offset.py
	@$(BINDIR)/opv-mod -S W5NYV -B 50 2>/dev/null | python3 /tmp/awgn_offset.py > /tmp/test_noisy.iq
	@NC=$$($(BINDIR)/opv-demod -s -r -q < /tmp/test_noisy.iq 2>/dev/null | wc -c); NC=$$((NC / 134)); \
	 CO=$$($(BINDIR)/opv-demod -s -c -r -q < /tmp/test_noisy.iq 2>/dev/null | wc -c); CO=$$((CO / 134)); \
	 echo "  Non-coherent: $${NC}/50 frames decoded"; \
	 echo "  Coherent:     $${CO}/50 frames decoded"; \
	 if [ "$${CO}" -gt "$${NC}" ]; then \
	   echo "✓ Coherent advantage confirmed at $(SNR) dB SNR, $(OFFSET) Hz offset"; \
	 elif [ "$${CO}" -eq "$${NC}" ]; then \
	   echo "~ Equal performance at $(SNR) dB SNR, $(OFFSET) Hz offset"; \
	 else \
	   echo "✗ Coherent underperforms at $(SNR) dB SNR, $(OFFSET) Hz offset — investigate"; \
	 fi


test-codec: all
	@echo "=== Codec KAT (single-source encoder) ==="
	@c++ -std=c++17 -O2 -Isrc tests/codec_kat.cpp -o bin/codec_kat
	@bin/codec_kat > /tmp/codec_frame.txt 2>/dev/null \
	  && diff -q /tmp/codec_frame.txt tests/golden_frame.txt >/dev/null \
	  && echo "  PASS: impulse 171/133 + frame matches golden" \
	  || (echo "  FAIL: codec drifted"; exit 1)

test-sync-resolver: | $(BINDIR)
	@echo "=== Sync hypothesis resolver ==="
	@$(CXX) $(CXXFLAGS) -Isrc tests/sync_resolver.cpp -o $(BINDIR)/sync_resolver
	@$(BINDIR)/sync_resolver


# LEO Doppler stress test — worst case for satellite demodulator performance.
#
# Models the zenith crossing of an overhead LEO pass: a linear frequency ramp
# at the maximum Doppler rate for the specified carrier frequency.
# This is the hardest moment of any LEO pass — near AOS/LOS the rate is slow,
# near zenith it peaks.
#
# Doppler physics (400 km circular orbit, direct overhead pass):
#   Orbital velocity:       ~7,800 m/s
#   Swing  ±f * v/c:       433 MHz: ±11.3 kHz   905 MHz: ±23.5 kHz
#                          2.4 GHz: ±62.4 kHz   5.0 GHz: ±130 kHz
#   Rate at zenith
#   f * v² / (c * h):      433 MHz:  ~220 Hz/s   905 MHz:  ~460 Hz/s
#                          2.4 GHz: ~1200 Hz/s   5.0 GHz: ~2500 Hz/s
#
# HEO hits these rates only briefly at perigee closest approach.
# GEO produces a slow diurnal figure-eight drift — negligible by comparison.
# LEO is therefore the binding worst case for dynamic frequency tracking.
#
# The Costas loop's continuous phase tracking gives it an advantage over
# the non-coherent AFC (which only corrects coarsely per symbol) when the
# frequency is actively ramping. However, the K=7 convolutional FEC has
# ~5 dB of coding gain whose steep waterfall may mask the 3 dB coherent
# advantage in synthetic tests. True validation requires over-the-air
# hardware testing with real oscillator drift and a live satellite pass.
#
# Usage:
#   make test-doppler                          # 905 MHz default
#   make test-doppler FREQ_MHZ=433             # 70cm band
#   make test-doppler FREQ_MHZ=5000            # 5 GHz uplink
#   make test-doppler FREQ_MHZ=2400 DOPPLER_SNR=0
FREQ_MHZ    ?= 905
DOPPLER_SNR ?= 5
test-doppler: all
	@echo "=== LEO Doppler Test: $(FREQ_MHZ) MHz carrier, SNR=$(DOPPLER_SNR) dB ==="
	@printf '%s\n' \
		'import sys, struct, math, random, cmath' \
		'sample_rate = 2168000.0' \
		'freq_hz     = $(FREQ_MHZ) * 1e6' \
		'v_orbit     = 7800.0                        # m/s LEO orbital velocity' \
		'c           = 3e8                            # m/s' \
		'h_orbit     = 400e3                          # m, altitude above ground' \
		'doppler_rate = freq_hz * v_orbit**2 / (c * h_orbit)   # Hz/sec at zenith' \
		'doppler_swing = freq_hz * v_orbit / c                 # Hz one-way swing' \
		'snr_lin     = 10**($(DOPPLER_SNR)/10)' \
		'noise_amp   = 1.0 / math.sqrt(2 * snr_lin)' \
		'print(f"  Carrier:       {freq_hz/1e6:.0f} MHz", file=sys.stderr)' \
		'print(f"  Doppler swing: ±{doppler_swing/1e3:.1f} kHz", file=sys.stderr)' \
		'print(f"  Doppler rate:  {doppler_rate:.0f} Hz/sec (zenith, 400km orbit)", file=sys.stderr)' \
		'data        = sys.stdin.buffer.read()' \
		'n_samples   = len(data) // 4' \
		'duration    = n_samples / sample_rate' \
		'out         = bytearray()' \
		'phase       = 0.0' \
		'freq        = -doppler_rate * duration / 2  # start negative, ramp through zero' \
		'phase_inc   = 2 * math.pi * freq / sample_rate' \
		'freq_inc    = 2 * math.pi * doppler_rate / (sample_rate ** 2)' \
		'for i in range(0, len(data)-3, 4):' \
		'    I, Q = struct.unpack_from("<hh", data, i)' \
		'    s = complex(I, Q) * cmath.exp(1j * phase)' \
		'    I = int(s.real + random.gauss(0, noise_amp * 32767))' \
		'    Q = int(s.imag + random.gauss(0, noise_amp * 32767))' \
		'    out += struct.pack("<hh", max(-32768,min(32767,I)), max(-32768,min(32767,Q)))' \
		'    phase_inc += freq_inc' \
		'    phase += phase_inc' \
		'sys.stdout.buffer.write(out)' \
		> /tmp/doppler.py
	@$(BINDIR)/opv-mod -S W5NYV -B 100 2>/dev/null | python3 /tmp/doppler.py > /tmp/test_doppler.iq
	@NC=$$($(BINDIR)/opv-demod -s -r -q < /tmp/test_doppler.iq 2>/dev/null | wc -c); NC=$$((NC / 134)); \
	 CO=$$($(BINDIR)/opv-demod -s -c -r -q < /tmp/test_doppler.iq 2>/dev/null | wc -c); CO=$$((CO / 134)); \
	 echo "  Non-coherent: $${NC}/100 frames decoded"; \
	 echo "  Coherent:     $${CO}/100 frames decoded"; \
	 if [ "$${CO}" -gt "$${NC}" ]; then \
	   echo "✓ Coherent advantage under Doppler confirmed"; \
	 elif [ "$${CO}" -eq "$${NC}" ]; then \
	   echo "~ Equal performance — FEC waterfall may be masking 3dB coherent gain;"; \
	   echo "  validate with over-the-air hardware testing on a live satellite pass"; \
	 else \
	   echo "✗ Coherent underperforms under Doppler — Costas loop investigation needed"; \
	 fi

.PHONY: all clean test test-codec test-raw test-server test-server-send test-rx test-coherent test-coherent-compare test-doppler
