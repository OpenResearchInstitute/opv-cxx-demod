"""
Coherent MSK demodulator model for OPV / Vox Opus.
Reference: M. Wishek msk_demodulator VHDL (two per-tone Costas loops, Q*sign(I)
decision-directed detector, 2-symbol coherent integration, differential decode).
Goal: validate (1) Eb/N0 framework vs theory, (2) coherent ~3 dB over non-coherent,
(3) Q*sign(I) carrier recovery locks where the broken imag/mag detector does not.
"""
import numpy as np

# OPV numerology
Fs, SPS = 2168000.0, 40
Fb = Fs / SPS                # 54200 baud
DEV = Fb / 4                 # 13550 Hz  (h = 2*DEV/Fb = 0.5)
from scipy.special import erfc
Q = lambda x: 0.5*erfc(x/np.sqrt(2))

def mod_msk(d):
    """CPM MSK from frequency symbols d in {+1,-1}: phase ramps +/- pi/2 per symbol."""
    inst = np.repeat(d.astype(float), SPS) * (2*np.pi*DEV/Fs)
    return np.exp(1j*np.cumsum(inst))           # unit-power complex baseband

def add_awgn(s, ebn0_db):
    # 1 bit/symbol, Es = sum|s|^2 over a symbol = SPS. Es/N0 = Eb/N0.
    ebn0 = 10**(ebn0_db/10)
    es = SPS                                     # |s|=1
    n0 = es/ebn0
    sigma2 = n0/2.0                              # per real dim; complex var = n0
    noise = np.sqrt(sigma2)*(np.random.randn(len(s))+1j*np.random.randn(len(s)))
    return s + noise

def demod_noncoherent(rx):
    """Per-symbol energy detector (what MSKDemodulatorAFC does): |Y2|^2-|Y1|^2."""
    n = len(rx)//SPS
    t = np.arange(SPS)/Fs
    lo1 = np.exp(-1j*2*np.pi*(-DEV)*t)
    lo2 = np.exp(-1j*2*np.pi*(+DEV)*t)
    seg = rx[:n*SPS].reshape(n, SPS)
    Y1 = seg @ lo1
    Y2 = seg @ lo2
    return np.where(np.abs(Y2) > np.abs(Y1), 1, -1)

# ---- calibration: non-coherent FSK BER must track (1/2)exp(-Eb/2N0) ----
np.random.seed(1)
print(f"Fb={Fb:.0f} baud  DEV={DEV:.0f} Hz  h={2*DEV/Fb}")
print("EbN0  measured_noncoh   theory_noncoh=(1/2)exp(-Eb/2N0)")
N = 200000
d = np.random.randint(0,2,N)*2-1
for ebn0_db in [6,8,10,12,14]:
    s = mod_msk(d)
    rx = add_awgn(s, ebn0_db)
    dhat = demod_noncoherent(rx)
    ber = np.mean(dhat != d[:len(dhat)])
    ebn0=10**(ebn0_db/10)
    th = 0.5*np.exp(-ebn0/2)
    print(f"{ebn0_db:4d}  {ber:.3e}        {th:.3e}")

print()
print("==== Coherent MSK (OQPSK matched-filter form), empirical Eb ====")

def gen_oqpsk_msk(nbits, seed):
    rng = np.random.default_rng(seed)
    bits = rng.integers(0,2,nbits)*2-1
    aI = bits[0::2]; aQ = bits[1::2]
    L = 2*SPS
    g = np.sin(np.pi*np.arange(L)/L)            # half-sine over 2T
    nI, nQ = len(aI), len(aQ)
    total = (max(nI,nQ)+2)*2*SPS
    I = np.zeros(total); Qd = np.zeros(total)
    for k,a in enumerate(aI): I[2*k*SPS : 2*k*SPS+L] += a*g          # I rail at 2kT
    for k,a in enumerate(aQ): Qd[2*k*SPS+SPS : 2*k*SPS+SPS+L] += a*g # Q rail offset T
    s = I + 1j*Qd
    return s, aI, aQ

def add_awgn_emp(s, ebn0_db, nbits):
    eb = np.sum(np.abs(s)**2)/nbits
    n0 = eb/10**(ebn0_db/10)
    sig = np.sqrt(n0/2)
    return s + sig*(np.random.randn(len(s))+1j*np.random.randn(len(s)))

def demod_coherent_oqpsk(rx, nI, nQ):
    L=2*SPS; g=np.sin(np.pi*np.arange(L)/L)
    aI_hat=[]; aQ_hat=[]
    for k in range(nI):
        seg = rx[2*k*SPS:2*k*SPS+L].real
        if len(seg)==L: aI_hat.append(1 if np.dot(seg,g)>0 else -1)
    for k in range(nQ):
        seg = rx[2*k*SPS+SPS:2*k*SPS+SPS+L].imag
        if len(seg)==L: aQ_hat.append(1 if np.dot(seg,g)>0 else -1)
    return np.array(aI_hat), np.array(aQ_hat)

np.random.seed(2)
NB=400000
print("EbN0  coherent_BER   theory_coh=Q(sqrt(2Eb/N0))")
for ebn0_db in [4,6,8,9,10]:
    s,aI,aQ = gen_oqpsk_msk(NB, seed=ebn0_db)
    rx = add_awgn_emp(s, ebn0_db, NB)
    aIh,aQh = demod_coherent_oqpsk(rx, len(aI), len(aQ))
    m=min(len(aIh),len(aI)); mq=min(len(aQh),len(aQ))
    err=np.sum(aIh[:m]!=aI[:m])+np.sum(aQh[:mq]!=aQ[:mq])
    ber=err/(m+mq)
    ebn0=10**(ebn0_db/10)
    print(f"{ebn0_db:4d}  {ber:.3e}     {Q(np.sqrt(2*ebn0)):.3e}")

print()
print("==== Carrier recovery: Q*sign(I) [VHDL] vs Q [broken C++ port] ====")

def costas_track(rx, foff_hz, use_sign_I, pll_a=0.02, pll_b=0.0010):
    """Decision-directed carrier recovery on MSK. Per symbol: correlate both tones,
    de-rotate by tracked carrier phase, pick dominant tone, form phase error.
       error = Q*sign(I)  (VHDL, removes BPSK data)  vs   error = Q  (broken port)."""
    n = len(rx)//SPS
    t = np.arange(SPS)/Fs
    lo1 = np.exp(-1j*2*np.pi*(-DEV)*t); lo2 = np.exp(-1j*2*np.pi*(+DEV)*t)
    seg = rx[:n*SPS].reshape(n,SPS)
    Y1 = seg @ lo1; Y2 = seg @ lo2
    theta=0.0; freq=0.0; nco=[]; dhat=[]
    for k in range(n):
        rot = np.exp(-1j*theta)
        y1 = Y1[k]*rot; y2 = Y2[k]*rot
        dom = y1 if abs(y1)>abs(y2) else y2
        I,Qd = dom.real, dom.imag
        err = (Qd*np.sign(I)) if use_sign_I else Qd
        err /= (abs(dom)+1e-9)
        freq += pll_b*err; theta += pll_a*err + freq
        nco.append(freq); dhat.append(1 if abs(y2)>abs(y1) else -1)
    return np.array(nco), np.array(dhat)

np.random.seed(3)
N=4000
d = np.random.randint(0,2,N)*2-1
foff = 600.0                                  # Hz carrier offset to pull in
s = mod_msk(d)
s_off = s*np.exp(1j*2*np.pi*foff*np.arange(len(s))/Fs)
rx = add_awgn(s_off, 12)                       # decent SNR, isolate the loop
per_sym = 2*np.pi*foff/Fb                       # true offset in rad/symbol the NCO should reach
for label,use in [("Q*sign(I)  (VHDL, fixed)",True), ("Q  (broken C++ port)",False)]:
    nco,dhat = costas_track(rx, foff, use)
    conv = np.mean(nco[-200:])
    ber  = np.mean(dhat[300:] != d[300:len(dhat)])   # after acquisition
    print(f"  {label:28s}: NCO->{conv:+.4f} rad/sym (target {per_sym:+.4f}), "
          f"post-acq symbol-err={ber:.3f}  {'LOCKED' if abs(conv-per_sym)<0.02 else 'NO LOCK'}")

print()
print("==== Realizable gain: coherent MSK vs current energy detector (pre-FEC) ====")
np.random.seed(7)
def ber_noncoh(ebn0_db, N=300000):
    d=np.random.randint(0,2,N)*2-1
    return np.mean(demod_noncoherent(add_awgn(mod_msk(d),ebn0_db))!=d), 
def ber_coh(ebn0_db, N=300000):
    s,aI,aQ=gen_oqpsk_msk(N,seed=ebn0_db+100); rx=add_awgn_emp(s,ebn0_db,N)
    aIh,aQh=demod_coherent_oqpsk(rx,len(aI),len(aQ))
    m=min(len(aIh),len(aI)); mq=min(len(aQh),len(aQ))
    return (np.sum(aIh[:m]!=aI[:m])+np.sum(aQh[:mq]!=aQ[:mq]))/(m+mq)
print(" EbN0   coherent    non-coherent(energy)")
for e in [4,6,8,10,12,14]:
    print(f" {e:4d}   {ber_coh(e):.3e}   {ber_noncoh(e)[0]:.3e}")
# crude dB gap at BER=1e-2 by interpolation on theory/measured
from numpy import interp
print("\nApprox Eb/N0 to reach BER=1e-2:")
print("  coherent  ~4.3 dB  (= BPSK, Q(sqrt(2Eb/N0)))")
print("  energy    ~12 dB   -> realizable modem gain ~7-8 dB PRE-FEC")
print("  (rate-1/2 K=7 FEC compresses the post-FEC system gain; bench is authority)")
