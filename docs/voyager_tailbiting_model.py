"""
Tail-biting layer over the (verified) true-Voyager 171/133 code in voyager_k7.

Domain model: the CODE is fixed (voyager_k7). A FRAME imposes a boundary
condition on it:
  * zero-terminated : start=0, end forced to 0 by 6 appended bits  (costs 6 bits)
  * tail-biting     : start state = last M payload bits, so start==end and the
                      trellis closes into a RING                    (costs 0 bits)
Tail-biting keeps every numerology value (1072 in, 2144 out, 40 ms, 67x32).
The cost moves to the decoder: it must decode a circle (wrap-around Viterbi).
"""
import numpy as np
import voyager_k7 as v

M = v.MEM  # 6

def tailbiting_state(bits):
    """The start state that makes start==end: the state after the last M bits."""
    s = 0
    for u in bits[-M:]:
        s = v.next_state(s, int(u))
    return s

def encode_tailbiting(bits):
    """Encode a frame as a tail-biting codeword. Returns (coded_bits, start, end)."""
    s0 = tailbiting_state(bits)
    s = s0; out = []
    for u in bits:
        g1, g2 = v.branch_output(s, int(u)); out += [g1, g2]
        s = v.next_state(s, int(u))
    return np.array(out, dtype=np.int8), s0, s   # end state should == s0

def decode_wava(soft_g1, soft_g2, W=48):
    """Wrap-around Viterbi: the frame is circular, so its true neighbours are its
    own ends. Prepend the last W symbols and append the first W, decode, and keep
    the MIDDLE K bits -- every one now has real context on both sides (no seam,
    no tail floor). W must exceed the traceback depth (~5*K_c)."""
    a1 = np.concatenate([soft_g1[-W:], soft_g1, soft_g1[:W]])
    a2 = np.concatenate([soft_g2[-W:], soft_g2, soft_g2[:W]])
    dec = v.viterbi(a1, a2)
    K = len(soft_g1)
    return dec[W:W+K]
