"""End-to-end self-test for airsense_compare.py without an AirSense.

Takes a real logged run (total_raw.pkl: external flow + simulator ground
truth), pretends it was an AirSense run, and writes fake SD-card files with a
KNOWN device-clock offset (+137.4 s), inverted flow sign, and planted
Hypopnea annotations 8 s after each delivered hypopnea's onset. The compare
tool must recover the offset and attribute each planted event correctly.
"""
import datetime as dt
import os
import pickle
import struct
import sys

import numpy as np

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
sys.path.insert(0, HERE)
import airsense_compare as ac
from spike_analysis import ext_times

D = 137.4            # device clock = PC clock + D
OUT = SP + "/fake_datalog"


def pad(s, w):
    return s.encode("latin-1").ljust(w)[:w]


def write_edf(path, start, rec_s, sigs, annots=None):
    """sigs: [(label, unit, fs, x, pmin, pmax)]; annots: [(onset_s, dur_s, text)]."""
    n_rec = max(int(np.ceil(len(x) / (fs * rec_s))) for _, _, fs, x, _, _ in sigs) if sigs else 1
    if annots is not None and not sigs:
        n_rec = 1
    labels = [s[0] for s in sigs] + (["EDF Annotations"] if annots is not None else [])
    ns = len(labels)
    nsamp = [int(fs * rec_s) for _, _, fs, _, _, _ in sigs]
    tal_bytes = []
    if annots is not None:
        per = [[] for _ in range(n_rec)]
        for a in annots:
            per[min(int(a[0] // rec_s), n_rec - 1) if sigs else 0].append(a)
        for r in range(n_rec):
            b = b"+%g\x14\x14\x00" % (r * rec_s)
            for on, du, t in per[r]:
                b += b"+%g\x15%g\x14%s\x14\x00" % (on, du, t.encode())
            tal_bytes.append(b)
        n_ann = (max(len(b) for b in tal_bytes) + 1) // 2 + 1
        nsamp.append(n_ann)
    hdr = (pad("0", 8) + pad("X X X X", 80) + pad("Startdate X X X X", 80)
           + pad(start.strftime("%d.%m.%y"), 8) + pad(start.strftime("%H.%M.%S"), 8)
           + pad(str(256 * (ns + 1)), 8) + pad("EDF+C", 44) + pad(str(n_rec), 8)
           + pad("%g" % rec_s, 8) + pad(str(ns), 4))
    hdr += b"".join(pad(l, 16) for l in labels) + pad("", 80) * ns
    hdr += b"".join(pad(s[1], 8) for s in sigs) + (pad("", 8) if annots is not None else b"")
    hdr += b"".join(pad("%g" % s[4], 8) for s in sigs) + (pad("-1", 8) if annots is not None else b"")
    hdr += b"".join(pad("%g" % s[5], 8) for s in sigs) + (pad("1", 8) if annots is not None else b"")
    hdr += b"".join(pad("-32768", 8) for _ in sigs) + (pad("-32768", 8) if annots is not None else b"")
    hdr += b"".join(pad("32767", 8) for _ in sigs) + (pad("32767", 8) if annots is not None else b"")
    hdr += pad("", 80) * ns + b"".join(pad(str(n), 8) for n in nsamp) + pad("", 32) * ns
    body = b""
    for r in range(n_rec):
        for k, (lab, unit, fs, x, pmin, pmax) in enumerate(sigs):
            n = nsamp[k]
            seg = np.asarray(x[r * n:(r + 1) * n], dtype=float)
            seg = np.pad(seg, (0, n - len(seg)), constant_values=seg[-1] if len(seg) else 0)
            dig = np.clip(np.round(-32768 + (seg - pmin) * 65535 / (pmax - pmin)), -32768, 32767)
            body += struct.pack("<%dh" % n, *dig.astype(int))
        if annots is not None:
            body += tal_bytes[r].ljust(nsamp[-1] * 2, b"\x00")
    with open(path, "wb") as fh:
        fh.write(hdr + body)


raw = pickle.load(open(SP + "/total_raw.pkl", "rb"))
raw["script_name"] = "selftest"
raw["wall_off"] = 1_792_000_000.0 - raw["t_script"]
pickle.dump(raw, open(SP + "/airsense_selftest_raw.pkl", "wb"))

E = np.array(raw["ext"], dtype=float)
t_pc = ext_times(E[:, 0]) + raw["wall_off"]
w0 = raw["t_script"] + raw["wall_off"]
grid = np.arange(t_pc[0] + 1, t_pc[-1] - 1, 0.04)
flow = -(np.interp(grid, t_pc, E[:, 2]) - 10.0) / 60.0        # L/s, sign inverted
dev0 = dt.datetime.fromtimestamp(np.floor(grid[0] + D))
rel = grid + D - dev0.timestamp()
os.makedirs(OUT, exist_ok=True)
write_edf(OUT + "/20261020_000000_BRP.edf", dev0, 60.0,
          [("Flow.40ms", "L/s", 25.0, flow, -2.0, 2.0),
           ("Press.40ms", "cmH2O", 25.0, np.full(len(flow), 8.0), 0.0, 30.0)])

events = ac.ground_truth(raw)
planted = []
for e in events:
    if e["type"] == "Hypopnea":
        planted.append((w0 + e["onset"] + 8.0 + D - dev0.timestamp(), 25.0, "Hypopnea"))
write_edf(OUT + "/20261020_000000_EVE.edf", dev0, 60.0, [], planted)

g2 = np.arange(rel[0], rel[-1], 2.0)
p2 = np.full(len(g2), 8.0)
fl2 = np.zeros(len(g2))
for e in events:
    a = w0 + e["onset"] + D - dev0.timestamp()
    z = w0 + e["end"] + D - dev0.timestamp()
    fl2[(g2 >= a) & (g2 <= z)] = 0.4
    p2[g2 >= z] += 0.5
write_edf(OUT + "/20261020_000000_PLD.edf", dev0, 60.0,
          [("MaskPress.2s", "cmH2O", 0.5, p2, 0.0, 30.0),
           ("Press.2s", "cmH2O", 0.5, p2, 0.0, 30.0),
           ("FlowLim.2s", "", 0.5, fl2, 0.0, 1.0)])

print("planted: device clock = PC %+.1f s; %d Hypopnea annotations at onset + 8 s"
      % (D, len(planted)))
ac.main([SP + "/airsense_selftest_raw.pkl", OUT])
