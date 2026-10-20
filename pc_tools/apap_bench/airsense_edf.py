"""Minimal EDF / EDF+ reader for ResMed AirSense SD-card files (no dependencies).

ResMed writes EDF+ (often EDF+D, discontinuous) per session in DATALOG/<date>/:
  *_EVE.edf  annotations only: scored events ("Central Apnea", "Hypopnea", ...)
  *_BRP.edf  ~25 Hz flow and pressure (labels like "Flow.40ms", "Press.40ms")
  *_PLD.edf  2 s values (mask pressure, leak, tidal volume, flow limitation...)
  *_SA2.edf  oximetry (unused here)
and STR.edf (daily summary) at the card root.

read_edf(path) returns an Edf with:
  start      datetime of the file start (device clock, local time)
  signals    {label: Sig(fs, t, x, unit)} for every data signal; t is seconds
             from `start` for each sample - EDF+D record onsets are honoured
  annots     [(onset_s, duration_s, text)] from every "EDF Annotations"
             signal, excluding the per-record time-keeping entries
Labels are kept exactly as stored; callers match them by substring because
the exact names differ between models and firmware.
"""
import datetime as _dt
import struct
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Sig:
    label: str
    unit: str
    fs: float
    t: np.ndarray
    x: np.ndarray


@dataclass
class Edf:
    path: str
    start: _dt.datetime
    edf_plus: str            # "", "EDF+C" or "EDF+D"
    record_s: float
    n_records: int
    signals: dict = field(default_factory=dict)
    annots: list = field(default_factory=list)


def _f(b):
    return b.decode("latin-1").strip()


def _parse_tals(raw):
    """EDF+ TAL list -> [(onset, duration, [texts])]. The first TAL of a record
    is its time-keeping entry (onset with an empty text)."""
    out = []
    for tal in raw.split(b"\x00"):
        if not tal or tal[0:1] not in (b"+", b"-"):
            continue
        parts = tal.split(b"\x14")
        head = parts[0]
        if b"\x15" in head:
            on, du = head.split(b"\x15", 1)
            dur = float(du) if du else 0.0
        else:
            on, dur = head, 0.0
        try:
            onset = float(on)
        except ValueError:
            continue
        texts = [p.decode("latin-1") for p in parts[1:]]
        out.append((onset, dur, texts))
    return out


def read_edf(path):
    with open(path, "rb") as fh:
        data = fh.read()
    h = data[:256]
    start_date, start_time = _f(h[168:176]), _f(h[176:184])
    header_bytes = int(_f(h[184:192]))
    reserved = _f(h[192:236])
    n_records = int(_f(h[236:244]))
    record_s = float(_f(h[244:252]))
    ns = int(_f(h[252:256]))
    dd, mm, yy = (int(v) for v in start_date.split("."))
    hh, mi, ss = (int(v) for v in start_time.split("."))
    year = 2000 + yy if yy < 85 else 1900 + yy
    start = _dt.datetime(year, mm, dd, hh, mi, ss)

    def field_(off, width):
        return [_f(data[256 + off + i * width: 256 + off + (i + 1) * width]) for i in range(ns)]

    o = 0
    labels = field_(o, 16); o += 16 * ns
    o += 80 * ns                                       # transducer
    units = field_(o, 8); o += 8 * ns
    pmin = [float(v) for v in field_(o, 8)]; o += 8 * ns
    pmax = [float(v) for v in field_(o, 8)]; o += 8 * ns
    dmin = [int(v) for v in field_(o, 8)]; o += 8 * ns
    dmax = [int(v) for v in field_(o, 8)]; o += 8 * ns
    o += 80 * ns                                       # prefilter
    nsamp = [int(v) for v in field_(o, 8)]; o += 8 * ns

    rec_bytes = sum(nsamp) * 2
    body = data[header_bytes:]
    if n_records < 0:                                  # unknown in header
        n_records = len(body) // rec_bytes
    n_records = min(n_records, len(body) // rec_bytes)
    is_annot = [lab.startswith("EDF Annotations") for lab in labels]

    edf = Edf(path=path, start=start, edf_plus=reserved if reserved.startswith("EDF+") else "",
              record_s=record_s, n_records=n_records)
    rec_onset = np.arange(n_records, dtype=float) * record_s
    chunks = {i: [] for i in range(ns)}
    for r in range(n_records):
        rec = body[r * rec_bytes:(r + 1) * rec_bytes]
        p = 0
        for i in range(ns):
            nb = nsamp[i] * 2
            seg = rec[p:p + nb]
            p += nb
            if is_annot[i]:
                tals = _parse_tals(seg)
                for k, (onset, dur, texts) in enumerate(tals):
                    if k == 0 and (not texts or all(t == "" for t in texts)):
                        rec_onset[r] = onset          # time-keeping TAL
                        continue
                    for t in texts:
                        if t:
                            edf.annots.append((onset, dur, t))
            else:
                chunks[i].append(np.frombuffer(seg, dtype="<i2"))

    for i in range(ns):
        if is_annot[i] or nsamp[i] == 0:
            continue
        raw = np.concatenate(chunks[i]).astype(float) if chunks[i] else np.zeros(0)
        span = (dmax[i] - dmin[i]) or 1
        x = pmin[i] + (raw - dmin[i]) * (pmax[i] - pmin[i]) / span
        fs = nsamp[i] / record_s if record_s > 0 else 0.0
        k = np.arange(nsamp[i]) / fs if fs > 0 else np.zeros(nsamp[i])
        t = (rec_onset[:, None] + k[None, :]).ravel()[:len(x)]
        edf.signals[labels[i]] = Sig(labels[i], units[i], fs, t, x)
    edf.annots.sort(key=lambda a: a[0])
    return edf


def find(edf, *subs):
    """First signal whose label contains every substring (case-insensitive)."""
    for lab, s in edf.signals.items():
        if all(x.lower() in lab.lower() for x in subs):
            return s
    return None


def describe(edf):
    print("%s: start %s  %s  %d records x %.1f s" % (edf.path, edf.start, edf.edf_plus or "EDF",
                                                   edf.n_records, edf.record_s))
    for lab, s in edf.signals.items():
        print("   %-18s %-8s %6.2f Hz  %7d samples  %.3g..%.3g"
              % (lab, s.unit, s.fs, len(s.x), s.x.min() if len(s.x) else 0, s.x.max() if len(s.x) else 0))
    if edf.annots:
        kinds = {}
        for _, _, t in edf.annots:
            kinds[t] = kinds.get(t, 0) + 1
        print("   annotations: %s" % ", ".join("%s x%d" % kv for kv in sorted(kinds.items())))


def write_test_edfplus(path):
    """Synthetic EDF+D file (one 2-sample/s signal + annotations, a gap between
    records) - used only to self-test this reader."""
    ns, rec_s = 2, 10.0
    n_sig, n_ann = 20, 60                                # samples per record
    recs = []
    onsets = [0.0, 10.0, 40.0]                           # discontinuous
    events = {1: [(14.0, 12.0, "Central Apnea")], 2: [(45.5, 11.0, "Hypopnea")]}
    for r, on in enumerate(onsets):
        sig = struct.pack("<%dh" % n_sig, *[int(1000 * np.sin(on + k)) for k in range(n_sig)])
        tal = b"+%g\x14\x14\x00" % on
        for (o, d, t) in events.get(r, []):
            tal += b"+%g\x15%g\x14%s\x14\x00" % (o, d, t.encode())
        tal = tal.ljust(n_ann * 2, b"\x00")
        recs.append(sig + tal)
    def pad(s, w):
        return s.encode("latin-1").ljust(w)[:w]
    hdr = (pad("0", 8) + pad("X X X X", 80) + pad("Startdate X X X X", 80) + pad("20.10.26", 8)
           + pad("14.30.00", 8) + pad(str(256 * (ns + 1)), 8) + pad("EDF+D", 44)
           + pad(str(len(recs)), 8) + pad("10", 8) + pad(str(ns), 4))
    labels = ["Flow.40ms", "EDF Annotations"]
    hdr += b"".join(pad(l, 16) for l in labels) + pad("", 80) * ns
    hdr += pad("L/s", 8) + pad("", 8)
    hdr += pad("-1", 8) + pad("-1", 8) + pad("1", 8) + pad("1", 8)
    hdr += pad("-1000", 8) + pad("-32768", 8) + pad("1000", 8) + pad("32767", 8)
    hdr += pad("", 80) * ns + pad(str(n_sig), 8) + pad(str(n_ann), 8) + pad("", 32) * ns
    with open(path, "wb") as fh:
        fh.write(hdr + b"".join(recs))


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        describe(read_edf(p))
