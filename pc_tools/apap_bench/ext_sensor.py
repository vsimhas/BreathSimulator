#!/usr/bin/env python3
"""External pressure/flow sensor logger (the mask-end reference).

Same wire protocol as D:/Docs/OneDrive - somnalink/Code/Pres_Sens/PlotFlowP.py:
    AA 55 VER TYPE LEN PAYLOAD CRC16-CCITT-FALSE (little-endian, over VER..PAYLOAD)
    TYPE 0x01 payload = <dd  pressure cmH2O, flow LPM
The sensor sends no timestamp, so each sample is stamped with the host's
time.monotonic() on arrival. Aligning with the CPAP telemetry is done later by
cross-correlating pressure (see align()).

Only one program can hold the port: close PlotFlowP.py while logging.

As a module:
    ext = ExtSensorLogger("ext_run.csv"); ext.start(); ...; ext.stop()
    ext.samples -> list of (t_host_s, pressure, flow)
Standalone check:
    python ext_sensor.py            # 10 s, prints rate and ranges
    python ext_sensor.py 30 out.csv
"""
import struct
import sys
import threading
import time

EXT_PORT, EXT_BAUD = "COM4", 230400
SOF1, SOF2, TYPE_SAMPLE = 0xAA, 0x55, 0x01


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) & 0xFFFF) ^ 0x1021 if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class _Parser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, chunk):
        self.buf.extend(chunk)

    def packets(self):
        while True:
            i = self.buf.find(bytes((SOF1, SOF2)))
            if i < 0:
                del self.buf[:-1]
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 7:
                return
            length = self.buf[4]
            total = 7 + length
            if len(self.buf) < total:
                return
            pkt = bytes(self.buf[:total])
            del self.buf[:total]
            body = pkt[2:5 + length]
            if (pkt[-2] | (pkt[-1] << 8)) != crc16_ccitt_false(body):
                self.crc_errors += 1
                continue
            yield pkt[3], pkt[5:5 + length]


class ExtSensorLogger(threading.Thread):
    def __init__(self, csv_path=None, port=EXT_PORT, baud=EXT_BAUD):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.csv = open(csv_path, "w", encoding="utf-8") if csv_path else None
        if self.csv:
            self.csv.write("t_host_s,pressure_cmh2o,flow_lpm\n")
        self.samples = []
        self.parser = _Parser()
        self._stop = threading.Event()
        self.error = None

    def run(self):
        try:
            self.ser.reset_input_buffer()
            while not self._stop.is_set():
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                now = time.monotonic()
                self.parser.feed(chunk)
                for mtype, payload in self.parser.packets():
                    if mtype == TYPE_SAMPLE and len(payload) == 16:
                        p, f = struct.unpack("<dd", payload)
                        self.samples.append((now, p, f))
                        if self.csv:
                            self.csv.write("{:.4f},{:.4f},{:.3f}\n".format(now, p, f))
        except Exception as e:                      # port lost etc.
            self.error = e
        finally:
            try:
                self.ser.close()
            except Exception:
                pass
            if self.csv:
                self.csv.close()

    def stop(self):
        self._stop.set()
        self.join(timeout=2.0)


def looks_frozen(samples):
    """True if the stream is the sensor board's hung-I2C pattern: one value
    re-sent at ~12 Hz instead of ~100 Hz live data. Needs >= 1 s of data."""
    if len(samples) < 2:
        return True
    span = samples[-1][0] - samples[0][0]
    if span <= 0:
        return True
    rate = (len(samples) - 1) / span
    distinct = len({(round(x[1], 3), round(x[2], 2)) for x in samples})
    return rate < 50.0 or distinct < 2


def load_csv(path):
    out = []
    with open(path, encoding="utf-8") as fh:
        next(fh)
        for line in fh:
            t, p, f = line.strip().split(",")
            out.append((float(t), float(p), float(f)))
    return out


def align(dev, ext, max_lag_s=3.0, step_s=0.01):
    """Offset (s) to add to device time (s) to land on host time.

    dev: list of (t_dev_s, value) - e.g. device P_sensor or P_mask
    ext: list of (t_host_s, value) - external pressure
    Coarse start: the device clock and the host clock are unrelated, so the
    caller passes dev already shifted by a first guess (arrival-time based);
    this refines it by maximising the correlation of the two pressure traces
    over +/- max_lag_s. Returns (refinement_s, correlation)."""
    import bisect
    et = [e[0] for e in ext]
    ev = [e[1] for e in ext]

    def interp(t):
        i = bisect.bisect_left(et, t)
        if i <= 0 or i >= len(et):
            return None
        t0, t1 = et[i - 1], et[i]
        if t1 - t0 > 0.2:
            return None
        return ev[i - 1] + (ev[i] - ev[i - 1]) * (t - t0) / (t1 - t0)

    best = (0.0, -2.0)
    n_steps = int(max_lag_s / step_s)
    for k in range(-n_steps, n_steps + 1):
        lag = k * step_s
        pairs = []
        for t, v in dev[::5]:
            x = interp(t + lag)
            if x is not None:
                pairs.append((v, x))
        if len(pairs) < 50:
            continue
        n = len(pairs)
        ma = sum(a for a, _ in pairs) / n
        mb = sum(b for _, b in pairs) / n
        sa = sum((a - ma) ** 2 for a, _ in pairs) ** 0.5
        sb = sum((b - mb) ** 2 for _, b in pairs) ** 0.5
        if sa == 0 or sb == 0:
            continue
        r = sum((a - ma) * (b - mb) for a, b in pairs) / (sa * sb)
        if r > best[1]:
            best = (lag, r)
    return best


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    path = sys.argv[2] if len(sys.argv) > 2 else None
    try:
        lg = ExtSensorLogger(path)
    except Exception as e:
        print("cannot open {}: {}  (is PlotFlowP.py still running?)".format(
            EXT_PORT, e))
        sys.exit(1)
    lg.start()
    time.sleep(dur)
    lg.stop()
    s = lg.samples
    if lg.error:
        print("error:", lg.error)
    if len(s) < 2:
        print("no samples ({} CRC errors)".format(lg.parser.crc_errors))
        sys.exit(1)
    span = s[-1][0] - s[0][0]
    print("{} samples in {:.1f} s = {:.1f} Hz, CRC errors {}".format(
        len(s), span, (len(s) - 1) / span, lg.parser.crc_errors))
    print("pressure {:.2f} .. {:.2f} cmH2O, flow {:.1f} .. {:.1f} LPM".format(
        min(x[1] for x in s), max(x[1] for x in s),
        min(x[2] for x in s), max(x[2] for x in s)))
    if looks_frozen(s):
        print("!! FROZEN - the sensor board's I2C bus has hung. Power-cycle it.")
        sys.exit(2)
    print("live")
