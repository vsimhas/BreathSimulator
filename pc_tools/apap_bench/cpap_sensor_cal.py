#!/usr/bin/env python3
"""#84 sensor calibration bench check on our CPAP (COM13). Blower stays
stopped throughout (therapy is only started where it must be refused).

    python cpap_sensor_cal.py run      # zero learn / refuse, field span, log
    python cpap_sensor_cal.py status   # after a reset: what came back from CAL.CSV

`run` ends with the ambient offset left at +0.50 mbar, so a reset can show
it reloading; `python cpap_sensor_cal.py clear` sets it back to 0.
"""
import sys
import time

import serial

KEEP = ("#", "ACK", "ERR", "ack", "err", "S,")
LOG = "runs/cpap_sensor_cal_%s_%s.log"


class Cpap:
    def __init__(self):
        self.s = serial.Serial("COM13", 115200, timeout=0.05)
        self.cmd("telem off", 0.5)

    def cmd(self, c, dur=0.8):
        self.s.reset_input_buffer()
        self.s.write((c + "\r\n").encode("ascii"))
        out, buf, t0 = [], b"", time.time()
        while time.time() - t0 < dur:
            buf += self.s.read(4096)
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                ln = ln.decode("ascii", "replace").strip()
                if ln.startswith(KEEP):
                    out.append(ln)
        return out

    def line(self, c, prefix, dur=0.8):
        for ln in self.cmd(c, dur):
            if ln.startswith(prefix):
                return ln
        return ""


def f(line, key):
    for tok in line.split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


def say(log, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


def cal(c, log, label):
    ln = c.line("cal status", "# CAL")
    say(log, "   %-28s ps_off=%s gain=%s src=%s rej=%s cand=%s n=%s | fs_zero=%s rej=%s cand=%s az=%s"
        " | as_off=%s src=%s | rows=%s err=%s" % (
            label, f(ln, "ps_off"), f(ln, "ps_gain"), f(ln, "ps_src"), f(ln, "ps_rej"),
            f(ln, "ps_cand"), f(ln, "ps_n"), f(ln, "fs_zero"), f(ln, "fs_rej"),
            f(ln, "fs_cand"), f(ln, "fs_az"), f(ln, "as_off"), f(ln, "as_src"),
            f(ln, "rows"), f(ln, "csverr")))
    return ln


def pre(c, log, label):
    ln = c.line("therapy precheck", "# PRECHECK")
    say(log, "   %-28s precheck ok=%s fail=%s warn=%s ps=%s q=%s amb=%s" % (
        label, f(ln, "ok"), ln.split("fail=")[1].split(" inj=")[0] if "fail=" in ln else "?",
        f(ln, "warn"), f(ln, "ps"), f(ln, "q"), f(ln, "amb")))
    return ln


def wait_ps(c, n_before, max_s=15.0):
    t0 = time.time()
    while time.time() - t0 < max_s:
        time.sleep(1.0)
        ln = c.line("cal status", "# CAL")
        if f(ln, "ps_n") not in ("?", n_before):
            return round(time.time() - t0, 1)
    return None


def wait_fs(c, az_before, zero_before, rej_before="0", max_s=40.0):
    t0 = time.time()
    while time.time() - t0 < max_s:
        time.sleep(1.0)
        ln = c.line("cal status", "# CAL")
        newly_refused = (f(ln, "fs_rej") == "1") and (rej_before != "1")
        if newly_refused or (f(ln, "fs_az") != az_before
                             and f(ln, "fs_zero") != zero_before):
            time.sleep(2.0)
            return round(time.time() - t0, 1)
    return None


def run(c, log):
    cal(c, log, "start")
    pre(c, log, "start")

    say(log, "== 1. PS +0.8 cmH2O offset: learned and corrected")
    n = f(cal(c, log, "before"), "ps_n")
    c.cmd("cal test ps 0.8")
    pre(c, log, "offset in, not learned")
    c.cmd("cal ps zero")
    say(log, "   learned after %s s" % wait_ps(c, n))
    cal(c, log, "after")
    pre(c, log, "after")

    say(log, "== 2. PS +2.0 cmH2O offset: refused, start refused")
    n = f(cal(c, log, "before"), "ps_n")
    c.cmd("cal test ps 2.0")
    c.cmd("cal ps zero")
    say(log, "   decided after %s s" % wait_ps(c, n))
    cal(c, log, "after")
    pre(c, log, "after")
    for ln in c.cmd("therapy start", 1.2):
        if ln.startswith("# PRECHECK") or ln.startswith("S,"):
            say(log, "   start:", ln[:160])
    time.sleep(1.0)
    c.cmd("therapy stop", 1.0)

    say(log, "== 3. offset removed: learned back")
    n = f(cal(c, log, "before"), "ps_n")
    c.cmd("cal test none")
    c.cmd("cal ps zero")
    say(log, "   learned after %s s" % wait_ps(c, n))
    cal(c, log, "after")
    pre(c, log, "after")

    say(log, "== 4. flow +0.02 mbar (~2 L/min): learned")
    ln = cal(c, log, "before")
    c.cmd("cal test fs 0.02")
    time.sleep(1.0)
    pre(c, log, "offset in, not learned")
    c.cmd("cal fs zero")
    say(log, "   done after %s s" % wait_fs(c, f(ln, "fs_az"), f(ln, "fs_zero"), f(ln, "fs_rej")))
    cal(c, log, "after")
    pre(c, log, "after")

    say(log, "== 5. flow +0.08 mbar (~8 L/min): refused, warning")
    ln = cal(c, log, "before")
    c.cmd("cal test fs 0.08")
    time.sleep(1.0)
    c.cmd("cal fs zero")
    say(log, "   done after %s s" % wait_fs(c, f(ln, "fs_az"), f(ln, "fs_zero"), f(ln, "fs_rej")))
    cal(c, log, "after")
    pre(c, log, "after")

    say(log, "== 6. flow offset removed: learned back")
    ln = cal(c, log, "before")
    c.cmd("cal test none")
    time.sleep(1.0)
    c.cmd("cal fs zero")
    say(log, "   done after %s s" % wait_fs(c, f(ln, "fs_az"), f(ln, "fs_zero"), f(ln, "fs_rej")))
    cal(c, log, "after")
    pre(c, log, "after")

    say(log, "== 7. field span / ambient")
    say(log, "   ps gain 1.2:", c.cmd("cal ps gain 1.2")[:1])
    say(log, "   ps gain 1.02:", c.cmd("cal ps gain 1.02")[:1])
    cal(c, log, "gain 1.02")
    say(log, "   ps gain 1.0:", c.cmd("cal ps gain 1.0")[:1])
    pre(c, log, "before as offset")
    say(log, "   as offset 0.5:", c.cmd("cal as offset 0.5")[:1])
    pre(c, log, "as offset 0.5")
    time.sleep(2.0)
    cal(c, log, "end (as +0.5 left for reload)")


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "status"
    c = Cpap()
    with open(LOG % (what, time.strftime("%Y%m%d_%H%M%S")), "w", encoding="utf-8") as log:
        try:
            if what == "run":
                run(c, log)
            elif what == "clear":
                say(log, c.cmd("cal as offset 0"))
            cal(c, log, "final")
            pre(c, log, "final")
        finally:
            c.cmd("cal test none", 0.4)
            c.s.close()
