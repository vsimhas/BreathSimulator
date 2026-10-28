#!/usr/bin/env python3
"""Synthetic SUMMARY.CSV for the #65 adherence bench check.

~115 nights ending last night, built to hit the edge cases:
- first night 115 days ago; nights 1-30 mostly >= 4 h (reimbursement window
  should pass with exactly 22/30), then a mixed period;
- sessions starting after midnight (count to the previous night), two
  sessions in one night (summed), missed nights, sub-4 h nights;
- one record with rtc_valid 0 (undated, excluded).
Writes SUMMARY_ADH_TEST.CSV next to this script.
"""
import datetime as dt
import random

HDR = ("schema,start,end,rtc_valid,run_s,mask_on_s,mask_off_s,mask_off_n,mode,p_cpap,apap_min,"
       "apap_max,epr_type,epr_level,ramp_min,p_avg,p_min,p_max,p50,p90,p95,leak_avg,leak_p50,"
       "leak_p95,leak_max,leak_high_s,leak_low_s,events_scored,oa,ca,hyp,fl,snore,ahi,rdi,si,edf")


def rec(start, on_s, valid=1):
    end = start + dt.timedelta(seconds=on_s)
    f = lambda t: t.strftime("%Y-%m-%d %H:%M:%S")
    return ("1,%s,%s,%d,%d,%d,0,0,AutoSet,8.0,8.0,12.0,0,1,0,9.10,0.00,11.2,9.0,10.4,10.8,"
            "6.0,5.0,14.0,22.0,0,0,1,1,0,2,1,0,1.5,1.9,0.0,%s"
            % (f(start), f(end), valid, on_s, on_s, start.strftime("%Y%m%d_%H%M%S")))


random.seed(65)
last_night = dt.date(2026, 10, 25)          # the night before 2026-10-26
first_night = last_night - dt.timedelta(days=114)
lines = []
for i in range(115):
    night = first_night + dt.timedelta(days=i)
    if i < 30:                              # first 30 nights: 22 good, 4 short, 4 missed
        kind = "good" if i % 30 < 22 else ("short" if i % 30 < 26 else "miss")
    else:
        kind = random.choice(["good", "good", "short", "miss", "split", "late"])
    if kind == "miss":
        continue
    if kind == "good":
        lines.append(rec(dt.datetime.combine(night, dt.time(22, 30)), 6 * 3600 + 120 * (i % 7)))
    elif kind == "short":
        lines.append(rec(dt.datetime.combine(night, dt.time(23, 15)), 3 * 3600 + 1800))
    elif kind == "split":                   # two sessions in one night: 2 h + 2.5 h = 4.5 h
        lines.append(rec(dt.datetime.combine(night, dt.time(22, 0)), 2 * 3600))
        lines.append(rec(dt.datetime.combine(night + dt.timedelta(days=1), dt.time(2, 0)),
                         2 * 3600 + 1800))
    elif kind == "late":                    # starts 00:40 next date -> still this night
        lines.append(rec(dt.datetime.combine(night + dt.timedelta(days=1), dt.time(0, 40)),
                         4 * 3600 + 600))
    if i == 60:                             # an undated record (clock lost)
        lines.append(rec(dt.datetime(2026, 1, 1, 0, 5), 7 * 3600, valid=0))

with open(__file__.replace("make_adherence_test.py", "SUMMARY_ADH_TEST.CSV"), "w",
          newline="") as fh:
    fh.write("# SomnaLink session summary, schema 1\r\n" + HDR + "\r\n")
    for l in lines:
        fh.write(l + "\r\n")
print("records:", len(lines), "nights", first_night, "..", last_night)
