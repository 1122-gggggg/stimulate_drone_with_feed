"""Snapshot the observed wind at an NCDR station from https://watch.ncdr.nat.gov.tw/watch_page_wind
(the page's "風速/陣風監測" data feed: wh/cv_gustwind_list).

  python wind_data/ncdr_fetch.py                      # Xiyu (西嶼, C0W120) -> wind_data/xiyu_ncdr.json
  python wind_data/ncdr_fetch.py --station C0W190     # another station code, e.g. Wang-an

For each window it stores the maximum 10-minute-mean wind speed and the maximum gust within the last
h hours. The feed has no wind direction (only the map image does); direction is chosen in the simulator.
"""
import argparse
import csv
import io
import json
import os
import ssl
import time
import urllib.request

BASE = "https://watch.ncdr.nat.gov.tw/wh/cv_gustwind_list"
HOURS = [1, 3, 6, 12, 24, 48]
# NCDR's certificate lacks a Subject Key Identifier, which Python >= 3.13 rejects in strict mode.
# Only the strict-profile flag is dropped; the certificate chain and host name are still verified.
CTX = ssl.create_default_context()
CTX.verify_flags &= ~ssl.VERIFY_X509_STRICT

p = argparse.ArgumentParser()
p.add_argument("--station", default="C0W120")
p.add_argument("--out", default="")
a = p.parse_args()


def fetch(v, h):
    with urllib.request.urlopen(f"{BASE}?v={v}&h={h}&d=0", timeout=30, context=CTX) as r:
        rows = list(csv.reader(io.StringIO(r.read().decode("utf-8"))))
    head = rows[0]
    for row in rows[1:]:
        if row[head.index("測站代碼")] == a.station:
            return dict(zip(head, row))
    raise SystemExit(f"station {a.station} not in the feed for v={v} h={h}")


out = {"source": "https://watch.ncdr.nat.gov.tw/watch_page_wind", "windows": {}}
for h in HOURS:
    mean_row = fetch(2, h)  # v=2: ranked by mean wind
    gust_row = fetch(1, h)  # v=1: ranked by gust
    out["station"] = {k: mean_row[k] for k in ("縣市", "鄉鎮", "測站", "測站代碼", "lon", "lat")}
    out["windows"][f"max_{h}h"] = {
        "mean": float(mean_row["風速m/s"]),
        "gust": float(gust_row["陣風m/s"]),
        "gust_time": gust_row["發生時間"],
    }
out["fetched"] = time.strftime("%Y-%m-%d %H:%M:%S")
path = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "xiyu_ncdr.json")
json.dump(out, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
for k, v in out["windows"].items():
    print(f"{out['station']['測站']} {k}: mean {v['mean']} m/s, gust {v['gust']} m/s (at {v['gust_time']})")
print("wrote", path)
