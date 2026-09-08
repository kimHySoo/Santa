"""Isaac 공식 에셋에서 idealworks iw.hub 로봇을 받는다 (공개 S3, 인증 불필요)."""
import urllib.request, urllib.parse, xml.etree.ElementTree as ET, os, concurrent.futures
B = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
PREFIX = "Assets/Isaac/6.0/Isaac/Robots/Idealworks/iwhub/"
# paths.sh 의 $WAREHOUSE/robots/iwhub 와 같은 자리여야 한다 (2026-09-08 정정).
DEST = os.path.expanduser("~/khs/wh/warehouse/robots/iwhub/")
NS = "{http://s3.amazonaws.com/doc/2006-03-01/}"

def keys():
    out, tok = [], None
    while True:
        u = f"{B}/?list-type=2&prefix={urllib.parse.quote(PREFIX)}&max-keys=1000"
        if tok: u += "&continuation-token=" + urllib.parse.quote(tok)
        with urllib.request.urlopen(u, timeout=60) as r: root = ET.fromstring(r.read())
        for c in root.findall(NS + "Contents"):
            k = c.find(NS + "Key").text
            if not k.endswith("/") and "/.thumbs/" not in k: out.append(k)
        if root.findtext(NS + "IsTruncated") == "true": tok = root.findtext(NS + "NextContinuationToken")
        else: break
    return out

def dl(k):
    p = os.path.join(DEST, k[len(PREFIX):])
    os.makedirs(os.path.dirname(p), exist_ok=True)
    if os.path.exists(p) and os.path.getsize(p) > 0: return "skip"
    try:
        urllib.request.urlretrieve(f"{B}/{urllib.parse.quote(k)}", p); return "ok"
    except Exception as e: return f"ERR {k}: {e}"

ks = keys()
print(f"[iwhub] {len(ks)}개 -> {DEST}", flush=True)
ok = skip = err = 0
with concurrent.futures.ThreadPoolExecutor(max_workers=24) as ex:
    for r in ex.map(dl, ks):
        if r == "ok": ok += 1
        elif r == "skip": skip += 1
        else:
            err += 1
            if err <= 5: print(r, flush=True)
print(f"[iwhub] ok={ok} skip={skip} err={err}", flush=True)
