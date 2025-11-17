import argparse, json, csv, os, uuid
import configparser
from pathlib import Path
import paho.mqtt.client as mqtt
import requests
from datetime import datetime

# ---------- Config ----------
def load_api_config():
    cfg_path = Path(__file__).resolve().parent / "config" / "config.ini"
    ip = "127.0.0.1"; http_port = "7579"; mqtt_port = "1883"
    if cfg_path.exists():
        p = configparser.ConfigParser(interpolation=None)
        p.read(cfg_path)
        if p.has_section("API"):
            ip = p["API"].get("IOTPLATFORM_IP", ip)
            http_port = p["API"].get("IOTPLATFORM_HTTP_PORT", http_port)
            mqtt_port = p["API"].get("IOTPLATFORM_MQTT_PORT", mqtt_port)
    return ip, http_port, mqtt_port

IP, HTTP_PORT, MQTT_PORT = load_api_config()
MOBIUS_BASE = os.getenv("MOBIUS_BASE_URL", f"http://{IP}:{HTTP_PORT}/Mobius")
ORIGIN      = os.getenv("MOBIUS_ORIGIN", "CAdmin")

# ---------- Helpers ----------
def headers_ty(ty:int):
    return {
        "X-M2M-Origin": ORIGIN,
        "X-M2M-RI": f"ri-{uuid.uuid4().hex}",
        "X-M2M-RVI": "3",
        "Accept": "application/json",
        "Content-Type": f"application/json;ty={ty}",
    }

def fetch_cin_by_uri(uri: str):
    url = f"{MOBIUS_BASE}{uri}" if not uri.startswith("http") else uri
    r = requests.get(url, headers={"X-M2M-Origin": ORIGIN, "X-M2M-RVI":"3", "Accept":"application/json"}, timeout=5)
    r.raise_for_status()
    return r.json()

def parse_notification(payload_bytes: bytes):
    s = payload_bytes.decode("utf-8", errors="ignore")
    doc = json.loads(s)
    sgn = doc.get("m2m:sgn") or {}
    sur = sgn.get("sur")
    rqi = sgn.get("rqi")
    nev = sgn.get("nev") or {}
    rep = nev.get("rep") or {}

    cin = rep.get("m2m:cin")
    con = None
    ty  = None

    if cin:
        con = cin.get("con")
        ty  = 4
    else:
        uri = rep.get("m2m:uri") or rep.get("vrq") or rep.get("ri")
        if not uri:
            if isinstance(rep, str):
                uri = rep
        if uri:
            cin_doc = fetch_cin_by_uri(uri)
            cin = cin_doc.get("m2m:cin") or cin_doc.get("m2m:cin_", cin_doc)
            if isinstance(cin_doc.get("m2m:list"), list):
                items = cin_doc["m2m:list"]
                if items and isinstance(items[-1], dict):
                    cin = items[-1].get("m2m:cin") or items[-1]
            if cin:
                con = cin.get("con")
                ty  = 4

    return {"sur": sur, "cin": cin, "con": con, "rqi": rqi, "ty": ty, "raw": s}

# ---------- CSV logger ----------
class CSVLogger:
    def __init__(self, path):
        self.path = path
        self.fh = open(path, "a", newline="", encoding="utf-8")
        self.wr = csv.writer(self.fh)
        if Path(path).stat().st_size == 0:
            self.wr.writerow(["ts","topic","sur","ty","con_json"])
    def write(self, topic, parsed):
        self.wr.writerow([
            datetime.now().isoformat(timespec="seconds"),
            topic,
            parsed.get("sur"),
            parsed.get("ty"),
            json.dumps(parsed.get("con"), ensure_ascii=False),
        ])
        self.fh.flush()
    def close(self):
        self.fh.close()

# ---------- MQTT listener ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default=IP, help=f"MQTT broker IP (default {IP})")
    ap.add_argument("--port", type=int, default=int(MQTT_PORT))
    ap.add_argument("--topic", action="append", required=True,
                    help="subscribe topic (repeatable). ex) MetaSejong-PatrolRobot-FireSensor-Anomaly")
    ap.add_argument("--csv", default=None, help="optional csv log path")
    args = ap.parse_args()

    csvlog = CSVLogger(args.csv) if args.csv else None

    def on_connect(cli, u, f, rc):
        print(f"[MQTT] connected rc={rc}")
        for t in args.topic:
            cli.subscribe(t)
            print(f"[MQTT] subscribed: {t}")

    def on_message(cli, u, msg):
        try:
            parsed = parse_notification(msg.payload)
            print("─"*70)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] topic={msg.topic}")
            print(f"  sur: {parsed.get('sur')}")
            print(f"  ty : {parsed.get('ty')}")
            print(f"  con: {json.dumps(parsed.get('con'), ensure_ascii=False)}")
            if csvlog: csvlog.write(msg.topic, parsed)
        except Exception as e:
            print("[ERR] parse failed:", e)

    cli = mqtt.Client()
    cli.on_connect = on_connect
    cli.on_message = on_message
    cli.connect(args.broker, args.port, 60)
    try:
        cli.loop_forever()
    finally:
        if csvlog: csvlog.close()

if __name__ == "__main__":
    main()
