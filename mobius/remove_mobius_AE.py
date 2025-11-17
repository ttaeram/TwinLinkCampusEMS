import argparse
import json
import sys
import uuid
from pathlib import Path
from typing import Dict

import requests
import configparser

def load_config(cfg_path: Path) -> Dict[str, str]:
    if not cfg_path.exists():
        sys.exit(f"[ERR] config not found: {cfg_path}")
    p = configparser.ConfigParser(interpolation=None)
    p.read(cfg_path)

    if not p.has_section("API"):
        sys.exit("[ERR] [API] section missing in config.ini")

    api = p["API"]
    ip = api.get("IOTPLATFORM_IP", "").strip()
    http_port = api.get("IOTPLATFORM_HTTP_PORT", "").strip()
    http_tpl = api.get("IOTPLATFORM_URL_HTTP", "http://{}:{}").strip()
    origin = api.get("MOBIUS_ORIGIN", "CAdmin").strip()

    if not ip or not http_port:
        sys.exit("[ERR] IOTPLATFORM_IP or IOTPLATFORM_HTTP_PORT missing in config.ini")

    base_url = f"{http_tpl.format(ip, http_port).rstrip('/')}/Mobius"
    return {"base_url": base_url, "origin": origin}

def headers(origin: str) -> Dict[str, str]:
    return {
        "X-M2M-Origin": origin,
        "X-M2M-RI": str(uuid.uuid4()),
        "Accept": "application/json",
        "X-M2M-RVI": "3"
    }

def delete_ae(base_url: str, origin: str, rn: str, timeout: float = 10.0) -> None:
    url = f"{base_url.rstrip('/')}/{rn}"
    resp = requests.delete(url, headers=headers(origin), timeout=timeout)

    if resp.status_code in (200, 202, 204):
        print(f"[OK] AE deleted: rn={rn}")
        if resp.content:
            try:
                print(json.dumps(resp.json(), ensure_ascii=False, indent=2))
            except Exception:
                print(resp.text)
    else:
        print(f"[ERR] delete AE failed: {resp.status_code}")
        try:
            print(json.dumps(resp.json(), ensure_ascii=False, indent=2))
        except Exception:
            print(resp.text)
        sys.exit(1)

def main() -> None:
    parser = argparse.ArgumentParser(description="Delete Mobius AE using config/config.ini only")
    parser.add_argument("--rn", required=True, help="AE resourceName")
    parser.add_argument("--config", default=str(Path(__file__).parent / "config" / "config.ini"),
                        help="Path to config.ini (default: ./config/config.ini)")
    parser.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout seconds")
    args = parser.parse_args()

    cfg = load_config(Path(args.config))
    delete_ae(cfg["base_url"], cfg["origin"], args.rn, args.timeout)

if __name__ == "__main__":
    main()
