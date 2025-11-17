import os
import time
import base64
import json
import re
import uuid
import configparser
from datetime import datetime
from typing import Optional, Tuple

import requests
from requests.exceptions import RequestException

try:
    from PIL import Image
    from io import BytesIO
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False


# --------------------------- Config ---------------------------

DEFAULT_CONFIG_PATH = "/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/mobius/config/config.ini"

def load_config(path: Optional[str] = None) -> configparser.ConfigParser:
    cfg_path = os.environ.get("MOBIUS_CONFIG", path or DEFAULT_CONFIG_PATH)
    cp = configparser.ConfigParser()
    read_files = cp.read(cfg_path)
    if not read_files:
        raise FileNotFoundError(f"config.ini not found at: {cfg_path}")
    # sane defaults
    if "Mobius" not in cp:
        cp["Mobius"] = {}
    if "Runtime" not in cp:
        cp["Runtime"] = {}
    cp["Mobius"].setdefault("base_url", "http://127.0.0.1:7579")
    cp["Mobius"].setdefault("container_path", "/Mobius/myAE/myImageCNT")
    cp["Mobius"].setdefault("origin", "CAdmin")
    cp["Mobius"].setdefault("verify_ssl", "false")
    cp["Runtime"].setdefault("interval", "2.0")
    cp["Runtime"].setdefault("out_dir", "./images")
    cp["Runtime"].setdefault("show_image", "false")
    cp["Runtime"].setdefault("file_prefix", "img_")
    return cp


# --------------------------- Helpers ---------------------------

def to_bool(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "t", "yes", "y")

def now_str() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def guess_ext_from_dataurl(data_url: str) -> str:
    m = re.match(r"data:image/([a-zA-Z0-9+.-]+);base64,", data_url)
    if m:
        return "." + m.group(1).lower().replace("jpeg", "jpg")
    return ".jpg"

def decode_image_from_con(con_value) -> Tuple[bytes, str]:
    """
    con_value 가 다음 중 하나일 때 지원:
      1) 순수 base64 문자열
      2) data URL (data:image/png;base64,....)
      3) JSON 문자열 또는 객체 { "img": "<base64 or dataurl>", ... }
    반환: (바이너리이미지, 추천확장자)
    """
    ext = ".jpg"

    # dict나 list면 이미지 필드 추출 시도
    if isinstance(con_value, (dict, list)):
        # 가장 흔한 필드 이름 후보
        candidates = ["img", "image", "data", "payload", "b64", "content"]
        for key in candidates:
            if isinstance(con_value, dict) and key in con_value:
                con_value = con_value[key]
                break
        if isinstance(con_value, (dict, list)):
            raise ValueError("Unsupported nested JSON structure for image content.")

    # 문자열이 아니면 문자열화
    if not isinstance(con_value, str):
        con_value = json.dumps(con_value, ensure_ascii=False)

    s = con_value.strip()

    # JSON 문자열이면 다시 파싱해서 필드 찾기
    if s.startswith("{") and s.endswith("}"):
        try:
            obj = json.loads(s)
            return decode_image_from_con(obj)
        except Exception:
            # JSON 아니면 그냥 진행
            pass

    # data URL?
    if s.startswith("data:image/") and ";base64," in s:
        ext = guess_ext_from_dataurl(s)
        b64 = s.split(",")[1]
        try:
            return base64.b64decode(b64), ext
        except Exception as e:
            raise ValueError(f"Invalid data URL base64: {e}")

    # 순수 base64라고 가정
    # base64에 공백/개행 제거
    b64 = re.sub(r"\s+", "", s)
    # base64 패딩 보정
    missing = len(b64) % 4
    if missing:
        b64 += "=" * (4 - missing)
    try:
        raw = base64.b64decode(b64)
        # 간단히 헤더로 확장자 추정(PNG/JPEG/GIF)
        if raw.startswith(b"\x89PNG\r\n\x1a\n"):
            ext = ".png"
        elif raw.startswith(b"\xff\xd8"):
            ext = ".jpg"
        elif raw.startswith(b"GIF8"):
            ext = ".gif"
        return raw, ext
    except Exception as e:
        raise ValueError(f"Invalid base64 content: {e}")

def ensure_dir(d: str):
    os.makedirs(d, exist_ok=True)

def show_image_if_available(data: bytes):
    if not PIL_AVAILABLE:
        print("[WARN] Pillow가 설치되어 있지 않아 화면 표시를 건너뜁니다 (pip install pillow).")
        return
    try:
        img = Image.open(BytesIO(data))
        img.show()  # OS 기본 뷰어
    except Exception as e:
        print(f"[WARN] 이미지 표시 실패: {e}")


# --------------------------- Mobius Poller ---------------------------

class MobiusImagePoller:
    def __init__(self, base_url: str, cnt_path: str, origin: str, verify_ssl: bool,
                 interval: float, out_dir: str, show_image: bool, file_prefix: str):
        self.base_url = base_url.rstrip("/")
        self.cnt_path = cnt_path  # 예: /Mobius/AE/CNT
        self.origin = origin
        self.verify_ssl = verify_ssl
        self.interval = interval
        self.out_dir = out_dir
        self.show_image = show_image
        self.file_prefix = file_prefix

        self.session = requests.Session()
        self.session.verify = verify_ssl
        self.last_ri = None  # 마지막으로 처리한 리소스 ID

    def _headers(self) -> dict:
        return {
            "Accept": "application/json",
            "X-M2M-Origin": self.origin,
            "X-M2M-RI": str(uuid.uuid4()),
        }

    def _latest_url(self) -> str:
        # latest: 컨테이너 하위 최신 cin
        # ex) http://host:7579/Mobius/AE/CNT/latest
        return f"{self.base_url}{self.cnt_path}/latest"

    def fetch_latest(self) -> Optional[dict]:
        try:
            r = self.session.get(self._latest_url(), headers=self._headers(), timeout=10)
            if r.status_code == 200:
                return r.json()
            elif r.status_code == 404:
                # 아직 cin 없음
                return None
            else:
                print(f"[WARN] HTTP {r.status_code} 응답: {r.text[:200]}")
                return None
        except RequestException as e:
            print(f"[ERROR] 요청 실패: {e}")
            return None

    def extract_con_ri_ct(self, payload: dict) -> Tuple[Optional[object], Optional[str], Optional[str]]:
        """
        payload 형태 예:
        { "m2m:cin": { "con": "...", "ri": "cin-xxx", "ct": "20250101T123456" ... } }
        """
        if not payload:
            return None, None, None
        cin = payload.get("m2m:cin") or payload.get("cin") or payload
        con = cin.get("con") if isinstance(cin, dict) else None
        ri = cin.get("ri") if isinstance(cin, dict) else None
        ct = cin.get("ct") if isinstance(cin, dict) else None
        return con, ri, ct

    def save_image(self, data: bytes, ext: str, ct: Optional[str]) -> str:
        ensure_dir(self.out_dir)
        # 파일명: prefix + 생성시각(or now) + uuid 짧게
        ts = ct if ct else now_str()
        # ct가 oneM2M 형식(YYYYMMDDThhmmss)일 수 있으니 파일명친화적으로 변환
        ts = ts.replace("T", "_")
        fname = f"{self.file_prefix}{ts}_{uuid.uuid4().hex[:6]}{ext}"
        fpath = os.path.join(self.out_dir, fname)
        with open(fpath, "wb") as f:
            f.write(data)
        return fpath

    def run(self):
        print("[INFO] Mobius Image Poller 시작")
        print(f"  base_url      : {self.base_url}")
        print(f"  container_path: {self.cnt_path}")
        print(f"  origin        : {self.origin}")
        print(f"  verify_ssl    : {self.verify_ssl}")
        print(f"  interval(sec) : {self.interval}")
        print(f"  out_dir       : {self.out_dir}")
        print(f"  show_image    : {self.show_image}")
        print("----------------------------------------------------")

        try:
            while True:
                payload = self.fetch_latest()
                if payload:
                    con, ri, ct = self.extract_con_ri_ct(payload)
                    if ri and ri != self.last_ri and con is not None:
                        try:
                            img_bytes, ext = decode_image_from_con(con)
                            path = self.save_image(img_bytes, ext, ct)
                            print(f"[OK] 새 이미지 저장: {path} (ri={ri}, ct={ct})")
                            if self.show_image:
                                show_image_if_available(img_bytes)
                            self.last_ri = ri
                        except Exception as e:
                            print(f"[ERROR] 디코딩/저장 실패: {e}")
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("\n[INFO] 종료 요청(Ctrl+C). Poller 중단.")


# --------------------------- Main ---------------------------

def main():
    cfg = load_config()
    mob = cfg["Mobius"]
    run = cfg["Runtime"]

    base_url = mob.get("base_url", "http://127.0.0.1:7579")
    cnt_path = mob.get("container_path", "/Mobius/myAE/myImageCNT")
    origin = mob.get("origin", "CAdmin")
    verify_ssl = to_bool(mob.get("verify_ssl", "false"))

    interval = float(run.get("interval", "2.0"))
    out_dir = run.get("out_dir", "./images")
    show_image = to_bool(run.get("show_image", "false"))
    file_prefix = run.get("file_prefix", "img_")

    poller = MobiusImagePoller(
        base_url=base_url,
        cnt_path=cnt_path,
        origin=origin,
        verify_ssl=verify_ssl,
        interval=interval,
        out_dir=out_dir,
        show_image=show_image,
        file_prefix=file_prefix,
    )
    poller.run()


if __name__ == "__main__":
    main()
