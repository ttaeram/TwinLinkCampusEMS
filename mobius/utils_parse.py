import re
from datetime import datetime, timezone

_key_token = re.compile(r'([A-Za-z0-9_.\s\[\]%]+):\s*("[^"]*"|[^\s]+)')

def _normalize_key(k: str) -> str:
    k = k.strip()
    # 괄호 단위 추출 (예: Temperature[C] -> temperature_c)
    unit = None
    if '[' in k and ']' in k:
        base, unit = k.split('[', 1)
        k = base.strip()
        unit = unit.split(']', 1)[0].strip().lower()
    # 공백/점/퍼센트 기호 표준화
    k = k.lower().replace('%', 'pct').replace(' ', '_').replace('.', '_')
    # 단위가 있으면 접미사 추가
    if unit:
        k = f'{k}_{unit}'
    # 특수 케이스 맵핑
    if k == 'no_id': k = 'no_id'
    if k == 'utc': k = 'utc_epoch'
    if k == 'ecO2'.lower(): k = 'eco2_ppm'
    return k

def _to_number(v: str):
    v = v.strip().strip('"')
    # e-notation, int/float 자동 변환
    try:
        if v.lower().endswith('e+09') or 'e' in v.lower() or '.' in v:
            f = float(v)
            # 정수로 떨어지면 int로
            return int(f) if f.is_integer() else f
        return int(v)
    except ValueError:
        return v

def parse_sensor_line(line: str) -> dict:
    out = {}
    for m in _key_token.finditer(line):
        raw_k, raw_v = m.group(1), m.group(2)
        key = _normalize_key(raw_k)
        val = _to_number(raw_v)
        out[key] = val

    # UTC 보강: iso8601 추가
    if 'utc_epoch' in out and isinstance(out['utc_epoch'], (int, float)):
        sec = float(out['utc_epoch'])
        dt = datetime.fromtimestamp(sec, tz=timezone.utc)
        out['utc_iso'] = dt.isoformat().replace('+00:00', 'Z')
    return out
