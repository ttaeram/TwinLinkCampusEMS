import os
import uuid
import requests
import configparser
from pathlib import Path
from typing import Dict, List, Any, Optional

# ---------- Config loader ----------
def load_api_config() -> Dict[str, Optional[str]]:
    cfg_path = Path(__file__).resolve().parent / "config" / "config.ini"
    api = {}
    if cfg_path.exists():
        parser = configparser.ConfigParser(interpolation=None)
        parser.read(cfg_path)
        if parser.has_section("API"):
            sec = parser["API"]
            api = {
                "ip": sec.get("IOTPLATFORM_IP", fallback=None),
                "http_port": sec.get("IOTPLATFORM_HTTP_PORT", fallback=None),
                "mqtt_port": sec.get("IOTPLATFORM_MQTT_PORT", fallback=None),
                "http_tpl": sec.get("IOTPLATFORM_URL_HTTP", fallback="http://{}:{}"),
                "mqtt_tpl": sec.get("IOTPLATFORM_URL_MQTT", fallback="mqtt://{}/{}?ct=json"),
            }
    return api or {
        "ip": None,
        "http_port": None,
        "mqtt_port": None,
        "http_tpl": "http://{}:{}",
        "mqtt_tpl": "mqtt://{}/{}?ct=json",
    }

_API = load_api_config()

def _compose_http_base(api: Dict[str, Optional[str]]) -> Optional[str]:
    ip = api.get("ip")
    port = api.get("http_port")
    tpl = api.get("http_tpl") or "http://{}:{}"
    if ip and port:
        return tpl.format(ip, port)
    return None

_HTTP_BASE = _compose_http_base(_API)

# ---------- Effective settings (ENV > config.ini > defaults) ----------
MOBIUS_BASE_URL = os.getenv("MOBIUS_BASE_URL")
MOBIUS_ORIGIN   = os.getenv("MOBIUS_ORIGIN", "CAdmin")
FD_AE_NAME      = os.getenv("FD_AE_NAME", "CampusEMS")

def mqtt_nu(topic: str) -> str:
    ip = _API.get("ip") or "127.0.0.1"
    mport = _API.get("mqtt_port") or "1883"
    tpl = "mqtt://{}:{}/{}?ct=json"
    return tpl.format(ip, mport, topic)

# ---------- HTTP helpers ----------
HEADERS_BASE = {
    "X-M2M-Origin": MOBIUS_ORIGIN,
    "Accept": "application/json",
    "X-M2M-RVI": "3",
}

def _headers_with_type(ty: int) -> Dict[str, str]:
    h = dict(HEADERS_BASE)
    h["X-M2M-RI"] = f"ri-{uuid.uuid4().hex}"
    h["Content-Type"] = f"application/json;ty={ty}"
    return h

def _post(url: str, ty: int, payload: Dict[str, Any]) -> requests.Response:
    return requests.post(url, json=payload, headers=_headers_with_type(ty), timeout=10)

# ---------- oneM2M creators ----------
def _create_ae(cse_base_url: str, rn: str, api: str, rr: bool, poa: List[str]) -> None:
    body = {"m2m:ae": {"rn": rn, "api": api, "rr": rr, "poa": poa}}
    resp = _post(cse_base_url, ty=2, payload=body)
    if resp.status_code in (200, 201):
        print(f"[AE] created: {rn}")
    elif resp.status_code == 409:
        print(f"[AE] already exists: {rn}")
    else:
        raise RuntimeError(f"AE create failed {rn}: {resp.status_code} {resp.text}")

def _create_container(parent_url: str, rn: str, lbl: List[str] = None, mni: int = None) -> None:
    cnt: Dict[str, Any] = {"rn": rn}
    if lbl:
        cnt["lbl"] = lbl
    if mni is not None:
        cnt["mni"] = mni
    body = {"m2m:cnt": cnt}
    resp = _post(parent_url, ty=3, payload=body)
    if resp.status_code in (200, 201):
        print(f"[CNT] created: {parent_url}/{rn}")
    elif resp.status_code == 409:
        print(f"[CNT] already exists: {parent_url}/{rn}")
    else:
        raise RuntimeError(f"CNT create failed {parent_url}/{rn}: {resp.status_code} {resp.text}")
    
def _create_cin(parent_url: str, con: Any) -> None:
    body = {
        "m2m:cin": {
            "con": con,
            "cnf": "application/json"
        }
    }
    resp = _post(parent_url, ty=4, payload=body)
    if resp.status_code in (200, 201):
        print(f"[CIN] created under: {parent_url}")
    elif resp.status_code == 409:
        print(f"[CIN] already exists? {parent_url}: {resp.status_code} {resp.text}")
    else:
        raise RuntimeError(f"CIN create failed {parent_url}: {resp.status_code} {resp.text}")

def _create_subscription(parent_url: str, rn: str, enc_net: List[int], nct: int, nu: List[str]) -> None:
    body = {"m2m:sub": {"rn": rn, "enc": {"net": enc_net}, "nct": nct, "nu": nu}}
    resp = _post(parent_url, ty=23, payload=body)
    if resp.status_code in (200, 201):
        print(f"[SUB] created: {parent_url}/{rn}")
    elif resp.status_code == 409:
        print(f"[SUB] already exists: {parent_url}/{rn}")
    else:
        raise RuntimeError(f"SUB create failed {parent_url}/{rn}: {resp.status_code} {resp.text}")

def ensure_data_and_subs(parent_url: str, node: Dict[str, Any]) -> None:
    # 1) find data spec if present
    data_spec = None
    for c in node.get("cnt", []):
        if isinstance(c, dict) and c.get("rn") == "data":
            data_spec = c
            break

    if data_spec is None:
        _create_container(parent_url, "data", mni=3600)
        data_url = f"{parent_url}/data"
        subs_list = node.get("subs", [])
        for s in subs_list:
            _create_subscription(
                data_url,
                s["rn"],
                s["enc"]["net"],
                s["nct"],
                [mqtt_nu(_extract_topic(s["nu"][0]))],
            )
        return
    

    _create_container(parent_url, "data", mni=data_spec.get("mni"))
    data_url = f"{parent_url}/data"

    subs_at_parent = node.get("subs", [])
    subs_at_data = data_spec.get("subs", [])
    for s in subs_at_parent + subs_at_data:
        _create_subscription(
            data_url,
            s["rn"],
            s["enc"]["net"],
            s["nct"],
            [mqtt_nu(_extract_topic(s["nu"][0]))],
        )

def _create_subscriptions_here(parent_url: str, subs: List[Dict[str, Any]]) -> None:
    if not subs:
        return
    for s in subs:
        rn   = s["rn"]
        enc  = s.get("enc", {"net": [3]})
        nct  = s.get("nct", 2)
        nu   = s.get("nu", [])
        _create_subscription(parent_url, rn, enc["net"], nct, nu)

def build_tree_recursive(parent_url: str, nodes: List[Dict[str, Any]]) -> None:
    for node in nodes:
        rn  = node["rn"]
        lbl = node.get("lbl")
        # 1) 컨테이너 생성
        _create_container(parent_url, rn, lbl=lbl)
        here_url = f"{parent_url}/{rn}"

        # 2) 이 노드에 subs가 있으면 "현재 URL"에 SUB 생성
        if "subs" in node:
            _create_subscriptions_here(here_url, node["subs"])

        # 3) 이 노드에 cin 정의가 있으면, 여기 URL 아래에 CIN 생성
        cin_list = node.get("cin", [])
        for ci in cin_list:
            # 트리 정의에서 ci를 {"con": {...}} 형태로 넣었으니까 그걸 받아서 사용
            if isinstance(ci, dict) and "con" in ci:
                _create_cin(here_url, ci["con"])
            else:
                # 혹시 그냥 con만 넘겼으면 그대로 쓰기
                _create_cin(here_url, ci)

        # 4) 하위 cnt 재귀
        if "cnt" in node and isinstance(node["cnt"], list):
            build_tree_recursive(here_url, node["cnt"])

def _extract_topic(nu_url: str) -> str:
    try:
        if "mqtt://" in nu_url:
            # strip scheme
            rest = nu_url.split("mqtt://", 1)[1]
            # drop host
            after_host = rest.split("/", 1)[1] if "/" in rest else rest
            # drop query
            topic = after_host.split("?", 1)[0]
            return topic
        return nu_url
    except Exception:
        return nu_url

# ---------- Tree builder ----------
def build_tree(ae_url: str, tree: List[Dict[str, Any]]) -> None:
    for top in tree:
        top_rn = top["rn"]
        _create_container(ae_url, top_rn, lbl=top.get("lbl"))
        top_url = f"{ae_url}/{top_rn}"

        for child in top.get("cnt", []):
            child_rn = child["rn"]
            _create_container(top_url, child_rn, lbl=child.get("lbl"))
            child_url = f"{top_url}/{child_rn}"
            # ensure_data_and_subs(child_url, child)

# ---------- Main ----------
def main():
    _create_ae(
        cse_base_url=MOBIUS_BASE_URL,
        rn=FD_AE_NAME,
        api="N.Campus.EMS",
        rr=True,
        poa=[],
    )

    ae_url = f"{MOBIUS_BASE_URL}/{FD_AE_NAME}"

    tree = [
        {
            "rn": "CCTV",
            "lbl": ["type=camera", "cid=C"],
            "cnt": [
                {
                    "rn": "ChungmuHall",
                    "lbl": ["type=region", "rid=CH"],
                    "cnt": [
                        {
                            "rn": "CCTV01",
                            "lbl": ["type=camera", "rid=C-CH-01", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-CH-01-D"]
                                }
                            ],
                        },
                        {
                            "rn": "CCTV02",
                            "lbl": ["type=camera", "rid=C-CH-02", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-CH-02-D"]
                                }
                            ],
                        },
                    ]
                },
                {
                    "rn": "YongdukHall",
                    "lbl": ["type=region", "rid=YH"],
                    "cnt": [
                        {
                            "rn": "CCTV01",
                            "lbl": ["type=camera", "rid=C-YH-01", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-YH-01-D"]
                                }
                            ],
                        },
                        {
                            "rn": "CCTV02",
                            "lbl": ["type=camera", "rid=C-YH-02", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-YH-02-D"]
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "GwanggaetoHall",
                    "lbl": ["type=region", "rid=GH"],
                    "cnt": [
                        {
                            "rn": "CCTV01",
                            "lbl": ["type=camera", "rid=C-GH-01", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-GH-01-D"]
                                }
                            ],
                        },
                        {
                            "rn": "CCTV02",
                            "lbl": ["type=camera", "rid=C-GH-02", "x=10.0", "y=10.0", "z=0.0", "adjx=10.0", "adjy=10.5", "adjz=0.0"],
                            "cnt": [
                                {
                                    "rn": "ImageRawData",
                                    "lbl": ["type=data", "did=C-GH-02-D"]
                                }
                            ],
                        },
                    ],
                },
            ],
        },
        {
            "rn": "Robots",
            "lbl": ["type=robots", "rid=R"],
            "cnt": [
                {
                    "rn": "Robot01",
                    "lbl": ["type=robot", "region=ChungmuHall", "sid=R-CH-01"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-CH-01-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-CH-01-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot01-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-CH-01-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-CH-01-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-01-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-CH-01-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot01",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot01/camera",
                                                        "--result-topic", "/robot01/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-01-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-01-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot02",
                    "lbl": ["type=robot", "region=ChungmuHall", "sid=R-CH-02"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-CH-02-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-CH-02-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot02-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-CH-02-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-CH-02-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-02-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-CH-02-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot02",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot02/camera",
                                                        "--result-topic", "/robot02/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-02-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-02-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot03",
                    "lbl": ["type=robot", "region=ChungmuHall", "sid=R-CH-03"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-CH-03-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-CH-03-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot03-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-CH-03-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-CH-03-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-03-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-CH-03-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot03",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot03/camera",
                                                        "--result-topic", "/robot03/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-03-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-CH-03-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot04",
                    "lbl": ["type=robot", "region=YongdukHall", "sid=R-YH-04"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-YH-04-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-YH-04-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot04-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-YH-04-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-YH-04-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-04-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-YH-04-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot04",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot04/camera",
                                                        "--result-topic", "/robot04/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-04-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-04-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot05",
                    "lbl": ["type=robot", "region=YongdukHall", "sid=R-YH-05"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-YH-05-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-YH-05-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot05-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-YH-05-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-YH-05-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-05-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-YH-05-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot05",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot05/camera",
                                                        "--result-topic", "/robot05/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-05-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-05-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot06",
                    "lbl": ["type=robot", "region=YongdukHall", "sid=R-YH-06"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-YH-06-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-YH-06-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot06-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-YH-06-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-YH-06-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-06-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-YH-06-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot06",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot06/camera",
                                                        "--result-topic", "/robot06/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-06-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-YH-06-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot07",
                    "lbl": ["type=robot", "region=GwanggaetoHall", "sid=R-GH-07"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-GH-07-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-GH-07-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot07-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-GH-07-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-GH-07-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-07-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-GH-07-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot07",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot07/camera",
                                                        "--result-topic", "/robot07/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-07-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-07-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot08",
                    "lbl": ["type=robot", "region=GwanggaetoHall", "sid=R-GH-08"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-GH-08-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-GH-08-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot08-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-GH-08-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-GH-08-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-08-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-GH-08-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot08",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot08/camera",
                                                        "--result-topic", "/robot08/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-08-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-08-VLM"],
                                }
                            ],
                        },
                    ],
                },
                {
                    "rn": "Robot09",
                    "lbl": ["type=robot", "region=GwanggaetoHall", "sid=R-GH-09"],
                    "cnt": [
                        {
                            "rn": "ImageRawData",
                            "lbl": ["type=data", "did=R-GH-09-D"]
                        },
                        {
                            "rn": "Control",
                            "lbl": ["type=command", "cid=R-GH-09-C"],
                            "cnt": [
                                {
                                    "rn": "Nav",
                                    "lbl": ["type=command"],
                                    "subs": [
                                        {
                                            "rn": "Nav_sub",
                                            "enc": {"net": [3]},
                                            "nct": 2,
                                            "nu": [mqtt_nu("CampusEMS-Robots-Robot09-Control-Nav")]
                                        }
                                    ]
                                },
                            ],
                        },
                        {
                            "rn": "Report",
                            "lbl": ["type=report", "rid=R-GH-09-R"],
                        },
                        {
                            "rn": "ModelDeploymentList",
                            "lbl": ["type=modelDeploymentList", "mid=R-GH-09-MDL"],
                            "cnt": [
                                {
                                    "rn": "ObjectDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-09-ODM", "modelRef=ODM"],
                                    "cin": [
                                        {
                                            "con": {
                                                "kind": "deployment",
                                                "deploymentId": "R-GH-09-ODM",
                                                "modelRef": "ODM",
                                                "modelPath": "/CampusEMS/ModelRepo/ObjectDetectionModel",
                                                "status": "active",
                                                "version": "yolov8s-2024-11-15",
                                                "runtime": {
                                                    "host": "robot09",
                                                    "device": "cuda:0",
                                                    "num_workers": 2
                                                },
                                                "infer": {
                                                    "type": "python",
                                                    "entry": "python -m robot.perception.detector",
                                                    "args": [
                                                        "--weights", "/models/yolov8s.pt",
                                                        "--source-topic", "/robot09/camera",
                                                        "--result-topic", "/robot09/detections"
                                                    ]
                                                }
                                            }
                                        }
                                    ]
                                },
                                {
                                    "rn": "HumanDetectionModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-09-HDM"],
                                },
                                {
                                    "rn": "VisualLocalizationModel",
                                    "lbl": ["type=modelDeployment", "mid=R-GH-09-VLM"],
                                }
                            ],
                        },
                    ],
                },
            ],
        },
        {
            "rn": "ControlCenter",
            "lbl": ["type=controlCenter", "ccid=CC"],
            "cnt": [
                {
                    "rn": "Manager",
                    "lbl": ["type=manager", "mid=CC-M"],
                    "cnt": [
                        {
                            "rn": "VoiceRawData",
                            "lbl": ["type=data", "did=CC-M-D"],
                            "subs": [
                                {
                                    "rn": "Voice_Sub",
                                    "enc": {"net": [3]},
                                    "nct": 2,
                                    "nu": [mqtt_nu("CampusEMS-ControlCenter-Manager-VoiceRawData")]
                                }
                            ]
                        },
                    ],
                },
                {
                    "rn": "ModelDeploymentList",
                    "lbl": ["type=modelDeployment", "mid=CC-M-MDL"],
                    "cnt": [
                        {
                            "rn": "STTModel",
                            "lbl": ["type=modelDeployment", "mid=CC-M-STTM", "modelRef=STTM"],
                            "cin": [
                                {
                                    "con": {
                                        "kind": "deployment",
                                        "deploymentId": "CC-M-STTM",
                                        "modelRef": "STTM",
                                        "modelPath": "/CampusEMS/ModelRepo/STTModel",
                                        "status": "active",
                                        "runtime": {
                                            "host": "control-center-01",
                                            "device": "cuda:0"
                                        },
                                        "infer": {
                                            "type": "python",
                                            "entry": "python -m cc.stt.server",
                                            "args": [
                                                "--hf-repo-id", "GGarri/whisper_finetuned_ver241113_2",
                                                "--input-topic", "CampusEMS-ControlCenter-Manager-VoiceRawData",
                                                "--output-topic", "CampusEMS-ControlCenter-Manager-VoiceText",
                                            ],
                                        },
                                    },
                                },
                            ],
                        },
                    ],
                },
            ],
        },
        {
            "rn": "ModelRepo",
            "lbl": ["type=modelRepo", "rid=MR"],
            "cnt": [
                {
                    "rn": "ObjectDetectionModel",
                    "lbl": ["type=model", "mid=ODM", "task=object-detection"],
                    "cin": [
                        {
                            "con": {
                                "kind": "model",
                                "id": "ODM",
                                "name": "YOLOv8s Object Detection",
                                "task": "object-detection",
                                "framework": "ultralytics",
                                "upstream": {
                                    "type": "pypi",
                                    "package": "ultralytics",
                                    "version": "latest",
                                    "model_name": "yolov8s"
                                },
                                "artifact": {
                                    "type": "local",
                                    "path": "/models/yolov8s.pt"
                                },
                                "runtime": {
                                    "device": "cuda:0",
                                    "precision": "fp16"
                                }
                            }
                        }
                    ]
                },
                {
                    "rn": "STTModel",
                    "lbl": ["type=model", "mid=STTM", "task=stt"],
                    "cin": [
                        {
                            "con": {
                                "kind": "model",
                                "id": "STTM",
                                "name": "Whisper finetuned ver241113_2",
                                "task": "speech-to-text",
                                "framework": "transformers",
                                "upstream": {
                                    "type": "huggingface",
                                    "repo_id": "GGarri/whisper_finetuned_ver241113_2"
                                },
                                "runtime": {
                                    "device": "cuda:0",
                                    "language": "ko",
                                    "max_audio_sec": 30
                                }
                            }
                        }
                    ]
                },
                {
                    "rn": "HumanDetectionModel",
                    "lbl": ["type=model", "mid=HDM"],
                },
                {
                    "rn": "VisualLocalizationModel",
                    "lbl": ["type=model", "mid=VLM"],
                },
            ],
        },
        {
            "rn": "DatasetRepo",
            "lbl": ["type=datasetRepo", "rid=DR"],
            "cnt": [
                {
                    "rn": "RobotImage",
                    "lbl": ["type=data", "did=DR-RI"],
                    "cnt": [
                        {
                            "rn": "Robot01",
                            "lbl": ["type=data", "did=DR-RI-01"],
                        },
                        {
                            "rn": "Robot02",
                            "lbl": ["type=data", "did=DR-RI-02"],
                        },
                        {
                            "rn": "Robot03",
                            "lbl": ["type=data", "did=DR-RI-03"],
                        },
                        {
                            "rn": "Robot04",
                            "lbl": ["type=data", "did=DR-RI-04"],
                        },
                        {
                            "rn": "Robot05",
                            "lbl": ["type=data", "did=DR-RI-05"],
                        },
                        {
                            "rn": "Robot06",
                            "lbl": ["type=data", "did=DR-RI-06"],
                        },
                        {
                            "rn": "Robot07",
                            "lbl": ["type=data", "did=DR-RI-07"],
                        },
                        {
                            "rn": "Robot08",
                            "lbl": ["type=data", "did=DR-RI-08"],
                        },
                        {
                            "rn": "Robot09",
                            "lbl": ["type=data", "did=DR-RI-09"],
                        },
                    ],
                },
                {
                    "rn": "CCTVImage",
                    "lbl": ["type=data", "did=DR-CI"],
                    "cnt": [
                        {
                            "rn": "ChungmuHall01",
                            "lbl": ["type=data", "did=DR-CI-CH-01"],
                        },
                        {
                            "rn": "ChungmuHall02",
                            "lbl": ["type=data", "did=DR-CI-CH-02"],
                        },
                        {
                            "rn": "YongdukHall01",
                            "lbl": ["type=data", "did=DR-CI-YH-01"],
                        },
                        {
                            "rn": "YongdukHall02",
                            "lbl": ["type=data", "did=DR-CI-YH-02"],
                        },
                        {
                            "rn": "GwanggaetoHall01",
                            "lbl": ["type=data", "did=DR-CI-GH-01"],
                        },
                        {
                            "rn": "GwanggaetoHall02",
                            "lbl": ["type=data", "did=DR-CI-GH-02"],
                        },
                    ]
                },
                {
                    "rn": "Voice",
                    "lbl": ["type=data", "did=DR-V"],
                },
            ],
        },
        {
            "rn": "DataScientist",
            "lbl": ["type=user", "uid=DS"],
        },
    ]

    build_tree_recursive(ae_url, tree)
    print("Done.")

if __name__ == "__main__":
    main()
