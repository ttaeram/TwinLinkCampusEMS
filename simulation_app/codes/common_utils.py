# Copyright (c) 2025, IoT Convergence & Open Sharing System (IoTCOSS)
#
# All rights reserved. This software and its documentation are proprietary and confidential.
# The IoT Convergence & Open Sharing System (IoTCOSS) retains all intellectual property rights,
# including but not limited to copyrights, patents, and trade secrets, in and to this software
# and related documentation. Any use, reproduction, disclosure, or distribution of this software
# and related documentation without explicit written permission from IoTCOSS is strictly prohibited.
#

import hashlib

def check_missing_mandatory_key(data:dict, keys:list):
    """yaml 파일에 대한 검사로 필수키가 모두 있는지 확인하는 함수"""
    missing_keys = [key for key in keys if key not in data]
    if missing_keys:
        raise KeyError(f"Missing mandatory keys: {', '.join(missing_keys)}") 
    
def generate_md5_hash(plaintext:str, seed:str):
    """plaintext 문자열을 seed 문자열을 키로 하여 md5 해시 값을 생성하는 함수"""
    return hashlib.md5(f"{plaintext}{seed}".encode()).hexdigest()

def validate_md5_hash(hash:str, plaintext:str, seed:str):
    """hash된 문자열과 plaintext + seed 문자열 조합을 비교하여 동일한지 검증하는 함수"""
    generated_hash = generate_md5_hash(plaintext, seed)
    return hash == generated_hash
