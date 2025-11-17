import json
import subprocess
import os
import hashlib
import re
    
def generate_md5_hash(plaintext:str, seed:str):
    """plaintext 문자열을 seed 문자열을 키로 하여 md5 해시 값을 생성하는 함수"""
    return hashlib.md5(f"{plaintext}{seed}".encode()).hexdigest()

def validate_md5_hash(hash:str, plaintext:str, seed:str):
    """hash된 문자열과 plaintext + seed 문자열 조합을 비교하여 동일한지 검증하는 함수"""
    generated_hash = generate_md5_hash(plaintext, seed)
    return hash == generated_hash

def generate_team_id(team_name: str):
    trimmed_sentence = team_name.strip()
    trimmed_sentence = trimmed_sentence.lower()
    transformed_sentence = re.sub(r'[^a-zA-Z0-9]', '_', trimmed_sentence)
    return transformed_sentence

def generate_token_from_teamname(team_name: str, seed: str):
    # 딕셔너리를 JSON 문자열로 변환
    team_id = generate_team_id(team_name)
    token = generate_md5_hash(team_id, seed)

    return {'team': team_id, 'token': token}
    
# 사용 예시
if __name__ == "__main__":

    CONST_COMPETITOR_INTERFACE_NAMESPACE = 'metasejong2025'

    team_name_list = ["Robit", 'Anonymous', 'Team JungSeong', 'Temple Owl']
    for team_name in team_name_list:
        result = generate_token_from_teamname(team_name, CONST_COMPETITOR_INTERFACE_NAMESPACE)
        print(result)

   

