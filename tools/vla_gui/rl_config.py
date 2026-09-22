"""Persist editable reinforcement-learning experiment settings."""
import math


def defaults():
    return {'schema_version': 1, 'resume_candidate': False, 'general_release_reward': 3.0, 'hand_release_reward': 8.0, 'models': ['success-59-10000', 'success-mixed-84-15000'],
            'environments_per_model': 3, 'episode_seconds': 35.0,
            'episodes_per_environment': 100, 'success_reward': 20.0,
            'timeout_reward': -2.0, 'time_cost_per_second': .02,
            'save_successes': True, 'fast_success_reward': 5.0,
            'height_progress_reward': 2.0, 'approach_progress_reward': 1.0, 'contact_bonus_reward': 2.0, 'height_scale_m': .1,
            'approach_scale_m': 1.0, 'rediscovery_reward': .5,
            'wrong_release_reward': -5.0, 'search_depth_m': .03,
            'search_yaw_rad': .1, 'policy_groups': {}, 'reward_version': 'legacy',
            'evaluation_repeats': 1, 'ppo_learning_rate': .0003,
            'ppo_clip': .2, 'ppo_kl_limit': .01, 'ppo_drift_limit': .005,
            'ppo_gamma': .99, 'ppo_lambda': .95, 'ppo_entropy': 0.0}


def validate(data, models):
    result = defaults()
    data = {**result, **data}
    if set(data) != set(result):
        raise ValueError('설정 항목이 올바르지 않습니다.')
    if data['schema_version'] != 1 or data['save_successes'] is not True:
        raise ValueError('지원하지 않는 설정입니다.')
    if type(data['resume_candidate']) is not bool:
        raise ValueError('후보 재개 설정이 올바르지 않습니다.')
    if not 0 <= data['general_release_reward'] < data['hand_release_reward'] < data['success_reward']:
        raise ValueError('분리 보상은 일반 < 손 카메라 < 우측 포크 순서여야 합니다.')
    selected = data['models']
    if not isinstance(selected, list) or not selected or any(not isinstance(k, str) or k not in models for k in selected):
        raise ValueError('사용 가능한 시작 모델을 선택하세요.')
    if len(selected) != len(set(selected)):
        raise ValueError('시작 모델이 중복되었습니다.')
    if any(models[k].get('residual_checkpoint') for k in selected):
        raise ValueError('PPO 실행 후보는 모델 실행 화면에서 선택하세요. 강화학습은 원본 모델과 재개 설정을 사용합니다.')
    for key, maximum in [('environments_per_model', 6), ('episodes_per_environment', 10000)]:
        if type(data[key]) is not int or not 1 <= data[key] <= maximum:
            raise ValueError('환경 수 또는 시도 횟수가 범위를 벗어났습니다.')
    groups = data['policy_groups']
    if not isinstance(groups, dict):
        raise ValueError('정책별 환경 설정이 올바르지 않습니다.')
    total = len(selected) * data['environments_per_model']
    if groups:
        from pathlib import Path
        import re
        total = 0
        for name, group in groups.items():
            if not isinstance(name, str) or not re.fullmatch('[a-z0-9_-]+', name):
                raise ValueError('정책 그룹 이름이 올바르지 않습니다.')
            if not isinstance(group, dict) or set(group) != {'base_model', 'environments', 'checkpoint'}:
                raise ValueError('정책 그룹 항목이 올바르지 않습니다.')
            if group['base_model'] not in selected:
                raise ValueError('정책 그룹의 원본 모델을 선택하세요.')
            if type(group['environments']) is not int or not 1 <= group['environments'] <= 6:
                raise ValueError('정책별 환경 수는 1~6개입니다.')
            if not isinstance(group['checkpoint'], str) or not Path(group['checkpoint']).is_file():
                raise ValueError('시작 체크포인트를 찾을 수 없습니다.')
            total += group['environments']
        if not data['resume_candidate'] or data['reward_version'] != 'precision_v3':
            raise ValueError('정책 그룹은 precision_v3와 명시적 시작 정책을 사용합니다.')
    if total > 6:
        raise ValueError('전체 환경 수는 최대 6개입니다.')
    if data['reward_version'] not in ('legacy', 'precision_v3'):
        raise ValueError('지원하지 않는 보상 버전입니다.')
    if data['reward_version'] == 'precision_v3' and not groups:
        raise ValueError('정밀 보상은 명시적 정책 그룹이 필요합니다.')
    if type(data['evaluation_repeats']) is not int or not 1 <= data['evaluation_repeats'] <= 5:
        raise ValueError('평가 반복 수는 1~5입니다.')
    for key, low, high in [('ppo_learning_rate', 1e-6, .001), ('ppo_clip', .05, .3),
                           ('ppo_kl_limit', .001, .02), ('ppo_drift_limit', .001, .01),
                           ('ppo_gamma', .9, .9999), ('ppo_lambda', .8, .999),
                           ('ppo_entropy', 0, .01)]:
        value = data[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('PPO 설정 범위 오류: '+key)
    for key in ('general_release_reward', 'hand_release_reward', 'episode_seconds', 'success_reward', 'timeout_reward', 'time_cost_per_second',
                'fast_success_reward', 'approach_progress_reward', 'contact_bonus_reward', 'height_progress_reward', 'height_scale_m',
                'approach_scale_m', 'rediscovery_reward', 'wrong_release_reward',
                'search_depth_m', 'search_yaw_rad'):
        value = data[key]
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 10000:
            raise ValueError('보상과 시간에는 유효한 숫자를 입력하세요.')
    if not 0 < data['episode_seconds'] <= 600 or data['time_cost_per_second'] < 0:
        raise ValueError('제한 시간은 0~600초, 시간 비용은 0 이상이어야 합니다.')
    for key in ('height_scale_m', 'approach_scale_m', 'search_depth_m', 'search_yaw_rad'):
        if data[key] <= 0:
            raise ValueError('거리와 각도 기준은 0보다 커야 합니다.')
    for key in ('fast_success_reward', 'approach_progress_reward', 'contact_bonus_reward', 'height_progress_reward', 'rediscovery_reward'):
        if data[key] < 0:
            raise ValueError('추가 보상은 0 이상이어야 합니다.')
    return dict(data)
