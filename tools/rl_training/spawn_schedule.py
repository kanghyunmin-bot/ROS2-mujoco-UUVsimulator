"""Separate random training layouts from paired, reproducible held-out layouts."""
from schedule import round_plan


def spawn_request(region, wave, worker_index, repeats):
    kind, repeat = round_plan(wave, repeats)
    cycle = (wave-1)//(1+2*repeats) if wave else 0
    sequence = cycle*6+worker_index if kind == 'training' else repeat*6+worker_index
    # Disjoint parity domains; fixed evaluation bank is paired across policies.
    seed = region['seed'] + 2*sequence + int(kind != 'training')
    return dict(seed=seed, x_m=region['x_m'], y_m=region['y_m'])
