"""Separate parallel sampling from sequential PPO and repeated evaluation."""


def round_plan(wave, repeats=1):
    if wave == 0:
        return 'baseline_evaluation', 0
    position = (wave-1) % (1+2*repeats)
    if position == 0:
        return 'training', 0
    if position <= repeats:
        return 'incumbent_evaluation', position-1
    return 'candidate_evaluation', position-repeats-1
