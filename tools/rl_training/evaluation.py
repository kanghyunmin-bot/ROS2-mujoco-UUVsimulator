"""Conservative promotion of deterministic validation rollouts."""
def summary(episodes):
    if not episodes:
        raise ValueError('Empty evaluation')
    return {'n':len(episodes),'fork':sum(e.get('release_tier')=='right_fork' for e in episodes),
            'hand_or_fork':sum(e.get('release_tier') in ('hand_view','right_fork') for e in episodes),
            'released':sum(bool(e.get('released')) for e in episodes),
            'mean_reward':sum(e['reward'] for e in episodes)/len(episodes)}

def promote(baseline, candidate, *, parameter_changed=True, reward_margin=.25):
    """Screen a fresh paired evaluation, not proof of generalization.

    The reward margin suppresses tiny timing noise; tier counts take priority.
    The caller must compare matching resets/seeds from the same update cycle.
    """
    if not parameter_changed or baseline['n']!=candidate['n']:
        return False
    keys=('fork','hand_or_fork','released')
    no_worse=all(candidate[k]>=baseline[k] for k in keys)
    better=any(candidate[k]>baseline[k] for k in keys)
    return no_worse and (better or candidate['mean_reward']>=baseline['mean_reward']+reward_margin)


def assessment(baseline, candidate, *, parameter_changed=True, reward_margin=.25,
               require_outcome_gain=False):
    """Separate selecting a best checkpoint from retaining safe learning progress."""
    if not parameter_changed:
        return 'unchanged'
    if require_outcome_gain:
        if baseline['n'] != candidate['n']:
            return 'incomplete_evaluation'
        keys = ('fork', 'hand_or_fork', 'released')
        if any(candidate[k] < baseline[k] for k in keys):
            return 'rollback'
        if any(candidate[k] > baseline[k] for k in keys):
            return 'promote'
        return 'continue_training'
    if promote(baseline, candidate, reward_margin=reward_margin):
        return 'promote'
    if baseline['n'] != candidate['n']:
        return 'incomplete_evaluation'
    if any(candidate[k] < baseline[k] for k in ('fork', 'hand_or_fork', 'released')):
        return 'rollback'
    if candidate['mean_reward'] <= baseline['mean_reward']-reward_margin:
        return 'rollback'
    # Within the comparison's noise margin: preserve candidate policy + Adam
    # for the next on-policy rollout, but do not overwrite the best checkpoint.
    return 'continue_training'
