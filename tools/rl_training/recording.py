"""Finalize recorder output without confusing automatic saves with active recordings."""


def finish_recording(api, wait, *, previous_path, success):
    """Return saved metadata and whether the demonstration completed normally."""
    state = api()['recorder']
    if state.get('active'):
        try:
            api('recorder_action', action='success' if success else 'failure')
        except RuntimeError:
            # Automatic termination can race the stop request. Only a verified
            # new automatic save below may resolve this race.
            state = api()['recorder']
            if state.get('active'):
                raise
            if state.get('last_result', {}).get('termination_reason') != 'sampling_discontinuity':
                raise
    wait(lambda: not api()['recorder'].get('busy') and not api()['recorder'].get('active'), 'save')
    saved = api()['recorder'].get('last_result', {})
    if not saved.get('path') or saved['path'] == previous_path or saved.get('frames', 0) < 1:
        raise RuntimeError('Recording result missing or belongs to a previous episode')
    reason = saved.get('termination_reason')
    if reason == 'sampling_discontinuity' and saved.get('success') is False:
        return saved, False
    if reason != 'operator_stop' or saved.get('success') != success:
        raise RuntimeError('Recording result mismatch: ' + str(reason))
    return saved, True
