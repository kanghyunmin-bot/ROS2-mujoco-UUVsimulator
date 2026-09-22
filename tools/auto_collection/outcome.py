"""Conservative simulation labels; never fed to the visual teacher."""

def classify_release(initial_ids, rows, start_s):
    """Require an initially attached magnet and real rake contact at release."""
    for row in rows:
        if row.get('id') not in initial_ids or not row.get('detached'):
            continue
        if row.get('release_time_s', -1) < start_s:
            continue
        success = (row.get('eq_active') is False
                   and row.get('release_rake_contact') is True
                   and row.get('release_hand_contact') is True
                   and bool(row.get('release_reason')))
        return {'success': success, 'reason': 'fork_contact_release' if success
                else 'release_without_verified_fork_contact', 'buoy': row}
    return None
