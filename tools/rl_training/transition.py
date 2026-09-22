"""Associate action feedback by explicit command ID, including delayed inference."""
def attach_feedback(trajectory, data):
    feedback = data.get('feedback')
    if feedback is not None:
        index = feedback['action_id']
        if not 0 <= index < len(trajectory) or 'feedback' in trajectory[index]:
            raise ValueError('Unknown or duplicate action feedback')
        trajectory[index]['feedback'] = feedback
    if data.get('done'):
        last = data['last_applied_id']
        del trajectory[last+1:]
        if not trajectory or any('feedback' not in row for row in trajectory):
            raise ValueError('Missing applied-action feedback')
