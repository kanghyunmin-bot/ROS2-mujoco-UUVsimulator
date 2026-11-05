"""Conservative release diagnostics, independent of legacy reward versions.

The result is not a training reward or a complete approach-process success
metric. Old observations without contact evidence remain unknown. Instantaneous
contact cannot establish exclusive causality.
"""
import math


SCHEMA = "uuv.precision_validation.v1"


def validate_release(measurement, *, released, post_release=None):
    """Check contact and an optional, continuously sampled stability interval.

    ``released`` must come from the episode's time/equality release check.
    Post-release samples contain time_s [s], tilt_rad [rad] (body-up versus
    world-up), and angular_speed_rad_s [rad/s]. Draft diagnostic limits are
    2 s, 0.35 rad, 0.5 rad/s, with at most 0.25 s between samples. These are
    explicit acceptance assumptions, not calibrated vehicle specifications.
    """
    result = dict(schema=SCHEMA, contact_and_stability_verified=False,
                  contact_status="unknown", stability_status="not_observed",
                  reasons=[])
    reasons = result["reasons"]
    if not released:
        reasons.append("no_valid_episode_release")
        return result
    if measurement.get("release_right_fork_region") is not True:
        reasons.append("outside_right_fork_region")
    if measurement.get("release_right_fork_stem_contact") is not True:
        reasons.append("no_right_fork_stem_flag")
    evidence = measurement.get("release_contact_evidence")
    if not isinstance(evidence, dict) or evidence.get("schema") != "uuv.release_contact.v1":
        reasons.append("missing_contact_evidence")
        return result
    try:
        release_time = float(measurement["release_time_s"])
        evidence_time = float(evidence["time_s"])
        if not math.isfinite(release_time) or not math.isfinite(evidence_time):
            raise ValueError
        if abs(evidence_time - release_time) > 1e-8:
            raise ValueError
        contacts = evidence["contacts"]
        if not isinstance(contacts, list):
            raise ValueError
        for contact in contacts:
            force = float(contact["force_n"])
            if not math.isfinite(force) or force < 0:
                raise ValueError
            if contact["kind"] not in {"right_fork_stem", "right_fork_other", "other_hand", "vehicle_body"}:
                raise ValueError
            if not isinstance(contact["solver_active"], bool):
                raise ValueError
        loaded = [c for c in contacts if c["solver_active"] and c["force_n"] > 0]
    except (KeyError, TypeError, ValueError):
        reasons.append("invalid_or_stale_contact_evidence")
        return result
    if not any(c["kind"] == "right_fork_stem" for c in loaded):
        reasons.append("no_loaded_right_fork_stem_contact")
    if any(c["kind"] != "right_fork_stem" for c in loaded):
        reasons.append("simultaneous_other_contact")
    result["contact_status"] = "verified" if not reasons else "rejected"
    if post_release is None:
        reasons.append("post_release_not_observed")
        return result
    try:
        if not post_release:
            raise ValueError
        previous = release_time
        stable = True
        for sample in post_release:
            now, tilt, speed = (float(sample[key]) for key in
                                ("time_s", "tilt_rad", "angular_speed_rad_s"))
            if (not all(math.isfinite(v) for v in (now, tilt, speed))
                    or not 0 <= now - previous <= .25 + 1e-8
                    or (now == previous and previous != release_time)
                    or not 0 <= tilt <= math.pi or speed < 0):
                raise ValueError
            stable = stable and tilt <= .35 and speed <= .5
            previous = now
        if previous - release_time < 2.0 - 1e-8:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        result["stability_status"] = "unknown"
        reasons.append("incomplete_or_invalid_stability_interval")
        return result
    result["stability_status"] = "stable" if stable else "unstable"
    if not stable:
        reasons.append("post_release_unstable")
    result["contact_and_stability_verified"] = not reasons
    return result
