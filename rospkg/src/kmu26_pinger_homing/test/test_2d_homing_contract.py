from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_standalone_package_has_no_hydrophone_repos_dependency():
    text = (ROOT / "package.xml").read_text()
    assert "hydrophone.repos" not in text
    assert "kmu26_auv_hydrophone" not in text


def test_alt_hold_and_private_rc_defaults():
    text = (ROOT / "src" / "pinger_homing_2d_controller.cpp").read_text()
    assert 'declare_parameter<std::string>("mode", "ALT_HOLD")' in text
    assert '"/control/pinger/rc_override"' in text
    assert 'msg.channels[2] = 1500' in text
    assert 'declare_parameter<std::string>("imu_topic", "/mavros/imu/data")' in text
    assert 'declare_parameter<bool>("invert_rc_yaw", true)' in text
    assert 'pwm(invert_rc_yaw_ ? -yaw : yaw, rc_span_)' in text
    assert '"mavros_imu_aligned"' in text


def test_frequency_selection_contract():
    selector = (ROOT / "src" / "pinger_frequency_selector.cpp").read_text()
    assert "monitor_s_" in selector
    assert "Eigen::FFT<double>" in selector
    assert "FFT scan" in selector
    assert "min_snr_db_" in selector
    assert "min_peak_prominence_db_" in selector
    assert "minimum_candidate_hits_" in selector
    assert "relative_to_top_snr_db_" in selector
    assert "average_power_" in selector
    assert "spectral_peaks" in selector
    assert 'declare_parameter<double>("monitor_s", 10.0)' in selector
    assert 'declare_parameter<double>("min_frequency_hz", 19000.0)' in selector
    assert 'declare_parameter<double>("max_frequency_hz", 22000.0)' in selector
    assert 'declare_parameter<bool>("combine_channels", true)' in selector
    assert "power[bin] / channel_floor" in selector
    assert "persistent_min_ratio_" in selector
    assert '"/pinger_homing/manual_selection"' in selector


def test_manual_selection_wrapper_waits_for_ros_discovery():
    wrapper = (ROOT / "scripts" / "start_pinger_homing_test_tank.sh").read_text()
    assert "wait_for_candidate_topic" in wrapper
    assert 'grep -Fxq "/pinger_homing/frequency_candidates"' in wrapper
    assert 'ros2 topic echo --once /pinger_homing/frequency_candidates' in wrapper
    assert "manual_selection" in wrapper
    assert "setsid ros2 launch" in wrapper
    assert 'kill -INT -- "-${launch_pid}"' in wrapper


def test_xy_only_controller():
    text = (ROOT / "src" / "pinger_homing_2d_controller.cpp").read_text()
    assert "direction_world_->y()" in text
    assert "direction.vector.z = 0.0" in text
    assert "depth" not in text.lower()
    assert '"/homing/direction"' in text
    assert '"/pinger_homing/direction_body"' in text
    assert "records_observation()" in text
    assert "update_direction_from_feedback(false)" in text
    assert "approach_dither_command_" in text
    assert '\\"feedback_updates\\"' in text


if __name__ == "__main__":
    # CTest invokes this file directly rather than through pytest.
    test_standalone_package_has_no_hydrophone_repos_dependency()
    test_alt_hold_and_private_rc_defaults()
    test_frequency_selection_contract()
    test_manual_selection_wrapper_waits_for_ros_discovery()
    test_xy_only_controller()
