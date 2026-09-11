from kmu26_auv_vla_data_collector.contract import ACTION_NAMES, STATE_NAMES
from kmu26_auv_vla_data_collector.export_lerobot import _modality


def test_exported_modality_matches_model_contract():
    modality = _modality()

    assert modality["state"]["validity"]["end"] == len(STATE_NAMES)
    assert modality["action"]["motion"]["end"] == len(ACTION_NAMES)
    assert modality["video"]["ego"]["original_key"] == "observation.images.ego"
    assert (
        modality["video"]["buoy_release"]["original_key"]
        == "observation.images.buoy_release"
    )
