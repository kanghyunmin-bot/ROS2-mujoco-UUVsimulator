"""Installer/source contracts that prevent known fresh-PC failures."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_launchers_parse_and_help_without_hardware():
    for path in [ROOT/'release.sh',*list((ROOT/'release').glob('*.sh')),ROOT/'docker/ubuntu-dev/dev.sh']:
        subprocess.run(['bash','-n',str(path)],check=True)
    result=subprocess.run(['bash',str(ROOT/'release.sh'),'--help'],capture_output=True,text=True,check=True)
    assert 'install' in result.stdout and 'Ubuntu 22.04/24.04' in result.stdout

def test_vla_execution_does_not_depend_on_private_experiments():
    for name in ['server.py','warmup.py','runtime_config.py','run_rollout.py']:
        source=(ROOT/'tools/vla_gui'/name).read_text()
        assert '/home/khm' not in source
        assert 'outputs/vla-first-training' not in source
        assert 'outputs/vla-transfer-audit' not in source

def test_docker_context_includes_policy_requirements():
    ignore=(ROOT/'.dockerignore').read_text()
    assert '!rospkg/src/kmu26_auv_vla_policy/requirements-ros.txt' in ignore

def test_release_clock_matches_pinned_source():
    import importlib.util
    spec=importlib.util.spec_from_file_location('clock_patch',ROOT/'setup/patch_ardusub_json_clock.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.patch_json_clock(module.ORIGINAL)
    assert result==module.ROUNDED and module.patch_json_clock(result)==result
