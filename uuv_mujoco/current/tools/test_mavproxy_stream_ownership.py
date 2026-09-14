"""Verify launcher defaults do not periodically overwrite per-message rates."""

import ast
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
from types import SimpleNamespace
import unittest


CURRENT = Path(__file__).resolve().parents[1]


def _effective_args(*, explicit: str = "", cli_override: bool = False) -> list[str]:
    source = (CURRENT / "start_ardusub_sitl_mj311.sh").read_text()
    block = re.search(
        r'if \[\[ "\$USER_SET_MAVPROXY_ARGS" -eq 0 \]\]; then\n'
        r'[\s\S]*?\nfi', source,
    ).group(0)
    environment = dict(os.environ, SITL_MAVPROXY_ARGS=explicit,
                       USER_SET_MAVPROXY_ARGS=str(int(cli_override)))
    result = subprocess.check_output(
        ["bash", "-c", block + '\nprintf "%s" "$SITL_MAVPROXY_ARGS"'],
        env=environment, text=True,
    )
    return shlex.split(result)


class MavproxyStreamOwnershipTest(unittest.TestCase):
    def test_default_stops_periodic_all_stream_requests(self):
        spec = importlib.util.find_spec("MAVProxy")
        if spec is None:
            self.skipTest("MAVProxy is available in the simulator container")
        source = Path(next(iter(spec.submodule_search_locations))) / "mavproxy.py"
        tree = ast.parse(source.read_text())
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "set_stream_rates")
        args = _effective_args()
        rate = 4  # Installed MAVProxy CLI default when --streamrate is absent.
        for index, item in enumerate(args):
            if item.startswith("--streamrate="):
                rate = int(item.split("=", 1)[1])
            elif item == "--streamrate":
                rate = int(args[index + 1])
        sent = []
        namespace = {
            "msg_period": SimpleNamespace(trigger=lambda: True),
            "mavutil": SimpleNamespace(mavlink=SimpleNamespace(MAV_DATA_STREAM_ALL=0)),
            "mpstate": SimpleNamespace(
                status=SimpleNamespace(last_streamrate1=-1, last_streamrate2=-1),
                settings=SimpleNamespace(streamrate=rate, streamrate2=rate),
                mav_master=[SimpleNamespace(linknum=0, mav=SimpleNamespace(
                    request_data_stream_send=lambda *values: sent.append(values)))],
                vehicle_link_map={0: {(1, 1)}},
            ),
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
        for _ in range(3):
            namespace["set_stream_rates"]()
        self.assertEqual(sent, [], "periodic group requests overwrite accepted ATTITUDE intervals")

    def test_explicit_env_and_cli_stream_rates_are_preserved(self):
        explicit = "--non-interactive --streamrate=7"
        self.assertEqual(_effective_args(explicit=explicit), shlex.split(explicit))
        self.assertEqual(_effective_args(explicit=explicit, cli_override=True), shlex.split(explicit))


if __name__ == "__main__":
    unittest.main()
