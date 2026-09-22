"""Validate and serve a KMU26 CAP-free checkpoint using the training transform."""
import argparse
import importlib.util
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("u0_root", type=Path)
parser.add_argument("checkpoint", type=Path)
parser.add_argument("--check_only", action="store_true")
parser.add_argument("--port", type=int, default=8000)
parser.add_argument("--residual_checkpoint", type=Path)
args = parser.parse_args()

package_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(package_root))
sys.path.insert(0, str(package_root.parent / "kmu26_auv_vla_policy"))
from kmu26_auv_vla_data_collector.deployment_config import validate_deployment_contract
from kmu26_auv_vla_policy.kmu26_contract import validate_checkpoint

validate_checkpoint(str(args.checkpoint))
contract = validate_deployment_contract(args.checkpoint)
print("Checkpoint matches CAP-free 23-state / 4-action / PWM-span-400 contract")
residual = None
if args.residual_checkpoint is not None:
    repository = Path(__file__).resolve().parents[4]
    sys.path.insert(0, str(repository / 'tools/rl_training'))
    from deployment import ResidualDeployment
    residual = ResidualDeployment(args.residual_checkpoint, args.checkpoint, repository)
    print(f"Loaded PPO residual: {residual.metadata}", flush=True)
if not args.check_only:
    sys.path.insert(0, str(args.u0_root.resolve()))
    spec = importlib.util.spec_from_file_location(
        "kmu26_inference", args.u0_root / "scripts/inference_service_u0.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    config = module.ArgsConfig(
        model_path=str(args.checkpoint), data_config=contract["data_config"],
        port=args.port, host="127.0.0.1", http_server=True, server=True, client=False,
    )
    import numpy as np
    from kmu26_auv_vla_policy.kmu26_contract import motion_chunk

    class BoundedTransferPolicy(module.Gr00tPolicy):
        """Project generated continuous actions onto the declared RC action space."""

        def get_action(self, observations):
            # The upstream transform may modify the observation mapping in place.
            residual_observation = ({k: np.array(v, copy=True) for k, v in observations.items()
                                     if k.startswith('state.')} if residual else None)
            actions, target = super().get_action(observations)
            if target is not None:
                raise ValueError("CAP-free checkpoint unexpectedly returned a target")
            raw = np.asarray(actions["action.motion"])
            if not np.isfinite(raw).all():
                raise ValueError("Model generated nonfinite motion")
            clipped = np.clip(raw, -1.0, 1.0)
            if residual is not None:
                clipped = residual.apply(residual_observation, clipped)
            # Keep adapter validation strict; constrain at the policy boundary.
            motion_chunk({"action.motion": clipped})
            if np.any(raw != clipped):
                count = getattr(self, "_bounded_requests", 0) + 1
                self._bounded_requests = count
                if count == 1 or count % 100 == 0:
                    print(f"KMU26 action-space projection: requests={count}, raw_range=({raw.min():.4f},{raw.max():.4f})", flush=True)
            return {**actions, "action.motion": clipped}, None

    module.Gr00tPolicy = BoundedTransferPolicy
    module.main(config)
