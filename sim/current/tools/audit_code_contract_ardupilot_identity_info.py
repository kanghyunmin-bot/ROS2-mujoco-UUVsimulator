"""ArduPilot git identity metadata collection."""

from __future__ import annotations

from dataclasses import dataclass

from audit_code_contract_common import ARDUPILOT_ROOT, REPO_ROOT, git_output


@dataclass(frozen=True)
class ArduPilotIdentity:
    describe: str
    status_short: str
    inner_commit: str
    root_gitlink_commit: str

    def metadata(self) -> dict[str, str]:
        return {
            "ardupilot_describe": self.describe,
            "ardupilot_status_short": self.status_short,
            "ardupilot_inner_commit": self.inner_commit,
            "ardupilot_top_level_gitlink": self.root_gitlink_commit,
        }


def read_ardupilot_identity() -> ArduPilotIdentity:
    return ArduPilotIdentity(
        describe=git_output(["describe", "--tags", "--always", "--dirty"], ARDUPILOT_ROOT),
        status_short=git_output(["status", "--short"], ARDUPILOT_ROOT),
        inner_commit=git_output(["rev-parse", "HEAD"], ARDUPILOT_ROOT),
        root_gitlink_commit=root_gitlink_commit(),
    )


def root_gitlink_commit() -> str:
    root_gitlink_line = git_output(["ls-tree", "HEAD", "ardupilot"], REPO_ROOT)
    parts = root_gitlink_line.split()
    if len(parts) >= 3 and parts[1] == "commit":
        return parts[2]
    return ""


__all__ = ["ArduPilotIdentity", "read_ardupilot_identity", "root_gitlink_commit"]
