"""Compatibility facade for development OS system checks."""

from __future__ import annotations

from dev_os_compat_docker import check_docker, check_docker_host_contract
from dev_os_compat_ros import check_ros2_env
from dev_os_compat_sitl_paths import check_sitl_paths
from dev_os_compat_ubuntu import check_ubuntu_migration_contract


__all__ = [
    "check_docker",
    "check_docker_host_contract",
    "check_ros2_env",
    "check_sitl_paths",
    "check_ubuntu_migration_contract",
]
