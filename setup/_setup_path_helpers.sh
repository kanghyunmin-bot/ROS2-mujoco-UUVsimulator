#!/usr/bin/env bash

preferred_setup_script_names() {
  local user_shell
  user_shell="$(basename "${SHELL:-}")"
  if [[ "$user_shell" == "zsh" ]]; then
    printf '%s\n' \
      setup.zsh local_setup.zsh \
      setup.bash local_setup.bash \
      setup.sh local_setup.sh
    return 0
  fi

  printf '%s\n' \
    setup.bash local_setup.bash \
    setup.zsh local_setup.zsh \
    setup.sh local_setup.sh
}

resolve_setup_script_in_dir() {
  local dir="$1"
  local script_name
  [[ -d "$dir" ]] || return 1

  while IFS= read -r script_name; do
    if [[ -f "${dir}/${script_name}" ]]; then
      printf '%s\n' "${dir}/${script_name}"
      return 0
    fi
  done < <(preferred_setup_script_names)

  return 1
}

resolve_ros_workspace_dir() {
  local workspace_dir="$1"
  if [[ -n "${ROS_WORKSPACE_DIR:-}" ]]; then
    printf '%s\n' "${ROS_WORKSPACE_DIR}"
    return 0
  fi
  if [[ -d "${workspace_dir}/rospkg" ]]; then
    printf '%s\n' "${workspace_dir}/rospkg"
    return 0
  fi
  printf '%s\n' "${workspace_dir}"
}

resolve_kmu26_auv_dir() {
  local workspace_dir="$1"
  local ros_workspace_dir="$2"
  if [[ -n "${KMU26_AUV_DIR:-}" ]]; then
    printf '%s\n' "${KMU26_AUV_DIR}"
    return 0
  fi
  if [[ -d "${ros_workspace_dir}/kmu26_auv" ]]; then
    printf '%s\n' "${ros_workspace_dir}/kmu26_auv"
    return 0
  fi
  if [[ -d "${ros_workspace_dir}/src/kmu26_auv" ]]; then
    printf '%s\n' "${ros_workspace_dir}/src/kmu26_auv"
    return 0
  fi
  if [[ -d "${workspace_dir}/kmu26_auv" ]]; then
    printf '%s\n' "${workspace_dir}/kmu26_auv"
    return 0
  fi
  printf '%s\n' "${ros_workspace_dir}/src/kmu26_auv"
}

resolve_ros_install_setup() {
  local workspace_dir="$1"
  local ros_workspace_dir="$2"
  local setup_script
  if [[ -n "${ROS_INSTALL_SETUP:-}" && -f "${ROS_INSTALL_SETUP}" ]]; then
    printf '%s\n' "${ROS_INSTALL_SETUP}"
    return 0
  fi
  if setup_script="$(resolve_setup_script_in_dir "${ros_workspace_dir}/install")"; then
    printf '%s\n' "${setup_script}"
    return 0
  fi
  if setup_script="$(resolve_setup_script_in_dir "${workspace_dir}/install")"; then
    printf '%s\n' "${setup_script}"
    return 0
  fi
  printf '%s\n' "${ros_workspace_dir}/install/setup.bash"
}

resolve_ros_env_setup() {
  local ros_distro="${1:-${ROS_DISTRO:-humble}}"
  local setup_script

  if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
    printf '%s\n' "${ROS_ENV_SETUP}"
    return 0
  fi

  for base_dir in \
    "/opt/ros/${ros_distro}" \
    "/usr/local/ros/${ros_distro}" \
    "/opt/homebrew/opt/ros/${ros_distro}" \
    "/usr/local/opt/ros/${ros_distro}"
  do
    if setup_script="$(resolve_setup_script_in_dir "${base_dir}")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
  done

  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    if setup_script="$(resolve_setup_script_in_dir "${CONDA_PREFIX}")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
  fi

  return 1
}

resolve_qgc_app() {
  local os_name
  local workspace_dir

  if [[ -n "${QGC_APP:-}" ]]; then
    printf '%s\n' "${QGC_APP}"
    return 0
  fi

  workspace_dir="${WORKSPACE_DIR:-$PWD}"

  for candidate in \
    "${workspace_dir}/dist/ubuntu22.04/QGroundControl.AppImage" \
    "${workspace_dir}/dist/ubuntu22.04/QGroundControl-x86_64.AppImage" \
    "${workspace_dir}/QGroundControl.AppImage" \
    "${workspace_dir}/QGroundControl-x86_64.AppImage" \
    "$HOME/Downloads/QGroundControl.AppImage" \
    "$HOME/Downloads/QGroundControl-x86_64.AppImage" \
    "$HOME/Applications/QGroundControl.AppImage" \
    "$HOME/bin/QGroundControl.AppImage" \
    "/opt/QGroundControl/QGroundControl.AppImage" \
    "/usr/local/bin/QGroundControl.AppImage" \
    "${workspace_dir}/QGroundControl.app" \
    "/Applications/QGroundControl.app" \
    "$HOME/Applications/QGroundControl.app"
  do
    if [[ -n "$candidate" && -e "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  if command -v QGroundControl >/dev/null 2>&1; then
    command -v QGroundControl
    return 0
  fi
  if command -v qgroundcontrol >/dev/null 2>&1; then
    command -v qgroundcontrol
    return 0
  fi

  os_name="$(uname -s)"
  if [[ "$os_name" == "Darwin" ]]; then
    printf '%s\n' "/Applications/QGroundControl.app"
  else
    printf '%s\n' "${workspace_dir}/QGroundControl.AppImage"
  fi
}
