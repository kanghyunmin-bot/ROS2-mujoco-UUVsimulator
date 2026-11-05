#!/usr/bin/env bash
set -euo pipefail

DISPLAY="${DISPLAY:-:99}"
SCREEN_RESOLUTION="${SCREEN_RESOLUTION:-1920x1080x24}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

dnum="${DISPLAY#:}"

# X lock/socket 정리 (display 번호 기반)
rm -f "/tmp/.X${dnum}-lock" "/tmp/.X11-unix/X${dnum}" || true

# Xvfb
Xvfb "$DISPLAY" -screen 0 "$SCREEN_RESOLUTION" -ac -nolisten tcp -noreset &
sleep 0.5

# WM + 터미널 (dev 계정이 있으면 dev로 띄우고, 없으면 그냥 띄움)
if id dev >/dev/null 2>&1; then
  su -s /bin/bash dev -c "fluxbox >/tmp/fluxbox.log 2>&1 &" || true
  su -s /bin/bash dev -c "xterm -geometry 120x40+20+20 >/tmp/xterm.log 2>&1 &" || true
else
  fluxbox >/tmp/fluxbox.log 2>&1 &
  xterm -geometry 120x40+20+20 >/tmp/xterm.log 2>&1 &
fi

# VNC
x11vnc -display "$DISPLAY" -rfbport "$VNC_PORT" -forever -shared -nopw -listen 0.0.0.0 >/tmp/x11vnc.log 2>&1 &

# noVNC (PID 1)
exec websockify --web=/usr/share/novnc/ "$NOVNC_PORT" "localhost:$VNC_PORT"
