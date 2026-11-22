#!/usr/bin/env bash
# Provision a separate desktop; retain the local GNOME display on :1.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for required in tigervncserver tigervncpasswd startxfce4 openssl; do
  command -v "$required" >/dev/null || {
    echo "Missing $required; run install_nuc_host.sh first." >&2
    exit 2
  }
done
if [[ -S /tmp/.X11-unix/X2 ]] &&
   ! systemctl --user is-active --quiet nyush-sentry-vnc.service; then
  echo 'Display :2 is already occupied by another session.' >&2
  exit 1
fi
install -d -m 0700 "$HOME/.vnc" "$HOME/.config/nyush-sentry"
install -d "$HOME/.config/systemd/user"
if [[ ! -e "$HOME/.vnc/passwd" ]]; then
  # VNC uses eight password characters. Keep both files local and private.
  (
    umask 077
    openssl rand -base64 6 | tr -d '\n' > "$HOME/.config/nyush-sentry/vnc-password"
    tigervncpasswd -f < "$HOME/.config/nyush-sentry/vnc-password" > "$HOME/.vnc/passwd"
  )
fi
restart_required=false
if systemctl --user is-active --quiet nyush-sentry-vnc.service &&
   { ! cmp -s "$script_dir/sentry-xstartup" "$HOME/.vnc/sentry-xstartup" ||
     ! cmp -s "$script_dir/nyush-sentry-vnc.service" \
       "$HOME/.config/systemd/user/nyush-sentry-vnc.service"; }; then
  restart_required=true
fi
install -m 0755 "$script_dir/sentry-xstartup" "$HOME/.vnc/sentry-xstartup"
install -m 0644 "$script_dir/nyush-sentry-vnc.service" \
  "$HOME/.config/systemd/user/nyush-sentry-vnc.service"
systemctl --user daemon-reload
systemctl --user enable --now nyush-sentry-vnc.service
if "$restart_required"; then
  systemctl --user restart nyush-sentry-vnc.service
fi
echo 'VNC is bound to localhost:5902. Use SSH port forwarding to connect.'
echo 'A generated password, if needed, is stored privately at ~/.config/nyush-sentry/vnc-password.'
