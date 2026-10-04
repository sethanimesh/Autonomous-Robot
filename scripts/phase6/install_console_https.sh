#!/usr/bin/env bash
# Run on the Jetson with sudo. Adds LAN HTTPS without changing port 8080.
set -euo pipefail
if [[ $(id -u) != 0 ]]; then
  echo 'Run this installer with sudo on the Jetson.' >&2
  exit 1
fi
console_runtime=/home/animesh/echora
console_address=192.168.1.48
if ! command -v caddy >/dev/null; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y caddy
  # This is a fresh package installation; use our dedicated service instead.
  systemctl disable --now caddy
fi
console_caddy=$(command -v caddy)
console_backup="$console_runtime/backups/phone-https-$(date +%Y%m%d-%H%M%S)"
install -d -m 0755 /etc/caddy "$console_runtime/phone_https" "$console_backup"
install -d -o caddy -g caddy -m 0700 /var/lib/echora-https
for console_file in /etc/caddy/echora.Caddyfile /etc/systemd/system/echora-https.service; do
  if [[ -f "$console_file" ]]; then cp -p "$console_file" "$console_backup/"; fi
done
cat > /etc/caddy/echora.Caddyfile <<'CADDY'
{
    admin off
    auto_https disable_redirects
    skip_install_trust
}
https://192.168.1.48 {
    bind 192.168.1.48
    tls internal
    reverse_proxy 127.0.0.1:8080 {
        flush_interval 100ms
    }
}
CADDY
cat > /etc/systemd/system/echora-https.service <<UNIT
[Unit]
Description=Echora private home Wi-Fi HTTPS console
After=network-online.target echora-enrollment-console.service
Wants=network-online.target

[Service]
User=caddy
Group=caddy
Environment=XDG_DATA_HOME=/var/lib/echora-https
Environment=XDG_CONFIG_HOME=/var/lib/echora-https/config
ExecStart=$console_caddy run --config /etc/caddy/echora.Caddyfile --adapter caddyfile
Restart=on-failure
RestartSec=3
AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/var/lib/echora-https
PrivateTmp=true

[Install]
WantedBy=multi-user.target
UNIT
sudo -u caddy env XDG_DATA_HOME=/var/lib/echora-https XDG_CONFIG_HOME=/var/lib/echora-https/config \
  "$console_caddy" validate --config /etc/caddy/echora.Caddyfile --adapter caddyfile
systemctl daemon-reload
systemctl enable echora-https.service
systemctl restart echora-https.service
console_root=/var/lib/echora-https/caddy/pki/authorities/local/root.crt
for ((console_attempt=0; console_attempt<20; console_attempt++)); do
  if [[ -f "$console_root" ]]; then break; fi
  sleep 0.5
done
# Export only the public trust certificate. The private CA key stays with Caddy.
install -m 0644 "$console_root" "$console_runtime/phone_https/echora-local-ca.crt"
curl --fail --silent --show-error --max-time 10 --cacert "$console_root" "https://$console_address/api/status" >/dev/null
openssl x509 -in "$console_root" -noout -subject -fingerprint -sha256
echo "HTTPS is ready: https://$console_address/"
echo "Phone setup: http://$console_address:8080/phone-setup"
echo "Configuration backup: $console_backup"
