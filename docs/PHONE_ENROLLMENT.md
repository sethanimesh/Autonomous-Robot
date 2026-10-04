# Phone enrollment and profiles

Implemented and validated locally on 2026-09-09. Deployment, phone certificate trust, and real-phone capture remain pending; the operator requested local validation only.

- **Settings → Profiles:** add people, rename them, update enrollment, delete profiles, and manage remembered clothes.
- **Face enrollment:** choose Phone camera or Robot camera before starting. Phone capture defaults to the selfie camera, supports switching cameras, and collects clear front/side views automatically. Existing photo uploads remain available.
- **Find Me:** choose a saved profile and use **Find [name]**. The server validates the displayed profile ID and enrollment revision before starting.

Phone enrollment uses the existing Jetson detection and embedding models. The EV3 and robot camera can be offline. Phone frames are separate from robot observations and never enter navigation. Samples carry their capture source; stale session requests are rejected. Saving an update preserves the profile ID and other people's data, and changes its enrollment revision. Cancellation leaves saved profiles intact.

Camera tracks stop on pause, camera switching, leaving Settings, backgrounding, loss of connection, finish, or cancellation. Resume is explicit. A closed/reloaded page can resume its own session; another page can explicitly cancel an abandoned session before starting a new one. Camera permission denial leaves photo upload available.

## Local validation

Run the Python checks with the project's vision environment, then the isolated browser-media tests:

```sh
/Users/animesh/Library/Caches/Echora/depth-venv/bin/python scripts/phase6/check_build.py
node --test tests/test_phone_enrollment_ui.mjs
bash -n scripts/phase6/install_console_https.sh
```

All 1,023 Python tests and eight browser-media tests passed. Validation covers HTTP/session binding, quality rejection, profile persistence, revision checks, capture lifecycle and existing motor safety. Desktop and 390-pixel phone layouts were inspected in the browser with test profiles. The Caddy configuration was validated using its official binary; no HTTPS server or system trust changes were made locally.

## Later Jetson setup

Back up and replace only the runtime `enrollment_console.py`, then restart `echora-enrollment-console` when no enrollment or mission is active. Other runtime modules and services do not need changes. Copy `scripts/phase6/install_console_https.sh` to the Jetson and run it with sudo.

The installer adds `echora-https.service`, binds HTTPS to `192.168.1.48`, proxies the existing console on port 8080, preserves existing HTTP integrations, and exports only the public certificate at `/phone-ca.crt`. Its configuration backups are under the Jetson runtime's `backups/phone-https-*` directory. The CA private key stays in `/var/lib/echora-https`.

On each phone, open `http://192.168.1.48:8080/phone-setup`, install/trust the downloaded local certificate, and then use `https://192.168.1.48/`. The setup page includes [Apple's certificate-trust steps](https://support.apple.com/102390) and [Android certificate instructions](https://support.google.com/pixelphone/answer/2844832). [Caddy's local HTTPS documentation](https://caddyserver.com/docs/automatic-https) explains the trust requirement.

Keep the EV3 off during enrollment checks. Confirm actual camera capture in Safari/Chrome and stationary robot recognition of a phone-enrolled person after deployment.
