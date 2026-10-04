# Robot-delivered messages

The operator can approve typed text or an edited speech transcript for one enrolled person, preview the generated audio through the Jetson speaker, then start a supervised one-room search. Search-only remains a separate action. Playback begins only after a supported arrival report, fresh unique identity for the selected profile, a validated measured person distance in the configured standoff interval, stopped left/right motor feedback, and a stopped camera head. An estimated arrival, stale or ambiguous identity, changed profile/message revision, unavailable speech service, or uncertain feedback leaves the robot silent.

`played` means the ALSA playback process completed. The robot has no recipient acknowledgement and does not claim that the person heard or understood the message. Recorded operator audio is forwarded by the Mac service to the configured transcription provider; recordings and synthesized MP3s are held only in memory by the services. Fish Audio is used to synthesize the reviewed text.

## Mac speech service

The narrow FastAPI service binds only to `127.0.0.1:8766`. It reuses the existing communication backend's configured transcription provider and Fish Audio settings. Its LaunchAgent is `robot/mac/com.echora.voice-backend.plist`; the SSH LaunchAgent forwards Jetson loopback port `18766` to Mac loopback port `8766`.

The Mac user needs the existing `communication/.venv`, its Python dependencies, and configured provider settings in `communication/.env` or the repository `.env`. The Mac must stay awake and connected to the Jetson Wi-Fi during transcription and synthesis. To install/update the agents in the current user's LaunchAgents directory:

```sh
mkdir -p "$HOME/Library/LaunchAgents" "$HOME/Library/Logs/Echora"
cp robot/mac/com.echora.voice-backend.plist "$HOME/Library/LaunchAgents/"
cp robot/mac/com.echora.jetson-backend-tunnel.plist "$HOME/Library/LaunchAgents/"
launchctl bootstrap "gui/$(id -u)" "$HOME/Library/LaunchAgents/com.echora.voice-backend.plist"
launchctl bootstrap "gui/$(id -u)" "$HOME/Library/LaunchAgents/com.echora.jetson-backend-tunnel.plist"
```

For an already loaded agent, use `launchctl bootout` for that label before bootstrapping its updated plist. Check `/health` locally at `http://127.0.0.1:8766/health`; the tunnel and service are not activated by repository changes alone.

## Jetson audio and delivery

Deploy `robot/jetson/perception/enrollment_console.py` and its sibling `speech_delivery.py` together into the runtime directory used by `echora-enrollment-console.service`; the console imports the playback/client module at startup. Install the runtime audio tools on the Jetson (`ffmpeg` and `alsa-utils`). Connect and power the speaker, inspect available ALSA outputs with `aplay -L`, and set `--speech-device` in the enrollment-console service to the desired device. The unit currently uses ALSA's `default` device and the loopback tunnel URL. Restart the service only when no enrollment or mission is active. Play an approved message with **Preview on robot** before moving the robot; successful preview verifies that the selected output can play that MP3.

Before a supervised delivery trial, verify the selected room is clear and measure a conservative stopping gap using the actual robot, payload, surface, battery state, speed, and camera/front offset. Configure the person-range calibration so its validated interval is conservative and repeatable. The current approximate camera-range mode explicitly reports an unverified physical gap; it cannot authorize speech. Read [family clothing memory range and approach notes](FAMILY_CLOTHING_MEMORY.md) before enabling a physical approach trial.

For each trial, select the enrolled person, edit and approve the exact message, preview it, position the robot at the marked cable-neutral heading, confirm the cable is clear, then choose **Find & deliver** while supervising. **Find person** never speaks. **STOP ROBOT** interrupts both mission motion and active audio. `delivery_failed` explains why speech was withheld; `played` reports playback completion only.

Voice-command starts, open conversation, and recipient acknowledgement are outside this feature.
