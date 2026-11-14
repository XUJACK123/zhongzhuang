# chassis_ref_new

This is STM32H723 DM-MC02 reinforcement-learning wheel-leg firmware. `Makefile`
is the authoritative build. Legacy Keil/CubeMX/VS Code configurations are
archived under `archive/legacy_configs_*.zip`; their old active entries were
removed. Do not regenerate over runtime initialization without review.

- Six motors share CAN2 PB5/PB6, 1Mbps. Left DM1 hip/DM2 knee; right DM4
  hip/DM3 knee. M3508 left ID2/right ID3. Group commands DM 0x3FE little
  endian, DJI 0x200 big endian. Feedback uses separate 0x30x/0x20x namespaces.
- Policy order is RIGHT hip/knee/wheel then LEFT hip/knee/wheel.
- Read `user/robot_config.h` and `docs/DEPLOYMENT.md` before changing mappings,
  calibration, sensor frames, time steps or output authorization flags.
- `user/algorithm/model` is generated from the actual sample checkpoint.
  Regenerate with `make export-policy`; do not hand-edit weights or manifests.
- Keep the 31D observation contract separate from other 25D/history policies.
- BMI088 freshness uses the real gyro DRDY rising edge on PE12; close both
  PC2/PC3 analog switches. Do not replace freshness with a fabricated loop tick.
  Physical pose data and unresolved yaw-scale limits are recorded in
  `docs/BOARD_BRINGUP_20261113.md`; do not treat raw encoder values as zero marks.
- The single active firmware is `build/rl/wheel_leg_rl.elf`. `make flash-check`
  only checks the intended command; `make flash` explicitly programs and runs
  through the known working Horco DAP/OpenOCD configuration. `make monitor`
  only reads current RL telemetry. Read `docs/COMMISSIONING.md` for calibration.
- Run `make test` and `make` after control changes. Host tests and successful
  cross-build do not establish board timing, bench behavior or balance.
- Do not flash, enable, move motors, or mark commissioning flags verified as
  part of source-only work. Hardware work requires its own user instruction.
- No CAN parameter writes, auto zeroing, arbitrary ID scanning or pulse motion
  belongs in this control application. Original scan code is in `archive`.
