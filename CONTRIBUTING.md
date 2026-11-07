# Contributing to CULFW32

Thank you for considering a contribution.

## License of contributions

CULFW32 is licensed under the GNU General Public License, version 3 or (at your
option) any later version. Contributions are accepted under the same license
(inbound = outbound). There is no contributor license agreement.

## Developer Certificate of Origin

Every commit must carry a `Signed-off-by:` line with your real name
(`git commit -s`). With it you certify the Developer Certificate of Origin 1.1
(<https://developercertificate.org/>) for that contribution.

## Relationship to q-culfw

Dirk Tostmann also publishes a separate, closed-source firmware for the same
hardware, q-culfw. Code that Dirk Tostmann wrote himself may appear in both
firmwares. Code contributed by others stays in CULFW32 under the GPL. If we would
like to use a particular contributed patch in q-culfw as well, we ask its author
for a separate written permission for that patch. You are free to decline; it has
no effect on whether your contribution is accepted into CULFW32.

## Practical notes

- One topic per pull request; Conventional Commit titles
  (`fix(slowrf): ...`, `feat(web): ...`).
- Keep the license header and origin notices of every file intact. Code taken
  from another project needs its original copyright lines and a note what was
  changed.

## Frequency calibration (FSCTRL0)

The CC1101 frequency offset belongs to the board, not to a protocol. The driver
(`cc1101_driver`) holds the value: the stored `?rf.freqoff` override, otherwise
the factory value from eFuse, otherwise 0. It writes the value over a table's
FSCTRL0 in `write_reg()` and restores it after every `SRES`.

- A protocol class does not calibrate. Its register table sets FSCTRL0 (`0x0C`)
  to `0x00` or leaves it out, and its nominal carrier goes into FREQ2..0.
- `write_reg_raw()` bypasses the driver's value. It is meant for manual register
  access (`Cw`) and measurements, not for protocol code.
- The one intended exception is the SlowRF EEPROM byte at address `0x0E` (`W0E`):
  a non-zero value is used as FSCTRL0 as written, `0x00` means the calibration
  applies. This is not implemented yet; as of 0.2.9 a calibrated stick ignores
  `W0E`.
