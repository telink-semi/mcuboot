# tl_mcuboot

Telink maintained fork of [MCUboot](https://github.com/mcu-tools/mcuboot), the
secure bootloader for 32-bit MCUs. It is the bootloader component of the
Telink Zephyr SDK ([tl_zephyr](https://github.com/telink-semi/tl_zephyr)) and
is pulled in through the SDK west manifests.

This repository was previously published as `telink-semi/mcuboot` and has been
renamed to `tl_mcuboot`, so that the Telink maintained components of the SDK
are easy to identify.

## Upstream base

This branch is based on upstream MCUboot `main` at commit
[346f7374ff4](https://github.com/mcu-tools/mcuboot/commit/346f7374ff4467e40b5594658f8ac67a5e9813c9)
(*boot: bootutil: Use BOOT_IMG_AREA to get boot_loader_state area*, 2025-02-12).
The original upstream README is preserved in
[README.upstream.md](README.upstream.md).

## Telink modifications

On top of the upstream base, this branch carries the following Telink
modifications:

- **Validate the primary slot image only on the first boot**
  (`boot: bootutil: validate once`): migrates `BOOT_VALIDATE_SLOT0_ONCE`
  (removing its `SINGLE_APPLICATION_SLOT` restriction) so that the image in
  the primary slot is validated on the first boot after an upgrade instead of
  on every boot, reducing boot time.
- **Automatic watchdog on TLSR9268J**
  (`telink: tlsr9268j: add automatic watchdog`): skips MCUboot's default
  watchdog setup and feeding when `CONFIG_WATCHDOG_AUTO` is enabled, leaving
  the watchdog to the automatic watchdog handling of the SoC.
