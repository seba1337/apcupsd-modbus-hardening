# apcupsd — hardened APC MODBUS-over-USB driver

This is a fork of [apcupsd](http://www.apcupsd.org/) 3.14.14 with a hardened
`UPSTYPE modbus` / `UPSCABLE usb` driver (`src/drivers/modbus/`). The base
driver is Adam Kropelin's 2013 MODBUS implementation (see file headers,
GPLv2); this fork adds a series of fixes for USB/HID-level communication
issues that showed up running it against real hardware over time.

## Hardware this was developed/validated against

- **Model:** APC Smart-UPS 3000
- **Firmware:** UPS 18.0 / 00.5
- **Connection:** USB (HID), not RS-232

Nothing in the fixes is model-specific — they live in the shared transport
code (`ModbusUsbComm.cpp` / `ModbusComm.cpp` / `modbus.cpp`) used by any APC
UPS configured with `UPSTYPE modbus`, not just this unit. The specific
desync symptom described below may be more or less likely depending on the
UPS's USB-HID gateway firmware revision, but the fixes cost nothing on units
that never hit it.

## Configuration (`/etc/apcupsd/apcupsd.conf`)

No physical/firmware settings on the UPS itself needed to change — this is
purely an `apcupsd.conf` + driver-code matter. The relevant lines:

```
UPSCABLE usb
UPSTYPE modbus
DEVICE
```

`DEVICE` is left blank; the driver auto-detects the APC USB HID device.

## What's fixed vs. upstream

1. **Batched measurement reads.** The ~10 live measurement registers
   (battery %, battery voltage, temperature, load %, output current/voltage,
   frequency, input status/voltage) were found to intermittently
   cross-contaminate each other when polled as separate back-to-back MODBUS
   transactions — almost certainly racing the UPS's own internal
   measurement refresh cycle. They're now fetched as three contiguous
   blocks per cycle instead.

2. **Stale-duplicate detection.** The UPS can asynchronously re-emit an old
   response some time after the original exchange completed. If it happens
   to match the size of a *different*, later request, it can look like a
   valid response to that request. `ModbusComm::SendAndWait` keeps a short
   history of recently-accepted responses and discards anything that's a
   byte-for-byte match, since two distinct registers legitimately holding
   the same value at the same instant is rare.

3. **No same-size collision risk from decomposed reads.** When a batched
   block fetch fails, the driver does *not* fall back to reading its
   individual registers one at a time — many are the same size, and that's
   exactly the pattern that produces the stale-duplicate collisions in (2).
   A failed block is skipped for the cycle and retried next time.

4. **Tolerant capability probing.** `get_capabilities()` runs immediately
   after `Open()` — the noisiest point in the connection, most likely to
   still be settling from earlier USB traffic. A single failed probe isn't
   reliable evidence a register is actually unsupported, so each capability
   is retried a few times before being marked unavailable.

5. **Live status polling bypasses the block cache.** `check_state()` needs
   to detect an online/on-battery transition within about a second. It does
   a guaranteed-fresh, single-register live read of `CI_STATUS` rather than
   waiting on the (much larger, slower, cached) measurement-block cycle.

6. **Debounced COMMLOST.** This UPS's USB/MODBUS channel produces brief,
   self-recovering noise bursts at roughly the polling granularity.
   Declaring COMMLOST (and closing/reopening the port) on the very first
   missed poll was firing on noise, not real communication loss. A few
   consecutive failures are now required before COMMLOST is declared.

7. **NIS server no longer starves on slow reads.** A single MODBUS read
   cycle can take anywhere from a few seconds to (with retries) tens of
   seconds. The UPS-state lock is now held only for the brief moment of
   copying an already-decoded value into the shared struct, not for the
   whole read — holding it for the whole read was starving `apcaccess`
   (via the NIS server) for just as long, causing client timeouts even
   though the daemon itself was healthy.

8. **Recovery from a stuck measurement channel.** `check_state()`'s
   single-register live poll can keep succeeding — and resetting its own
   failure counter — even while the much larger batched measurement blocks
   stay desynced indefinitely, since the two use different-sized reads on
   the same wire. `read_volatile_data()` now tracks how long it's been
   since *all three* measurement blocks last fetched cleanly; past 180
   seconds of that (well above the longest self-recovering noise burst
   observed in practice) it forces a `Close()`/`Open()` of the device to
   make the OS and UPS renegotiate the USB HID session, independent of
   whatever `check_state()` is doing.

## Known limitations / possible future work

`get_capabilities()`'s per-CI probe retry (point 4) treats a timeout or
garbled response exactly the same as a genuine MODBUS exception response
(the actual "register not supported" signal). If the channel is noisy for
longer than the retry budget during that one-time startup probe, a field
can be marked permanently unavailable for the life of the connection — with
no way to recover short of restarting apcupsd — even though the channel
goes on to work fine afterward. A cleaner fix would only let a genuine
MODBUS exception permanently disable a capability, and let ordinary
noise/timeouts just skip that probe pass without penalty, consistent with
how the steady-state read path (point 8) already tolerates transient
failures.

## License

GPLv2, same as upstream apcupsd. See `COPYING`. Original MODBUS driver
Copyright (C) 2013 Adam Kropelin.
