# Remaining Code Quality and Safety Issues

Tracked during PR #1 (code quality) and PR #2 (safety hardening) reviews.

## High priority — `tcm_args_download/` has same bugs we fixed in `laser_engine/`

- [ ] `tcm_args_download.ino`: `analyzingHostFrame` and `analyzingTCMFrame` have no buffer overflow guards
- [ ] `tcm_args_download.ino`: `analyzingHostFrame` has same OOB read when `host_protocol_buf_length == 1`
- [ ] `pc-settings.py:240`: `received_loop` crashes on `msg[-1]` when msg is empty
- [ ] `pc-settings.py:210`: `STATE_NAMES`-style subscript has no bounds check
- [ ] `pc-settings.py`: no exception handling in thread loops (USB disconnect kills threads)
- [ ] `pc-settings.py:119,128,135`: per-caller lock pattern not consolidated into `_send_packet`

## Medium priority — `laser_engine/` remaining improvements

- [ ] Buffer overflow: implement "skip to terminator" pattern instead of one-byte drain per loop iteration
- [ ] `tcm_parse_failure_count` not incremented in `ENABLETCM`/`SWITCHTCMSTATUS` inline parse paths
- [ ] Consecutive error circuit breaker in Python thread loops (stop after N repeated errors)
- [ ] `run()` sends redundant queries alongside `query_loop` thread (double query rate to firmware)

## Low priority — cosmetic

- [ ] Mixed tabs/spaces indentation throughout `.ino` file
- [ ] Remove pre-existing commented-out code (`//enableLaser(i)` in `.ino`, stale example calls in Python `__main__`)
