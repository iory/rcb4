# Provenance of the RCB-4mini protocol

Where every fact in `rcb4mini.py` and `framing.py` comes from, and what a host
driving an RCB-4mini through them has to do because of it.

## Kondo RCB-4mini control board + ICS servos — serial gateway

The servos are the same ICS servos as on the ARM-H7. A different board from
the ARM-H7: the H7 firmware grew out of the RCB-4, so the framing, ack,
version and batch move are shared, and the memory is not.

| Fact used in code | Source |
| --- | --- |
| Framing, `ACK=0xFE`, `VERSION=0xFD`, batch move `0x10`/`0x11` are byte-for-byte the H7's | [17], pinned by `test_protocol_framing.py` |
| **Never `0x11`** (a move byte per servo). A host should send only `0x10` and refuse servos with different move bytes | **measured** [19]: a `0x11` packet with a real target (servo 5 at its own position, every move byte 1) was answered with a lone `00` and the board then answered nothing until power-cycled; streaming `0x11` (servo 5 at move byte 2, the rest 1) reset the board (it answered again at 115200). The same targets as `0x10` never failed, incl. a new packet every 20 ms during a 2-frame interpolation with record reads in between (300 packets). The byte order sent was kxreus's `:m-s-mv` [20] `rcb4asm.l`; rcb4 [17] defines the opcode and never sends it |
| From the OFF reference (`0x8000`) the board does not interpolate toward a target; servo ON (`0x7FFF`, which copies the measured pulse into the reference) must come first, in its own packet, for a servo going from released to commanded | **measured** [19]: the record's interpolation width read `target - 0x8000` and the reference stayed `0x8000` |
| One move-byte unit is one frame; with config frame bits 4-5 = `01` that is 15 ms | **measured** [19], servo 5, +10 deg: move byte 20 ramped linearly for ~300 ms, 127 reached 90 % at ~1.8 s; frame-field meaning from [20] `:sysreg` (`00` 10 ms, `01` 15 ms, `10` 20 ms, `11` 25 ms) |
| Move byte 1 streamed at 50 Hz tracks a +-10 deg, 0.5 Hz sine with a 42 ms lag and 0.20 deg rms error after the lag; the measured path is within 0.25 deg rms (0.63 max) of a smooth sine | **measured** [19], 200 cycles, 19.8..20.2 ms period, no missed read, no refusal |
| RAM read: `00 20 00 00 00 addr_lo addr_hi size`; reply `[00, data...]` | [17] `move_ram_to_com_command`; reply **recorded** [19] |
| RAM write: `00 02 addr_lo addr_hi 00 data...`; reply `[00, 06]` | [17] `move_com_to_ram_command`; reply **recorded** [19] |
| A RAM read can carry up to 252 bytes (the length-byte limit), but `plan_record_reads` plans reads of at most **126** (`READ_CHUNK_BYTES`) | **measured** [19]: `$01FC`+242 repeated alone was refused exactly every second time (30 of 60), and `$00BC`+162 alternating with it had the 242 refused every time; every board hang whose last request was logged ended on a read of 162 bytes or more, after which the board answered nothing at any rate until power-cycled. Reads of up to 126 (180 in a row, incl. the same span in 126-byte pieces) drew no refusal and no hang. [17] `_angle_vector` and [20] `:read-servos-three` read 126 at a time |
| Config bit 2 makes the board send a command out of COM when a servo's interpolation ends; bits 4-5 are the output cycle (`00` 10 ms, `01` 15 ms, `10` 20 ms, `11` 25 ms). `flat_config` clears bit 2 and sets the cycle to `01`, so a project's settings do not change the protocol or the move-byte unit | [21] firmware reference p.4-5 (システムレジスタ); `01` is what a board without a project powered up with (`0x8313`), **measured** [19]; a board with a project read `0xc291` (bit 2 clear, cycle `01`) on 2026-10-20 |
| Record `+8..+12` is the interpolation state (speed, end position, steps, width); with `+8` (speed) non-zero the board rewrites `+6` every cycle, so `+8` must be written 0 with every OFF written into a record | [21] firmware reference p.16-17 (補間動作) |
| Record `+14`/`+17` (uint16) name each of two mixings' source data (RAM address, or ICS number + offset), `+16`/`+19` its ratio byte (sign, factor); the board drives `+6` + mixing 1 + mixing 2 every cycle, each source x ratio. A host that wants the servos driven to their reference alone sets the ratio of any mixing with a source to 0 and switches that servo off | [21] firmware reference p.17-18 (ミキシング); offsets also [21] programming manual (`Mixing1AddressOffset` 14 ... `Mixing2RatioAddressOffset` 19) and [20] `:servo-state`. Source `$FFFF` taken as "none": every one of the 35 records on a board with a project read `$FFFF`, ratio `01` (**recorded** 2026-10-20), with servos driven exactly to their reference; as a source it is no RAM address (RAM ends at `$048F`) and no ICS number (bits 8-13 = 63), **inferred** from [21] |
| Config bit 1 runs the ROM program, bit 3 enables vector jumps; `flat_config` clears both | bit names: [17] `ConfigData`, [20] `*rcb4-sysreg-bits*`; [20] `:send-servo-vector-com` clears ROM-run around every servo command, [17] `suspend_motion` clears both. A board without a project powered up with ROM-run set (`0x8313`), once with vector jumps too (`0x831b`), **measured** [19]. Not shown to cause the refusals: they persisted with both cleared |
| The board sometimes **refuses** a request with `[x, 0x15]` (x seen: 0x00, 0x01, 0x02, 0x0b), and answers the same request moments later | **measured** [19]; [18] retries reads for the same reason |
| Firmware reply to `VERSION`: `fd "CB-4 V1.0      090715          " c7` | [17] `check_firmware_version`; **recorded** [19] |
| The board's **replies carry the checksum** (`(len + sum(body)) & 0xFF`), like the requests, so `unframe` raises `ChecksumError` on a mismatch, and `RCB4Interface.serial_read` with it. [17] used to discard the reply's last byte without checking it, which is why this was once left open | **measured** [22]: all 7 replies matched (`VERSION`, a config read, five ICS 2 record reads with the servo limp, held and at -80, 0, 45 deg), and then 500 RAM reads in a row (4, 2 and 126 bytes in turn) parsed with the strict `unframe`: 500 ok, no `ChecksumError`, no NAK; the replies recorded in [19] and pinned in `test_protocol_rcb4mini.py` (ack, RAM reads, record reads) match as well. **Not measured on an ARM-H7**, which shares `unframe` |
| COM port: **even parity** | [17] `RCB4Interface.open` |
| Config word at `$0000`; COM rate in bits 6-7: `0` = 115200, `2` = 1.25 Mbit/s; the board acks a config write at the old rate, then switches | **measured** [19] (switched to 2, answered at 1.25 M only) |
| ICS rate in config bits 13-14: KRS servos set to 1.25 Mbit/s answer with `2`, not `0` or `1` (`3` also answered, meaning unknown, unused) | **measured** [19] |
| ICS rate code `0` is 115200 bit/s on the ICS line | **measured** 2026-10-16 with a scope on the same rig (lab note, URL not captured); KRS servos set to 115200 (IDs 0, 2, 8) answered with it here, through rcb4 `assign_ics_servos` (config `0x8391`) |
| ICS assignment table at `$0044`, one `uint16` per ICS number, `$FFFF` = unassigned, else the record address; loaded from the ROM project at power-up. A project may point a number at any 20 bytes of user RAM; a host that wants one record per number maps every such number back to its own record (`$0090` + 20 x number), servo off | [18] `servo_bus_rcb4.cpp` (cites the RCB-4 firmware reference); [21] firmware reference p.15 (records placed anywhere in user RAM `$0090..$048F`); **recorded** [19] |
| Servo records at `$0090`, 20 bytes each; `+4` measured pulse, `+6` reference pulse (`0x8000` = off) | [17] `_angle_vector`, [18]; **recorded** [19] |
| The board sends a record's reference to the servo the moment its number is assigned, so it is set to off first | [18] `try_assign` |
| A record keeps its last pulse after its servo goes quiet, so it is cleared before assigning | **measured** [19]: a stale pulse read as a servo until cleared |
| A board without a project powers up at COM 115200 with every ICS number unassigned (config `0x8313`) | **measured** [19] |
| A record's `+1` byte is the servo's ICS ID, and ICS number = 2 x ID + port (port 0 = SIO1-4, 1 = SIO5-8) | **recorded** [19]: ICS 2, 5, 8, 10, 18, 30 held 1, 2, 4, 5, 9, 15; port numbering from [21] `Rcb4.xml` `CommunicateDevice` |
| A MOV with an ICS source (`00 21 00 00 00 offset ics size`) reads the board's RAM record through the assignment table, not the servo | **measured** [19]: ICS 5, offset 0, 20 bytes returned the 20 bytes of record `$00F4` exactly; encoding from [20] `rcb4asm.l` `rcb4-src-bytes` and [21] command reference p.3 (MOV) |
| `0x12` sets a servo parameter of the ICS numbers whose bit is set in a 5-byte mask (bit n = ICS number n), then a sub-command (`01` stretch 1..127, `02` speed) and one value per set bit in ICS order; answered `SIZE CMD 06 SUM`. The reference's table prints the command code as `11h`, the batch move it copies its layout from. `build_stretch` builds it | [21] command reference p.20-21 (サーボのストレッチ・スピードの変更); code `0x12` and the bit order from [17] `rcb4` `set_servo_parameters_command` (the vectors in `test_protocol_rcb4mini.py`, generated at rcb4 `e5cd18d`). kxr_controller sends it only to an ARM-H7 (`rcb4_ros_bridge.py`, `if not self.use_rcb4`). **Not yet measured** on an RCB-4mini |
| **Never `0x0E`** (ICS relay: `[09, 0E, port, count, tx_lo, tx_hi, rx_lo, rx_hi, sum]`, which the reference says sends RAM bytes to the ICS bus and stores the reply in RAM). Nothing in `rcb4.protocol` builds it | format from [21] command reference p.14 (ICS通信); **measured** [19]: relaying `a2 04` (temperature read, ICS 5 = ID 2) on port 1 drew no reply in 1 s and wrote nothing to the reply area; the same on port 0 left the board answering nothing at any rate until power-cycled |
| The user-variable area (`$0462`) is not cleared by a power cycle | **measured** [19]: bytes written there were still present after the board was power-cycled |
| The board rewrites the `+4` measured pulse of every assigned number whose servo answers, once per frame, limp or held; one with no servo stays at whatever was last written. Zeroing `+4` and reading it back is therefore a presence test | **measured** [19]: ICS 18 unplugged read 0 within 50 ms of a clear and came back after replugging without reassigning; assigned unwired ICS 0, 1, 3 stayed 0 |
| A record's `+2` (int16) is the board's per-servo **trim**: the board reports `servo pulse - trim` at `+4` and drives `reference + trim`. A project in the board's ROM sets it at power-up, so a host that wants the servos' own zeros switches any servo with a trim off and zeroes the trim at connect | [21] firmware reference p.15-16 (`基準値`: drive = mixing 1 + mixing 2 + `+6` + `+2`; `+4` = servo − `+2`). **measured** 2026-10-20 on a board with a project: ICS 5 and 7 held trims -2700 and +2000 (ICS 4, 6, 8, 9 and 12-17 held trims as well); with both servos released, writing 0 to `+2` changed their `+4` by exactly -2700 and +2000 (7497 -> 4797, 7500 -> 9500). rcb4 [17] `_angle_vector` reads the same word as its "error" slot |
| Writing a record's `+6` reference in RAM drives the servo to it, as a batch move does | **measured** 2026-10-20: a position servo (ICS 21..23, before they were set to rotation mode) went to 7500 when its `+6` was written to 7500. A wheel (rotation-mode servo) is best started this way, because servo ON (`0x7FFF`) would copy the pot into its speed |
| A servo's port is found by assigning both 2 x ID and 2 x ID + 1 and seeing which `+4` the board rewrites, never by the `0x0E` relay | follows from the two rows above; **not yet measured** with a servo moved between SIO1-4 and SIO5-8, and whether polling the extra, empty numbers lengthens the board's frame is not measured either |

## Kondo ICS 3.5 / 3.6 — a servo spoken to directly (`ics.py`)

The protocol on the servo's own line (a Dual USB Adapter in ICS mode, red LED,
or the board's SIO ports), not the RCB-4 board protocol above. No length byte
and no checksum.

| Fact used in code | Source |
| --- | --- |
| Command byte = main command in bits 5-7 (`0x80` position, `0xA0` read, `0xC0` write, `0xE0` ID) + servo ID 0..31 in bits 0-4; every other byte keeps its MSB 0 (ID writes excepted) | [23] p.11-12 |
| Sub-commands: `00` EEPROM, `01` stretch, `02` speed, `03` current (read) / current limit (write), `04` temperature / temperature limit, `05` position read (ICS 3.6 only, read only) | [23] p.12 |
| The line is shared, so the host first receives every byte it sent (**loopback**), then the servo's reply | [23] p.10 (送信コマンドのループバック), p.15 |
| Position: TX `CMD POS_H POS_L`; RX loopback + `R_CMD TCH_H TCH_L`, `R_CMD` = `CMD & 0x7F`. A position is 14 bits sent as two 7-bit halves; 0 frees the servo; 3500..11500, neutral 7500 on a 270 deg servo. Example: `81 3A 4C` -> `01 3A 4C` | [23] p.13-15 |
| A position reply from **ID 0 at 115.2 kbit/s keeps the MSB** (ICS 2.0 compatibility); `parse_reply` accepts `0x80` for ID 0 at any rate, since it does not know the rate | [23] p.14-15 |
| Read: TX `CMD SC`; RX loopback + `R_CMD SC DATA`, `R_CMD` = `0x20 | ID`; `DATA` is 64 bytes for EEPROM, two 7-bit halves for the position, one byte otherwise. Example: `A1 01` -> `21 01 1E` | [23] p.16-17 |
| Write: TX `CMD SC DATA`; RX loopback + `R_CMD SC DATA` (one value) or `R_CMD SC` (EEPROM), `R_CMD` = `0x40 | ID`. Example: `CA 02 64` -> `4A 02 64` | [23] p.18-20 |
| ID: TX `CMD SC SC SC` (`FF 00 00 00` reads, `E0|id 01 01 01` writes); RX loopback + `R_CMD` = `0xE0 | ID`, **not** masked. Only with one servo on the line | [23] p.21-22 (the text on p.22 says the read reply has its MSB cleared, but its own example `0xF3`/`0xF4` and the note under it keep it set; the example is followed) |
| EEPROM: 64 bytes, each one 4-bit half of a field (byte 1-2 = `0x5A` header, 27-28 baud rate, factory-calibration fields to be written back as read) | [23] p.23-24 |
| A wrong loopback, header, sub-command or written-back value raises `IcsReplyError` (a `ProtocolError`) instead of being taken for data; `ICSServoController` used the bytes at fixed offsets without checking them | follows from the rows above; **measured** [24]: every reply of a KRS servo passed the checks (ID, EEPROM, stretch, speed, current, temperature, position read and four position commands, then 300 reads in a row: 300 ok) |

[17] `rcb4` — the host library, at commit
     `dfa222528e0f21a9ecd1495b7398ae7eb12e25bc`, which added the ICS assignment
     and the 1.25 M switch to `RCB4Interface.open`.
     <https://github.com/iory/rcb4>
[18] `build-your-own-arduino-rtos`, `code/13_quadruped/src/servo_bus_rcb4.cpp`
     — an RCB-4mini driven from an Arduino, which assigns ICS numbers the same
     way. <https://github.com/iory/build-your-own-arduino-rtos>
[19] Measured on an RCB-4mini (no HeartToHeart project, six KRS servos at ICS
     numbers 2, 5, 8, 10, 18, 30), 2026-10-12/13, through a Kondo Dual USB
     Adapter HS (165c:0008).
[20] `kxreus` — EusLisp interface to RCB-4 boards, at commit
     `76bb12718f09bad3c309b62993059a747130588b`: `rcb4asm.l`
     `*rcb4-sysreg-bits*` (config bit names), `rcb4interface.l`
     `:send-servo-vector-com` (ROM-run off around servo commands) and
     `:read-servos-three` (126-byte record reads).
     <https://github.com/inabajsk/kxreus>
[21] Kondo Kagaku, RCB-4 reference set Ver.2.0 (`RCB4RefSetV200R20120420.zip`):
     `RCB-4コマンドリファレンス20100805.pdf` (p.3 MOV, p.14 ICS通信),
     `RCB-4ファームウェアリファレンス20100805.pdf` and
     `RCB-4プログラミングマニュアル 20120420.pdf` (RAM maps), and the `Rcb4.dll`
     documentation `Rcb4.xml`.
     <https://kondo-robot.com/faq/rcb-4-reference-ver-2_0>,
     <https://kondo-robot.com/w/wp-content/uploads/RCB4RefSetV200R20120420.zip>
[22] Measured 2026-11-10 on an RCB-4mini (firmware `CB-4 V1.0 090715`, one
     KRS servo at ICS 2 with a bare horn, OWON SPE6103 at 7.4 V) through a
     Kondo Dual USB Adapter HS (165c:0008), at 1.25 Mbit/s: requests built
     and replies parsed with `rcb4.protocol` on the port
     `RCB4Interface.open` had opened, every read compared with
     `RCB4Interface`'s own.
[23] Kondo Kagaku, ICS3.5/3.6 ソフトウェアマニュアル コマンドリファレンス
     (Ver 2.6), page numbers as printed ("n／30").
     <https://kondo-robot.com/w/wp-content/uploads/ICS3.5_3.6_SoftwareManual_2_6.pdf>;
     English ICS3.5 edition:
     <https://kondo-robot.com/w/wp-content/uploads/ICS3_5SoftwareManual_V1English.pdf>
[24] Measured 2026-11-10 on one KRS servo (ID 2, ICS 3.6: it answers the
     position read, EEPROM baud field 115200) on a Kondo Dual USB Adapter HS
     in ICS mode, through `ICSServoController` with `rcb4.protocol.ics`. It
     answered nothing at 1.25 M and 625 k (`OSError`, no `IcsReplyError`).

### Not sourced — deliberately absent

- **625000 bit/s** on COM, and every ICS rate but 115200 and 1.25 M: their
  config values were not tried, so asking for one raises.
- **Why large reads are refused or hang the board.** Not the address (a
  16-byte scan of `$0090..$0400` and `$01FC`+242 alone both answered), not
  the request spacing (2-byte reads at 0.5 ms gaps never refused), not the
  ROM program (cleared, refusals persisted). Unknown; `plan_record_reads`
  avoids it.
- **What `x` in a refusal means**, and whether a refused move was applied.
  Resending is safe either way, because every request here is idempotent.
- **A move time in milliseconds.** One unit is one frame (15 ms with the frame
  field this board powered up with, measured above), but the frame length is
  a config field a project can change and only `01` was measured, so no
  constant converts move bytes to milliseconds here.
- **Servo EEPROM, rotation mode, current and temperature.** The ICS-source
  MOV reads only RAM and the `0x0E` relay does not work on this firmware
  (both measured above), so none of them is reachable through the board. They
  can be read with the servo on the adapter in ICS mode ([17] `rcb4/ics.py`).
