# UART Frame Parser

A lightweight, robust UART protocol parser written in C for a custom framed message format. It is designed for embedded systems where bytes arrive over a serial interface and need to be validated, timed, and decoded into a clean command + payload structure.

## Overview

This parser accepts incoming UART bytes and reconstructs frames using a simple protocol:

- Start-of-frame (SOF): `0xAA`
- Command byte: 1 byte
- Length byte: 1 byte
- Payload: `LEN` bytes, where `0 <= LEN <= 16`
- Checksum: XOR of `CMD`, `LEN`, and each payload byte

The parser detects:

- valid frames
- checksum errors
- timeout-based frame resets
- invalid length conditions
- back-to-back frame parsing without leftover state

## Frame Format

```text
+--------+--------+-----------+-----------+------------+
| SOF    | CMD    | LEN      | PAYLOAD  | CHECKSUM   |
| 0xAA   | 1 byte | 1 byte   | LEN bytes| 1 byte     |
+--------+--------+-----------+-----------+------------+
```

Checksum calculation:

```text
checksum = CMD ^ LEN ^ payload[0] ^ payload[1] ^ ... ^ payload[LEN-1]
```

The receiving node validates the final checksum byte and only accepts the frame if it matches exactly.

## Features

- Byte-by-byte UART frame decoding
- Start-of-frame synchronization
- Maximum payload length enforcement (`16 bytes`)
- Timeout detection using millisecond timestamps
- Automatic reset on invalid or incomplete frames
- Supports recovery after a timeout or checksum failure
- Includes built-in example test sequences for validation

## State Machine

The parser operates with the following states:

1. `UART_STATE_WAITING_FOR_SOF`
2. `UART_STATE_RECEIVING_CMD`
3. `UART_STATE_RECEIVING_LEN`
4. `UART_STATE_RECEIVING_PAYLOAD`
5. `UART_STATE_RECEIVING_CHECKSUM`

This sequence is enforced to ensure that only valid frames are accepted and that the parser can recover cleanly from corrupted UART streams.

## API Summary

### Core types

```c
typedef struct
{
    uart_parser_state_t state;
    uint8_t cmd;
    uint8_t payload_len;
    uint8_t payload[UART_MAX_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t checksum;
    uint32_t last_byte_timestamp_ms;
    uint32_t timeout_ms;
} uart_parser_t;
```

### Initialization

```c
uart_parser_init(&parser, UART_DEFAULT_TIMEOUT_MS);
```

- `parser`: parser instance
- `timeout_ms`: maximum allowed inter-byte gap before frame timeout
- `0` disables timeout checking

### Feeding one byte

```c
int status = uart_parser_feed_byte(&parser, byte, timestamp_ms);
```

Return values:

- `UART_FRAME_COMPLETE` = valid frame received
- `UART_FRAME_IN_PROGRESS` = parser is still collecting data
- `UART_CHECKSUM_ERROR` = checksum mismatch
- `UART_TIMEOUT_ERROR` = inter-byte timeout exceeded

### Feeding a burst of bytes

```c
uart_feed_stream(&parser, bytes, timestamps, count);
```

This helper is useful when bytes are read from a UART buffer or a stored packet trace and need to be processed sequentially with timing information.

## Protocol Behavior

### Timeout handling

The parser checks for a timeout before accepting the next byte. If the elapsed time between consecutive bytes exceeds the configured timeout, the current frame state is reset and a timeout error is reported.

This prevents a partial frame from being mistaken for a valid one after a long delay.

### Invalid length handling

If the `LEN` byte is greater than `UART_MAX_PAYLOAD_LEN` (`16`), the parser discards the frame and waits for the next SOF. This prevents buffer overflow and malformed input from damaging state.

### Recovery behavior

The parser automatically resets on:

- timeout
- checksum mismatch
- invalid length
- undefined state

After reset, the parser resumes waiting for the next `0xAA` start byte.

## Example Use Case

```c
uart_parser_t parser;
uint8_t data[] = { 0xAA, 0x01, 0x03, 0x10, 0x20, 0x30, 0x02 };
uint32_t ts[] = { 0, 5, 10, 15, 20, 25, 30 };

uart_parser_init(&parser, 50U);
uart_feed_stream(&parser, data, ts, 7U);
```

Expected result:

```text
FRAME OK CMD=0x01 LEN=3 PAYLOAD=[10 20 30]
```

## Build and Run

From the project directory:

```bash
gcc uart_parser.c -o uart_parser
./uart_parser
```

The program includes four test cases:

1. Clean valid frame
2. Timeout mid-frame with recovery
3. Two valid frames back-to-back
4. Timeout disabled mode

## Notes for Embedded Use

- Use a timer tick or millisecond counter to provide `timestamp_ms` values.
- Keep the timeout value realistic for your UART baud rate and application latency.
- This implementation is intentionally simple and deterministic, which makes it suitable for embedded firmware and low-resource MCUs.
- The parser assumes a single byte stream and does not include OS-level UART drivers.

## Summary

This code is a production-friendly parser foundation for a compact binary UART protocol. It is easy to integrate into firmware, robust against malformed data, and designed to recover cleanly from protocol errors without blocking or corrupting the parser state.
