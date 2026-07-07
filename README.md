# frame_arq

Provides transport framing, error checking, and **guaranteed delivery** for dirty comms lines.

This library accepts simple read and write callbacks to read or write bytes to and from a transport. On top of plain framing it adds a stop-and-wait ARQ (Automatic Repeat reQuest) layer: each frame is acknowledged, corrupt or missing frames are detected, and delivery is retried until it succeeds.

This is a fork of `htcw_frame`. The wire format is **not** compatible with `htcw_frame` — it adds a sequence byte and a real CRC — so the two ends of a link must both use `htcw_frame_arq`.

## Wire format

Each frame is framed using 8 identical marker bytes, followed by a single sequence/type byte, a 4-byte unsigned little-endian payload length, a 4-byte little-endian CRC, and then the payload itself.

- **Marker bytes:** For a data frame the marker is the command value plus 128, so a command of 1 through 127 becomes 129 through 255 on the wire. This keeps markers out of the ASCII range and prevents collisions with log text. Command 0 is reserved — its marker value (128) identifies a **control frame** (an ACK or NACK).
- **Sequence/type byte:** The high 2 bits are the frame type (data, ACK, or NACK); the low 6 bits are the sequence number (0-63). Stop-and-wait only needs enough sequence space to spot duplicates, so ACKs and NACKs ride entirely in this byte with no payload — no command values are stolen from your 1-127 range.
- **CRC:** A standard IEEE CRC-32. Unlike the original library, the CRC now covers the sequence byte and the length field in addition to the payload, so a corrupted header is caught too.

The full header is `FRAME_ARQ_HEADER_LENGTH` (17) bytes.

## Setup

Setting it up involves giving `frame_arq_create()` a couple of read/write callbacks and a max payload size.

Here's an example with Arduino:
```cpp
#include <Arduino.h>
#include "frame_arq.h"
#define PAYLOAD_MAX_SIZE (1024)
static frame_arq_handle_t frame_handle = NULL;
static int serial_read(void* state) {
    (void)state;
    return Serial.read();
}
static int serial_write(uint8_t value, void* state) {
    (void)state;
    Serial.write((uint8_t)value);
    return 0;
}
void setup() {
    Serial.begin(115200);
    frame_handle = frame_arq_create(PAYLOAD_MAX_SIZE,serial_read,NULL,serial_write,NULL);
}
```

Once it's configured and bound to a transport, you read and write frames using `frame_arq_get()` and `frame_arq_put()`.

## Reading frames

`frame_arq_get()` drives the ARQ state machine. It pumps whatever bytes are available and returns immediately (it never blocks waiting for a frame), so call it repeatedly in a loop. When it receives a valid frame it automatically sends the appropriate ACK or NACK for you before returning.

```cpp
void* ptr;
size_t length;
int cmd = frame_arq_get(frame_handle, &ptr, &length);
// cmd  > 0 : a payload was delivered. cmd is the command (1-127);
//            ptr points to the payload and length is its size in bytes.
// cmd == 0 : nothing to do — no frame yet, or an ACK/duplicate/control
//            frame was handled internally.
// cmd == FRAME_ARQ_RESEND_NEEDED (-1) : a NACK arrived from the peer;
//            call frame_arq_resend(). This is a notification, not an error.
// cmd  < -1 : a genuine error (e.g. FRAME_ARQ_ERROR_CRC).
```

The error codes are ordered so that `FRAME_ARQ_RESEND_NEEDED` is `-1` and every real failure is `< -1`. That lets you write `if (cmd < -1)` to test for a genuine error while treating a resend request as the non-fatal notification it is.

`ptr` points into internal storage that is overwritten on the next call to `frame_arq_get()`, so consume the payload before calling again.

## Writing frames

Unlike reading, you provide the buffer. `frame_arq_put()` copies the frame internally (so your buffer need not stay valid after the call) and retains it for possible retransmission.

```cpp
// typedef struct { ... } data_t; data_t my_data;
int res = frame_arq_put(frame_handle, my_cmd, &my_data, sizeof(my_data));
// FRAME_ARQ_SUCCESS on success.
// FRAME_ARQ_ERROR_BUSY if a previous frame is still awaiting its ACK.
// < -1 on other errors.
```

Because this is stop-and-wait, only one frame may be outstanding at a time. If you call `frame_arq_put()` while a previously sent frame has not yet been acknowledged, it returns `FRAME_ARQ_ERROR_BUSY` rather than blocking. You can check this ahead of time with `frame_arq_awaiting_ack()`.

## Guaranteed delivery

Delivery is guaranteed by two mechanisms working together, and retransmission timing is left to you so you keep control over timeouts and retry counts:

- **Fast path (NACK):** When the peer receives a corrupt or out-of-sequence frame it sends a NACK. Your `frame_arq_get()` returns `FRAME_ARQ_RESEND_NEEDED`; you respond by calling `frame_arq_resend()`, which retransmits the outstanding frame with the same sequence number.
- **Total-loss path (timeout):** If a frame is lost so completely that nothing comes back, no NACK is generated. You detect this with your own timer: while `frame_arq_awaiting_ack()` is true, resend if too much time has passed.

A minimal receive/retransmit loop:
```cpp
#define ACK_TIMEOUT_MS 500
static uint32_t last_send_ms = 0;

void loop() {
    void* ptr;
    size_t length;

    int cmd = frame_arq_get(frame_handle, &ptr, &length);
    if (cmd > 0) {
        handle_message(cmd, ptr, length);   // arrived and was ACKed for us automatically
    } else if (cmd == FRAME_ARQ_RESEND_NEEDED) {
        frame_arq_resend(frame_handle);      // the peer NACKed our outstanding frame
        last_send_ms = millis();
    }

    // caller-owned timeout: retransmit if we've waited too long for an ACK
    if (frame_arq_awaiting_ack(frame_handle) &&
        (uint32_t)(millis() - last_send_ms) >= ACK_TIMEOUT_MS) {
        frame_arq_resend(frame_handle);
        last_send_ms = millis();
    }
}
```

And to send, respecting the one-in-flight rule:
```cpp
if (!frame_arq_awaiting_ack(frame_handle)) {
    if (frame_arq_put(frame_handle, my_cmd, &my_data, sizeof(my_data)) == FRAME_ARQ_SUCCESS) {
        last_send_ms = millis();
    }
}
```

Duplicate delivery is handled for you: if an ACK is lost and the peer retransmits, `frame_arq_get()` re-ACKs the duplicate but does not deliver it to you a second time.

The library never abandons a frame or advances the sequence number on its own — it retransmits the same frame until it is acknowledged. If you want to give up on a dead link, do it in your own retry logic and recover by reconnecting (which resets the sequence for a fresh session).

## Other functions

Discard the next waiting incoming frame without reading it:
```cpp
frame_arq_discard(frame_handle); // may never need this
```

Poll the sender state:
```cpp
bool waiting = frame_arq_awaiting_ack(frame_handle);   // a sent frame is unacked
bool resend  = frame_arq_resend_needed(frame_handle);  // a NACK was seen and a resend is pending
```

Free the handle when you're done:
```cpp
frame_arq_destroy(frame_handle);
```

## Zero-allocation creation

If you'd rather not use the heap, `frame_arq_create_za()` lets you supply the storage. It needs a `frame_arq_t` state structure and **two** buffers — one for receiving and one for retaining the last sent frame — each `FRAME_ARQ_HEADER_LENGTH + max_payload_size` bytes long:
```cpp
#define PAYLOAD_MAX_SIZE (1024)
static frame_arq_t     frame_state;
static uint8_t         read_buffer[FRAME_ARQ_HEADER_LENGTH + PAYLOAD_MAX_SIZE];
static uint8_t         retain_buffer[FRAME_ARQ_HEADER_LENGTH + PAYLOAD_MAX_SIZE];

frame_handle = frame_arq_create_za(PAYLOAD_MAX_SIZE,
    &frame_state, read_buffer, retain_buffer,
    serial_read, NULL, serial_write, NULL);
// do not call frame_arq_destroy() on a handle created this way
```

## CRC without a table

By default the CRC-32 uses a 256-entry lookup table, which lives in flash (not RAM) on an MCU. If flash is tight, define `FRAME_ARQ_NO_CRC_TABLE` (e.g. `-DFRAME_ARQ_NO_CRC_TABLE`) to compute the CRC bit-by-bit instead. This removes the table entirely — nothing is precomputed in RAM — at the cost of roughly 8x more CPU per payload byte. Both modes produce identical CRC values, so a tableless build interoperates with a table build.

## PlatformIO

```
; PlatformIO INI entry
[env:node32s]
platform = espressif32
board = node32s
framework = arduino
lib_deps = 
	codewitch-honey-crisis/htcw_frame_arq
```

## About the Demo

The PlatformIO repo portion contains a Demo project under its examples tree which demonstrates using this library in tandem with [htcw_buffers](https://github.com/codewitch-honey-crisis/htcw_buffers) to create a complete, framed, and reliably delivered serial wire protocol for communicating between the SerialFrameDemo C# app running on a Windows PC and a connected ESP32. Because delivery is acknowledged in both directions, the demo recovers from a noisy line — corrupt frames are detected, NACKed, and retransmitted without losing messages.