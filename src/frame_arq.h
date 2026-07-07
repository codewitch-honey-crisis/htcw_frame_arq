#ifndef HTCW_FRAME_ARQ_H
#define HTCW_FRAME_ARQ_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/// @brief The on-wire header length: 8 marker bytes + 1 seq/type byte + 4 length + 4 CRC
#define FRAME_ARQ_HEADER_LENGTH (8 + 1 + 4 + 4)

/// @brief Define FRAME_ARQ_NO_CRC_TABLE (e.g. -DFRAME_ARQ_NO_CRC_TABLE) to compute
///        the CRC-32 with a bit-by-bit loop instead of a 256-entry lookup table.
///        This removes the ~1 KB const table (which otherwise lives in flash, not
///        SRAM) at the cost of ~8x more CPU per payload byte. No table is built at
///        runtime in this mode. Both modes produce identical CRC values, so a
///        tableless build interoperates with a table build.

/// @brief frame error / status codes
/// @remarks Ordering is deliberate: every genuine error is < -1, so a caller can
///          test `if (r < -1)` to catch real failures while treating -1
///          (FRAME_ARQ_RESEND_NEEDED) as the non-fatal "please resend" notification.
enum {
    /// @brief A send was attempted while still awaiting an ACK for the
    ///        previously sent frame. Stop-and-wait allows one frame in flight.
    FRAME_ARQ_ERROR_BUSY = -7,
    /// @brief An invalid argument was passed
    FRAME_ARQ_ERROR_ARG = -6,
    /// @brief There was not enough data to complete the request (deprecated)
    FRAME_ARQ_ERROR_UNDERFLOW = -5,
    /// @brief The frame received length is too large
    FRAME_ARQ_ERROR_OVERFLOW = -4,
    /// @brief The frame CRC did not match (a NACK was auto-emitted)
    FRAME_ARQ_ERROR_CRC = -3,
    /// @brief Internal sentinel: no complete frame is available yet. Never
    ///        returned to the caller (frame_arq_get() reports it as 0).
    FRAME_ARQ_INCOMPLETE = -2,
    /// @brief A NACK was received; the caller should resend the outstanding
    ///        frame via frame_arq_resend(). This is a *notification*, not a
    ///        failure. It sits at -1 so that `r < -1` excludes it from errors.
    FRAME_ARQ_RESEND_NEEDED = -1,
    /// @brief The operation completed successfully
    FRAME_ARQ_SUCCESS = 0
};

/// @brief The callback used to read a byte from the transport
typedef int(*frame_arq_read_callback_t)(void* state);
/// @brief The callback used to write a byte to the transport
typedef int(*frame_arq_write_callback_t)(uint8_t value, void* state);

// private state
typedef struct {
    size_t payload_max_size;
    frame_arq_read_callback_t read_cb;
    void* read_state;
    frame_arq_write_callback_t write_cb;
    void* write_state;
    uint8_t* read_buffer;    // incoming frame under assembly
    uint8_t* retain_buffer;  // last sent DATA frame, kept for resend
    size_t retain_length;    // bytes valid in retain_buffer
    uint8_t start;           // marker byte candidate during sync
    size_t byte_count;       // bytes assembled in read_buffer so far
    uint8_t tx_seq;          // seq (0-63) of the last DATA frame sent
    uint8_t expected_rx_seq; // seq (0-63) we next expect to receive
    bool awaiting_ack;       // a sent DATA frame has not yet been ACKed
    bool resend_needed;      // a NACK was seen; resend is pending
} frame_arq_t;

/// @brief A handle to a frame controller
typedef void* frame_arq_handle_t;

/// @brief Creates a frame controller
/// @param max_payload_size The maximum size of a payload for a frame. This should be at least the size of the largest message to be received
/// @param on_read_callback The read callback used to read a byte from the transport
/// @param on_read_callback_state User defined state to pass to the read callback
/// @param on_write_callback The write callback used to write a byte to the transport
/// @param on_write_callback_state User defined state to pass to the write callback
/// @return A frame handle, or NULL on error (out of memory or invalid arg)
frame_arq_handle_t frame_arq_create(size_t max_payload_size,
    frame_arq_read_callback_t on_read_callback, void* on_read_callback_state,
    frame_arq_write_callback_t on_write_callback, void* on_write_callback_state
);

/// @brief Creates a frame controller without allocating. The caller supplies the buffers.
/// @param max_payload_size The maximum size of a payload for a frame. This should be at least the size of the largest message to be received
/// @param in_out_frame_state A caller supplied frame_arq_t structure used for bookkeeping. Effectively opaque.
/// @param frame_read_buffer A caller supplied buffer for the life of the controller. Must be FRAME_ARQ_HEADER_LENGTH + max_payload_size in length.
/// @param frame_retain_buffer A second caller supplied buffer of the same size, used to hold the last sent frame for retransmission.
/// @param on_read_callback The read callback used to read a byte from the transport
/// @param on_read_callback_state User defined state to pass to the read callback
/// @param on_write_callback The write callback used to write a byte to the transport
/// @param on_write_callback_state User defined state to pass to the write callback
/// @return A frame handle, or NULL on error (invalid arg)
/// @remarks Do not call frame_arq_destroy() on this handle
frame_arq_handle_t frame_arq_create_za(size_t max_payload_size,
    frame_arq_t* in_out_frame_state,
    void* frame_read_buffer,
    void* frame_retain_buffer,
    frame_arq_read_callback_t on_read_callback, void* on_read_callback_state,
    frame_arq_write_callback_t on_write_callback, void* on_write_callback_state
);

/// @brief Releases the resources used by a frame controller
/// @param handle The handle to destroy
void frame_arq_destroy(frame_arq_handle_t handle);

/// @brief Attempts to retrieve the next waiting frame in the buffer, driving the ARQ state machine.
/// @param handle The handle to the frame controller
/// @param out_data A pointer to the payload data received. The frame controller handles the lifetime.
/// @param out_size The size of the payload data received.
/// @return 0 = no data frame delivered (nothing waiting, or an ACK/duplicate/control frame was handled internally).
///         > 0 = the cmd marker byte (1-127) of a freshly delivered payload.
///         FRAME_ARQ_RESEND_NEEDED (-1) = a NACK arrived; call frame_arq_resend(). Not an error.
///         < -1 = a genuine error (e.g. FRAME_ARQ_ERROR_CRC, after which a NACK was auto-emitted).
/// @remarks Non-blocking / coroutine friendly: pumps whatever bytes are available and returns.
///          Call repeatedly in a loop. ACKs and NACKs for received frames are emitted automatically.
///          out_data points into internal storage that is overwritten on the next call, so consume it first.
int frame_arq_get(frame_arq_handle_t handle, void** out_data, size_t* out_size);

/// @brief Unconditionally discards the next waiting (incoming) frame
/// @param handle The handle to the frame controller
/// @return 0 on success. < 0 on error (frame handle invalid)
int frame_arq_discard(frame_arq_handle_t handle);

/// @brief Writes a DATA frame to the transport and retains it for possible resend.
/// @param handle The handle to the frame controller
/// @param cmd The cmd marker byte to write (1-127)
/// @param payload The payload to send
/// @param size The size of the payload
/// @return FRAME_ARQ_SUCCESS on success. FRAME_ARQ_ERROR_BUSY if a prior frame is still awaiting an ACK. Otherwise, error.
/// @remarks Copies the frame internally; the caller's payload need not remain valid after return.
int frame_arq_put(frame_arq_handle_t handle, uint8_t cmd, const void* payload, size_t size);

/// @brief Re-streams the last sent DATA frame to the transport (identical bytes, same seq).
/// @param handle The handle to the frame controller
/// @return FRAME_ARQ_SUCCESS on success (or if nothing is outstanding). < 0 on transport/arg error.
/// @remarks Caller-driven: invoke on receipt of FRAME_ARQ_RESEND_NEEDED, or on your own timeout.
int frame_arq_resend(frame_arq_handle_t handle);

/// @brief Reports whether a sent frame is still awaiting its ACK.
/// @param handle The handle to the frame controller
/// @return true if awaiting an ACK (a new frame_arq_put() would return BUSY), false otherwise or if handle is NULL.
bool frame_arq_awaiting_ack(frame_arq_handle_t handle);

/// @brief Reports whether a NACK has been seen and a resend is pending.
/// @param handle The handle to the frame controller
/// @return true if frame_arq_resend() should be called, false otherwise or if handle is NULL.
bool frame_arq_resend_needed(frame_arq_handle_t handle);

/// @brief Resets ARQ state (sequence numbers, awaiting/resend flags, buffers) for a fresh session.
/// @remarks Use when a link is re-established without both ends restarting (e.g. a USB CDC
///          reconnect that does not reboot the MCU). Both ends should reset so sequences realign.
int frame_arq_reset(frame_arq_handle_t handle);
#ifdef __cplusplus
}
#endif
#endif // HTCW_FRAME_ARQ_H
