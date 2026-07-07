#include "frame_arq.h"
#include <memory.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/* Seq/type byte layout: [ttssssss]  t = 2 type bits, s = 6 seq bits.  */
#define FA_TYPE_DATA 0
#define FA_TYPE_ACK  1
#define FA_TYPE_NACK 2
#define FA_SEQ_MASK  0x3F
#define FA_TYPE_OF(b) (((b) >> 6) & 0x03)
#define FA_SEQ_OF(b)  ((b) & FA_SEQ_MASK)
#define FA_SEQBYTE(type, seq) ((uint8_t)(((type) << 6) | ((seq) & FA_SEQ_MASK)))

/* Marker byte for control (ACK/NACK) frames. cmd 0 is reserved, so 0+128. */
#define FA_CTRL_MARKER 128

/* Standard IEEE CRC-32 (reflected 0xEDB88320), init/final 0xFFFFFFFF.
   By default a 256-entry const table is used; it lives in flash rather than RAM
   on an MCU. Define FRAME_ARQ_NO_CRC_TABLE to compute the CRC bit-by-bit instead,
   dropping the table entirely (no flash table, no runtime-built RAM table) at the
   cost of ~8x more CPU per byte. Both paths yield identical CRC values. */
#ifndef FRAME_ARQ_NO_CRC_TABLE
static const uint32_t fa_crc_table[256] = {
    0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU, 0x076DC419U, 0x706AF48FU, 0xE963A535U, 0x9E6495A3U,
    0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U, 0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U,
    0x1DB71064U, 0x6AB020F2U, 0xF3B97148U, 0x84BE41DEU, 0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
    0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU, 0x14015C4FU, 0x63066CD9U, 0xFA0F3D63U, 0x8D080DF5U,
    0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U, 0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU,
    0x35B5A8FAU, 0x42B2986CU, 0xDBBBC9D6U, 0xACBCF940U, 0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
    0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U, 0x21B4F4B5U, 0x56B3C423U, 0xCFBA9599U, 0xB8BDA50FU,
    0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U, 0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU,
    0x76DC4190U, 0x01DB7106U, 0x98D220BCU, 0xEFD5102AU, 0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
    0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U, 0x7F6A0DBBU, 0x086D3D2DU, 0x91646C97U, 0xE6635C01U,
    0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU, 0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U,
    0x65B0D9C6U, 0x12B7E950U, 0x8BBEB8EAU, 0xFCB9887CU, 0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
    0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U, 0x4ADFA541U, 0x3DD895D7U, 0xA4D1C46DU, 0xD3D6F4FBU,
    0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U, 0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U,
    0x5005713CU, 0x270241AAU, 0xBE0B1010U, 0xC90C2086U, 0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
    0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U, 0x59B33D17U, 0x2EB40D81U, 0xB7BD5C3BU, 0xC0BA6CADU,
    0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU, 0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U,
    0xE3630B12U, 0x94643B84U, 0x0D6D6A3EU, 0x7A6A5AA8U, 0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
    0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU, 0xF762575DU, 0x806567CBU, 0x196C3671U, 0x6E6B06E7U,
    0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU, 0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U,
    0xD6D6A3E8U, 0xA1D1937EU, 0x38D8C2C4U, 0x4FDFF252U, 0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
    0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U, 0xDF60EFC3U, 0xA867DF55U, 0x316E8EEFU, 0x4669BE79U,
    0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U, 0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU,
    0xC5BA3BBEU, 0xB2BD0B28U, 0x2BB45A92U, 0x5CB36A04U, 0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
    0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU, 0x9C0906A9U, 0xEB0E363FU, 0x72076785U, 0x05005713U,
    0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U, 0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U,
    0x86D3D2D4U, 0xF1D4E242U, 0x68DDB3F8U, 0x1FDA836EU, 0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
    0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU, 0x8F659EFFU, 0xF862AE69U, 0x616BFFD3U, 0x166CCF45U,
    0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U, 0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU,
    0xAED16A4AU, 0xD9D65ADCU, 0x40DF0B66U, 0x37D83BF0U, 0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
    0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U, 0xBAD03605U, 0xCDD70693U, 0x54DE5729U, 0x23D967BFU,
    0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U, 0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU,};

static uint32_t fa_crc_update(uint32_t crc, const uint8_t* data, size_t length) {
    while (length--) {
        crc = fa_crc_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}
#else  /* FRAME_ARQ_NO_CRC_TABLE: bit-by-bit, no lookup table */
static uint32_t fa_crc_update(uint32_t crc, const uint8_t* data, size_t length) {
    while (length--) {
        int k;
        crc ^= *data++;
        for (k = 0; k < 8; ++k) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
#endif /* FRAME_ARQ_NO_CRC_TABLE */
/* CRC covers the seq byte + 4 length bytes (read_buffer[8..12]) then the
   payload (read_buffer[FRAME_ARQ_HEADER_LENGTH..]). The 8 marker bytes and the
   4 CRC bytes themselves are excluded. */
static uint32_t fa_frame_crc(const uint8_t* buf, size_t payload_size) {
    uint32_t crc = 0xFFFFFFFFU;
    crc = fa_crc_update(crc, buf + 8, 5);
    crc = fa_crc_update(crc, buf + FRAME_ARQ_HEADER_LENGTH, payload_size);
    return crc ^ 0xFFFFFFFFU;
}

/* ------------------------------------------------------------------ */
/* little-endian field access (byte-wise to avoid unaligned 32-bit loads) */
static uint32_t fa_rd_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void fa_wr_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static size_t size_from_frame(const frame_arq_t* frame) {
    return (size_t)fa_rd_u32(frame->read_buffer + 9);
}
static uint32_t crc_from_frame(const frame_arq_t* frame) {
    return fa_rd_u32(frame->read_buffer + 13);
}

/* ------------------------------------------------------------------ */
/* stream a header-only control frame (ACK or NACK) directly to the wire */
static int fa_write_control(frame_arq_t* frame, uint8_t type, uint8_t seq) {
    uint8_t hdr[FRAME_ARQ_HEADER_LENGTH];
    int i, res;
    for (i = 0; i < 8; ++i) hdr[i] = FA_CTRL_MARKER;
    hdr[8] = FA_SEQBYTE(type, seq);
    fa_wr_u32(hdr + 9, 0);                 /* length = 0 */
    fa_wr_u32(hdr + 13, fa_frame_crc(hdr, 0));
    for (i = 0; i < FRAME_ARQ_HEADER_LENGTH; ++i) {
        res = frame->write_cb(hdr[i], frame->write_state);
        if (res < 0) return res;
    }
    return FRAME_ARQ_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Assemble the 8 identical marker bytes (each >= 128). Returns 0 when the
   marker is complete, FRAME_ARQ_INCOMPLETE when more bytes are needed. */
static int read_frame_marker(frame_arq_t* frame) {
    while (frame->byte_count < 8) {
        int b = frame->read_cb(frame->read_state);
        if (b < 0) return FRAME_ARQ_INCOMPLETE;
        if (b < 128) {              /* not a marker byte: resync */
            frame->byte_count = 0;
            continue;
        }
        if (frame->byte_count == 0) {
            frame->start = (uint8_t)b;
            frame->read_buffer[0] = (uint8_t)b;
            frame->byte_count = 1;
        } else if ((uint8_t)b == frame->start) {
            frame->read_buffer[frame->byte_count++] = (uint8_t)b;
        } else {                    /* mismatch: restart with this candidate */
            frame->start = (uint8_t)b;
            frame->read_buffer[0] = (uint8_t)b;
            frame->byte_count = 1;
        }
    }
    return FRAME_ARQ_SUCCESS;
}

/* Pump bytes into read_buffer. Returns 0 for a complete, CRC-valid frame,
   FRAME_ARQ_INCOMPLETE when more bytes are needed, or a negative error. */
static int read_frame(frame_arq_t* frame) {
    if (frame->byte_count < 8) {
        int res = read_frame_marker(frame);
        if (res != FRAME_ARQ_SUCCESS) return res;
    }
    while (frame->byte_count < FRAME_ARQ_HEADER_LENGTH) {
        int b = frame->read_cb(frame->read_state);
        if (b < 0) return FRAME_ARQ_INCOMPLETE;
        frame->read_buffer[frame->byte_count++] = (uint8_t)b;
    }
    {
        size_t size = size_from_frame(frame);
        if (size > frame->payload_max_size) {
            frame->byte_count = 0;
            return FRAME_ARQ_ERROR_OVERFLOW;
        }
        while (frame->byte_count < FRAME_ARQ_HEADER_LENGTH + size) {
            int b = frame->read_cb(frame->read_state);
            if (b < 0) return FRAME_ARQ_INCOMPLETE;
            frame->read_buffer[frame->byte_count++] = (uint8_t)b;
        }
        if (crc_from_frame(frame) != fa_frame_crc(frame->read_buffer, size)) {
            frame->byte_count = 0;
            return FRAME_ARQ_ERROR_CRC;
        }
    }
    return FRAME_ARQ_SUCCESS;
}

/* ------------------------------------------------------------------ */
int frame_arq_get(frame_arq_handle_t handle, void** out_data, size_t* out_size) {
    frame_arq_t* frame;
    int res;
    uint8_t seqbyte, type, seq, marker;
    size_t size;
    if (handle == NULL || out_data == NULL || out_size == NULL) {
        return FRAME_ARQ_ERROR_ARG;
    }
    frame = (frame_arq_t*)handle;
    res = read_frame(frame);
    if (res == FRAME_ARQ_INCOMPLETE) {
        return FRAME_ARQ_SUCCESS; /* nothing complete yet */
    }
    if (res == FRAME_ARQ_ERROR_CRC) {
        /* corrupt: seq is untrustworthy, NACK our own expected seq */
        fa_write_control(frame, FA_TYPE_NACK, frame->expected_rx_seq);
        frame->byte_count = 0;
        return FRAME_ARQ_ERROR_CRC;
    }
    if (res < 0) {
        frame->byte_count = 0;
        return res;
    }

    marker = frame->read_buffer[0];
    seqbyte = frame->read_buffer[8];
    type = FA_TYPE_OF(seqbyte);
    seq = FA_SEQ_OF(seqbyte);

    if (marker == FA_CTRL_MARKER || type != FA_TYPE_DATA) {
        /* control frame */
        frame->byte_count = 0;
        if (type == FA_TYPE_ACK) {
            if (frame->awaiting_ack && seq == frame->tx_seq) {
                frame->awaiting_ack = false;
                frame->resend_needed = false;
            }
            return FRAME_ARQ_SUCCESS;
        }
        if (type == FA_TYPE_NACK) {
            if (frame->awaiting_ack) {
                frame->resend_needed = true;
                return FRAME_ARQ_RESEND_NEEDED;
            }
            return FRAME_ARQ_SUCCESS;
        }
        return FRAME_ARQ_SUCCESS; /* reserved control type: ignore */
    }

    /* DATA frame */
    size = size_from_frame(frame);
    if (seq == frame->expected_rx_seq) {
        *out_data = frame->read_buffer + FRAME_ARQ_HEADER_LENGTH;
        *out_size = size;
        fa_write_control(frame, FA_TYPE_ACK, seq);
        frame->expected_rx_seq = (uint8_t)((seq + 1) & FA_SEQ_MASK);
        frame->byte_count = 0;
        return (int)(marker - 128); /* cmd 1..127 */
    }
    if (seq == (uint8_t)((frame->expected_rx_seq - 1) & FA_SEQ_MASK)) {
        /* duplicate (our previous ACK was likely lost): re-ACK, do not deliver */
        fa_write_control(frame, FA_TYPE_ACK, seq);
        frame->byte_count = 0;
        return FRAME_ARQ_SUCCESS;
    }
    /* unexpected seq: ask for the one we actually want */
    fa_write_control(frame, FA_TYPE_NACK, frame->expected_rx_seq);
    frame->byte_count = 0;
    return FRAME_ARQ_SUCCESS;
}

int frame_arq_discard(frame_arq_handle_t handle) {
    frame_arq_t* frame;
    if (handle == NULL) {
        return FRAME_ARQ_ERROR_ARG;
    }
    frame = (frame_arq_t*)handle;
    frame->byte_count = 0;
    return FRAME_ARQ_SUCCESS;
}

int frame_arq_put(frame_arq_handle_t handle, uint8_t cmd, const void* payload, size_t size) {
    frame_arq_t* frame;
    uint8_t* p;
    int i, res;
    if (handle == NULL) {
        return FRAME_ARQ_ERROR_ARG;
    }
    if (cmd < 1 || cmd > 127) {
        return FRAME_ARQ_ERROR_ARG; /* cmd 0 is reserved for control frames */
    }
    frame = (frame_arq_t*)handle;
    if (size > frame->payload_max_size) {
        return FRAME_ARQ_ERROR_OVERFLOW;
    }
    if (frame->awaiting_ack) {
        return FRAME_ARQ_ERROR_BUSY; /* one frame in flight (stop-and-wait) */
    }

    frame->tx_seq = (uint8_t)((frame->tx_seq + 1) & FA_SEQ_MASK);

    /* build the full framed bytes into the retain buffer */
    p = frame->retain_buffer;
    for (i = 0; i < 8; ++i) p[i] = (uint8_t)(cmd + 128);
    p[8] = FA_SEQBYTE(FA_TYPE_DATA, frame->tx_seq);
    fa_wr_u32(p + 9, (uint32_t)size);
    if (size > 0 && payload != NULL) {
        memcpy(p + FRAME_ARQ_HEADER_LENGTH, payload, size);
    }
    fa_wr_u32(p + 13, fa_frame_crc(p, size));
    frame->retain_length = FRAME_ARQ_HEADER_LENGTH + size;

    /* mark outstanding before streaming so a mid-send failure can still resend */
    frame->awaiting_ack = true;
    frame->resend_needed = false;

    for (i = 0; (size_t)i < frame->retain_length; ++i) {
        res = frame->write_cb(p[i], frame->write_state);
        if (res < 0) return res;
    }
    return FRAME_ARQ_SUCCESS;
}

int frame_arq_resend(frame_arq_handle_t handle) {
    frame_arq_t* frame;
    size_t i;
    int res;
    if (handle == NULL) {
        return FRAME_ARQ_ERROR_ARG;
    }
    frame = (frame_arq_t*)handle;
    if (!frame->awaiting_ack || frame->retain_length == 0) {
        frame->resend_needed = false;
        return FRAME_ARQ_SUCCESS; /* nothing outstanding: no-op */
    }
    for (i = 0; i < frame->retain_length; ++i) {
        res = frame->write_cb(frame->retain_buffer[i], frame->write_state);
        if (res < 0) return res;
    }
    frame->resend_needed = false;
    return FRAME_ARQ_SUCCESS;
}

bool frame_arq_awaiting_ack(frame_arq_handle_t handle) {
    if (handle == NULL) return false;
    return ((frame_arq_t*)handle)->awaiting_ack;
}

bool frame_arq_resend_needed(frame_arq_handle_t handle) {
    if (handle == NULL) return false;
    return ((frame_arq_t*)handle)->resend_needed;
}

/* ------------------------------------------------------------------ */
static void fa_init_state(frame_arq_t* f, size_t max_payload_size,
                          uint8_t* read_buffer, uint8_t* retain_buffer,
                          frame_arq_read_callback_t rcb, void* rstate,
                          frame_arq_write_callback_t wcb, void* wstate) {
    memset(f, 0, sizeof(frame_arq_t));
    f->read_buffer = read_buffer;
    f->retain_buffer = retain_buffer;
    f->payload_max_size = max_payload_size;
    f->read_cb = rcb;
    f->read_state = rstate;
    f->write_cb = wcb;
    f->write_state = wstate;
    f->tx_seq = FA_SEQ_MASK;      /* first put() advances to seq 0 */
    f->expected_rx_seq = 0;
}

frame_arq_handle_t frame_arq_create(size_t max_payload_size,
                            frame_arq_read_callback_t on_read_callback, void* on_read_callback_state,
                            frame_arq_write_callback_t on_write_callback, void* on_write_callback_state) {
    uint8_t* rx_buffer = NULL;
    uint8_t* tx_buffer = NULL;
    frame_arq_t* result = NULL;
    if (on_read_callback == NULL || on_write_callback == NULL) {
        goto error;
    }
    rx_buffer = (uint8_t*)malloc(max_payload_size + FRAME_ARQ_HEADER_LENGTH);
    if (rx_buffer == NULL) goto error;
    tx_buffer = (uint8_t*)malloc(max_payload_size + FRAME_ARQ_HEADER_LENGTH);
    if (tx_buffer == NULL) goto error;
    result = (frame_arq_t*)malloc(sizeof(frame_arq_t));
    if (result == NULL) goto error;
    fa_init_state(result, max_payload_size, rx_buffer, tx_buffer,
                  on_read_callback, on_read_callback_state,
                  on_write_callback, on_write_callback_state);
    return result;
error:
    if (rx_buffer != NULL) free(rx_buffer);
    if (tx_buffer != NULL) free(tx_buffer);
    if (result != NULL) free(result);
    return NULL;
}

frame_arq_handle_t frame_arq_create_za(size_t max_payload_size,
                            frame_arq_t* in_out_frame_state,
                            void* frame_read_buffer,
                            void* frame_retain_buffer,
                            frame_arq_read_callback_t on_read_callback, void* on_read_callback_state,
                            frame_arq_write_callback_t on_write_callback, void* on_write_callback_state) {
    if (on_read_callback == NULL || on_write_callback == NULL ||
        in_out_frame_state == NULL || frame_read_buffer == NULL ||
        frame_retain_buffer == NULL) {
        return NULL;
    }
    fa_init_state(in_out_frame_state, max_payload_size,
                  (uint8_t*)frame_read_buffer, (uint8_t*)frame_retain_buffer,
                  on_read_callback, on_read_callback_state,
                  on_write_callback, on_write_callback_state);
    return in_out_frame_state;
}

void frame_arq_destroy(frame_arq_handle_t handle) {
    frame_arq_t* frame;
    if (handle == NULL) return;
    frame = (frame_arq_t*)handle;
    if (frame->read_buffer != NULL) {
        free(frame->read_buffer);
        frame->read_buffer = NULL;
    }
    if (frame->retain_buffer != NULL) {
        free(frame->retain_buffer);
        frame->retain_buffer = NULL;
    }
    free(frame);
}

int frame_arq_reset(frame_arq_handle_t handle) {
    frame_arq_t* frame;
    if (handle == NULL) {
        return FRAME_ARQ_ERROR_ARG;
    }
    frame = (frame_arq_t*)handle;
    frame->byte_count = 0;
    frame->retain_length = 0;
    frame->tx_seq = FA_SEQ_MASK;      /* first put -> seq 0 */
    frame->expected_rx_seq = 0;
    frame->awaiting_ack = false;
    frame->resend_needed = false;
    return FRAME_ARQ_SUCCESS;
}

