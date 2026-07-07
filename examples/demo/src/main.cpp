// THIS IS THE ARDUINO source file
#include <Arduino.h>
#include "frame_arq.h"
#include "interface_buffers.h"
// uncomment to test corruption on TX
// #define TEST_CORRUPTION

// How long to wait for an ACK on an outbound response before retransmitting it.
// Per the 1-with-3 model this never gives up and never advances the sequence:
// we keep retrying until the ACK lands or the ESP resets on reconnect. It must
// sit comfortably under the PC's reply watchdog so a lost response gets several
// retransmit attempts before the PC stops waiting.
#define ACK_TIMEOUT_MS 500

static int test_corruption=0;

static frame_arq_handle_t frame_handle = NULL;

static int serial_read(void* state) {
    (void)state;
    return Serial.read();
}
static int serial_write(uint8_t value, void* state) {
    (void)state;
#ifdef TEST_CORRUPTION
    if(test_corruption) {
        if(test_corruption==1) {
            test_corruption=0;
            Serial.write("\nTest corruption in stream\n");
        } else if(test_corruption>0) {
            --test_corruption;
        }
    }
#endif
    Serial.write((uint8_t)value);
    return 0;
}

void setup() {
    Serial.begin(115200);
    frame_handle = frame_arq_create(INTERFACE_MAX_SIZE, serial_read, NULL, serial_write, NULL);
}

typedef struct {
    uint8_t* ptr;
    size_t remaining;
} buffer_write_cursor_t;
typedef struct {
    const uint8_t* ptr;
    size_t remaining;
} buffer_read_cursor_t;
int on_write_buffer(uint8_t value, void* state) {
    buffer_write_cursor_t* cur = (buffer_write_cursor_t*)state;
    if(cur->remaining==0) {
        return BUFFERS_ERROR_EOF;
    }    
    *cur->ptr++=value;
    --cur->remaining;
    return 1;
}
int on_read_buffer(void* state) {
    buffer_read_cursor_t* cur = (buffer_read_cursor_t*)state;
    if(cur->remaining==0) {
        return BUFFERS_EOF;
    }
    uint8_t result = *cur->ptr++;
    --cur->remaining;
    return result;
}

// A single outstanding outbound response. This is enough for a request/response
// peer that waits for each reply before issuing the next request (as the demo PC
// does), so the ESP never has two responses in flight. A peer that pipelined
// requests would need a queue here instead of one slot.
static uint8_t resp_buffer[INTERFACE_MAX_SIZE];
static size_t  resp_len = 0;
static uint8_t resp_cmd = 0;
static bool    resp_pending = false;
static uint32_t last_send_ms = 0;

static void queue_response(uint8_t cmd, int count) {
    if (count < 0) {
        return; // message writer failed; nothing to send
    }
    resp_cmd = cmd;
    resp_len = (size_t)count;
    resp_pending = true;
}

// Process one received command. Responses are written into resp_buffer and queued
// (not sent inline) so the loop can honor stop-and-wait on the outbound side.
static void handle_command(int cmd, void* ptr, size_t length) {
    buffer_read_cursor_t read_cur = {(const uint8_t*)ptr, length};
    buffer_write_cursor_t write_cur = {resp_buffer, INTERFACE_MAX_SIZE};
    switch((st_message_command_t)cmd) {
        case CMD_RNG: {
            st_rng_message_t msg;
            if(-1<st_rng_message_read(&msg,on_read_buffer,&read_cur)) {
                Serial.println("RNG generation requested");
                st_rng_response_message_t resp;
                randomSeed(millis());
                for(size_t i = 0;i<msg.count;++i) {
                    resp.values[i] = random();
                }
                resp.values_size = msg.count;
                int count = st_rng_response_message_write(&resp,on_write_buffer,&write_cur);
                queue_response(CMD_RNG_RESPONSE, count);
            }
        }
        break;
        case CMD_GPIO_GET: {
            st_gpio_get_message_t msg;
            uint64_t result = 0;
            if(-1<st_gpio_get_message_read(&msg,on_read_buffer,&read_cur)) {
                for(int i = 0; i<64;++i) {
                    if(0!=(msg.mask & (((uint64_t)1)<<i))) {
                        Serial.print("GPIO get request for ");
                        Serial.println((int)i);
                        if(digitalRead(i)==HIGH) {
                            result |= (((uint64_t)1)<<i);
                        }
                    }
                }
                st_gpio_get_response_message_t resp;
                resp.values = result;
                int count = st_gpio_get_response_message_write(&resp,on_write_buffer,&write_cur);
                queue_response(CMD_GPIO_GET_RESPONSE, count);
            }
        }
        break;
        case CMD_GPIO_SET: {
            st_gpio_set_message_t msg;
            if(-1<st_gpio_set_message_read(&msg,on_read_buffer,&read_cur)) {
                for(int i = 0; i<64;++i) {
                    uint64_t mask_cmp = (((uint64_t)1)<<i);
                    if(0!=(msg.mask & mask_cmp)) {
                        Serial.print("GPIO set level request for ");
                        Serial.print((int)i);
                        if(0!=(msg.values & mask_cmp)) {
                            Serial.println(" to on");
                            digitalWrite(i,HIGH);
                        } else {
                            Serial.println(" to off");
                            digitalWrite(i,LOW);
                        }
                    }
                }
            }
        }
        break;
        case CMD_GPIO_MODE: {
            st_gpio_mode_message_t msg;
            if(-1<st_gpio_mode_message_read(&msg,on_read_buffer,&read_cur)) {
                Serial.print("GPIO set mode for ");
                Serial.println((int)msg.gpio);
                switch(msg.mode) {
                    case MODE_INPUT:
                        pinMode(msg.gpio,INPUT);
                        break;
                    case MODE_INPUT_PULLUP:
                        pinMode(msg.gpio,INPUT_PULLUP);
                    break;
                    case MODE_INPUT_PULLDOWN:
                        pinMode(msg.gpio,INPUT_PULLDOWN);
                        break;
                    case MODE_OUTPUT:
                        pinMode(msg.gpio,OUTPUT);
                        break;
                    case MODE_OUTPUT_OPEN_DRAIN:
                        pinMode(msg.gpio,OUTPUT_OPEN_DRAIN);
                        break;
                }
            }
        }
        break;

        default: {
            Serial.print("Unknown command received ");
            Serial.println((int)cmd);
        }
        break;
    }
}
void loop() {
    void* ptr;
    size_t length;
    static bool testing_corruption = false;
    if(!testing_corruption) {
        testing_corruption = true;
        if((random()&1)==1) {
            test_corruption = random() % (FRAME_ARQ_HEADER_LENGTH+1);
        }
    }
    // 1) Pump the receiver. get() auto-emits ACK/NACK for inbound frames; here we
    //    only act on a delivered command or a NACK against our outstanding response.
    int cmd = frame_arq_get(frame_handle, &ptr, &length);
    if (cmd == FRAME_ARQ_RESEND_NEEDED) {
        // The PC NACKed our outstanding response: retransmit it immediately.
        frame_arq_resend(frame_handle);
        last_send_ms = millis();
    } else if (cmd > 0) {
        handle_command(cmd, ptr, length);
    }
    // cmd == 0: nothing waiting, or an ACK/control frame was handled internally.
    // cmd < -1: a corrupt frame was seen; get() already auto-NACKed. Nothing to do.

    // 2) Send a queued response once the previous one has been acknowledged.
    if (resp_pending && !frame_arq_awaiting_ack(frame_handle)) {
        int r = frame_arq_put(frame_handle, resp_cmd, resp_buffer, resp_len);
        if (r == FRAME_ARQ_SUCCESS) {
            resp_pending = false;
            last_send_ms = millis();
            testing_corruption = false;
        } else if (r == FRAME_ARQ_ERROR_BUSY) {
            // still awaiting a prior ACK; leave it queued and retry next loop
        } else {
            resp_pending = false; // unexpected (e.g. oversized); drop rather than spin
            testing_corruption = false;
        }
    }

    // 3) Caller-owned retransmit timer. Per 1-with-3 this keeps retrying forever;
    //    it never gives up and never advances the sequence. A dead link is cleared
    //    when the PC reconnects and the ESP resets.
    if (frame_arq_awaiting_ack(frame_handle) &&
        (uint32_t)(millis() - last_send_ms) >= ACK_TIMEOUT_MS) {
        testing_corruption = false;
        frame_arq_resend(frame_handle);
        last_send_ms = millis();
    }
    
}