/**
 * sensors/gnss_driver.c — GNSS Driver (u-blox M9/M10, UBX binary)
 *
 * Parses NAV-PVT messages streamed at 10 Hz over UART0.
 * Also configures the receiver on startup via UBX-CFG.
 * Runs in Core 1 polling loop.
 */

#include "sensor_hal.h"
#include "../comms/comms.h"

/* ─── UBX DEFINES ────────────────────────────────────────── */
#define UBX_HDR0            0xB5
#define UBX_HDR1            0x62
#define UBX_CLS_NAV         0x01
#define UBX_CLS_CFG         0x06
#define UBX_ID_NAV_PVT      0x07
#define UBX_ID_CFG_RATE     0x08
#define UBX_ID_CFG_PRT      0x00
#define UBX_ID_CFG_MSG      0x01
#define UBX_NAV_PVT_LEN     92u

/* Minimum fix quality to trust */
#define GNSS_MIN_FIX        GNSS_FIX_3D

/* ─── UBX CHECKSUM ───────────────────────────────────────── */
static void ubx_cksum(const uint8_t *payload, uint16_t len,
                       uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += payload[i];
        *ck_b += *ck_a;
    }
}

/* ─── SEND UBX COMMAND ───────────────────────────────────── */
static void ubx_send(uint8_t cls, uint8_t id,
                      const uint8_t *payload, uint16_t len) {
    uint8_t hdr[6] = { UBX_HDR0, UBX_HDR1, cls, id,
                        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uart_write_blocking(COMMS_UART_GNSS, hdr, 6);
    if (len) uart_write_blocking(COMMS_UART_GNSS, payload, len);
    uint8_t ck_a, ck_b;
    /* Checksum over class+id+length+payload */
    uint8_t tmp[4] = { cls, id, (uint8_t)(len&0xFF), (uint8_t)(len>>8) };
    ubx_cksum(tmp, 4, &ck_a, &ck_b);
    if (len) { /* add payload to running sum */
        uint8_t a = ck_a, b = ck_b;
        for (uint16_t i = 0; i < len; i++) { a += payload[i]; b += a; }
        ck_a = a; ck_b = b;
    }
    uint8_t ck[2] = { ck_a, ck_b };
    uart_write_blocking(COMMS_UART_GNSS, ck, 2);
}

/* ─── RECEIVER CONFIGURATION ─────────────────────────────── */
static void gnss_configure(void) {
    /* Set measurement rate: 100 ms (10 Hz), GPS time ref */
    uint8_t cfg_rate[] = { 0x64, 0x00,   /* measRate = 100 ms */
                            0x01, 0x00,   /* navRate  = 1      */
                            0x01, 0x00 }; /* timeRef  = GPS    */
    ubx_send(UBX_CLS_CFG, UBX_ID_CFG_RATE, cfg_rate, sizeof(cfg_rate));
    sleep_ms(100);

    /* Enable NAV-PVT on UART1 at 1 message per epoch */
    uint8_t cfg_msg[] = { UBX_CLS_NAV, UBX_ID_NAV_PVT, 0,1,0,0,0,0 };
    ubx_send(UBX_CLS_CFG, UBX_ID_CFG_MSG, cfg_msg, sizeof(cfg_msg));
    sleep_ms(50);

    /* Disable NMEA on UART to reduce bandwidth */
    /* (omitted for brevity — set via CFG-PRT port protocol mask) */
}

/* ─── NAV-PVT PARSER ─────────────────────────────────────── */
static bool parse_nav_pvt(const uint8_t *payload, GnssMeas *out) {
    /* Payload offsets per u-blox M9 Integration Manual */
    out->pos.lat_deg  = (int32_t)(payload[28] | payload[29]<<8 |
                                   payload[30]<<16 | payload[31]<<24) * 1e-7;
    out->pos.lon_deg  = (int32_t)(payload[24] | payload[25]<<8 |
                                   payload[26]<<16 | payload[27]<<24) * 1e-7;
    out->pos.alt_m    = (int32_t)(payload[36] | payload[37]<<8 |
                                   payload[38]<<16 | payload[39]<<24) * 1e-3f;

    out->vel_ned.x    = (int32_t)(payload[48] | payload[49]<<8 |
                                   payload[50]<<16 | payload[51]<<24) * 1e-3f; /* N */
    out->vel_ned.y    = (int32_t)(payload[52] | payload[53]<<8 |
                                   payload[54]<<16 | payload[55]<<24) * 1e-3f; /* E */
    out->vel_ned.z    = (int32_t)(payload[56] | payload[57]<<8 |
                                   payload[58]<<16 | payload[59]<<24) * 1e-3f; /* D */

    out->h_acc_m      = (uint32_t)(payload[40]|payload[41]<<8|
                                    payload[42]<<16|payload[43]<<24) * 1e-3f;
    out->v_acc_m      = (uint32_t)(payload[44]|payload[45]<<8|
                                    payload[46]<<16|payload[47]<<24) * 1e-3f;
    out->vel_acc_mss  = (uint32_t)(payload[68]|payload[69]<<8|
                                    payload[70]<<16|payload[71]<<24) * 1e-3f;

    out->fix_type     = (GnssFix)payload[20];
    out->num_sv       = payload[23];
    out->ts_us        = time_us_64();

    /* Quality gates */
    out->valid = (out->fix_type >= GNSS_MIN_FIX)
              && (out->num_sv  >= NAV_GNSS_MIN_SATS)
              && (out->h_acc_m <= NAV_GNSS_MAX_HACC_M);
    return out->valid;
}

/* ─── FRAME STATE MACHINE ────────────────────────────────── */
typedef enum {
    UBX_SM_SYNC1, UBX_SM_SYNC2,
    UBX_SM_CLASS, UBX_SM_ID,
    UBX_SM_LEN_L, UBX_SM_LEN_H,
    UBX_SM_PAYLOAD, UBX_SM_CKA, UBX_SM_CKB
} UbxSmState;

static struct {
    UbxSmState  st;
    uint8_t     cls, id;
    uint16_t    len, idx;
    uint8_t     payload[UBX_NAV_PVT_LEN + 4];
    uint8_t     ck_a_rx, ck_b_rx;
} s_ubx;

/* Feed one byte from UART ring buffer */
static void gnss_feed_byte(uint8_t b) {
    switch (s_ubx.st) {
    case UBX_SM_SYNC1:
        if (b == UBX_HDR0) s_ubx.st = UBX_SM_SYNC2; break;
    case UBX_SM_SYNC2:
        s_ubx.st = (b == UBX_HDR1) ? UBX_SM_CLASS : UBX_SM_SYNC1; break;
    case UBX_SM_CLASS:
        s_ubx.cls = b; s_ubx.st = UBX_SM_ID; break;
    case UBX_SM_ID:
        s_ubx.id = b; s_ubx.st = UBX_SM_LEN_L; break;
    case UBX_SM_LEN_L:
        s_ubx.len = b; s_ubx.st = UBX_SM_LEN_H; break;
    case UBX_SM_LEN_H:
        s_ubx.len |= (uint16_t)(b << 8);
        s_ubx.idx  = 0;
        if (s_ubx.len == 0) { s_ubx.st = UBX_SM_CKA; break; }
        if (s_ubx.len > sizeof(s_ubx.payload)) { s_ubx.st = UBX_SM_SYNC1; break; }
        s_ubx.st = UBX_SM_PAYLOAD;
        break;
    case UBX_SM_PAYLOAD:
        s_ubx.payload[s_ubx.idx++] = b;
        if (s_ubx.idx >= s_ubx.len) s_ubx.st = UBX_SM_CKA;
        break;
    case UBX_SM_CKA:
        s_ubx.ck_a_rx = b; s_ubx.st = UBX_SM_CKB; break;
    case UBX_SM_CKB: {
        s_ubx.ck_b_rx = b;
        s_ubx.st = UBX_SM_SYNC1;
        /* Verify checksum */
        uint8_t a = 0, bv = 0;
        uint8_t hdr[4] = { s_ubx.cls, s_ubx.id,
                            (uint8_t)(s_ubx.len & 0xFF),
                            (uint8_t)(s_ubx.len >> 8) };
        for (int i = 0; i < 4; i++) { a += hdr[i]; bv += a; }
        for (uint16_t i = 0; i < s_ubx.len; i++) {
            a += s_ubx.payload[i]; bv += a;
        }
        if (a != s_ubx.ck_a_rx || bv != s_ubx.ck_b_rx) break;

        /* Dispatch valid message */
        if (s_ubx.cls == UBX_CLS_NAV && s_ubx.id == UBX_ID_NAV_PVT &&
            s_ubx.len == UBX_NAV_PVT_LEN) {
            parse_nav_pvt(s_ubx.payload, &g_sensors.gnss);
        }
        break;
    }
    }
}

/* ─── PUBLIC API ─────────────────────────────────────────── */
bool gnss_init(void) {
    gnss_configure();
    memset(&s_ubx, 0, sizeof(s_ubx));
    s_ubx.st = UBX_SM_SYNC1;
    return true;
}

/* Called in Core 1 loop — drain UART ring into state machine */
void gnss_poll(void) {
    uint8_t b;
    while (uart_ring_pop(&g_ring_gnss, &b)) {
        gnss_feed_byte(b);
    }
}

/* Check for GNSS timeout */
bool gnss_is_healthy(float now_s) {
    return g_sensors.gnss.valid &&
           (now_s - g_nav.gnss_last_fix_s) < (NAV_GNSS_TIMEOUT_MS * 1e-3f);
}
