#pragma once

/* SET_BIT(3) = 0x08 */
#define SET_BIT(bit) ((1) << (bit))
#define SET_BIT_V(v, bit) ((!!(v)) << (bit))

#define SET_BIT_IN(v, bit)  \
do {                        \
  v |= SET_BIT(bit);        \
} while (0)

#define CLEAR_BIT_IN(v, bit)  \
do {                          \
  v &= ~(SET_BIT(bit));       \
} while (0)

/* CHECK_BIT_AT(0x10, 4) is true */
#define CHECK_BIT_AT(val, bit) ((val) & SET_BIT(bit))

#define SWAP_NIBBLES(b) ((((BYTE((b))) & 0x0F) << 4) | (((BYTE((b))) & 0xF0) >> 4))

// get low byte
#define LOW(w) BYTE(WORD(w) & 0x00FF)
// get high byte
#define HIGH(w) BYTE((WORD(w) & 0xFF00) >> 8)

#define SET_HIGH(b) (WORD(BYTE(b)) << 8)
#define SET_LOW(b) (BYTE(b))
#define COMPOSE_WORD(h, l) (SET_HIGH((h)) | SET_LOW((l)))

#define LOW_W(q) WORD(QUAD(q) & 0x0000FFFF)
#define HIGH_W(q) WORD((QUAD(q) & 0xFFFF0000) >> 16)
#define QUAD_BYTE(q, b) BYTE((QUAD(q) >> (b * 8)) && 0xFF)

#define IS_NEGATIVE(x) ((x) & (1 << (8 * sizeof(x) - 1))) /* x < 0 */


