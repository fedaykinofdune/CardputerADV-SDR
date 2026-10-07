/* Cardputer ADV keyboard: TCA8418 7x8 matrix scanner on the system I2C bus.
 * Key events are polled from its FIFO; the interrupt line is not needed. */
#include "cardputer.h"
#include "driver/i2c_master.h"
#include "esp_check.h"

#define PIN_SDA 8
#define PIN_SCL 9
#define TCA8418_ADDR 0x34
#define REG_CFG 0x01
#define REG_INT_STAT 0x02
#define REG_KEY_LCK_EC 0x03
#define REG_KEY_EVENT_A 0x04
#define REG_KP_GPIO_1 0x1D
#define REG_KP_GPIO_2 0x1E
#define REG_KP_GPIO_3 0x1F

static i2c_master_dev_handle_t dev;
static bool fn, shift, ready;

/* Logical layout shared with the original Cardputer: 4 rows x 14 columns.
 * Each entry: plain, shifted, Fn layer. 0 marks modifiers/unused keys. */
#define FN_KEY 1
#define SHIFT_KEY 2
static const int keymap[4][14][3] = {
    {{'`', '~', CP_KEY_ESC}, {'1', '!', 0}, {'2', '@', 0}, {'3', '#', 0}, {'4', '$', 0}, {'5', '%', 0},
     {'6', '^', 0}, {'7', '&', 0}, {'8', '*', 0}, {'9', '(', 0}, {'0', ')', 0}, {'-', '_', 0},
     {'=', '+', 0}, {CP_KEY_BACKSPACE, CP_KEY_BACKSPACE, CP_KEY_DEL}},
    {{CP_KEY_TAB, CP_KEY_TAB, 0}, {'q', 'Q', 0}, {'w', 'W', 0}, {'e', 'E', 0}, {'r', 'R', 0}, {'t', 'T', 0},
     {'y', 'Y', 0}, {'u', 'U', 0}, {'i', 'I', 0}, {'o', 'O', 0}, {'p', 'P', 0}, {'[', '{', 0},
     {']', '}', 0}, {'\\', '|', 0}},
    {{-FN_KEY, -FN_KEY, -FN_KEY}, {-SHIFT_KEY, -SHIFT_KEY, -SHIFT_KEY}, {'a', 'A', 0}, {'s', 'S', 0},
     {'d', 'D', 0}, {'f', 'F', 0}, {'g', 'G', 0}, {'h', 'H', 0}, {'j', 'J', 0}, {'k', 'K', 0},
     {'l', 'L', 0}, {';', ':', CP_KEY_UP}, {'\'', '"', 0}, {CP_KEY_ENTER, CP_KEY_ENTER, CP_KEY_ENTER}},
    {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {'z', 'Z', 0}, {'x', 'X', 0}, {'c', 'C', 0}, {'v', 'V', 0},
     {'b', 'B', 0}, {'n', 'N', 0}, {'m', 'M', 0}, {',', '<', CP_KEY_LEFT}, {'.', '>', CP_KEY_DOWN},
     {'/', '?', CP_KEY_RIGHT}, {' ', ' ', ' '}},
};

static esp_err_t wr(uint8_t reg, uint8_t val) {
    const uint8_t b[2] = {reg, val};
    return i2c_master_transmit(dev, b, 2, 20);
}
static int rd(uint8_t reg) {
    uint8_t v;
    if (i2c_master_transmit_receive(dev, &reg, 1, &v, 1, 20) != ESP_OK) return -1;
    return v;
}

void cp_kbd_init(void) {
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1, .sda_io_num = PIN_SDA, .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = TCA8418_ADDR, .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));
    /* Rows R0..R6 and columns C0..C7 form the matrix; event FIFO enabled. */
    ready = wr(REG_KP_GPIO_1, 0x7f) == ESP_OK && wr(REG_KP_GPIO_2, 0xff) == ESP_OK &&
            wr(REG_KP_GPIO_3, 0x00) == ESP_OK && wr(REG_CFG, 0x01) == ESP_OK;
    while (ready && rd(REG_KEY_EVENT_A) > 0) {}
    if (ready) wr(REG_INT_STAT, 0x03);
}

int cp_kbd_read(void) {
    if (!ready) return 0;
    for (;;) {
        int count = rd(REG_KEY_LCK_EC);
        if (count <= 0 || !(count & 0x0f)) {
            wr(REG_INT_STAT, 0x03);
            return 0;
        }
        int ev = rd(REG_KEY_EVENT_A);
        if (ev <= 0) return 0;
        bool pressed = ev & 0x80;
        unsigned raw = (unsigned)(ev & 0x7f) - 1u, r = raw / 10, c = raw % 10;
        /* Hardware row/column to the logical 4x14 grid (as M5Cardputer). */
        unsigned col = r * 2 + (c > 3), row = c % 4;
        if (row > 3 || col > 13) continue;
        int k = keymap[row][col][0];
        if (k == -FN_KEY) {fn = pressed; continue;}
        if (k == -SHIFT_KEY) {shift = pressed; continue;}
        if (!pressed) continue;
        int out = keymap[row][col][fn ? 2 : shift ? 1 : 0];
        if (out) return out;
    }
}
