#ifdef ARDUINO_ARCH_RP2040

#include "pio_registry.h"

void pio_registry_unclaim_unnamed();
void pio_registry_register(PIO pio, uint sm, const char* name, int8_t len);

//Note: NUM_PIOS, NUM_PIO_STATE_MACHINES are defined in platform_defs.h

char pio_registry[NUM_PIOS][NUM_PIO_STATE_MACHINES][8] = {};
int8_t prog_len[NUM_PIOS][NUM_PIO_STATE_MACHINES] = {};

//workaround for pico-sdk v2.2.0, see https://github.com/raspberrypi/pico-sdk/pull/2792
//bool pio_claim_free_sm_and_add_program_for_gpio_range(const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base, uint gpio_count, bool set_gpio_base)
bool pio_registry_claim(const char* name, const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base, uint gpio_count, bool set_gpio_base) {
    char name2[8] = {};
    strncpy(name2 + 1, name, 6);

    // name any unnamed claimed items before the claim
    name2[0] = '1';
    pio_registry_name_unclaimed(name2);

    // claim
    bool rv = pio_claim_free_sm_and_add_program_for_gpio_range(program, pio, sm, offset, gpio_base, gpio_count, set_gpio_base);
    if(!rv) return rv;

    // name the claimed item
    pio_registry_register(*pio, *sm, name, program->length);

    #if PICO_RP2350 && !PICO_RP2350A
        //RP2350B unclaim unnamed registry entries - these are claims made by buggy pio_claim_free_sm_and_add_program_for_gpio_range()
        pio_registry_unclaim_unnamed();
    #endif

    // name any unnamed claimed items (should not be any)
    name2[0] = '2';
    pio_registry_name_unclaimed(name2);

    return rv;
}

//name unnamed registry entries
void pio_registry_name_unclaimed(const char* name) {
    for(uint p = 0; p < NUM_PIOS; p++) {
        PIO pio = PIO_INSTANCE(p);
        for(uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
            if(pio_sm_is_claimed(pio, sm) && !pio_registry[p][sm][0]) {
                pio_registry_register(pio, sm, name, -1);
            }
        }
    }
}

//unclaim unnamed registry entries
void pio_registry_unclaim_unnamed() {
    for(uint p = 0; p < NUM_PIOS; p++) {
        PIO pio = PIO_INSTANCE(p);
        for(uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
            if(pio_sm_is_claimed(pio, sm) && !pio_registry[p][sm][0]) {
                pio_sm_unclaim(pio, sm);
            }
        }
    }
}

//name an entry
void pio_registry_register(PIO pio, uint sm, const char* name, int8_t len) {
    strncpy(pio_registry[PIO_NUM(pio)][sm], name, 7);
    prog_len[PIO_NUM(pio)][sm] = len;
}

//show list of SM
void pio_registry_print() {
    Serial.printf("\nPIO RESOURCES\n");
    for(uint p = 0; p < NUM_PIOS; p++) {
        PIO pio = PIO_INSTANCE(p);
        for(uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
            int base = pio_get_gpio_base(pio);
            bool claimed = pio_sm_is_claimed(pio, sm);
            Serial.printf("  pio%d-sm%d ", p, sm);
            Serial.printf("%-10s %-7s ", (claimed ? "claimed by" : "free"), pio_registry[p][sm]);
            Serial.printf("base=%2d ", base);
            if(claimed) {
                Serial.printf("prog_len=%2d ", prog_len[p][sm]);
                float mhz = (float)clock_get_hz(clk_sys) * 65536e-6 / pio->sm[sm].clkdiv;
                Serial.printf("%6.2fMHz ", mhz);
                //Serial.printf("addr=%02X ", pio->sm[sm].addr);
                Serial.printf("clkdiv=%7.2f ", pio->sm[sm].clkdiv / 65536.0);
                Serial.printf("execctrl=%08X ", pio->sm[sm].execctrl);
                Serial.printf("pinctrl=%08X ", pio->sm[sm].pinctrl);
                Serial.printf("gpio_out=%2d",    base + ((pio->sm[sm].pinctrl & 0x0000001f) >> 0));
                Serial.printf("(cnt=%d) ",       (pio->sm[sm].pinctrl & 0x03f00000) >> 20);
                Serial.printf("gpio_side=%2d",   base + ((pio->sm[sm].pinctrl & 0x000003e0) >> 5));
                Serial.printf("(cnt=%d) ",      (pio->sm[sm].pinctrl & 0x1c000000) >> 26);
                Serial.printf("wrap=%2d-", (pio->sm[sm].execctrl & 0x00000f80) >> 7);
                Serial.printf("%2d ", (pio->sm[sm].execctrl & 0x0001f000) >> 12);
                Serial.printf("enabled=%d ", (pio->ctrl >> sm) & 1);
            }
            Serial.println();
        }
    }
    Serial.printf("\nDMA RESOURCES\n");
    for(uint ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        Serial.printf("  dma%02d ", ch);
        Serial.printf("%-7s ", (dma_channel_is_claimed(ch) ? "claimed" : "free"));
        Serial.printf("%s ", (dma_channel_is_busy(ch) ? "busy" : "idle"));
        Serial.println();
    }
}

#endif //#ifdef ARDUINO_ARCH_RP2040
