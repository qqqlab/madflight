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
    Serial.printf("\n=== PIO RESOURCES ===\n\n");
    Serial.printf("PIO  SM   Name    En Base PLen   Freq     ClkDiv  GPIO-Out   GPIO-Set   GPIO-Side  Wrap   ExecCtrl  PinCtrl\n");
    for(uint p = 0; p < NUM_PIOS; p++) {
        PIO pio = PIO_INSTANCE(p);
        for(uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
            int base = pio_get_gpio_base(pio);
            bool claimed = pio_sm_is_claimed(pio, sm);
            Serial.printf("PIO%d-SM%d  ", p, sm);
            Serial.printf("%-7s  ", (claimed ? pio_registry[p][sm] : "-Free-") );
            Serial.printf("%c  ", ((pio->ctrl >> sm) & 1 ? 'Y' : '-'));
            Serial.printf("%2d  ", base);
            if(claimed) {
                Serial.printf("%2d  ", prog_len[p][sm]);
                float mhz = (float)clock_get_hz(clk_sys) * 65536e-6 / pio->sm[sm].clkdiv;
                Serial.printf("%6.2fMHz  ", mhz);
                Serial.printf("%7.2f  ", pio->sm[sm].clkdiv / 65536.0);
                int out_cnt = (pio->sm[sm].pinctrl & 0x03f00000) >> 20;
                if(out_cnt) {
                    Serial.printf("%2d", base + ((pio->sm[sm].pinctrl & 0x0000001f) >> 0)); //0x0000001f [4:0]   OUT_BASE
                    Serial.printf("(cnt=%1d)  ", out_cnt);
                } else Serial.printf("           ");
                int set_cnt = (pio->sm[sm].pinctrl & 0x1c000000) >> 26; //0x1c000000 [28:26] SET_COUNT
                if(set_cnt) {
                    Serial.printf("%2d", base + ((pio->sm[sm].pinctrl & 0x000003e0) >> 5)); //0x000003e0 [9:5]   SET_BASE
                    Serial.printf("(cnt=%1d)  ", set_cnt);
                } else Serial.printf("           ");
                int side_cnt = (pio->sm[sm].pinctrl & 0xe0000000) >> 29; //0xe0000000 [31:29] SIDESET_COUNT
                if(side_cnt) {
                    Serial.printf("%2d", base + ((pio->sm[sm].pinctrl & 0x00007c00) >> 10)); //0x00007c00 [14:10] SIDESET_BASE
                    Serial.printf("(cnt=%1d)  ", side_cnt);
                } else Serial.printf("           ");
                Serial.printf("%2d-", (pio->sm[sm].execctrl & 0x00000f80) >> 7);  // 0x00000f80 [11:7]  WRAP_BOTTOM  (0x00) After reaching wrap_top, execution is wrapped to this address
                Serial.printf("%-2d  ", (pio->sm[sm].execctrl & 0x0001f000) >> 12); // 0x0001f000 [16:12] WRAP_TOP     (0x1f) After reaching this address, execution is wrapped to wrap_bottom
                Serial.printf("%08X  ", pio->sm[sm].execctrl);
                Serial.printf("%08X  ", pio->sm[sm].pinctrl);
            }
            Serial.println();
        }
    }
    Serial.printf("\n=== DMA RESOURCES ===\n\n");
    Serial.printf("DMA   Status  State Treq   TxCnt  CtrlTrig\n");
    for(uint ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        Serial.printf("DMA%-2d ", ch);
        Serial.printf("%-7s  ", (dma_channel_is_claimed(ch) ? "Claimed" : "-Free-"));
        Serial.printf("%s  ", (dma_channel_is_busy(ch) ? "Busy" : "Idle"));
        Serial.printf("%2d  ", (dma_hw->ch[ch].ctrl_trig & 0x007e0000) >> 17);//0x007e0000 [22:17] TREQ_SEL     (0x00) Select a Transfer Request signal
        Serial.printf("%5d*", dma_hw->ch[ch].transfer_count & 0x0fffffff ); 
        Serial.printf("%1d  ", (int)pow(2, (dma_hw->ch[ch].ctrl_trig & 0x0000000c) >> 2 ));//0x0000000c [3:2]   DATA_SIZE    (0x0) Set the size of each bus transfer (byte/halfword/word)
        Serial.printf("%08X  ", dma_hw->ch[ch].ctrl_trig);
        Serial.println();
    }
}

#endif //#ifdef ARDUINO_ARCH_RP2040
