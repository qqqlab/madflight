#pragma once

#include <Arduino.h>

void pio_registry_name_unclaimed(const char* name);
bool pio_registry_claim(const char* name, const pio_program_t *program, PIO *pio, uint *sm, uint *offset, uint gpio_base, uint gpio_count, bool set_gpio_base);
void pio_registry_print();
