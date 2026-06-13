#pragma once
// ============================================================
// dfu_jump.h — Reliable software entry into the STM32 ROM DFU
//
// Lets you enter upload mode over USB instead of grounding B0.
//
// WHY THE TWO-STEP DANCE:
//   Jumping to the ROM bootloader while USB is already running is
//   unreliable — the dirty USB state makes the jump bounce back
//   into the app. Instead we:
//     1. jumpToBootloader(): set a magic flag in an RTC backup
//        register (survives reset), then do a clean NVIC reset.
//     2. _dfu_boot_check(): a high-priority constructor that runs
//        EARLY on the next boot — before the USB/CDC stack starts —
//        sees the flag, and jumps to the ROM bootloader from a
//        clean state. That makes the jump stick.
//
// USAGE (header-only):
//   #include "dfu_jump.h"
//   ... call jumpToBootloader() from a serial command ('D').
//
// After it runs, the board enumerates as "STM32 BOOTLOADER" and
// `pio run -t upload` (dfu) finds it. No B0 jumper needed.
//
// Fallback: if firmware ever hangs before it can run this, force
// DFU the hardware way (B0 -> 3V, tap RESET).
// ============================================================

#include <Arduino.h>

// STM32F405 system memory (ROM bootloader) entry point.
#ifndef STM32_SYSMEM_ADDR
#define STM32_SYSMEM_ADDR 0x1FFF0000UL
#endif

// Magic flag + which backup register to stash it in.
// RTC has 20 backup regs (BKP0R..BKP19R); BKP19R is the last and
// least likely to collide with anything else.
#define DFU_MAGIC       0xB00710ADUL
#define DFU_BKP_REG     (RTC->BKP19R)

// Enable write access to the RTC backup registers (PWR clock + DBP).
static inline void _dfu_bkp_unlock(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;   // clock the PWR block
    __DSB();
    PWR->CR |= PWR_CR_DBP;               // disable backup write protection
}

// Step 1 — request DFU: stash the flag and reset cleanly.
static inline void jumpToBootloader(void) {
    Serial.println(F("[DFU] Rebooting into bootloader — run 'upload' now."));
    Serial.flush();
    delay(50);

    _dfu_bkp_unlock();
    DFU_BKP_REG = DFU_MAGIC;
    NVIC_SystemReset();   // clean reset; flag is read on the way back up
}

// Step 2 — early boot check. constructor(101) runs during
// __libc_init_array, BEFORE the USB/CDC global constructors, so the
// jump happens from a clean state and actually sticks.
__attribute__((constructor(101)))
static void _dfu_boot_check(void) {
    _dfu_bkp_unlock();
    if (DFU_BKP_REG == DFU_MAGIC) {
        DFU_BKP_REG = 0;   // clear first -> next boot is normal, no loop

        // Jump to the ROM bootloader. At this point no peripherals or
        // interrupts are configured yet, so no deinit is needed.
        __set_MSP(*(volatile uint32_t *)STM32_SYSMEM_ADDR);
        void (*bootJump)(void) =
            (void (*)(void))(*(volatile uint32_t *)(STM32_SYSMEM_ADDR + 4));
        bootJump();
        while (1) { /* never returns */ }
    }
}
