/*
  MCU tables for STM32U5
 */

#if defined(STM32U5)

#define STM32_UNKNOWN 0
#define STM32_U575    0x482

mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN,     "STM32U5??"},
    { STM32_U575,        "STM32U575/585"},
};

const mcu_rev_t silicon_revs[] = {
    {0x3007, 'U'},
};

#endif // STM32U5
