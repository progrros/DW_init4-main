# STM32 DWM3000 Positioning System

This project implements a UWB (Ultra-Wideband) positioning system using an STM32 microcontroller and the DWM3000 module.

## 📦 Repository Structure

example_selection.h # Defines the device role

config_option.c # Contains UWB configuration parameters

ds_twr_initiator_sts.c # Initiator configuration file

## 🛠️ Configuration

### 1. Set Device Role
Define your device's role in `example_selection.h`:

#define INITIATOR_TYPE 'E'  // 'E' or 'F'

### 2. Adjust the UWB settings in config_option.c:

## UWB Configuration Options

The system uses the following default configuration in `config_option.c`:

```c
dwt_config_t config_options = {
    .chan = 5,               // Channel number (5 or 9)
    .txPreambLength = DWT_PLEN_1024,  // Preamble length
    .rxPAC = DWT_PAC32,      // Preamble acquisition chunk size
    .txCode = 9,             // TX preamble code
    .rxCode = 9,             // RX preamble code
    .nsSFD = 3,              // SFD mode
    .dataRate = DWT_BR_850K,  // Data rate
    .phrMode = DWT_PHRMODE_STD, // PHY header mode
    .phrRate = DWT_PHRRATE_STD, // PHY header rate
    .sfdTO = (1024 + 1 + 8 - 8), // SFD timeout
    .stsMode = DWT_STS_MODE_1, // STS mode
    .stsLength = DWT_STS_LEN_128, // STS length
    .pdoaMode = DWT_PDOA_M0   // PDOA mode
};

## 

Configuration Notes:
Channel Number: Choose between 5 or 9 based on regional regulations

Preamble Length: Higher values (e.g., DWT_PLEN_1024) increase range but may reduce throughput

If you use DWT_PLEN_1024 instead of DWT_PLEN_128, you're increasing the preamble by:

1024 - 128 = 896 symbols

Given that each symbol at PRF 64 MHz is approximately 1017.63 ns, the total additional preamble time is:

896 × 1017.63 ns ≈ 911,798 ns ≈ 912 µs

and so on.

### 3. Build and Flash
Use your preferred STM32 development environment to compile and flash the firmware.