# ESP32 Interactive UART Terminal

An ESP32 project implementing an interactive UART command-line interface with LED blinking control and I2S audio tone generation.

## Features

### Blink Control
- `blink on` - Enable LED blinking
- `blink off` - Disable LED blinking
- `blink freq <ms>` - Set blink period in milliseconds

### I2S Audio
- `i2s init <channel> <sample_rate> <bit_width> <stereo/mono>` - Initialize I2S peripheral
  - Example: `i2s init 0 44100 16 stereo`
- `i2s tone <freq>` - Play a tone at specified frequency (100 Hz to 15000 Hz)
- `i2s start` - Start I2S transmission
- `i2s stop` - Stop I2S transmission

### General
- `help` - Display all available commands

## Hardware Requirements

- ESP32 development board
- LED on GPIO2 (built-in on most ESP32 dev boards)
- I2S DAC or amplifier on:
  - BCLK: GPIO26
  - LRCLK/WS: GPIO25
  - DOUT: GPIO22

## ESP-IDF Version

Developed using **ESP-IDF v6.0**

## Build and Flash

1. Set up ESP-IDF environment:
   ```bash
   source ~/.espressif/v6.0/esp-idf/export.sh
   ```

2. Build the project:
   ```bash
   cd blinky
   idf.py build
   ```

3. Flash to ESP32 (replace PORT with your serial port):
   ```bash
   idf.py -p /dev/cu.SLAB_USBtoUART flash
   ```

## Usage

Connect to the ESP32 via serial terminal at 115200 baud:
```bash
screen /dev/cu.SLAB_USBtoUART 115200
```

Type commands and press Enter. Start with `help` to see available commands.

## Project Structure

```
opencode/
├── blinky/
│   ├── main/
│   │   ├── blinky.c          # Main application with UART terminal and I2S
│   │   └── CMakeLists.txt   # Component configuration
│   ├── CMakeLists.txt       # Project configuration
│   └── sdkconfig.defaults  # Default SDK configuration
└── README.md
```

## Console Log Example

```
=== Blinky UART Terminal ===
Type 'help' for commands
> help

Commands:
  blink on       - Enable blinking
  blink off      - Disable blinking
  blink freq <ms>- Set blink period in ms
  i2s init <ch> <sr> <w> <s/m> - Init I2S
  i2s tone <freq>- Play tone 100-15000Hz
  i2s start      - Start I2S
  i2s stop       - Stop I2S
  help           - Show this help
> blink off
Blinking disabled
> blink freq 500
Blink frequency set
> blink on
Blinking enabled
> i2s init 0 44100 16 stereo
I2S initialized
> i2s tone 440
Tone playing
> i2s tone 880
Tone playing
> i2s tone 1760
Tone playing
> i2s stop
I2S stopped
> invalid_command
Unknown command. Type 'help'
> help

Commands:
  blink on       - Enable blinking
  blink off      - Disable blinking
  blink freq <ms>- Set blink period in ms
  i2s init <ch> <sr> <w> <s/m> - Init I2S
  i2s tone <freq>- Play tone 100-15000Hz
  i2s start      - Start I2S
  i2s stop       - Stop I2S
  help           - Show this help
>
```
