# AD9837-Based Version

Replaces DAC with AD9837 DDS generator

Maintains identical Julia communication protocol

Uses SPI control via provided AD9837 driver

OLED and UART result display preserved


Advantages:

Higher frequency stability (upto 2MHz)

Lower MCU overhead

Cleaner sine output, suitable for impedance spectroscopy
