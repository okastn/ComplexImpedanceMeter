# Complex Impedance Meter

An  impedance measurement system combining real-time data acquisition on an STM32F7 microcontroller with FFT-based signal processing and circuit parameter estimation in Julia.

The project includes two STM32 implementations:

DAC-based signal generation using the STM32 internal DAC

DDS-based signal generation using an AD9837 waveform generator chip

Both communicate with Julia for impedance calculation and display results on an OLED (SSD1306) screen.

### Functional Flow:

STM32 generates a sine excitation (via DAC or AD9837 DDS)

Analogue response signals are measured using dual ADCs (V1, V2)

ADC data is transmitted over UART to Julia

Julia performs FFT-based amplitude and phase extraction

Impedance and circuit parameters are computed

Results are sent back to STM32 and displayed on an OLED

OLED Display Example:

F=1000Hz SE=0.04

|Z|=1023.2   Ang=-45.3

Z=720-720j

Rs=720.0

Cs=2.34uF




