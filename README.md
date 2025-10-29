# Complex Impedance Meter

An  impedance measurement system combining real-time data acquisition on an STM32 microcontroller with FFT-based signal processing and circuit parameter estimation in Julia.

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


