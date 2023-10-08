# Krabi PAMIs

## board
electronics schematics & PCB designs

###
Elements:
- an STM32L0
- an 3.3V power supply
- a full H Drive driver for 2 motors (1A max)
- 4 LEDS outputs connected 2 the 2 motors PWM
- A switch connector
- 2 connectors for VL53L1X ToF sensors
- 2 connectors for CNY70 for line following
- A power connector with 9V for electronics & power and 3V to raise voltage up to 12V for motors, as well as AU connecting ground
- A STM32 debug connector
- A start connector that will inhibit H drives & CNY70 emitters when grounded. Pulled up to 3.3V

