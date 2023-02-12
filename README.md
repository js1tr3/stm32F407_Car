# stm32F407_Car
Follow setup directions here
https://www.mathworks.com/help/supportpkg/stmicroelectronicsstm32f4discovery/ug/Getting-started-stm32cubemx.html
Board is the IOT development board with can and ethernet


 [Development board overview] The development board uses STM32F407VET6 chip as the main controller, M4 super strong core, expansion with 1 RS232 interface, 1 RS485 interface, 2 CAN interface, 1 Ethernet interface, 1 USB HOST interface, 1 USB DEVICE interface, three user buttons, three user indicators. Very suitable for IoT application development, gateway server development, bus communication development and industrial control.
 
  Master MCU STM32F407VET6 -LQFP100
  Ethernet chip DP83848-LQFP48
  Serial port 1 RS232 interface with serial port 1 PA9 PA10
  RS485 interface 1 RS485 interface with serial port 2 PD5 PD6
  CAN interface 2 CAN interfaces with chip PD0 PD1 / PB5 PB6
  TF card 9 pin small card, controlled by PC and PD pins
  Wireless interface P3 leads to NRF24L01 interface
  USB interface: one master USB and one slave USB
  Storage with 24C02 and 25Q64 memory chips
  Temperature sensor interface leads to the DS18B20 sensor interface
  JTAG interface JTAG corresponding to JLINK
Buttons K1, K2, K3 Three user buttons
Indicator LED1, LED2, LED3 three user indicators 
