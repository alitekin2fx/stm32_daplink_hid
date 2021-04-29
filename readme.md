# Yet another STM32 CMSIS-DAP project for debugging ARM Cortex based microcontrollers.
Please follow the repository commit history for the integration steps.

# References
https://github.com/rkprojects/openlink-v1-cmsis-dap<br>
http://wiki.geniekits.com/usb_express/cmsis-dap<br>

# Configure Eclipse debugger for debugging with CMSIS-DAP
1. Goto "Run"->"Debug Configurations..."->"Debugger"<br>
2. Add "-s /usr/share/openocd/scripts" to the "OpenOCD Options"<br>
3. Set "User Defined" radio button<br>
4. Edit *Debug.cfg file and apply below changes<br>
-source [find interface/stlink.cfg]<br>
+source [find interface/cmsis-dap.cfg]<br>
-transport select "hla_swd"<br>
+transport select swd<br>

# Program and verify, verify and reset are optional.
openocd -f openocd-daplink.cfg -c "program filename.elf verify reset exit"<br>
openocd -f openocd-daplink.cfg -c "program filename.bin exit 0x08000000"<br>

