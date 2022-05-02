# iceduino-neorv32
 NEORV32 setup for Iceduino board
## Folder structure
* neorv32: [NEORV32 version 1.6.9](https://github.com/stnolting/neorv32)
* osflow: Setup for Iceduino board
* simulation: Testbench for simulation
* sw: Softwareframework

## Flashing Iceduino board 
* Open terminal navigate to osflow/boards/iceduino and use make clean all to create iceduino_impl.bin  
* Use make program to flash iceduino_impl.bin

## Using Bootloader with UART
* Set bootloader in osflow/boards/iceduino/neorv32_iceduino_top.vhd to true
* After flashing use the settings below

baudrate 19200  
8 databits  
1 stopbit  
no flow control or parity  

## Compile Application

Compilation for uploading via UART  
* Open terminal navigate to sw/programs/<application folder> and use make clean_all exe to create neorv32_exe.bin  

Compilation for installing into memory without bootloader  
* Open terminal navigate to sw/programs/<application folder> and use make clean_all install to create neorv32_application_image.vhd  




