# 3D Vector Graphic Engine with LPC1769 #

This project is an implementation of a FreeRTOS webserver-controlled 3D vector graphic representation on a SPI color display with an LPC 1769 CPU module. 

The project includes soldering RJ-45 connector onto the PCB wire wrapping board. The graph implementations are a 3D cube with 3D transformation pipeline on the center of the screen and the two screensavers from previous lab printed onto the two sides of the cube with linear decoration. On the top of the cube has a printed F letter with diffusion interpolation with DDA algorithm. 

The webserver is a FreeRTOS html website that is connected via TCP/IP protocol which is capable to start the screensaver by clicking the button on the html GUI interface.

# System Layout and Setup #
![system layout]()