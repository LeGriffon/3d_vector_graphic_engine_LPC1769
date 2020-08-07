# 3D Vector Graphic Engine with LPC1769 #

This project is an implementation of a vector graphic representation on a SPI color display with an LPC 1769 CPU module. The project includes soldering the SPI LCD color display and the CPU module onto a PCB wire wrapping board. 

The fitst graph implementations is a 2D vector graph engine that can produce two screensavers using two different functions. The implementation then upgraded to a 3D vector graphic representation on the SPI color display with FreeRTOS webserver. 

<img src="https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/IMG_3999.jpg?raw=true" width="300"> <img src="https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/IMG_4017.jpg?raw=true" width="300">


The final graph implementations is a cube with 3D transformation pipeline on the center of the screen and the two screensavers from previous step printed onto the two sides of the cube with linear decoration. On the top of the cube has a printed F letter with diffusion interpolation with DDA algorithm. 

<img src="https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/IMG_5224.jpg?raw=true" width="400">

The webserver is a FreeRTOS html website that is connected via TCP/IP protocol which is capable to start the screensaver by clicking the button on the html GUI interface.



# System Layout and Setup #
![system layout](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/System%20Layout%20and%20Setup.jpg?raw=true)

The overall system layout is using a LPC 1769 CPU module which acts as a server that is able to be connected by a computer. The LPC 1769 module then transfer data and power to a SPI LCD display using SPI protocol.

# LPC 1769 Interface to LCD Color Display #
![LPC 1769 Interface to LCD Color Display](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/SPI%20display%20connection.png?raw=true)
The association between the LPC 1769 CPU module and the LCD color display is a parent-child relationship. The CPU module acts as the parent to control the SPI LCD display, and in turn the display acts as the child to receive data and display images. The interface between the two modules implemented here has to carry out the SPI communication.

The LPC 1769 CPU is an ARM Cortex-M3 based microcontroller. The SPI LCD display applies the serial peripheral interface. The parent board sends out the MOSI, serial clock, chip select, data and reset signals. The child sends out the MISO signal.

# The LPC 1769 Interface to RJ-45 Jack #
![LPC 1769 interface to RJ-45 Jack](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/The%20LPC%201769%20interface%20to%20RJ-45%20Jack.jpg?raw=true)

# Flowchart of the Software Implementation #
![Flowchart of the Software Implementation](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/The%20flowchart%20of%20the%20software%20implementation.jpg?raw=true)

# Real-World Implementation #

![Implementation1](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/IMG_5227.jpg?raw=true)
![Implementation2](https://github.com/LeGriffon/3d_vector_graphic_engine_LPC1769/blob/master/imgs/IMG_5228.jpg?raw=true)

