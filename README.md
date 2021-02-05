# PRMG-Signal-Generator

This project is a signal generator of landing navigation beacon - PRMG.
This project can simulate the azimuth and glide path in static and dynamic.
It can be used for testing airborne equipment.
The output swing is approximately 3.3 V.
The generator sends data to AD7303 DACs via SPI interface.
This project can be fit into a 256 macrocell CPLD.
The source code was written in VHDL (~1100 sloc + test bench ~ 300 sloc).
The design is completed.
All known bugs were fixed.
