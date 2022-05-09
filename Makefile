all: gauge_configure battery_parameters

gauge_configure: battery.c battery.h gauge_configure.c I2C.c I2C.h
	aarch64-linux-gnu-gcc battery.c gauge_configure.c I2C.c -o gauge_configure

battery_parameters: battery.c battery.h gauge_parameters.c I2C.c I2C.h
	aarch64-linux-gnu-gcc battery.c gauge_parameters.c I2C.c -o battery_parameters

