# freemodbus sources
set(freemodbus_sources
    port/portevent.c
    port/portserial.c
    port/porttimer.c
    modbus/mb.c
    modbus/rtu/mbcrc.c
    modbus/rtu/mbrtu.c
    modbus/functions/mbfunccoils.c
    modbus/functions/mbfuncdiag.c
    modbus/functions/mbfuncdisc.c
    modbus/functions/mbfuncholding.c
    modbus/functions/mbfuncinput.c
    modbus/functions/mbfuncother.c
    modbus/functions/mbutils.c
    freemodbus.c
)

# add sources to lib
add_library(freemodbus_lib ${freemodbus_sources})

# add include
target_include_directories(freemodbus_lib 
                            PUBLIC port 
                            PUBLIC modbus/include
                            PUBLIC modbus/rtu 
                            )

# link stm32 library to freemodbus
target_link_libraries(freemodbus_lib PUBLIC stm32f4xx)
