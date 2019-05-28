# freertos core sources
set(freertos_core_sources
    croutine.c
    event_groups.c
    list.c
    queue.c
    tasks.c
    timers.c
    )

# freertos portable sources
set(freertos_portable_sources
    portable/RVDS/ARM_CM4F/port.c
    portable/MemMang/heap_4.c
    )

# add freertos library
add_library(freertos_lib ${freertos_core_sources} ${freertos_portable_sources})

# add include directories for freertos
target_include_directories(freertos_lib PUBLIC include)
target_include_directories(freertos_lib PUBLIC portable/RVDS/ARM_CM4F)