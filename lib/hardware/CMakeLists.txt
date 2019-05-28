# find all the source files
file(GLOB hardware_sources src/*.c)

#add system library
add_library(hardware_lib ${hardware_sources})

# add head files
target_include_directories(hardware_lib PUBLIC inc)

# link system library to project
target_link_libraries(hardware_lib PUBLIC stm32f4xx)