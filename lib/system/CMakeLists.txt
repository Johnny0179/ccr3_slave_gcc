# find all the source files
file(GLOB system_src src/*.c)

# set sources
# set(system)

#add system library
add_library(system_lib ${system_src})

# add head files
target_include_directories(system_lib PUBLIC inc)

# link stm32 library to system library
target_link_libraries(system_lib PUBLIC stm32f4xx)