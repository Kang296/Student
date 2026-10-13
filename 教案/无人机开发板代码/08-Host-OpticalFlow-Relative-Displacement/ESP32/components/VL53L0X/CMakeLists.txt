idf_component_register(
    SRCS
    "VL53L0X.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)

# include(${CMAKE_SOURCE_DIR}/cmake/custom.cmake)
# find_component_headers()
# set_component_lib()