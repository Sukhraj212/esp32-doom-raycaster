idf_component_register(SRCS "lcd.c" "main.c"
                    INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB} PUBLIC m)