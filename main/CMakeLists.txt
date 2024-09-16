# idf_component_register(SRCS "main.c"
#                     INCLUDE_DIRS ".")

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES sht4x driver)
