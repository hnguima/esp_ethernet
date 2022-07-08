# # Menor versão com suporte a cross-compiling
# cmake_minimum_required(VERSION 2.6)

# project(lib_modutr_slave C)

idf_component_register(SRCS 
                        "modutr_slave.c"
                        "ringbuffer.c"
                    INCLUDE_DIRS "include")
                    
# Força utilização de código no padrão ANSI, para permitir compatibilidade com
# outros compiladores (ex. Keil)
set(CMAKE_C_FLAGS "-mlongcalls")

# install(TARGETS ${INSTALL_TARGETS}
#         DESTINATION ${CMAKE_FIND_ROOT_PATH}/usr/lib)
