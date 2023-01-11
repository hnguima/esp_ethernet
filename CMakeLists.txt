# # Menor versão com suporte a cross-compiling
cmake_minimum_required(VERSION 3.16)

idf_component_register(SRCS 
                        "ethernet.c"
                    INCLUDE_DIRS 
                        "include"
                    REQUIRES 
                        nlohmann_json
                        esp_event
                        esp_eth
                        esp_netif
                        lwip
                    )
                    