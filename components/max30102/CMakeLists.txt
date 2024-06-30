idf_component_register(SRCS "heart_rate_algorithm.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES freertos driver esp_timer )