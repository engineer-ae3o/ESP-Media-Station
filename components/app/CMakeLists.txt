idf_component_register(SRCS "src/audio.cpp"
                            "src/tasks.cpp"
                            "src/codec.cpp"
                            "src/display.cpp"
                       INCLUDE_DIRS "inc/"
                       REQUIRES drivers freertos unity esp_timer
                                espressif__esp_audio_codec espressif__esp-dsp
                                lvgl__lvgl joltwallet__littlefs espressif__esp_peer
                                espressif__esp_h264 espressif__esp-dl
)
