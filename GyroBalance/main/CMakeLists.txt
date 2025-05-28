idf_component_register(SRCS "dshot_esc_example_main.c" "dshot_esc_encoder.c" "tuning_websocket_server.c" "websocket_server.c" "websocket.c" "wifi_handler.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES ../frontend/index.html)
