idf_component_register(
    SRCS
        "main.c"
        "modules/WIFI/wifi_lib.c"
        "modules/TCP/tcp_lib.c"
        "modules/MQTT/mqtt_lib.c"
        "modules/camera/camera_lib.c"

    INCLUDE_DIRS
        "."
        "modules/WIFI"
        "modules/TCP"
        "modules/MQTT"
        "modules/camera"

    REQUIRES
        mdns
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        espressif__mqtt
)
