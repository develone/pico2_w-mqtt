cmake_minimum_required(VERSION 3.12)
set(WIFI_SSID $ENV{WIFI_SSID})
set(WIFI_PASSWORD $ENV{WIFI_PASSWORD})
message("Using WIFI_SSID from environment ('${WIFI_SSID}')")
message("Using WIFI_PASSWORD from environment")
message("Using WIFI_PASSWORD from environment ('${WIFI_PASSWORD}')")

        add_subdirectory(wifi)
            add_subdirectory(bt)
