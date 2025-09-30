#ifndef CONFIG_H
#define CONFIG_H

    #ifdef ESP32
        #include "esp32_hardware.h"
        #include "conf_network.h"
    #else
        #include "teensy_hardware.h"
    #endif
    #include "PIDF_config.h"

#endif