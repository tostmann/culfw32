idf_component_register(
    SRCS "src/freq_cal.cpp"
    INCLUDE_DIRS "include"
    REQUIRES efuse nvs_eeprom cc1101_driver
)
