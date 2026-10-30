idf_component_register(
    SRCS         "src/rf_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol cc1101_driver
)
