# culfw_version.hpp wird aus dem .hpp.in-Template generiert. Quelle des Build-
# Counters ist firmware/build_number.txt (von scripts/bump_build.sh bei jedem
# idf.py build inkrementiert). Generiert in CMAKE_CURRENT_BINARY_DIR und ueber
# INCLUDE_DIRS dem Component zur Verfuegung gestellt.
set(BUILD_NUM_FILE  "${CMAKE_SOURCE_DIR}/build_number.txt")
set(VERSION_FILE    "${CMAKE_SOURCE_DIR}/version.txt")

if(EXISTS ${BUILD_NUM_FILE})
    file(READ ${BUILD_NUM_FILE} CULFW_BUILD_NUM)
    string(STRIP "${CULFW_BUILD_NUM}" CULFW_BUILD_NUM)
else()
    set(CULFW_BUILD_NUM "0")
endif()

if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} CULFW_VERSION_FULL)
    string(STRIP "${CULFW_VERSION_FULL}" CULFW_VERSION_FULL)
else()
    set(CULFW_VERSION_FULL "0.2.${CULFW_BUILD_NUM}")
endif()

set(VERSION_HDR_OUT "${CMAKE_CURRENT_BINARY_DIR}/include/culfw_version.hpp")
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/include/culfw_version.hpp.in"
    "${VERSION_HDR_OUT}"
    @ONLY
)

# CMake reconfigure triggern wenn build_number.txt oder version.txt sich
# aendern (passiert ueber bump_build.sh) → configure_file laeuft neu →
# culfw_version.hpp ist auf der neuen Build-Nummer. Ohne expliziten
# DIRECTORY-Pfad gilt das fuer den aktuellen Komponenten-Scope; das Wrapper-
# Skript bump_build.sh `touch`t deshalb auch dieses CMakeLists.txt nicht —
# CONFIGURE_DEPENDS reicht.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${BUILD_NUM_FILE} ${VERSION_FILE})

idf_component_register(
    SRCS         "src/frontend_culfw.cpp"
    INCLUDE_DIRS "include" "${CMAKE_CURRENT_BINARY_DIR}/include"
    REQUIRES     frontend renderer rf_manager nvs_eeprom led_drv wifi_drv eth_drv transport_tcpserver config_registry rf_stats system_monitor
    PRIV_REQUIRES rf_proto_moritz rf_proto_asksin rf_proto_slowrf rf_proto_native rf_proto_fastrf rf_proto_wmbus rf_proto_hmip rf_proto_dudette esp_timer transport_usbcdc transport_websocket transport_txq
)

# Damit der generated header VOR dem Compile da ist (configure_file haengt
# nicht am Component-Lib-Target). Hat keinen Effekt wenn Cache schon greift —
# configure_file laeuft bei jedem CMake-configure-step.
add_custom_target(culfw_version_hdr ALL DEPENDS "${VERSION_HDR_OUT}")
add_dependencies(${COMPONENT_LIB} culfw_version_hdr)
