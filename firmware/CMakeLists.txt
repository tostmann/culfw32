cmake_minimum_required(VERSION 3.16)

# improv-wifi-busware-Library: ueber firmware/main/idf_component.yml
# (Git-URL + Commit-SHA, public-clean, reproduzierbar fuer CI / Fremd-Maschinen).
# Wie man gegen eine lokale Kopie der Lib baut, steht dort. Der fruehere
# Override IMPROV_WIFI_BUSWARE_PATH -> EXTRA_COMPONENT_DIRS griff nicht: er
# meldete "local override", gebaut wurde trotzdem die gepinnte Kopie aus
# managed_components/ (gemessen 2026-10-11).

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(culfw32)

# C++20 global, weil mehrere Komponenten std::span / Concepts nutzen.
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++20" APPEND)
