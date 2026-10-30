# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Dirk Tostmann
idf_component_register(
    SRCS         "src/hmip_cfg.cpp"
                 "src/hmip_rf.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     rf_protocol rf_stats cc1101_driver freertos esp_timer rf_proto_asksin
)
