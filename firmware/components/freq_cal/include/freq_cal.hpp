// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Dirk Tostmann

#pragma once

#include "cc1101_driver.hpp"
#include "esp_err.h"
#include <cstdint>

namespace culfw32 {

class NvsEeprom;

// Frequenzoffset-Kalibrierung (CC1101 FSCTRL0) — Board-Eigenschaft, kein
// Protokoll-Parameter.
//
// Warum es das gibt: der 26-MHz-Quarz jedes Boards zieht anders. Auf der
// CUL32-Serienhardware sind es ~44 kHz (~50 ppm) gegen ein CUL868/a-culfw
// (gemessen 2026-08-31, Memory cul32_freqoffset_44khz_2026-08-31). Wirkung:
// FSK (AskSin/Moritz/HmIP) kommt in KEINE Richtung an, OOK/SlowRF laeuft
// unbeeindruckt weiter — SWRS061I §14.1: "Frequency offset compensation is not
// supported for ASK or OOK". Der Fix steht im selben Kapitel: FREQEST (0x32)
// auslesen und in FSCTRL0.FREQOFF schreiben (DN015). LSB = f_XOSC/2^14 =
// 1,587 kHz bei 26 MHz, Wertebereich +-128 LSB = +-203 kHz.
//
// Zwei Quellen, absteigende Prioritaet:
//
//   1. NVS/EEPROM (`rf.freqoff`) — Feldkalibrierung, jederzeit korrigierbar.
//   2. eFuse BLK3 (USER_DATA) — Werkskalibrierung aus dem Produktionstest.
//      Ueberlebt Flash-Erase, OTA und NVS-Wipe und haengt am Board, nicht am
//      Image.
//   (3. 0 = keine Korrektur.)
//
// Warum nicht eFuse allein: BLK3 ist auf dem C6 Reed-Solomon-codiert und laesst
// sich exakt EINMAL brennen (Fertigung) — zusammen mit
// Produktcode, HW-Revision, Serie, Baudatum und CUSTOM_MAC. Ein Messfehler
// waere damit unkorrigierbar, und jedes bereits ausgelieferte Geraet bliebe
// ohne Wert. Der NVS-Override ist die Sicherheitsleine, nicht der Normalfall.
//
// BLK3-Layout (Fertigung, Layout-Version 2):
//   0..1 "BW" · 2 version · 3..6 product · 7 hw_rev · 8..9 serial
//   10..11 build date · 12 RF-Cal-Marker 'F' · 13 FSCTRL0 (int8)
//   14..24 reserviert · 25..30 CUSTOM_MAC · 31 reserviert
// Version 1 (die TUL32-Bestandssticks) hat 12..24 auf 0x00 — dort gibt es
// keinen Kalibrierwert, und genau das meldet efuse_read() dann auch.
namespace freqcal {

inline constexpr uint8_t kBlk3RfCalMarker = 'F';
inline constexpr size_t  kBlk3MarkerOff   = 12;
inline constexpr size_t  kBlk3ValueOff    = 13;

// kHz pro FSCTRL0-LSB bei 26 MHz Quarz (f_XOSC / 2^14).
inline constexpr float kHzPerLsb = 26000.0f / 16384.0f;

enum class Source : uint8_t { None, Efuse, Nvs };

const char* source_name(Source s);

// Werkskalibrierung aus eFuse BLK3. false, wenn der Block unbeschrieben ist,
// eine fremde/aeltere Layout-Version traegt oder keinen RF-Cal-Marker hat.
bool efuse_read(int8_t& out);

// Feldkalibrierung aus dem EEPROM. false, wenn kein gueltiger Eintrag steht.
bool nvs_read(NvsEeprom& ee, int8_t& out);
void nvs_write(NvsEeprom& ee, int8_t value);
void nvs_clear(NvsEeprom& ee);

// Effektiver Wert nach der Prioritaet oben. `src` sagt, woher er kam.
int8_t effective(NvsEeprom* ee, Source& src);

// --- Suchlauf ---------------------------------------------------------------
//
// Henne-Ei: FREQEST ist erst dann aussagekraeftig, wenn ueberhaupt Signal im
// Empfaenger ankommt — und bei einem Board, das 44 kHz danebenliegt, wird kein
// Frame mehr dekodiert, aus dem man samplen koennte. Der Suchlauf loest das,
// indem er NICHT auf dekodierte Frames wartet, sondern auf Traeger: er faehrt
// FSCTRL0 in Schritten durch, misst je Schritt RSSI, und sammelt FREQEST immer
// dann, wenn etwas ueber der Rauschschwelle liegt. Der Offset-Kandidat mit den
// meisten Treffern gewinnt; sein FREQEST-Median ist die Feinkorrektur.
//
// Voraussetzung ist eine Signalquelle in Reichweite — im Feld die vorhandenen
// HomeMatic-/MAX!-Geraete, in der Produktion der Referenzsender am Pruefplatz.
// Ohne Quelle endet der Lauf mit ok=false und laesst den Offset unveraendert.
struct ScanState {
    bool    running   = false;
    bool    finished  = false;
    bool    ok        = false;
    int8_t  result    = 0;   // vorgeschlagener neuer FSCTRL0-Wert
    int8_t  best_step = 0;   // Kandidat mit den meisten Treffern
    int8_t  median    = 0;   // FREQEST-Median auf diesem Kandidaten
    size_t  samples   = 0;   // Treffer auf dem Gewinner
    int8_t  rssi_dbm  = 0;   // staerkstes gesehenes Signal
};

// Startet den Suchlauf als eigenen Task. dwell_ms = Verweildauer je Schritt.
// Laeuft nicht neu, solange einer laeuft. Der Aufrufer pollt scan_state().
esp_err_t scan_start(CC1101Driver& drv, uint32_t dwell_ms = 1200);
ScanState scan_state();

} // namespace freqcal
} // namespace culfw32
