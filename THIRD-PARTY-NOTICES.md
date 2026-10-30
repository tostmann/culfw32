# Third-Party Notices

CULFW32 is licensed under the GNU General Public License, version 3 or (at your
option) any later version (`GPL-3.0-or-later`); see [COPYING](COPYING). This file
lists third-party material that CULFW32 contains or is linked with, together with
the notices its licenses require.

## 1. Code ported into CULFW32

### a-culfw

Source: <https://github.com/tostmann/a-culfw>. Repository license: GNU GPL
version 2 or (at your option) any later version.

| Original file | Notice in the original | Used in |
|---|---|---|
| `culfw/clib/rf_receive.c` | Copyright by R.Koenig, Copyright 2015 B. Hempel, inspired by code from Dirk Tostmann; GNU GPL version 2 or any later version | `firmware/components/rf_proto_slowrf/src/slowrf_rf.cpp` |
| `culfw/clib/rf_receive_bucket.c` | Copyright (C) 2015 B. Hempel; GNU GPL version 2 or any later version | `slowrf_rf.cpp` |
| `culfw/clib/cc1100.c` | no individual notice | register tables in `rf_proto_slowrf` (`slowrf_cfg.*`), `rf_proto_fastrf` (`fastrf_cfg.*`) and `nvs_eeprom` (`culfw_ee_defaults.hpp`) |

Further files carry the note "Portions derived from a-culfw" in their header;
they follow a-culfw's command handling, output formats and protocol sequences.

The register tables in `rf_proto_asksin/src/asksin_cfg.cpp` (from
`culfw/clib/rf_asksin.c`), `rf_proto_moritz/src/moritz_cfg.cpp` (from
`culfw/clib/rf_moritz.c`) and `rf_proto_native/src/native_cfg.cpp` (from
`culfw/clib/rf_native.c`), and the receive/transmit sequence in
`rf_proto_wmbus/src/wmbus_rf.cpp` (from `culfw/clib/rf_mbus.c`), come from
a-culfw code written by Dirk Tostmann, who licenses them here under GPL-3.0-or-later.
`rf_native.c` and `rf_mbus.c` carry "Copyright by D.Tostmann", "License: GPL v2"
in the original; `rf_mbus.c` also notes "Inspired by code from TI.com AN".

### SIGNALduino

Source: <https://github.com/RFD-FHEM/SIGNALDuino>.

| Original | Notice in the original | Used in |
|---|---|---|
| Pattern Decoder Library V3 (`signalDecoder.h`, `signalDecoder.cpp`) | 2014-2015 N.Butzek, S.Butzek; 2015-2018 S.Butzek; 2018-2020 S.Butzek, HomeAutoUser; GNU GPL version 3 or any later version | `firmware/components/rf_proto_signalduino` |
| `send.h` | no individual notice; the repository is licensed under the GNU GPL version 3 | `firmware/components/frontend_signalduino` (transmit path) |

### wM-Bus

The wM-Bus codec (`firmware/components/rf_proto_wmbus/src/wmbus_codec.cpp`) was
written from EN 13757-4 and contains no third-party code. The CC1101 register
tables (`wmbus_cfg.cpp`) are written anew; the values determined by EN 13757-4
are derived there from the CC1101 data sheet, while the tuning values (AGC,
filter, calibration and test registers) are the values a-culfw uses for the
CC1101 (`culfw/clib/mbus/smode_rf_settings.h`, `tmode_rf_settings.h`, Texas
Instruments origin).

## 2. Protocol references

The HMUARTLGW and HB-RF-ETH frontends were written independently; the following
projects served as protocol references: FHEM `00_HMUARTLGW.pm`
(<https://svn.fhem.de/trac/browser/trunk/fhem/FHEM/00_HMUARTLGW.pm>), the piVCCU
kernel modules by Alexander Reinert, and the HB-RF-ETH firmware by Alexander
Reinert (`hmframe.h`, `radiomoduledetector.cpp`, Apache-2.0).

## 3. Components linked into the firmware image

| Component | License |
|---|---|
| Espressif ESP-IDF 5.5.3 | Apache-2.0, see [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt) |
| Espressif binary libraries shipped with ESP-IDF (Wi-Fi, PHY, coexistence) | Apache-2.0 |
| espressif/mdns 1.11.1 | Apache-2.0 |
| Mbed TLS (part of ESP-IDF) | Apache-2.0 OR GPL-2.0-or-later; used under Apache-2.0 |
| FreeRTOS kernel (part of ESP-IDF) | MIT, text below |
| lwIP (part of ESP-IDF) | BSD-3-Clause, text below |
| SPIFFS (part of ESP-IDF) | MIT, text below |
| cJSON (part of ESP-IDF) | MIT, text below |
| improv-wifi-busware | MIT, text below |
| newlib C library (ESP toolchain) | BSD-style licenses, see `COPYING.NEWLIB` in the toolchain distribution |
| libgcc, libstdc++ (ESP toolchain) | GPL-3.0 with the GCC Runtime Library Exception |

## 4. License texts

### FreeRTOS kernel

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### lwIP

```
Copyright (c) 2001, 2002 Swedish Institute of Computer Science.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
```

### SPIFFS

```
The MIT License (MIT)

Copyright (c) 2013-2017 Peter Andersson (pelleplutt1976<at>gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

### cJSON

```
Copyright (c) 2009-2017 Dave Gamble and cJSON contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

### improv-wifi-busware

```
MIT License

Copyright (c) 2023 Jonathas Amaral Barbosa (@jnthas)
Copyright (c) 2026 Dirk Tostmann / Busware (modifications)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
