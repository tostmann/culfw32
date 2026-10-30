/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (c) 2026 Dirk Tostmann */
/* CULFW32 Web-UI — vanilla JS, kein Framework. */
(function(){
'use strict';

// ============ Tab-Switch ============
document.querySelectorAll('.tab').forEach(t => {
  t.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach(x => x.classList.remove('active'));
    t.classList.add('active');
    document.getElementById('tab-' + t.dataset.tab).classList.add('active');
    // Lazy-Open Terminal-Connection beim ersten Switch
    if (t.dataset.tab === 'terminal' && !termWS) termConnect();
  });
});

// ============ Helpers ============
function $(id) { return document.getElementById(id); }
function wsURL(path) {
  return (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + path;
}
function logTerm(text, cls) {
  const out = $('term-output');
  const span = document.createElement('span');
  if (cls) span.className = cls;
  span.textContent = text;
  out.appendChild(span);
  out.scrollTop = out.scrollHeight;
}

// ============ JSON-Frontend Connection (Config-Tab) ============
let cfgWS = null;
let cfgItems = [];
let cfgPending = null;   // Dump im Gang: hier sammeln, bei dump_end uebernehmen

function cfgConnect() {
  console.log('[CULFW32] connecting JSON WS', wsURL('/ws/json'));
  cfgWS = new WebSocket(wsURL('/ws/json'));
  cfgWS.onopen = () => {
    console.log('[CULFW32] JSON WS open — sending dump');
    cfgWS.send(JSON.stringify({cmd:'dump'}));
  };
  cfgWS.onmessage = ev => {
    let m;
    try { m = JSON.parse(ev.data); } catch(e) { console.warn('[CULFW32] JSON-parse fail:', ev.data); return; }
    if (m.event === 'dump_begin') {
      // Erst bei dump_end umschalten: ein Live-Push mitten im Dump sieht sonst
      // eine halbe Liste (Firmware-Karte blinkt weg, Zustandslogik irrt).
      cfgPending = [];
    } else if (m.event === 'dump_item') {
      (cfgPending || (cfgPending = [])).push(m);
    } else if (m.event === 'dump_end') {
      if (cfgPending) { cfgItems = cfgPending; cfgPending = null; }
      console.log('[CULFW32] dump received,', cfgItems.length, 'items');
      renderSettings();
      otaAfterDump();     // vor renderStatus(): wertet eine Installation vor dem Neustart aus
      renderStatus();
    } else if (m.event === 'dump' && m.items) {
      // Legacy-Form falls jemand mit alter Firmware verbindet.
      cfgItems = m.items;
      renderSettings();
      otaAfterDump();     // wie im Streaming-Pfad: gibt otaAwaitDump/otaTarget frei
      renderStatus();
    } else if (m.event === 'value') {
      // cfgItems ebenfalls patchen, damit renderProtoTable() & co. den
      // aktuellen Wert sehen (sie lesen aus cfgItems statt aus dem DOM).
      const it = cfgItems.find(i => i.key === m.key);
      if (it) it.value = m.value;
      // Kam der Eintrag im laufenden Dump schon vorbei, ist der Push neuer.
      const pit = cfgPending && cfgPending.find(i => i.key === m.key);
      if (pit) pit.value = m.value;
      updateValue(m.key, m.value);
    } else if (m.event === 'tx_ack') {
      handleTxAck(m);
    } else if (m.event === 'ack') {
      const row = document.querySelector(`[data-key="${m.key}"]`);
      if (row) {
        row.classList.remove('dirty');
        if (!m.ok) flashRow(row, 'err');
        else       flashRow(row, 'ok');
      }
      if (m.key === 'ota' && !m.ok) otaRefused();
      // Re-fetch um echten neuen Wert anzuzeigen (Setter koennte normalisieren).
      cfgWS.send(JSON.stringify({cmd:'get', key: m.key}));
    } else if (m.event === 'err') {
      console.warn('cfg err:', m);
    }
  };
  cfgWS.onclose = () => {
    otaOnClose();
    setBadge('reconnecting …', 'status-unknown', 'No connection to the stick, retrying every 2 s');
    setTimeout(cfgConnect, 2000);  // Auto-Reconnect
  };
  cfgWS.onerror = () => { /* close folgt */ };
}

function flashRow(row, kind) {
  const orig = row.style.background;
  row.style.background = kind === 'ok' ? '#d4edda' : '#f8d7da';
  setTimeout(() => row.style.background = orig, 700);
}

function updateValue(key, value) {
  // Settings-Row aktualisieren — falls vorhanden. Bei readonly-rf.*-Keys wird
  // die Row vom Renderer ausgeblendet (HF-Card oben zeigt sie schon), daher
  // kann row==null sein. Das ist OK — wir gehen unten weiter zu den
  // HF-Card-/Status-Box-Updates.
  const row = document.querySelector(`[data-key="${key}"]`);
  if (row) {
    const inp = row.querySelector('.setting-input');
    if (inp && document.activeElement !== inp) {
      if (inp.classList.contains('setting-readonly')) {
        inp.textContent = value;
        inp.title       = value;
      } else if (inp.tagName === 'INPUT' || inp.tagName === 'SELECT') {
        inp.value = (inp.type === 'password') ? '' : value;
        if (inp.type === 'password' && value === '***') inp.placeholder = '*** (set)';
        inp.title = value;
        inp.dataset.original = value;
      }
    }
  }
  // Status-Box (build absichtlich nicht hier — laeuft raus, steht in MISC-Card)
  // renderNet() liest aus cfgItems: vor dem ersten dump_end ist die Liste leer,
  // ein Live-Push davor zeigte sonst "?" / "no network" bis zum dump_end.
  if ((key.startsWith('wifi.') || key.startsWith('eth.')) && cfgItems.length) renderNet();
  if (key === 'fw.version') showVersion(value);
  if (key === 'build')      $('footer-build').textContent  = value;
  // HF-Frontend-Karte
  if (key === 'rf.chip')      $('rf-chip').textContent     = value;
  if (key === 'rf.proto')     $('rf-proto').textContent    = value;
  if (key === 'rf.freq')      $('rf-freq').textContent     = value;
  if (key === 'rf.mod')       $('rf-mod').textContent      = value;
  if (key === 'rf.datarate')  $('rf-datarate').textContent = value;
  if (key === 'rf.bw')        $('rf-bw').textContent       = value;
  if (key === 'rf.deviation') $('rf-dev').textContent      = value;
  if (key === 'rf.pa.idx') {
    // PA-Idx in HF-Card spiegeln (Wert aus enum_labels wenn vorhanden).
    const it = cfgItems.find(i => i.key === 'rf.pa.idx');
    if (it && it.enum_labels) {
      const idx = parseInt(value, 10);
      $('rf-pa').textContent = isNaN(idx) ? value : `${idx} (${it.enum_labels[idx] || '?'})`;
    } else {
      $('rf-pa').textContent = value;
    }
  }
  if (key === 'rf.state') {
    const p = $('rf-state-pill');
    p.textContent = value;
    p.className = 'rf-state-pill ' +
      (value === 'RX' ? 'rx' :
       value === 'TX' ? 'tx' :
       value === 'IDLE' ? 'idle' :
       value === '-' ? '' : 'err');
  }
  if (key === 'rf.rssi') {
    const r = $('rf-rssi');
    const m = $('rf-bar-marker');
    const isNumeric = /^-?\d+$/.test(value);
    r.textContent = value;
    r.classList.toggle('na', !isNumeric);
    if (isNumeric) {
      // Marker-Position auf der Heat-Skala -120..-30 dBm.
      const n = parseInt(value, 10);
      const minDbm = -120, maxDbm = -30;
      let pct = ((n - minDbm) / (maxDbm - minDbm)) * 100;
      pct = Math.max(0, Math.min(100, pct));
      m.style.left = pct.toFixed(1) + '%';
      m.classList.remove('hidden');
    } else {
      m.classList.add('hidden');
    }
  }
  // ============ Stats / Credit ============
  // HF-Karte Credit-Bar + Stats-Tab Credit-Card teilen sich die selben Werte.
  if (key === 'stats.rf.credit_pct')   updateCreditPct(value);
  if (key === 'stats.rf.credit')       updateCreditAbs(value);
  if (key === 'stats.rf.lovf')         updateLovf(value);
  if (key === 'stats.rf.tx_total')     setText('st-tx-total', value);
  if (key === 'stats.rf.rx_total')     setText('st-rx-total', value);
  if (key === 'stats.rf.tx_airtime')   updateAirtime(value);
  if (key === 'stats.sys.heap_free')   updateHeap('free', value);
  if (key === 'stats.sys.heap_min')    updateHeap('min',  value);
  // ============ System / Stabilitaet ============
  if (key === 'sys.uptime')            { setText('brand-uptime', value); setText('st-uptime', value); }
  if (key === 'sys.uptime_total')      setText('st-uptime-total', value);
  if (key === 'sys.boot_count')        setText('st-boot-count', value);
  if (key === 'sys.crash_count')       setText('st-crash-count', value);
  if (key === 'sys.reset_reason')      updateResetReason(value);
  if (key === 'sys.reset_reason_text') setText('st-reset-text', value);
  if (key === 'sys.temp')              updateChipTemp('now', value);
  if (key === 'sys.temp_min')          updateChipTemp('min', value);
  if (key === 'sys.temp_max')          updateChipTemp('max', value);
  if (key.startsWith('stats.') && key.indexOf('.', 6) > 0) {
    // Per-Protocol Update — Tabelle neu rendern (lazy: erst wenn Tab sichtbar
    // oder zumindest Element existiert; renderProtoTable ist guarded).
    renderProtoTable();
  }
  // ============ Firmware-Karte ============
  if (key === 'fw.version' || key === 'ota' || key.startsWith('ota.')) renderOta();
}

// Reset-Reason mit Stabilitaets-Farbcode (gruen/gelb/rot) im Header-Badge.
function updateResetReason(value) {
  const v = String(value || '?');
  const headerBadge = $('brand-reset');
  const statsBadge  = $('st-reset-reason');
  if (headerBadge) {
    headerBadge.textContent = v;
    let cls = 'brand-meta brand-reset-ok';
    if (v === 'WDT' || v === 'TASK_WDT' || v === 'INT_WDT' ||
        v === 'BROWN' || v === 'GLITCH') {
      cls = 'brand-meta brand-reset-warn';
    } else if (v === 'PANIC' || v === 'LOCK') {
      cls = 'brand-meta brand-reset-crash';
    }
    headerBadge.className = cls;
  }
  if (statsBadge) statsBadge.textContent = v;
}

// Die-Temperatur des ESP32-C6 (nicht Umgebung), dazu Min/Max seit Boot.
// Die Registry liefert Zehntelgrad als Text ("41.3") oder "n/a".
const chipTemp = { now: null, min: null, max: null };
function updateChipTemp(kind, value) {
  chipTemp[kind] = String(value);
  const n = chipTemp.now;
  if (n === null) return;
  if (n === 'n/a') { setText('st-chip-temp', 'n/a'); return; }
  let s = n + ' \u00B0C';
  if (chipTemp.min !== null && chipTemp.max !== null && chipTemp.min !== 'n/a') {
    s += ' (min ' + chipTemp.min + ' / max ' + chipTemp.max + ')';
  }
  setText('st-chip-temp', s);
}

function setText(id, v) { const el = $(id); if (el) el.textContent = v; }

function updateCreditPct(value) {
  const pct = parseInt(value, 10);
  if (isNaN(pct)) return;
  // HF-Karte
  const hfPct = $('rf-credit-pct');
  const hfFill = $('rf-credit-fill');
  if (hfPct) hfPct.textContent = pct + '%';
  if (hfFill) {
    hfFill.style.width = pct + '%';
    hfFill.className = 'rf-credit-fill ' + creditClass(pct);
  }
  // Stats-Tab
  const stPct = $('st-credit-pct');
  const stFill = $('st-credit-fill');
  if (stPct) stPct.textContent = pct + '%';
  if (stFill) {
    stFill.style.width = pct + '%';
    stFill.className = 'rf-credit-fill ' + creditClass(pct);
  }
}
function creditClass(pct) {
  // Schwellen so gewaehlt, dass user "wann Schluss" sieht: <40 % = mid (gelb),
  // <15 % = low (rot), 0 % = empty (dunkelrot).
  if (pct >= 40) return '';        // grün via default-Linear-Gradient
  if (pct >= 15) return 'mid';
  if (pct >  0)  return 'low';
  return 'empty';
}

function updateCreditAbs(value) {
  // value = absolute Einheiten. Wir kennen MAX=900 statisch (dump enthält
  // stats.rf.credit_max). Anzeige als "<value>/900 (= 9 s burst)".
  const v = parseInt(value, 10);
  if (isNaN(v)) return;
  const abs = `${v}/900 (= ${(v/100).toFixed(1)} s burst)`;
  setText('rf-credit-abs', abs);
  setText('st-credit-abs', abs);
}

function updateLovf(value) {
  setText('st-lovf', value);
  const card = document.querySelector('.stats-card.stats-lovf');
  if (card) {
    const n = parseInt(value, 10);
    card.classList.toggle('alert', !isNaN(n) && n > 0);
  }
}

function updateAirtime(value) {
  const units = parseInt(value, 10);
  if (isNaN(units)) return;
  // 1 Unit = 10 ms = 0.01 s.
  setText('st-airtime-s', (units / 100).toFixed(1));
  setText('st-airtime-units', units + ' × 10 ms');
}

// Heap free/min in human-readable + relativer Bar (relativ zum hoechsten
// gesehenen Wert, da wir keinen festen "max heap" kennen).
let heapMaxSeen = 0;
function updateHeap(kind, value) {
  const v = parseInt(value, 10);
  if (isNaN(v)) return;
  const human = formatBytes(v);
  if (kind === 'free') {
    setText('st-heap-free', human);
    if (v > heapMaxSeen) heapMaxSeen = v;
    const bar = $('st-heap-bar');
    if (bar && heapMaxSeen > 0) {
      const pct = Math.min(100, (v / heapMaxSeen) * 100);
      bar.style.width = pct.toFixed(1) + '%';
    }
  } else if (kind === 'min') {
    setText('st-heap-min', human);
  }
}
function formatBytes(b) {
  if (b > 1024*1024) return (b / (1024*1024)).toFixed(2) + ' MB';
  if (b > 1024)      return (b / 1024).toFixed(1) + ' KB';
  return b + ' B';
}

// Per-Protocol Counter-Tabelle. Eine Zeile pro Protocol mit Zellen aus
// stats.<proto>.rx_msgs etc.
const PROTOS = ['moritz','asksin','slowrf','native','fastrf'];
function renderProtoTable() {
  const tbody = $('st-proto-tbody');
  if (!tbody) return;
  tbody.innerHTML = '';
  const get = key => {
    const it = cfgItems.find(i => i.key === key);
    return it ? it.value : null;
  };
  PROTOS.forEach(p => {
    const rxMsgs = get(`stats.${p}.rx_msgs`)  || '0';
    const rxBytes = get(`stats.${p}.rx_bytes`) || '0';
    const txMsgs = get(`stats.${p}.tx_msgs`)  || '0';
    const txBytes = get(`stats.${p}.tx_bytes`) || '0';
    const txAir   = get(`stats.${p}.tx_airtime`) || '0';
    const rxErr   = get(`stats.${p}.rx_err`)  || '0';
    const txErr   = get(`stats.${p}.tx_err`)  || '0';
    const crcErr  = get(`stats.${p}.crc_err`) || '0';
    const rssi    = get(`stats.${p}.last_rssi`) || 'n/a';
    const ago     = get(`stats.${p}.last_rx_ago`) || 'n/a';

    const tr = document.createElement('tr');
    const empty = (rxMsgs === '0' && txMsgs === '0' &&
                   rxErr === '0' && txErr === '0');
    if (empty) tr.classList.add('empty');

    tr.innerHTML = `
      <td>${p}</td>
      <td>${rxMsgs}</td>
      <td>${rxBytes}</td>
      <td>${txMsgs}</td>
      <td>${txBytes}</td>
      <td>${txAir} <small>(${(parseInt(txAir,10)/100).toFixed(1)}s)</small></td>
      <td class="err-cell ${(parseInt(rxErr,10)+parseInt(txErr,10))>0?'has-err':''}">
        <b>${rxErr}/${txErr}</b>
        <small>${crcErr>0?'crc:'+crcErr:''}</small>
        ${errorBar(rxErr, rxMsgs)}
      </td>
      <td class="${rssiClass(rssi)}">${rssi}${rssi !== 'n/a' ? ' dBm' : ''}</td>
      <td class="${agoClass(ago)}">${formatAgo(ago)}</td>
    `;
    tbody.appendChild(tr);
  });
}
function errorBar(err, total) {
  const e = parseInt(err, 10), t = parseInt(total, 10);
  if (isNaN(e) || isNaN(t) || t === 0) return '';
  const pct = Math.min(100, (e / (e + t)) * 100);
  if (pct < 1) return '';   // unter 1% nicht zeigen
  return `<span class="err-bar"><span class="err-bar-fill" style="width:${pct.toFixed(1)}%"></span></span>`;
}
function rssiClass(v) {
  if (v === 'n/a') return 'rssi-na';
  const n = parseInt(v, 10);
  if (isNaN(n)) return 'rssi-na';
  if (n >= -60) return 'rssi-strong';
  if (n >= -80) return 'rssi-mid';
  return 'rssi-weak';
}
function agoClass(v) {
  if (v === 'n/a') return 'ago-stale';
  const n = parseInt(v, 10);
  if (isNaN(n)) return 'ago-stale';
  if (n < 30)  return 'ago-fresh';
  if (n < 600) return 'ago-old';
  return 'ago-stale';
}
function formatAgo(v) {
  if (v === 'n/a') return 'never';
  const n = parseInt(v, 10);
  if (isNaN(n)) return v;
  if (n < 60)    return n + 's';
  if (n < 3600)  return Math.floor(n/60) + 'm ' + (n%60) + 's';
  return Math.floor(n/3600) + 'h ' + Math.floor((n%3600)/60) + 'm';
}

// Version im Kopf; der Quelltext-Link im Footer zeigt auf den Release-Tag
// genau dieser Fassung (GPLv3 §6).
function showVersion(v) {
  $('brand-version').textContent = 'v' + v;
  $('footer-source').href = 'https://github.com/tostmann/culfw32/tree/v' + encodeURIComponent(v);
}

function renderStatus() {
  const find = k => (cfgItems.find(i => i.key === k) || {}).value;
  renderNet();
  $('footer-build').textContent = find('build') || '';
  const fv = find('fw.version');
  if (fv) showVersion(fv);
  // HF-Card initial-Render via updateValue (gleicher Pfad wie Live-Push)
  ['rf.chip','rf.proto','rf.freq','rf.mod','rf.datarate','rf.bw','rf.deviation','rf.pa.idx','rf.state','rf.rssi'].forEach(k => {
    const v = find(k);
    if (v !== undefined) updateValue(k, v);
  });
  // Stats initial-Render — globale Karten + Per-Protocol-Tabelle.
  ['stats.rf.credit','stats.rf.credit_pct','stats.rf.lovf','stats.rf.tx_total',
   'stats.rf.rx_total','stats.rf.tx_airtime',
   'stats.sys.heap_free','stats.sys.heap_min',
   'sys.uptime','sys.uptime_total','sys.boot_count','sys.crash_count',
   'sys.reset_reason','sys.reset_reason_text',
   'sys.temp','sys.temp_min','sys.temp_max'].forEach(k => {
    const v = find(k);
    if (v !== undefined) updateValue(k, v);
  });
  renderProtoTable();
  renderOta();
}

// ============ Netz: Status-Karte + Kopf-Badge ============
// Der Stick haengt am WLAN, am Ethernet (W5500 an J1) oder an beidem. Das Badge
// zeigt die Verbindung, ueber die er erreichbar ist -- bis 0.1.885 zeigte es nur
// wifi.state, ein Ethernet-Stick stand also mit rotem "off" da.
// Rohzustaende der Firmware (eth.state / wifi.state) als lesbare Worte.
const ETH_WORDS  = {stopped: 'stopped', nolink: 'no link', link: 'link up, no address',
                    connected: 'connected'};
const WIFI_WORDS = {off: 'off', disconnected: 'not connected', connecting: 'connecting',
                    connected: 'connected'};

function renderNet() {
  const eth   = cfgValue('eth.state');              // fehlt in Builds ohne W5500
  const ethIp = cfgValue('eth.ip');
  const wifi  = cfgValue('wifi.state') || '?';
  const ap    = cfgValue('wifi.ap') === 'on';       // Provisioning-AP (seit 0.1.887)
  const ethShown = eth !== undefined && eth !== 'absent';
  const ethWord  = ETH_WORDS[eth] || eth;
  const wifiWord = (WIFI_WORDS[wifi] || wifi) + (ap ? ', access point on' : '');
  $('s-eth-row').hidden = !ethShown;
  if (ethShown) setText('s-eth', ethWord + (eth === 'connected' && ethIp && ethIp !== '0.0.0.0' ? ', ' + ethIp : ''));
  setText('s-wifi', wifiWord);
  setText('s-ip',   cfgValue('wifi.ip')   || '-');
  setText('s-rssi', cfgValue('wifi.rssi') || '-');
  setText('s-ssid', cfgValue('wifi.ssid') || '-');
  // Adresse und Pegel gibt es nur verbunden; die SSID zeigt, wohin er will.
  $('s-ip-row').hidden   = (wifi !== 'connected');
  $('s-rssi-row').hidden = (wifi !== 'connected');
  $('s-ssid-row').hidden = (wifi === 'off');
  let text, cls;
  if (eth === 'connected')        { text = 'Ethernet';        cls = 'status-online'; }
  else if (wifi === 'connected')  { text = 'WiFi';            cls = 'status-online'; }
  // Ueber den Provisioning-AP ist die Seite erreichbar, obwohl die Station
  // nicht verbunden ist (wifi.state beschreibt nur die Station).
  else if (ap)                    { text = 'WiFi AP';         cls = 'status-unknown'; }
  else if (wifi === 'connecting') { text = 'WiFi connecting'; cls = 'status-unknown'; }
  else if (ethShown && wifi === 'off') { text = 'Ethernet ' + ethWord; cls = 'status-offline'; }
  else                            { text = 'WiFi ' + wifiWord; cls = 'status-offline'; }
  setBadge(text, cls, (ethShown ? 'Ethernet: ' + ethWord + ', ' : '') + 'WiFi: ' + wifiWord);
}

function setBadge(text, cls, title) {
  const b = $('badge-state');
  b.textContent = text;
  b.className = 'status-badge ' + cls;
  b.title = title;
}

// ============ Firmware / Online-Update ============
// Pruefen und Installieren macht die Firmware selbst (components/ota_update):
// `set ota=check|install` stoesst es an, ota.state/.latest/.progress melden den
// Zustand (readonly -> Live-Push alle 5 s). Solange ein Auftrag laeuft, fragt
// die Karte jede Sekunde nach. Die Installation endet mit einem Neustart: die
// Verbindung bricht ab, cfgConnect() verbindet neu, otaAfterDump() vergleicht
// die laufende Version mit dem Ziel.
const OTA_CARD_KEYS = ['ota', 'ota.state', 'ota.latest', 'ota.progress'];
let otaPoll      = null;    // setInterval-Handle
let otaPollUntil = 0;       // so lange mindestens nachfragen (der OTA-Task greift erst auf)
let otaTarget    = null;    // Version, deren Installation diese Seite angestossen hat
let otaNote      = null;    // {text, cls, st}: Meldung statt Zustandstext, bis der Zustand wechselt
let otaLastState = null;
let otaAutoCheck = null;    // Timer: einmal pruefen, wenn das Geraet es seit dem Start nie tat
let otaAwaitDump = false;   // Verbindung riss waehrend der Installation ab: urteilen erst nach dem Dump

function cfgValue(k) { const it = cfgItems.find(i => i.key === k); return it ? it.value : undefined; }
function cfgOpen()   { return !!cfgWS && cfgWS.readyState === WebSocket.OPEN; }

function renderOta() {
  if (!$('ota-card')) return;                          // HTML ohne Firmware-Karte
  const st = cfgValue('ota.state');
  $('ota-card').hidden = (st === undefined);          // Build ohne Online-Update
  if (st === undefined) { $('brand-update').hidden = true; return; }
  const latest  = cfgValue('ota.latest') || '';
  const running = cfgValue('fw.version') || '';
  const prog    = parseInt(cfgValue('ota.progress'), 10);
  const detail  = cfgValue('ota') || '';
  if (st !== otaLastState) {
    otaLastState = st;
    // Die Klartextzeile ?ota (traegt die Fehlermeldung) wird nicht gepusht,
    // ota.latest pollt otaPollOnce() nicht mit -- beide beim Zustandswechsel.
    if (cfgOpen()) ['ota', 'ota.latest'].forEach(k => cfgWS.send(JSON.stringify({cmd:'get', key: k})));
  }
  if (otaNote && otaNote.st !== st) otaNote = null;
  // Eine Installation, die ohne Neustart endet (Fehler, nichts Neueres da),
  // gibt die Knoepfe wieder frei.
  // Nach einem Verbindungsabriss entscheidet allein otaAfterDump(): ein
  // Live-Push zwischen Neuverbindung und dump_end zeigt schon "validating".
  if (otaTarget !== null && !otaAwaitDump && cfgOpen() && Date.now() > otaPollUntil &&
      st !== 'checking' && st !== 'installing') otaTarget = null;

  setText('ota-running', running || '…');
  setText('ota-latest', latest || 'not checked yet');
  let text = '', cls = '';
  if (st === 'checking')        text = 'Checking install.busware.de …';
  else if (st === 'uptodate')   { text = 'Up to date.'; cls = 'ok'; }
  else if (st === 'available')  { text = 'Update available: ' + latest; cls = 'avail'; }
  else if (st === 'installing') text = 'Downloading ' + latest + ' … ' + (prog >= 0 ? prog : 0) + ' %';
  else if (st === 'validating') text = 'Confirming the new firmware (about one minute) …';
  else if (st === 'error') {
    const msg = detail.startsWith('error: ') ? detail.slice(7).replace(/ \(running [^)]*\)$/, '') : '';
    text = msg ? 'Error: ' + msg : 'Error.';
    cls = 'err';
  }
  if (otaNote) { text = otaNote.text; cls = otaNote.cls; }
  const s = $('ota-status');
  s.textContent = text;
  s.className = 'ota-status' + (cls ? ' ' + cls : '');
  $('ota-bar').hidden = (st !== 'installing');
  $('ota-bar-fill').style.width = (prog >= 0 ? prog : 0) + '%';

  const busy = st === 'checking' || st === 'installing' || otaTarget !== null;
  $('ota-check').disabled   = !cfgOpen() || busy;
  $('ota-install').disabled = !cfgOpen() || busy || st !== 'available';
  $('ota-install').textContent = (st === 'available' && latest) ? 'Install ' + latest : 'Install update';
  const pill = $('brand-update');
  pill.hidden = (st !== 'available');
  pill.textContent = 'update ' + latest;

  // Laeuft ein Auftrag -- aus diesem Tab, einem anderen, der Konsole oder dem
  // Tagesplan --, fragt die Karte selbst nach: der Live-Push schickt waehrend-
  // dessen nur noch ota.* alle 5 s (frontend_json, seit 0.1.885).
  if (!otaPoll && cfgOpen() && (st === 'checking' || st === 'installing')) otaStartPoll();
  if (otaPoll && Date.now() > otaPollUntil && st !== 'checking' && st !== 'installing') {
    clearInterval(otaPoll);
    otaPoll = null;
  }
}

function otaPollOnce() {
  if (!cfgOpen()) return;
  // Waehrend der Pruefung (TLS-Handshake im Geraet, der knappste Heap-Moment)
  // nur der Zustand; Fortschritt gibt es erst beim Download, ota.latest und die
  // Klartextzeile holt renderOta() beim Zustandswechsel.
  const keys = cfgValue('ota.state') === 'installing' ? ['ota.state', 'ota.progress'] : ['ota.state'];
  keys.forEach(k => cfgWS.send(JSON.stringify({cmd:'get', key: k})));
}

function otaStartPoll() {
  otaPollUntil = Date.now() + 4000;
  setTimeout(otaPollOnce, 300);         // der OTA-Task hat den Auftrag dann schon
  if (!otaPoll) otaPoll = setInterval(otaPollOnce, 1000);
}

function otaRequest(action) {
  if (!cfgOpen()) return;
  otaNote = null;
  cfgWS.send(JSON.stringify({cmd:'set', key:'ota', value: action}));
  otaStartPoll();
  renderOta();
}

function otaInstall() {
  const latest = cfgValue('ota.latest') || '';
  if (!confirm('Install firmware ' + latest + '?\n\nThe stick downloads it from install.busware.de and ' +
               'restarts. Radio and network are interrupted for about half a minute.')) return;
  otaTarget = latest;
  otaRequest('install');
  otaNote = {text: 'Starting the update …', cls: '', st: cfgValue('ota.state')};
  renderOta();
}

function otaRefused() {
  otaTarget = null;
  otaNote = {text: 'The stick refused the request (e.g. while it confirms a new firmware).',
             cls: 'err', st: cfgValue('ota.state')};
  renderOta();
}

function otaOnClose() {
  if (otaPoll) { clearInterval(otaPoll); otaPoll = null; }
  if (otaAutoCheck) { clearTimeout(otaAutoCheck); otaAutoCheck = null; }
  if (otaTarget !== null) {
    otaAwaitDump = true;
    otaNote = {text: 'Restarting with ' + otaTarget + ' …', cls: '', st: cfgValue('ota.state')};
  }
  if (cfgItems.length) renderOta();     // Knoepfe sperren, solange keine Verbindung steht
}

function otaAfterDump() {
  otaAwaitDump = false;
  if (otaTarget !== null) {
    const running = cfgValue('fw.version');
    const st = cfgValue('ota.state');
    if (running === otaTarget) {
      otaNote = {text: 'Updated to ' + running + '.', cls: 'ok', st};
      otaTarget = null;
    } else if (st !== 'installing' && st !== 'checking') {
      otaNote = {text: 'Not updated - still running ' + running + '.', cls: 'err', st};
      otaTarget = null;
    } else {
      otaNote = null;   // "Restarting with ..." war verfrueht: kein Neustart,
      otaStartPoll();   // der Download laeuft noch, die Verbindung war nur kurz weg
    }
  }
  renderOta();
  // Versionsvergleich beim Oeffnen: hat das Geraet seit dem Start noch nie
  // geprueft (erste Pruefung sonst nach 5 min), einmal anstossen -- verzoegert,
  // damit der TLS-Abruf nicht mit dem Seitenaufbau um den Heap konkurriert.
  if (!otaAutoCheck && cfgValue('ota.state') === 'idle' && !cfgValue('ota.latest')) {
    otaAutoCheck = setTimeout(() => {
      otaAutoCheck = null;
      if (cfgValue('ota.state') === 'idle' && !cfgValue('ota.latest')) otaRequest('check');
    }, 3000);
  }
}

// ============ Settings-Form-Generator ============
function renderSettings() {
  const host = $('settings-host');
  host.innerHTML = '';
  // Gruppieren per Prefix vor erstem '.'.
  // Filter: readonly-rf.*-Eintraege ausblenden — die werden bereits in der
  // HF-Frontend-Karte oben angezeigt; nur editierbare RF-Settings (rf.pa.idx)
  // bleiben im Form-Generator.
  const groups = {};
  cfgItems.forEach(it => {
    if (it.key.startsWith('rf.') && it.readonly) return;
    if (OTA_CARD_KEYS.includes(it.key)) return;   // stehen in der Firmware-Karte
    const grp = (it.key.indexOf('.') >= 0) ? it.key.split('.')[0] : 'misc';
    (groups[grp] ||= []).push(it);
  });
  // Sortierte Gruppen mit Priorität: rf zuerst (HF-Frontend-Status), dann
  // wifi/tcp/json (Connectivity), dann der Rest alphabetisch.
  const priority = ['rf', 'wifi', 'tcp', 'json'];
  const ordered = priority.filter(g => groups[g]).concat(
    Object.keys(groups).filter(g => !priority.includes(g)).sort()
  );
  ordered.forEach(grp => {
    const sec = document.createElement('div');
    sec.className = 'section';
    const hdr = document.createElement('div');
    hdr.className = 'section-header';
    hdr.textContent = grp;
    sec.appendChild(hdr);
    groups[grp].forEach(it => sec.appendChild(renderRow(it)));
    host.appendChild(sec);
  });
}

function renderRow(it) {
  const row = document.createElement('div');
  row.className = 'setting-row';
  row.dataset.key = it.key;

  const keyDiv = document.createElement('div');
  keyDiv.className = 'setting-key';
  // Als Text, nicht als HTML: Hilfetexte enthalten Platzhalter wie <ipv4> oder
  // <n>, die innerHTML als Tags schluckte ("any" or /).
  const keySpan = document.createElement('span');
  keySpan.textContent = it.key;
  const helpSmall = document.createElement('small');
  helpSmall.textContent = it.help || '';
  keyDiv.append(keySpan, helpSmall);
  row.appendChild(keyDiv);

  const inputCell = document.createElement('div');
  inputCell.className = 'setting-input-cell';
  let input;
  if (it.readonly) {
    // Readonly-Werte als scrollbarer Text statt <input> rendern, damit lange
    // Strings (z.B. build="rf:moritz,rf:asksin,...") horizontal scrollbar sind
    // statt rechts abgeschnitten zu werden.
    input = document.createElement('div');
    input.className = 'setting-input setting-readonly';
    input.textContent = it.value;
    input.title = it.value;          // Tooltip mit voller Wert
  } else if (it.type === 'intenum' && it.enum_labels) {
    input = document.createElement('select');
    it.enum_labels.forEach((lbl, idx) => {
      const opt = document.createElement('option');
      opt.value = idx; opt.textContent = `${idx} — ${lbl}`;
      input.appendChild(opt);
    });
    input.value = it.value;
    input.className = 'setting-input';
  } else {
    input = document.createElement('input');
    input.type = (it.type === 'secret') ? 'password' :
                 (it.type === 'u8' || it.type === 'u16' || it.type === 'u32') ? 'number' : 'text';
    input.value = (it.type === 'secret' && it.value === '***') ? '' : it.value;
    if (it.type === 'secret' && it.value === '***') input.placeholder = '*** (set)';
    input.title = it.value;          // Tooltip mit voller Wert
    if (it.unit) input.title = it.value + ' (unit: ' + it.unit + ')';
    input.className = 'setting-input';
  }
  input.dataset.original = it.value;
  if (input.tagName === 'INPUT' || input.tagName === 'SELECT') {
    input.addEventListener('input', () => {
      if (input.value != input.dataset.original) row.classList.add('dirty');
      else                                       row.classList.remove('dirty');
      // Tooltip live updaten
      input.title = input.value;
    });
    input.addEventListener('keydown', e => {
      if (e.key === 'Enter') { e.preventDefault(); saveRow(row, it); }
    });
  }
  inputCell.appendChild(input);
  row.appendChild(inputCell);

  const act = document.createElement('div');
  act.className = 'setting-actions';
  if (!it.readonly) {
    const btn = document.createElement('button');
    btn.className = 'btn-primary save';
    btn.textContent = 'Save';
    btn.addEventListener('click', () => saveRow(row, it));
    act.appendChild(btn);
  }
  const flags = document.createElement('span');
  flags.className = 'setting-flags';
  flags.textContent = (it.persist?'P':'-') + (it.readonly?'R':'-') + (it.sensitive?'S':'-');
  flags.title = (it.persist?'persist ':'') + (it.readonly?'readonly ':'') + (it.sensitive?'sensitive':'');
  act.appendChild(flags);
  row.appendChild(act);

  return row;
}

function saveRow(row, it) {
  const inp = row.querySelector('.setting-input');
  const val = inp.value.toString();
  if (cfgWS && cfgWS.readyState === WebSocket.OPEN) {
    cfgWS.send(JSON.stringify({cmd:'set', key: it.key, value: val}));
    inp.dataset.original = val;
  }
}

window.refresh = () => { if (cfgWS && cfgWS.readyState === WebSocket.OPEN) cfgWS.send(JSON.stringify({cmd:'dump'})); };
window.downloadJson = () => {
  const blob = new Blob([JSON.stringify(cfgItems, null, 2)], {type: 'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'culfw32-config.json';
  a.click();
};

// ============ Terminal-Connection (TTY-Tab) ============
let termWS = null;
const HISTORY_KEY = 'culfw32.term.history';
let history = JSON.parse(localStorage.getItem(HISTORY_KEY) || '[]');
let histIdx = history.length;     // pointer; histIdx == length = "neue Zeile"

function termSetState(s) {
  const el = $('term-state');
  el.textContent = s;
  el.classList.toggle('connected', s === 'connected');
}

function termConnect() {
  if (termWS) return;
  termSetState('connecting…');
  termWS = new WebSocket(wsURL('/ws/tty'));
  termWS.onopen = () => {
    termSetState('connected');
    logTerm('-- connected --\n', 'info');
    // Initial-Befehl `V` schicken damit der User den Banner sieht.
    termWS.send('V\r\n');
  };
  termWS.onmessage = ev => {
    logTerm(typeof ev.data === 'string' ? ev.data : '<binary>\n', 'reply');
  };
  termWS.onclose = () => {
    termSetState('disconnected');
    logTerm('-- disconnected --\n', 'info');
    termWS = null;
    setTimeout(() => { if ($('tab-terminal').classList.contains('active')) termConnect(); }, 2000);
  };
  termWS.onerror = () => { logTerm('-- error --\n', 'err'); };
}

window.termClear = () => { $('term-output').innerHTML = ''; };

const termInput = $('term-input');
termInput.addEventListener('keydown', e => {
  if (e.key === 'Enter') {
    const cmd = termInput.value;
    if (cmd === '') return;
    if (termWS && termWS.readyState === WebSocket.OPEN) {
      logTerm('> ' + cmd + '\n', 'echo');
      termWS.send(cmd + '\r\n');
    } else {
      logTerm('-- not connected --\n', 'err');
    }
    if (history[history.length-1] !== cmd) {
      history.push(cmd);
      if (history.length > 200) history.shift();
      localStorage.setItem(HISTORY_KEY, JSON.stringify(history));
    }
    histIdx = history.length;
    termInput.value = '';
  } else if (e.key === 'ArrowUp') {
    e.preventDefault();
    if (histIdx > 0) {
      histIdx--;
      termInput.value = history[histIdx] || '';
    }
  } else if (e.key === 'ArrowDown') {
    e.preventDefault();
    if (histIdx < history.length) {
      histIdx++;
      termInput.value = history[histIdx] || '';
    }
  } else if (e.key === 'l' && (e.ctrlKey || e.metaKey)) {
    e.preventDefault();
    termClear();
  }
});

// ============ Send Tab ============
const SEND_LOG_KEY = 'culfw32.send.log';
const SEND_LAST_KEY = 'culfw32.send.last';
let sendLog = JSON.parse(localStorage.getItem(SEND_LOG_KEY) || '[]');

function sendShowConditional() {
  const proto = $('send-proto').value;
  document.querySelectorAll('.send-cond').forEach(el => {
    const show = el.dataset.show;
    el.classList.toggle('hidden', show !== proto);
  });
}

function sendValidateHex(hex, proto, kind) {
  // Strip ' ' / ':', uppercase
  const clean = hex.replace(/[\s:]/g, '').toUpperCase();
  if (clean.length === 0) return {ok:false, msg:'empty'};
  if (clean.length % 2 !== 0) return {ok:false, msg:'odd number of hex digits'};
  if (!/^[0-9A-F]+$/.test(clean)) return {ok:false, msg:'invalid hex characters'};
  const nbytes = clean.length / 2;
  if (proto === 'slowrf' && kind === 'em' && nbytes !== 9) {
    return {ok:false, msg:'EM needs exactly 9 bytes (got '+nbytes+')'};
  }
  if (proto === 'slowrf' && kind === 'fs20' && (nbytes < 4 || nbytes > 8)) {
    return {ok:false, msg:'FS20 expects 4–8 bytes (got '+nbytes+')'};
  }
  if (proto === 'slowrf' && kind === 'ks300' && (nbytes < 1 || nbytes > 12)) {
    return {ok:false, msg:'KS300 expects 1–12 bytes (got '+nbytes+')'};
  }
  if ((proto === 'moritz' || proto === 'asksin') && nbytes >= 2) {
    const lenByte = parseInt(clean.substr(0,2), 16);
    if (lenByte + 1 !== nbytes) {
      return {ok:false, msg:'length byte says '+lenByte+' but '+(nbytes-1)+' bytes follow'};
    }
  }
  return {ok:true, msg:nbytes+' byte'+(nbytes===1?'':'s')+' ready', clean};
}

function sendUpdateValidate() {
  const proto = $('send-proto').value;
  const kind  = $('send-kind').value;
  const hex   = $('send-hex').value;
  const v = sendValidateHex(hex, proto, kind);
  const el = $('send-validate');
  el.textContent = v.msg;
  el.className = 'send-validate ' + (v.ok ? 'ok' : 'err');
}

function sendDoSend() {
  if (!cfgWS || cfgWS.readyState !== WebSocket.OPEN) {
    sendAddLogEntry({proto:'?', kind:'-', hex:'', ok:false, err:'no WS connection'});
    return;
  }
  const proto = $('send-proto').value;
  const kind  = $('send-kind').value;
  const hex   = $('send-hex').value;
  const v = sendValidateHex(hex, proto, kind);
  if (!v.ok) {
    sendAddLogEntry({proto, kind: proto==='slowrf'?kind:'-', hex, ok:false, err:'client: '+v.msg});
    return;
  }
  const payload = {cmd:'tx', proto, hex: v.clean};
  if (proto === 'slowrf') {
    payload.kind = kind;
    const r = parseInt($('send-repeat').value, 10);
    if (!isNaN(r) && r >= 1 && r <= 15 && r !== 3) payload.repeat = r;
  }
  if (proto === 'moritz') payload.long_preamble = $('send-longpre').checked;

  // Letzte Auswahl persistieren fuer nächsten Page-Load.
  localStorage.setItem(SEND_LAST_KEY, JSON.stringify({proto, kind, hex: v.clean,
    longpre: $('send-longpre').checked, repeat: $('send-repeat').value}));

  // Pending-Eintrag ins Log (wird beim tx_ack gepatcht).
  pendingSend = {proto, kind: proto==='slowrf'?kind:'-', hex: v.clean,
                 ts: new Date(), pending: true};
  cfgWS.send(JSON.stringify(payload));
}

let pendingSend = null;

function handleTxAck(m) {
  const entry = pendingSend || {proto: m.proto, kind: '-', hex: '', ts: new Date()};
  entry.ok = !!m.ok;
  entry.err = m.err || '';
  entry.pending = false;
  pendingSend = null;
  sendAddLogEntry(entry);
}

function sendAddLogEntry(e) {
  if (!e.ts) e.ts = new Date();
  // Save serializable form
  const ser = {ts: e.ts.toISOString ? e.ts.toISOString() : e.ts, ...e};
  delete ser.pending;
  sendLog.unshift(ser);
  if (sendLog.length > 50) sendLog.length = 50;
  localStorage.setItem(SEND_LOG_KEY, JSON.stringify(sendLog));
  renderSendLog();
}

function renderSendLog() {
  const tbody = $('send-log-tbody');
  if (!tbody) return;
  tbody.innerHTML = '';
  $('send-log-count').textContent = sendLog.length + (sendLog.length===1 ? ' entry' : ' entries');
  sendLog.forEach(e => {
    const tr = document.createElement('tr');
    const t = new Date(e.ts);
    const ts = isNaN(t) ? e.ts : t.toLocaleTimeString();
    // Als Text, nicht als HTML (wie renderRow seit 0.1.885): e.hex ist bei
    // fehlgeschlagener Validierung die rohe Eingabe, e.err kommt vom Geraet.
    const cell = (text, cls, title) => {
      const td = document.createElement('td');
      td.textContent = text;
      if (cls) td.className = cls;
      if (title !== undefined) td.title = title;
      return td;
    };
    tr.append(cell(ts), cell(e.proto || '-'), cell(e.kind || '-'),
              cell(e.hex || '', 'hex-cell', e.hex || ''),
              cell(e.ok ? '✓' : '✗', e.ok ? 'ok-yes' : 'ok-no'),
              cell(e.err || ''));
    tbody.appendChild(tr);
  });
}

// Restore last-used form values from localStorage.
function sendRestoreLast() {
  try {
    const last = JSON.parse(localStorage.getItem(SEND_LAST_KEY) || '{}');
    if (last.proto)  $('send-proto').value  = last.proto;
    if (last.kind)   $('send-kind').value   = last.kind;
    if (last.hex)    $('send-hex').value    = last.hex;
    if (last.repeat) $('send-repeat').value = last.repeat;
    if (last.longpre !== undefined) $('send-longpre').checked = !!last.longpre;
  } catch(e) {}
  sendShowConditional();
  sendUpdateValidate();
}

// Wire up Send-Tab UI (after DOM ready — IIFE runs at end of body so OK).
$('send-proto').addEventListener('change', () => { sendShowConditional(); sendUpdateValidate(); });
$('send-kind').addEventListener('change',  sendUpdateValidate);
$('send-hex').addEventListener('input',    sendUpdateValidate);
$('send-go').addEventListener('click',     sendDoSend);
$('send-clear-log').addEventListener('click', () => {
  sendLog = []; localStorage.setItem(SEND_LOG_KEY, '[]'); renderSendLog();
});
$('send-hex').addEventListener('keydown', e => {
  if (e.key === 'Enter') { e.preventDefault(); sendDoSend(); }
});
document.querySelectorAll('.send-presets .preset').forEach(b => {
  b.addEventListener('click', () => {
    $('send-proto').value = b.dataset.proto;
    if (b.dataset.kind) $('send-kind').value = b.dataset.kind;
    $('send-hex').value = b.dataset.hex;
    sendShowConditional();
    sendUpdateValidate();
    $('send-hex').focus();
  });
});
sendRestoreLast();
renderSendLog();

// Firmware-Karte + Hinweis im Kopf (fehlt die Karte im HTML, bleibt der Rest heil)
if ($('ota-check') && $('ota-install') && $('brand-update')) {
  $('ota-check').addEventListener('click', () => otaRequest('check'));
  $('ota-install').addEventListener('click', otaInstall);
  $('brand-update').addEventListener('click', e => {
    e.preventDefault();
    document.querySelector('.tab[data-tab="config"]').click();
    $('ota-card').scrollIntoView({behavior: 'smooth', block: 'center'});
  });
}

// ============ Init ============
console.log('[CULFW32] app.js loaded — build', '2026-04-28-version-in-header');
cfgConnect();

})();
