#pragma once

void tmcSetDir(bool dir);   // definiert in firmware.ino
void servoMoveTo(int angle, bool tracking = false); // definiert in firmware.ino

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ESP32Servo.h>

// ─── Globals defined in firmware.ino ──────────────────────────────────────
extern WebServer     server;
extern DNSServer     dnsServer;
extern Preferences   prefs;
extern Servo         elevationServo;
extern int           servoMin, servoMax, servoManualAngle;
extern int           servoCalAngle, servoTarget, servoActual;
extern int           elevAxisMode;
extern bool          servoEnabled, servoInitDone;
extern bool          manualControl;
extern unsigned long lastServoCmd, lastMotorTime;
extern long          currentSteps, azStepMin, azStepMax;
extern float         heading_yaw, pitch, roll, targetAzimuth, targetElevation;
extern float         gz_offset;
extern double        gps_lat, gps_lon, sat_lon;
extern float         az_offset;
extern char          wlan_ssid[64], wlan_pass[64];
extern bool          ak8963_ok, qmc_ok, hmc_ok, trackingEnabled;
extern float         azMountOffset;
extern float         dbg_ax, dbg_ay, dbg_az;
extern float         dbg_gx, dbg_gy, dbg_gz;
extern float         dbg_mx, dbg_my, dbg_mz;
extern float         dbg_gz_net, loopHz;
extern bool          dbg_dir;
extern bool          positionDirty;
extern bool          motorRunning, vibrationDetected, compassFrozen, compassStable;
extern float         compassHeading;
extern void          recalcSat();
extern void          calibrateGyroOffset();
extern void          stepMotorManual(bool dir, int count);

// ─── HTML ─────────────────────────────────────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoSat</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#1a1a2e;color:#eee;max-width:440px;margin:0 auto;padding:12px}
h2{color:#a0c4ff;text-align:center;padding:10px 0 14px}
.tabs{display:flex;gap:3px;margin-bottom:12px}
.tab{flex:1;padding:8px 2px;background:#16213e;border:none;color:#666;border-radius:8px;cursor:pointer;font-size:.75em}
.tab.on{background:#0f3460;color:#a0c4ff}
.box{background:#16213e;border-radius:12px;padding:14px;margin:10px 0}
.row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #222}
.row:last-child{border:none}
.lbl{color:#888;font-size:.88em}
.val{color:#a0c4ff;font-weight:bold}
.big{font-size:2em;color:#a0c4ff;font-weight:bold}
.center{text-align:center}
button{padding:11px 14px;margin:3px;border:none;border-radius:8px;cursor:pointer;color:#fff;font-size:.95em}
.bm{background:#334}.bmin{background:#8b0000}.bmax{background:#006400}
.bsave{background:#00408b;width:100%;padding:13px;font-size:1.05em;margin-top:8px}
.bwarn{background:#7a3800;width:100%;padding:13px;font-size:1.05em}
.bdis{background:#4a0000;width:100%;padding:11px;font-size:.95em;margin-top:6px}
.btrack-on {background:#006400;width:100%;padding:14px;font-size:1.1em;font-weight:bold}
.btrack-off{background:#8b0000;width:100%;padding:14px;font-size:1.1em;font-weight:bold}
.bsm{padding:9px 12px;font-size:.88em}
label{display:block;color:#888;font-size:.82em;margin:8px 0 2px}
input{background:#0f3460;color:#eee;border:1px solid #334;border-radius:6px;padding:8px;width:100%;font-size:.95em}
.pw-wrap{position:relative}.pw-wrap input{padding-right:44px}
.pw-eye{position:absolute;right:8px;top:50%;transform:translateY(-50%);background:none;border:none;color:#888;cursor:pointer;width:auto;padding:4px}
.msg{text-align:center;color:#ffd700;min-height:1.3em;margin-top:8px;font-size:.9em}
.note{color:#666;font-size:.8em;line-height:1.4;margin-bottom:10px}
.pill{display:inline-block;padding:2px 8px;border-radius:10px;font-size:.8em}
.pill.ok{background:#1a4a1a;color:#4f4}
.pill.off{background:#4a1a1a;color:#f44}
.track-state{text-align:center;font-size:1.3em;font-weight:bold;padding:8px 0 12px;letter-spacing:.05em}
.track-state.on{color:#4f4}.track-state.off{color:#f44}
.dbg-sec{color:#555;font-size:.72em;text-transform:uppercase;letter-spacing:.06em;padding:10px 0 3px;border-bottom:1px solid #1e1e3a;margin-bottom:2px}
.dbg-sec:first-child{padding-top:2px}
.dbg-row{display:grid;grid-template-columns:1.2em 1fr 1.2em 1fr 1.2em 1fr;gap:2px 6px;padding:4px 0;font-family:monospace;font-size:.88em;align-items:center}
.dbg-row .ax{color:#555;font-size:.8em}.dbg-row .av{color:#a0c4ff}
.dbg-kv{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #1e1e3a;font-family:monospace;font-size:.9em}
.dbg-kv:last-child{border:none}.dk{color:#666}.dv{color:#a0c4ff}
.dv.hi{color:#f44}.dv.warn{color:#fa0}.dv.ok{color:#4f4}.dv.off{color:#f44}
.lim-row{display:flex;justify-content:space-between;padding:5px 0;font-size:.88em}
.lim-row span:first-child{color:#888}.lim-row span:last-child{color:#a0c4ff;font-family:monospace}
.hide{display:none}
</style>
</head>
<body>
<h2>&#x1F4E1; AutoSat</h2>
<div class="tabs">
  <button class="tab on"  onclick="tab('s',this)">Status</button>
  <button class="tab"     onclick="tab('e',this)">Elev.</button>
  <button class="tab"     onclick="tab('g',this)">GPS</button>
  <button class="tab"     onclick="tab('m',this)">Motor</button>
  <button class="tab"     onclick="tab('w',this)">WLAN</button>
  <button class="tab"     onclick="tab('d',this)">Debug</button>
</div>

<!-- STATUS -->
<div id="ts">
  <div class="box" id="statBox"><div class="center" style="color:#666;padding:20px">Lade...</div></div>
</div>

<!-- ELEVATION -->
<div id="te" class="hide">
  <div class="box">
    <div class="dbg-sec">Servo-Freigabe</div>
    <div style="font-size:12px;color:#aaa;margin-bottom:10px">
      Servo ist nach dem Start <b>deaktiviert</b>. Erst freigeben wenn die Sch&uuml;ssel frei beweglich ist.<br>
      Nach Freigabe f&auml;hrt der Servo direkt auf den gespeicherten Kalibrierwinkel. Alle weiteren Bewegungen laufen langsam (Rampe).
    </div>
    <button class="btrack-on" onclick="srvEnable()">Servo aktivieren</button>
    <div class="msg" id="eSrvEnMsg"></div>
  </div>
  <div class="box center">
    <div class="lbl">Servo-Winkel</div>
    <div class="big" id="eAng">--</div>&deg;
    <br><br>
    <button class="bm" onclick="mv(-10)">&#x2212;10&deg;</button>
    <button class="bm" onclick="mv(-1)">&#x2212;1&deg;</button>
    <button class="bm" onclick="mv(1)">+1&deg;</button>
    <button class="bm" onclick="mv(10)">+10&deg;</button>
  </div>
  <div class="box">
    <div class="center" style="margin-bottom:10px">
      <button class="bmin" onclick="lim('min')">MIN setzen</button>
      <button class="bmax" onclick="lim('max')">MAX setzen</button>
    </div>
    <div class="row"><span class="lbl">MIN</span><span class="val" id="eMin">--</span></div>
    <div class="row"><span class="lbl">MAX</span><span class="val" id="eMax">--</span></div>
    <button class="bsave" onclick="saveElev()">Speichern</button>
    <div class="msg" id="eMsg"></div>
  </div>
  <div class="box">
    <div class="lbl" style="margin-bottom:8px">Elevation kalibrieren</div>
    <div style="font-size:12px;color:#aaa;margin-bottom:10px">Sch&uuml;ssel manuell auf Satellitensignal ausrichten, dann Kalibrieren dr&uuml;cken. Dieser Servo-Winkel wird als Referenz gespeichert.</div>
    <button class="bsave" onclick="calibrateElev()">Kalibrieren &amp; speichern</button>
    <div class="msg" id="eCalMsg"></div>
  </div>
  <div class="box">
    <div class="lbl" style="margin-bottom:8px">Kompass neu einlesen</div>
    <div style="font-size:12px;color:#aaa;margin-bottom:10px">Setzt Heading einmalig auf aktuellen Kompasswert. Nur im Stillstand m&ouml;glich (Motor aus, keine Vibration).</div>
    <button class="bwarn" onclick="compassReset()">Kompass neu einlesen</button>
    <div class="msg" id="eCompMsg"></div>
  </div>
  <div class="box">
    <div class="lbl" style="margin-bottom:8px">Korrektur-Achse</div>
    <div style="font-size:12px;color:#aaa;margin-bottom:10px">Welche IMU-Achse korrigiert die Neigung? Testen: Tracking an, Boot kippen &rarr; Schüssel muss nachfolgen. Sonst Achse wechseln.</div>
    <div id="axisLabel" style="text-align:center;font-size:1.1em;margin-bottom:8px;color:#4fc3f7">--</div>
    <div style="display:flex;gap:6px;flex-wrap:wrap;justify-content:center">
      <button class="bsm" onclick="setAxis(0)">+Pitch</button>
      <button class="bsm" onclick="setAxis(1)">+Roll</button>
      <button class="bsm" onclick="setAxis(2)">&minus;Pitch</button>
      <button class="bsm" onclick="setAxis(3)">&minus;Roll</button>
    </div>
    <div class="msg" id="eAxisMsg" style="margin-top:8px"></div>
  </div>
</div>

<!-- GPS / SAT -->
<div id="tg" class="hide">
  <div class="box">
    <label>Breitengrad (&deg;N)</label>
    <input type="number" id="gLat" step="0.0001">
    <label>L&auml;ngengrad (&deg;E)</label>
    <input type="number" id="gLon" step="0.0001">
    <label>Satellit (&deg;Ost)</label>
    <input type="number" id="gSat" step="0.1">
    <label>Azimut-Offset (&deg;)</label>
    <input type="number" id="gOff" step="0.1">
    <button class="bsave" onclick="saveCfg()">Speichern &amp; Neuberechnen</button>
    <div class="msg" id="gMsg"></div>
  </div>
</div>

<!-- MOTOR -->
<div id="tm" class="hide">
  <div class="box">
    <div class="row"><span class="lbl">Steps</span><span class="val" id="mSteps">--</span></div>
    <div class="row"><span class="lbl">Position</span><span class="val" id="mPos">--&deg;</span></div>
    <div class="row"><span class="lbl">Ziel-Azimut</span><span class="val" id="mTaz">--&deg;</span></div>
  </div>
  <div class="box">
    <div class="note">Position zur&uuml;cksetzen: Sch&uuml;ssel muss auf aktuellen Heading zeigen.</div>
    <button class="bwarn" onclick="resetSteps()">Position auf 0 zur&uuml;cksetzen</button>
    <div class="msg" id="mMsg"></div>
  </div>
</div>

<!-- WLAN -->
<div id="tw" class="hide">
  <div class="box">
    <div class="row"><span class="lbl">Hotspot</span><span class="val"><span class="pill ok">AutoSat aktiv</span></span></div>
    <div class="row"><span class="lbl">Netzwerk</span><span class="val"><span id="wNetPill" class="pill off">getrennt</span></span></div>
    <div class="row" id="wIpRow" style="display:none"><span class="lbl">IP / Hostname</span><span class="val" id="wIp">--</span></div>
    <div class="row" id="wSigRow" style="display:none"><span class="lbl">Signal</span><span class="val" id="wSig">--</span></div>
  </div>
  <div class="box">
    <label>WLAN-Name (SSID)</label>
    <input type="text" id="wSSID" autocomplete="off" placeholder="Netzwerkname">
    <label>Passwort</label>
    <div class="pw-wrap">
      <input type="password" id="wPass" autocomplete="new-password" placeholder="Passwort">
      <button class="pw-eye" onclick="togglePw()" type="button">&#x1F441;</button>
    </div>
    <button class="bsave" onclick="wlanSave()">Verbinden &amp; Speichern</button>
    <button class="bdis"  onclick="wlanDisc()">Verbindung l&ouml;schen</button>
    <div class="msg" id="wMsg"></div>
  </div>
</div>

<!-- DEBUG -->
<div id="td" class="hide">

  <!-- Tracking -->
  <div class="box">
    <div class="dbg-sec">Tracking</div>
    <div class="track-state off" id="dTrackState">■ TRACKING AUS</div>
    <button id="dTrackBtn" class="btrack-on" onclick="toggleTracking()">Tracking starten</button>
    <div class="msg" id="dTrackMsg"></div>
  </div>

  <!-- Süd-Referenz Kalibrierung -->
  <div class="box">
    <div class="dbg-sec">S&uuml;d-Referenz setzen</div>
    <div style="font-size:12px;color:#aaa;margin-bottom:10px">
      Plattform physisch nach S&uuml;den ausrichten &rarr; Knopf dr&uuml;cken.<br>
      Setzt Steps=0, Limits &plusmn;180&deg; und Richtungsoffset. Einmalig beim Aufbau.
    </div>
    <button class="bsave" onclick="southCal()">S&uuml;d-Referenz setzen</button>
    <div class="msg" id="dSouthCalMsg"></div>
  </div>

  <!-- Azimut Kalibrierung -->
  <div class="box">
    <div class="dbg-sec">Azimut Kalibrierung</div>
    <div class="note" style="margin-bottom:8px">Tracking muss AUS sein. Motor manuell zum Anfangs-/Endpunkt fahren, dann MIN/MAX setzen.</div>
    <div class="center" style="margin-bottom:8px">
      <button class="bm bsm" onclick="azStep(-10)">&#x2212;10&deg;</button>
      <button class="bm bsm" onclick="azStep(-1)">&#x2212;1&deg;</button>
      <button class="bm bsm" onclick="azStep(1)">+1&deg;</button>
      <button class="bm bsm" onclick="azStep(10)">+10&deg;</button>
    </div>
    <div class="lim-row"><span>Position</span><span id="dAzPos">-- Steps (--&deg;)</span></div>
    <div class="lim-row"><span>MIN</span><span id="dAzMin">--</span></div>
    <div class="lim-row"><span>MAX</span><span id="dAzMax">--</span></div>
    <div class="center" style="margin-top:8px">
      <button class="bmin bsm" onclick="setAzLim('min')">MIN setzen</button>
      <button class="bmax bsm" onclick="setAzLim('max')">MAX setzen</button>
    </div>
    <button class="bdis" style="margin-top:6px" onclick="resetHeading()">Heading auf 0 zur&uuml;cksetzen</button>
    <div class="msg" id="dAzMsg"></div>
  </div>

  <!-- Elevation Kalibrierung -->
  <div class="box">
    <div class="dbg-sec">Elevation Kalibrierung</div>
    <div class="note" style="margin-bottom:8px">Servo zum physischen Anschlag fahren, dann MIN/MAX setzen.</div>
    <div class="center" style="margin-bottom:8px">
      <button class="bm bsm" onclick="mv(-10)">&#x2212;10&deg;</button>
      <button class="bm bsm" onclick="mv(-1)">&#x2212;1&deg;</button>
      <button class="bm bsm" onclick="mv(1)">+1&deg;</button>
      <button class="bm bsm" onclick="mv(10)">+10&deg;</button>
    </div>
    <div class="lim-row"><span>Servo-Winkel</span><span id="dSrvAng">--&deg;</span></div>
    <div class="lim-row"><span>MIN</span><span id="dSrvMin">--&deg;</span></div>
    <div class="lim-row"><span>MAX</span><span id="dSrvMax">--&deg;</span></div>
    <div class="center" style="margin-top:8px">
      <button class="bmin bsm" onclick="lim('min')">MIN setzen</button>
      <button class="bmax bsm" onclick="lim('max')">MAX setzen</button>
    </div>
    <button class="bsave" style="margin-top:6px" onclick="saveElev()">Speichern</button>
    <div class="msg" id="dElMsg"></div>
  </div>

  <!-- IMU -->
  <div class="box">
    <div class="dbg-sec">Beschleunigung (g)</div>
    <div class="dbg-row">
      <span class="ax">X</span><span class="av" id="dAx">--</span>
      <span class="ax">Y</span><span class="av" id="dAy">--</span>
      <span class="ax">Z</span><span class="av" id="dAz">--</span>
    </div>
    <div class="dbg-sec">Gyro netto (&deg;/s)</div>
    <div class="dbg-row">
      <span class="ax">X</span><span class="av" id="dGx">--</span>
      <span class="ax">Y</span><span class="av" id="dGy">--</span>
      <span class="ax">Z</span><span class="av" id="dGz">--</span>
    </div>
    <div class="dbg-sec">Magnetometer (raw)</div>
    <div class="dbg-row">
      <span class="ax">X</span><span class="av" id="dMx">--</span>
      <span class="ax">Y</span><span class="av" id="dMy">--</span>
      <span class="ax">Z</span><span class="av" id="dMz">--</span>
    </div>
  </div>

  <!-- Lage -->
  <div class="box">
    <div class="dbg-sec">Lage</div>
    <div class="dbg-kv"><span class="dk">Pitch</span><span class="dv" id="dPitch">--</span></div>
    <div class="dbg-kv"><span class="dk">Roll</span><span class="dv" id="dRoll">--</span></div>
    <div class="dbg-kv"><span class="dk">Heading</span><span class="dv" id="dHdg">--</span></div>
    <div class="dbg-kv"><span class="dk">gz Offset</span><span class="dv" id="dGzOff">--</span></div>
    <div class="dbg-kv"><span class="dk">gz Heading</span><span id="dGzSt">--</span></div>
    <div class="dbg-kv"><span class="dk">Kompass-Status</span><span id="dCompSt">--</span></div>
    <div class="dbg-kv"><span class="dk">Kompass Heading</span><span class="dv" id="dCompHdg">--</span></div>
  </div>

  <!-- Motor Status -->
  <div class="box">
    <div class="dbg-sec">Motor Status</div>
    <div class="dbg-kv"><span class="dk">Steps</span><span class="dv" id="dSteps">--</span></div>
    <div class="dbg-kv"><span class="dk">Position</span><span class="dv" id="dMpos">--</span></div>
    <div class="dbg-kv"><span class="dk">Fehler</span><span class="dv" id="dErr">--</span></div>
    <div class="dbg-kv"><span class="dk">Richtung</span><span class="dv" id="dDir">--</span></div>
    <div class="dbg-kv"><span class="dk">Aktiv</span><span class="dv" id="dMact">--</span></div>
  </div>

  <!-- System -->
  <div class="box">
    <div class="dbg-sec">System</div>
    <div class="dbg-kv"><span class="dk">Uptime</span><span class="dv" id="dUp">--</span></div>
    <div class="dbg-kv"><span class="dk">Loop-Frequenz</span><span class="dv" id="dHz">--</span></div>
    <div class="dbg-kv"><span class="dk">Magnetometer</span><span class="dv" id="dMag">--</span></div>
  </div>

</div><!-- /debug -->

<script>
var cur=45,activeTab='s',dbgTimer=null,servoMin_js=0,servoMax_js=90;
var tids=['s','e','g','m','w','d'];
function tab(id,btn){
  activeTab=id;
  tids.forEach(function(t){document.getElementById('t'+t).classList.toggle('hide',t!==id);});
  document.querySelectorAll('.tab').forEach(function(b,i){b.classList.toggle('on',tids[i]===id);});
  if(id==='g')loadCfg();
  if(id==='w')updW();
  if(id==='d'){dbgTimer=setInterval(updD,500);updD();}
  else if(dbgTimer){clearInterval(dbgTimer);dbgTimer=null;}
}
function row(l,v){return '<div class="row"><span class="lbl">'+l+'</span><span class="val">'+v+'</span></div>';}
function fmt(v,d){return v.toFixed(d!==undefined?d:2);}
function fmtUp(ms){var s=Math.floor(ms/1000),h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;return('0'+h).slice(-2)+':'+('0'+m).slice(-2)+':'+('0'+sc).slice(-2);}
function sv(id,val){var el=document.getElementById(id);if(el)el.textContent=val;}
function sc(id,cls){var el=document.getElementById(id);if(el)el.className='dv '+(cls||'');}

function upd(){
  fetch('/status').then(function(r){return r.json();}).then(function(d){
    cur=d.servo; servoMin_js=d.smin; servoMax_js=d.smax;
    var axNames=['+Pitch','+Roll','-Pitch','-Roll'];
    var axEl=document.getElementById('axisLabel');
    if(axEl)axEl.textContent='Aktiv: '+axNames[d.elvaxis||0];
    document.getElementById('eAng').textContent=d.servo;
    document.getElementById('eMin').textContent=d.smin+'°';
    document.getElementById('eMax').textContent=d.smax+'°';
    document.getElementById('mSteps').textContent=d.steps;
    document.getElementById('mPos').textContent=d.mpos.toFixed(1)+'°';
    document.getElementById('mTaz').textContent=d.taz.toFixed(1)+'°';
    var err=d.taz-d.mpos;while(err>180)err-=360;while(err<-180)err+=360;
    document.getElementById('statBox').innerHTML=
      row('Tracking',d.tracking?'<span class="pill ok">AN</span>':'<span class="pill off">AUS</span>')+
      row('Heading',d.hdg.toFixed(1)+'°')+
      row('Ziel-Azimut',d.taz.toFixed(1)+'°')+
      row('Fehler',err.toFixed(1)+'°')+
      row('Ziel-Elevation',d.tel.toFixed(1)+'°')+
      row('Pitch (Boot)',d.pitch.toFixed(1)+'°')+
      row('Servo',d.servo+'°')+
      row('Netz',d.wconn?('<span class="pill ok">'+d.wip+'</span>'):'<span class="pill off">AP only</span>');
  }).catch(function(){});
}

function updD(){
  fetch('/debug').then(function(r){return r.json();}).then(function(d){
    // Tracking state
    var trk=d.tracking;
    var ts=document.getElementById('dTrackState');
    var tb=document.getElementById('dTrackBtn');
    ts.textContent=trk?'▶ TRACKING AN':'■ TRACKING AUS';
    ts.className='track-state '+(trk?'on':'off');
    tb.textContent=trk?'Tracking stoppen':'Tracking starten';
    tb.className=trk?'btrack-off':'btrack-on';
    // Azimut limits
    sv('dAzPos',d.steps+' Steps ('+fmt(d.mpos,1)+'°)');
    sv('dAzMin',d.azmin+' Steps ('+fmt(d.azmin/(200*16*2/360),1)+'°)');
    sv('dAzMax',d.azmax+' Steps ('+fmt(d.azmax/(200*16*2/360),1)+'°)');
    // Servo
    sv('dSrvAng',d.servo+'°');sv('dSrvMin',d.smin+'°');sv('dSrvMax',d.smax+'°');
    cur=d.servo;
    // IMU
    sv('dAx',fmt(d.ax,3));sv('dAy',fmt(d.ay,3));sv('dAz',fmt(d.az,3));
    sv('dGx',fmt(d.gx,2));sv('dGy',fmt(d.gy,2));sv('dGz',fmt(d.gzn,2));
    sc('dGz',Math.abs(d.gzn)>5?'hi':Math.abs(d.gzn)>1?'warn':'');
    sv('dMx',Math.round(d.mx));sv('dMy',Math.round(d.my));sv('dMz',Math.round(d.mz));
    var magCls=(d.qmc||d.hmc||d.ak)?'ok':'off';
    sc('dMx',magCls);sc('dMy',magCls);sc('dMz',magCls);
    // Lage
    sv('dPitch',fmt(d.pitch,1)+'°');sv('dRoll',fmt(d.roll,1)+'°');
    sv('dHdg',fmt(d.hdg,1)+'°');sv('dGzOff',fmt(d.gzoff,3)+' °/s');
    var gza=Math.abs(d.gzn);
    var gzc=gza<1?'':gza<10?'ok':gza<50?'warn':'hi';
    var gzl=gza<1?'○ ruhig':gza<10?'● aktiv':gza<50?'● schnell':'⚠ hoch';
    document.getElementById('dGzSt').innerHTML='<span class="dv '+gzc+'">'+fmt(d.gzn,2)+'°/s &nbsp;'+gzl+'</span>';
    // Motor
    sv('dSteps',d.steps);sv('dMpos',fmt(d.mpos,1)+'°');
    var err=d.err;sv('dErr',fmt(err,1)+'°');sc('dErr',Math.abs(err)>10?'hi':Math.abs(err)>2?'warn':'ok');
    document.getElementById('dDir').innerHTML=d.dir?'&#x2192; (CW)':'&#x2190; (CCW)';
    var ma=d.mact;sv('dMact',ma?'ja':'nein');sc('dMact',ma?'ok':'');
    // Kompass-Status
    var compLabel=d.compFrz?(d.motorRun?'⏸ Motor l\xe4uft':'⏸ Vibration'):(d.compStable?'✓ aktiv (stabil)':'○ aktiv');
    document.getElementById('dCompSt').innerHTML='<span class="dv '+(d.compFrz?'warn':'ok')+'">'+compLabel+'</span>';
    sv('dCompHdg',fmt(d.compHdg,1)+'\xb0');
    // System
    sv('dUp',fmtUp(d.uptime));
    sv('dHz',fmt(d.hz,0)+' Hz');sc('dHz',d.hz<50?'hi':d.hz<80?'warn':'ok');
    var ms=d.qmc?'QMC5883L':d.hmc?'HMC5883L':d.ak?'AK8963':'keiner';
    sv('dMag',ms);sc('dMag',(d.qmc||d.hmc||d.ak)?'ok':'off');
  }).catch(function(){});
}

function southCal(){fetch('/southcal').then(function(r){return r.text();}).then(function(t){document.getElementById('dSouthCalMsg').textContent=t;});}
function toggleTracking(){
  fetch('/debug').then(function(r){return r.json();}).then(function(d){
    var ep=d.tracking?'/trackingoff':'/trackingon';
    var btn=document.getElementById('dTrackBtn');
    if(!d.tracking){
      document.getElementById('dTrackMsg').textContent='Kalibriere Gyro \xe2\x80\x93 bitte stillhalten (2s)...';
      btn.disabled=true;btn.textContent='Kalibriere...';
    }
    fetch(ep).then(function(r){return r.text();}).then(function(t){
      btn.disabled=false;
      document.getElementById('dTrackMsg').textContent=t;updD();
    });
  });
}

function azStep(deg){
  fetch('/motorstep?delta='+deg).then(function(r){return r.text();}).then(function(t){
    document.getElementById('dAzMsg').textContent=t;
    updD();
  });
}
function setAzLim(w){
  fetch('/setaz'+w).then(function(r){return r.text();}).then(function(t){
    document.getElementById('dAzMsg').textContent=t;updD();
  });
}
function resetHeading(){
  fetch('/resetheading').then(function(r){return r.text();}).then(function(t){
    document.getElementById('dAzMsg').textContent=t;updD();
  });
}

function mv(d){cur=Math.max(servoMin_js,Math.min(servoMax_js,cur+d));fetch('/servo?angle='+cur).then(function(){
  if(activeTab==='d')updD();else upd();
});}
function lim(w){
  var msgId=activeTab==='d'?'dElMsg':'eMsg';
  fetch('/set'+w).then(function(r){return r.text();}).then(function(t){
    document.getElementById(msgId).textContent=t;
    if(activeTab==='d')updD();else upd();
  });
}
function saveElev(){
  var msgId=activeTab==='d'?'dElMsg':'eMsg';
  fetch('/saveelev').then(function(r){return r.text();}).then(function(t){document.getElementById(msgId).textContent=t;});
}
function calibrateElev(){fetch('/elevcal').then(function(r){return r.text();}).then(function(t){document.getElementById('eCalMsg').textContent=t;});}
function srvEnable(){fetch('/servoenable').then(function(r){return r.text();}).then(function(t){document.getElementById('eSrvEnMsg').textContent=t;});}
function setAxis(m){fetch('/elevaxis?m='+m).then(function(r){return r.text();}).then(function(t){document.getElementById('eAxisMsg').textContent=t;upd();});}
function compassReset(){fetch('/compassreset').then(function(r){return r.text();}).then(function(t){document.getElementById('eCompMsg').textContent=t;});}
function loadCfg(){fetch('/getcfg').then(function(r){return r.json();}).then(function(d){document.getElementById('gLat').value=d.lat;document.getElementById('gLon').value=d.lon;document.getElementById('gSat').value=d.sat;document.getElementById('gOff').value=d.off;});}
function saveCfg(){var q='lat='+document.getElementById('gLat').value+'&lon='+document.getElementById('gLon').value+'&sat='+document.getElementById('gSat').value+'&off='+document.getElementById('gOff').value;fetch('/savecfg?'+q).then(function(r){return r.text();}).then(function(t){document.getElementById('gMsg').textContent=t;upd();});}
function resetSteps(){fetch('/resetsteps').then(function(r){return r.text();}).then(function(t){document.getElementById('mMsg').textContent=t;upd();});}
function updW(){fetch('/wlanstatus').then(function(r){return r.json();}).then(function(d){var pill=document.getElementById('wNetPill');if(d.connected){pill.textContent=d.ssid;pill.className='pill ok';document.getElementById('wIpRow').style.display='';document.getElementById('wSigRow').style.display='';document.getElementById('wIp').innerHTML='<a href="http://autosat.local" style="color:#a0c4ff">autosat.local</a> &nbsp;'+d.ip;document.getElementById('wSig').textContent=d.rssi+' dBm';document.getElementById('wSSID').value=d.ssid;}else{pill.textContent='getrennt';pill.className='pill off';document.getElementById('wIpRow').style.display='none';document.getElementById('wSigRow').style.display='none';if(d.ssid)document.getElementById('wSSID').value=d.ssid;}}).catch(function(){});}
function wlanSave(){var ssid=document.getElementById('wSSID').value,pass=document.getElementById('wPass').value;document.getElementById('wMsg').textContent='Verbinde...';fetch('/wlansave?ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)).then(function(r){return r.text();}).then(function(t){document.getElementById('wMsg').textContent=t;updW();});}
function wlanDisc(){fetch('/wlandisc').then(function(r){return r.text();}).then(function(t){document.getElementById('wMsg').textContent=t;updW();});}
function togglePw(){var f=document.getElementById('wPass');f.type=f.type==='password'?'text':'password';}
// Status-Timer läuft nicht wenn Debug-Tab aktiv (dort läuft updD)
var updTimer=null;
function startUpdTimer(){if(!updTimer)updTimer=setInterval(upd,2000);}
function stopUpdTimer(){if(updTimer){clearInterval(updTimer);updTimer=null;}}
// Tab-Switch erweitern um upd-Timer zu pausieren
var origTab=tab;tab=function(id,btn){origTab(id,btn);if(id==='d')stopUpdTimer();else startUpdTimer();};
upd();startUpdTimer();
</script>
</body>
</html>
)rawliteral";

// ─── Captive Portal ────────────────────────────────────────────────────────
void handleRedirect(){server.sendHeader("Location","http://192.168.4.1/",true);server.send(302,"text/plain","");}
void handleRoot(){server.send_P(200,"text/html",PORTAL_HTML);}

// ─── Status ───────────────────────────────────────────────────────────────
void handleStatus(){
  float dp=heading_yaw+(currentSteps/(200.0f*16*2.0f/360.0f));
  bool wc=(WiFi.status()==WL_CONNECTED);
  String j="{\"hdg\":"+String(heading_yaw,2)+",\"taz\":"+String(targetAzimuth,2)+",\"tel\":"+String(targetElevation,2)+",\"pitch\":"+String(pitch,2)+",\"steps\":"+String(currentSteps)+",\"mpos\":"+String(dp,2)+",\"servo\":"+String(servoManualAngle)+",\"smin\":"+String(servoMin)+",\"smax\":"+String(servoMax)+",\"elvaxis\":"+String(elevAxisMode)+",\"lat\":"+String(gps_lat,4)+",\"lon\":"+String(gps_lon,4)+",\"sat\":"+String(sat_lon,1)+",\"off\":"+String(az_offset,1)+",\"tracking\":"+String(trackingEnabled?"true":"false")+",\"wconn\":"+String(wc?"true":"false")+",\"wip\":\""+(wc?WiFi.localIP().toString():String(""))+"\"}"  ;
  server.send(200,"application/json",j);
}

// ─── Debug ────────────────────────────────────────────────────────────────
void handleDebug(){
  float dp=heading_yaw+(float)currentSteps/STEPS_PER_DEGREE+azMountOffset;
  while(dp>=360.0f)dp-=360.0f; while(dp<0.0f)dp+=360.0f;
  float err=targetAzimuth-dp;
  while(err>180.0f)err-=360.0f;while(err<-180.0f)err+=360.0f;
  bool ma=(millis()-lastMotorTime<200);
  String j="{\"ax\":"+String(dbg_ax,4)+",\"ay\":"+String(dbg_ay,4)+",\"az\":"+String(dbg_az,4)+",\"gx\":"+String(dbg_gx,3)+",\"gy\":"+String(dbg_gy,3)+",\"gz\":"+String(dbg_gz,3)+",\"gzn\":"+String(dbg_gz_net,3)+",\"gzoff\":"+String(gz_offset,3)+",\"mx\":"+String(dbg_mx,1)+",\"my\":"+String(dbg_my,1)+",\"mz\":"+String(dbg_mz,1)+",\"pitch\":"+String(pitch,2)+",\"roll\":"+String(roll,2)+",\"hdg\":"+String(heading_yaw,2)+",\"steps\":"+String(currentSteps)+",\"mpos\":"+String(dp,2)+",\"err\":"+String(err,2)+",\"dir\":"+String(dbg_dir?1:0)+",\"mact\":"+String(ma?"true":"false")+",\"servo\":"+String(servoManualAngle)+",\"smin\":"+String(servoMin)+",\"smax\":"+String(servoMax)+",\"manual\":"+String(manualControl?"true":"false")+",\"azmin\":"+String(azStepMin)+",\"azmax\":"+String(azStepMax)+",\"tracking\":"+String(trackingEnabled?"true":"false")+",\"uptime\":"+String(millis())+",\"hz\":"+String(loopHz,1)+",\"qmc\":"+String(qmc_ok?"true":"false")+",\"hmc\":"+String(hmc_ok?"true":"false")+",\"ak\":"+String(ak8963_ok?"true":"false")+",\"motorRun\":"+String(motorRunning?"true":"false")+",\"vib\":"+String(vibrationDetected?"true":"false")+",\"compFrz\":"+String(compassFrozen?"true":"false")+",\"compHdg\":"+String(compassHeading,1)+",\"compStable\":"+String(compassStable?"true":"false")+"}";
  server.send(200,"application/json",j);
}

// ─── Tracking ─────────────────────────────────────────────────────────────
void handleTrackingOn(){
  calibrateGyroOffset();  // ~2.5s, Plattform stillhalten
  // Noch kein gespeicherter Offset: aktuelle Position als Startpunkt setzen.
  // Kein wildes Laufen beim ersten Start. Echte Kalibrierung per /southcal.
  if (azMountOffset == 0.0f) {
    azMountOffset = targetAzimuth - heading_yaw - (float)currentSteps / STEPS_PER_DEGREE;
  }
  trackingEnabled = true;
  Serial.printf("Tracking start – dish:%.1f  target:%.1f  offset:%.1f\n",
    heading_yaw + (float)currentSteps / STEPS_PER_DEGREE + azMountOffset,
    targetAzimuth, azMountOffset);
  server.send(200,"text/plain","Gyro kalibriert \xe2\x9c\x93  Tracking gestartet");
}
// Süd-Referenz: Plattform physisch nach Süden drehen, dann aufrufen.
// Setzt currentSteps=0, dynamische ±180°-Limits und Montageoffset.
// Einmalig beim ersten Aufbau, danach ist NVM-Zustand persistent.
void handleSouthCal(){
  trackingEnabled = false;  // Tracking stoppen während Kalibrierung
  currentSteps = 0;
  positionDirty = true;     // Steps sofort in NVM schreiben
  azMountOffset = 180.0f - heading_yaw;  // Kompass-Offset: steps=0 → Schüssel zeigt Süden
  azStepMin = -(long)(180.0f * STEPS_PER_DEGREE);
  azStepMax =  (long)(180.0f * STEPS_PER_DEGREE);
  prefs.begin("autosat", false);
  prefs.putFloat("azMntOff", azMountOffset);
  prefs.putLong ("azMin",    azStepMin);
  prefs.putLong ("azMax",    azStepMax);
  prefs.putLong ("steps",    0);
  prefs.end();
  char buf[72];
  snprintf(buf, sizeof(buf),
    "S\xc3\xbcd-Referenz: Offset=%.1f\xc2\xb0  Limits=\xc2\xb1%.0f Steps \xe2\x9c\x93",
    azMountOffset, 180.0f * STEPS_PER_DEGREE);
  Serial.printf("SuedCal: offset=%.1f  hdg=%.1f  limits=+/-%ld\n",
    azMountOffset, heading_yaw, azStepMax);
  server.send(200, "text/plain", buf);
}
void handleTrackingOff(){
  trackingEnabled=false;
  Serial.println("Tracking gestoppt.");
  server.send(200,"text/plain","Tracking gestoppt \xe2\x9c\x93");
}

// ─── GPIO-Diagnose + Motortest ────────────────────────────────────────────
void handleMotorTest(){
  trackingEnabled=false;

  // GPIO4 auf HIGH setzen und zurücklesen
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(100);
  int readHigh = digitalRead(DIR_PIN);

  // 200 Schritte mit DIR=HIGH bei 200µs (über Resonanz)
  for(int i=0;i<200;i++){
    digitalWrite(DIR_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(200);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(200);
  }
  delay(1500);

  // GPIO4 auf LOW setzen und zurücklesen
  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(100);
  int readLow = digitalRead(DIR_PIN);

  // 200 Schritte mit DIR=LOW (zurück)
  for(int i=0;i<200;i++){
    digitalWrite(DIR_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(200);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(200);
  }

  char buf[80];
  snprintf(buf,sizeof(buf),
    "DIR=HIGH gelesen:%d | DIR=LOW gelesen:%d | beide=1 bedeutet Pin haengt HIGH",
    readHigh, readLow);
  server.send(200,"text/plain",buf);
}

// ─── Manueller Motorschritt (stoppt Tracking automatisch) ────────────────
void handleMotorStep(){
  trackingEnabled = false;
  if(!server.hasArg("delta")){
    server.send(400,"text/plain","kein delta"); return;
  }
  String rawDelta = server.arg("delta");
  float deg = rawDelta.toFloat();
  bool dir = deg > 0;
  int steps = constrain((int)(fabsf(deg) * STEPS_PER_DEGREE), 1, 200);

  Serial.printf(">>> motorstep: raw='%s' deg=%.3f dir=%d  shaft=%s\n",
    rawDelta.c_str(), deg, (int)dir, dir ? "0(vor)" : "1(rück)");

  tmcSetDir(dir);  // Richtung per UART (shaft-Bit), nicht über DIR-Pin
  pinMode(STEP_PIN, OUTPUT);

  int ramp = min(steps/3, 10);
  for(int i = 0; i < steps; i++){
    unsigned int d;
    if(i < ramp)
      d = (unsigned int)(STEP_MAN_MAX - (long)(STEP_MAN_MAX-STEP_MAN_MIN)*i/max(ramp,1));
    else if(i >= steps-ramp)
      d = (unsigned int)(STEP_MAN_MIN + (long)(STEP_MAN_MAX-STEP_MAN_MIN)*(i-(steps-ramp))/max(ramp,1));
    else
      d = STEP_MAN_MIN;

    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(d);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(d);
    currentSteps += dir ? 1 : -1;
  }
  lastMotorTime = millis();
  positionDirty = true;
  dbg_dir = dir;

  char buf[64];
  snprintf(buf, sizeof(buf), "%s %d Steps  shaft=%s",
    dir ? "+" : "-", steps, dir ? "0(vor)" : "1(rück)");
  server.send(200, "text/plain", buf);
}

// ─── Azimut Limits ────────────────────────────────────────────────────────
void handleSetAzMin(){
  azStepMin=currentSteps;
  prefs.begin("autosat",false);prefs.putLong("azMin",azStepMin);prefs.end();
  Serial.printf("AZ MIN=%ld\n",azStepMin);
  server.send(200,"text/plain","AZ MIN = "+String(azStepMin)+" Steps");
}
void handleSetAzMax(){
  azStepMax=currentSteps;
  prefs.begin("autosat",false);prefs.putLong("azMax",azStepMax);prefs.end();
  Serial.printf("AZ MAX=%ld\n",azStepMax);
  server.send(200,"text/plain","AZ MAX = "+String(azStepMax)+" Steps");
}
void handleResetHeading(){
  heading_yaw=0.0f;
  Serial.println("Heading auf 0 gesetzt.");
  server.send(200,"text/plain","Heading = 0° \xe2\x9c\x93");
}

// ─── Elevation ────────────────────────────────────────────────────────────
// Servo erst aktivieren wenn User es explizit freigibt – kein automatisches Fahren beim Boot.
void handleServoEnable(){
  if (!servoEnabled) {
    elevationServo.attach(SERVO_PIN);
    int baseAngle = (servoCalAngle > 0) ? servoCalAngle : servoMin;
    // Zielwinkel aus Kalibrierung + aktueller IMU-Lage berechnen (identisch zu updateElevation)
    float corr = 0.0f;
    switch (elevAxisMode) {
      case 1:  corr =  roll;  break;
      case 2:  corr = -pitch; break;
      case 3:  corr = -roll;  break;
      default: corr =  pitch; break;
    }
    int target = constrain(baseAngle + (int)roundf(corr), servoMin, servoMax);
    // Ausgangspunkt = kalibrierter Winkel (letzte bekannte mechanische Position)
    elevationServo.write(baseAngle);
    servoActual      = baseAngle;
    servoTarget      = target;
    servoManualAngle = baseAngle;
    servoEnabled     = true;
    servoInitDone    = false;   // Sanfte Rampe (100ms/°) bis Ziel erreicht
    Serial.printf("Servo aktiv – Basis:%d° Korr:%.1f° Ziel:%d°\n", baseAngle, corr, target);
    if (servoCalAngle > 0)
      server.send(200,"text/plain","Servo aktiv \xe2\x9c\x93 \xe2\x80\x93 f\xc3\xa4hrt auf "+String(target)+"\xc2\xb0 (Kalibrierung + Lage)");
    else
      server.send(200,"text/plain","Servo aktiv \xe2\x9c\x93 \xe2\x80\x93 nicht kalibriert, manuell ausrichten");
  } else {
    server.send(200,"text/plain","Servo bereits aktiv");
  }
}
void handleServo(){
  if(server.hasArg("angle")){
    int a = constrain(server.arg("angle").toInt(), servoMin, servoMax);
    servoMoveTo(a);  // Langsame Rampe, Endstopps erzwungen
    manualControl=true; lastServoCmd=millis();
  }
  server.send(200,"text/plain","OK");
}
void handleSetMin(){servoMin=servoManualAngle;server.send(200,"text/plain","MIN = "+String(servoMin)+"\xc2\xb0");}
void handleSetMax(){servoMax=servoManualAngle;server.send(200,"text/plain","MAX = "+String(servoMax)+"\xc2\xb0");}
void handleSaveElev(){prefs.begin("autosat",false);prefs.putInt("servoMin",servoMin);prefs.putInt("servoMax",servoMax);prefs.end();server.send(200,"text/plain","Gespeichert \xe2\x9c\x93");}
void handleElevAxis(){
  if (!server.hasArg("m")) { server.send(400,"text/plain","kein m"); return; }
  elevAxisMode = constrain(server.arg("m").toInt(), 0, 3);
  prefs.begin("autosat",false); prefs.putInt("elvAxis", elevAxisMode); prefs.end();
  const char* names[] = {"+Pitch","+Roll","-Pitch","-Roll"};
  char buf[32]; snprintf(buf,sizeof(buf),"Achse: %s gespeichert",names[elevAxisMode]);
  Serial.printf("ElevAxis: mode=%d (%s)\n", elevAxisMode, names[elevAxisMode]);
  server.send(200,"text/plain",buf);
}
void handleElevCal(){
  // Speichert aktuellen Servo-Winkel direkt als Kalibrierwinkel.
  // Schüssel vorher manuell auf Satellitensignal ausrichten.
  servoCalAngle = servoManualAngle;
  prefs.begin("autosat",false); prefs.putInt("srvCalAng", servoCalAngle); prefs.end();
  char buf[48];
  snprintf(buf, sizeof(buf), "Elevation: %d\xc2\xb0 gespeichert \xe2\x9c\x93", servoCalAngle);
  Serial.printf("ElevCal: servoCalAngle=%d\n", servoCalAngle);
  server.send(200,"text/plain",buf);
}

// ─── Kompass-Reset (Stufe 2) ──────────────────────────────────────────────
void handleCompassReset() {
  if (motorRunning || vibrationDetected) {
    server.send(200,"text/plain","Abgelehnt \xe2\x9c\x97 \xe2\x80\x93 nicht im Stillstand");
    return;
  }
  heading_yaw = compassHeading;
  server.send(200,"text/plain","Heading auf Kompasswert gesetzt \xe2\x9c\x93");
}

// ─── GPS / Sat ────────────────────────────────────────────────────────────
void handleGetCfg(){server.send(200,"application/json","{\"lat\":"+String(gps_lat,4)+",\"lon\":"+String(gps_lon,4)+",\"sat\":"+String(sat_lon,1)+",\"off\":"+String(az_offset,1)+"}");}
void handleSaveCfg(){if(server.hasArg("lat"))gps_lat=server.arg("lat").toDouble();if(server.hasArg("lon"))gps_lon=server.arg("lon").toDouble();if(server.hasArg("sat"))sat_lon=server.arg("sat").toDouble();if(server.hasArg("off"))az_offset=server.arg("off").toFloat();recalcSat();prefs.begin("autosat",false);prefs.putDouble("gpsLat",gps_lat);prefs.putDouble("gpsLon",gps_lon);prefs.putDouble("satLon",sat_lon);prefs.putFloat("azOffset",az_offset);prefs.end();server.send(200,"text/plain","Gespeichert \xe2\x9c\x93");}

// ─── Motor ────────────────────────────────────────────────────────────────
void handleResetSteps(){currentSteps=0;prefs.begin("autosat",false);prefs.putLong("steps",0);prefs.end();server.send(200,"text/plain","Position = 0 \xe2\x9c\x93");}

// ─── WLAN ─────────────────────────────────────────────────────────────────
void handleWlanStatus(){bool c=(WiFi.status()==WL_CONNECTED);server.send(200,"application/json","{\"connected\":"+String(c?"true":"false")+",\"ssid\":\""+String(wlan_ssid)+"\",\"ip\":\""+(c?WiFi.localIP().toString():String(""))+"\",\"rssi\":"+String(c?WiFi.RSSI():0)+"}");}
void handleWlanSave(){if(server.hasArg("ssid")){server.arg("ssid").toCharArray(wlan_ssid,sizeof(wlan_ssid));server.arg("pass").toCharArray(wlan_pass,sizeof(wlan_pass));prefs.begin("autosat",false);prefs.putString("wifiSSID",wlan_ssid);prefs.putString("wifiPass",wlan_pass);prefs.end();WiFi.begin(wlan_ssid,wlan_pass);int t=0;while(WiFi.status()!=WL_CONNECTED&&t<20){delay(500);t++;}if(WiFi.status()==WL_CONNECTED){MDNS.begin("autosat");MDNS.addService("http","tcp",80);server.send(200,"text/plain","Verbunden \xe2\x9c\x93  "+WiFi.localIP().toString()+"  \xe2\x86\x92 http://autosat.local");}else{server.send(200,"text/plain","Verbindung fehlgeschlagen \xe2\x9c\x97");}}else{server.send(400,"text/plain","Fehlende Parameter");}}
void handleWlanDisc(){WiFi.disconnect();memset(wlan_ssid,0,sizeof(wlan_ssid));memset(wlan_pass,0,sizeof(wlan_pass));prefs.begin("autosat",false);prefs.remove("wifiSSID");prefs.remove("wifiPass");prefs.end();MDNS.end();server.send(200,"text/plain","Getrennt \xe2\x9c\x93");}

// ─── I2C Scan ────────────────────────────────────────────────────────────
void handleI2CScan(){
  String out = "I2C Scan:\n";
  int found = 0;
  for(byte addr=1;addr<127;addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission()==0){
      char buf[48];
      const char* lbl = "";
      if(addr==0x68) lbl=" (MPU AD0=LOW)";
      else if(addr==0x69) lbl=" (MPU AD0=HIGH)";
      else if(addr==0x0C) lbl=" (AK8963)";
      else if(addr==0x0D) lbl=" (QMC5883L)";
      else if(addr==0x1E) lbl=" (HMC5883L)";
      snprintf(buf,sizeof(buf),"  0x%02X%s\n",addr,lbl);
      out+=buf; found++;
    }
  }
  if(found==0) out+="  Keine Geraete gefunden!\n";
  server.send(200,"text/plain",out);
}

// ─── STA Verbindung ───────────────────────────────────────────────────────
void connectWiFiSTA(){if(strlen(wlan_ssid)==0)return;WiFi.begin(wlan_ssid,wlan_pass);Serial.printf("WLAN: verbinde mit %s",wlan_ssid);int t=0;while(WiFi.status()!=WL_CONNECTED&&t<20){delay(500);Serial.print(".");t++;}if(WiFi.status()==WL_CONNECTED){Serial.printf("\nWLAN: %s  IP: %s\n",wlan_ssid,WiFi.localIP().toString().c_str());MDNS.begin("autosat");MDNS.addService("http","tcp",80);Serial.println("mDNS: http://autosat.local");}else{Serial.println("\nWLAN: fehlgeschlagen.");}}

// ─── Initialisierung ──────────────────────────────────────────────────────
void setupWebPortal(const char* apSSID,const char* apPass){
  WiFi.mode(WIFI_AP_STA);WiFi.softAP(apSSID,apPass);
  Serial.printf("AP: %s  IP: %s\n",apSSID,WiFi.softAPIP().toString().c_str());
  dnsServer.start(53,"*",WiFi.softAPIP());
  server.on("/",handleRoot);
  server.on("/status",handleStatus);
  server.on("/debug",handleDebug);
  server.on("/trackingon",handleTrackingOn);
  server.on("/trackingoff",handleTrackingOff);
  server.on("/southcal",handleSouthCal);
  server.on("/motortest",handleMotorTest);
  server.on("/motorstep",handleMotorStep);
  server.on("/setazmin",handleSetAzMin);
  server.on("/setazmax",handleSetAzMax);
  server.on("/resetheading",handleResetHeading);
  server.on("/servoenable",handleServoEnable);
  server.on("/servo",handleServo);
  server.on("/setmin",handleSetMin);
  server.on("/setmax",handleSetMax);
  server.on("/saveelev",handleSaveElev);
  server.on("/elevcal",handleElevCal);
  server.on("/elevaxis",handleElevAxis);
  server.on("/compassreset",handleCompassReset);
  server.on("/getcfg",handleGetCfg);
  server.on("/savecfg",handleSaveCfg);
  server.on("/resetsteps",handleResetSteps);
  server.on("/i2cscan",handleI2CScan);
  server.on("/setdir",[](){
    bool h = server.hasArg("v") && server.arg("v")=="1";
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, h ? HIGH : LOW);
    server.send(200,"text/plain",
      h ? "DIR=HIGH gesetzt – miss jetzt GPIO4: erwartet ~3.3V"
        : "DIR=LOW gesetzt – miss jetzt GPIO4: erwartet ~0V");
  });
  server.on("/wlanstatus",handleWlanStatus);
  server.on("/wlansave",handleWlanSave);
  server.on("/wlandisc",handleWlanDisc);
  server.on("/generate_204",handleRoot);
  server.on("/hotspot-detect.html",handleRoot);
  server.on("/connecttest.txt",handleRoot);
  server.on("/redirect",handleRedirect);
  server.onNotFound(handleRedirect);
  server.begin();
}

void processWebPortal(){dnsServer.processNextRequest();server.handleClient();}
