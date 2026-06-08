// ==UserScript==
// @name         GeoFS -> Pico Cockpit Link
// @namespace    http://tampermonkey.net/
// @version      2.0
// @description  Send GeoFS telemetry to Raspberry Pi Pico over Web Serial
// @match        https://www.geo-fs.com/geofs.php*
// @match        https://*.geo-fs.com/geofs.php*
// @match        https://geofs.com/*
// @match        https://www.geofs.com/*
// @match        https://*.geofs.com/*
// @grant        none
// @run-at       document-idle
// ==/UserScript==

(function () {
    'use strict';

    let btn = null;

    let serialPort = null;
    let writer = null;
    let connected = false;

    function uiMount() {
        if (btn) return;
        const mount = document.body || document.documentElement;
        if (!mount) return;

        btn = document.createElement('button');
        btn.textContent = 'Connect Pico';
        btn.style.cssText = [
            'position:fixed',
            'top:8px',
            'right:8px',
            'z-index:2147483647',
            'padding:7px 10px',
            'background:#1976d2',
            'color:#fff',
            'border:none',
            'border-radius:6px',
            'font-size:12px',
            'cursor:pointer'
        ].join(';');

        mount.appendChild(btn);

        btn.addEventListener('click', async () => {
            if (!connected) {
                await connect();
            } else {
                await disconnect();
            }
        });
    }

    function syncButtonVisibility() {
        if (!btn) return;
        // Button is always visible: shows "Connect Pico" or "Disconnect Pico"
        btn.style.display = '';
    }

    function getNumber(obj, keys, fallback) {
        for (const key of keys) {
            if (obj && obj[key] !== undefined && obj[key] !== null) {
                const n = Number(obj[key]);
                if (Number.isFinite(n)) return n;
            }
        }
        return fallback;
    }

    function getFirstDefined(obj, keys, fallback) {
        for (const key of keys) {
            if (obj && obj[key] !== undefined && obj[key] !== null) return obj[key];
        }
        return fallback;
    }

    function getArrayNumber(arr, index, fallback) {
        if (Array.isArray(arr) && arr.length > index) {
            const n = Number(arr[index]);
            if (Number.isFinite(n)) return n;
        }
        return fallback;
    }

    function normalizeHeading(value) {
        const n = Number(value);
        if (!Number.isFinite(n)) return 0;
        return ((n % 360) + 360) % 360;
    }

    function clamp01(value) {
        if (!Number.isFinite(value)) return 0;
        if (value < 0) return 0;
        if (value > 1) return 1;
        return value;
    }

    function hasUsefulTelemetrySource(source) {
        if (!source) return false;
        return [
            'pitch', 'rawPitch', 'roll', 'invRoll', 'aroll',
            'heading360', 'heading', 'navHDG',
            'altitude', 'QNHFeet', 'altThousands',
            'kias', 'APIAS', 'airspeed',
            'verticalSpeed', 'VS', 'APVS',
            'mach', 'aoa', 'loadFactor', 'gLoad', 'g',
            'thrust', 'rpm'
        ].some((key) => Number.isFinite(Number(source[key])));
    }

    function buildTelemetryLine() {
        // ── Confirmed object paths from geofs.aircraft.Aircraft console dump ──
        // geofs.aircraft.instance           → the live Aircraft object
        // aircraft.htr                      → [heading_deg, pitch_deg, roll_deg]  (DEGREES, not radians)
        // aircraft.animationValue            → live animation dict (altitude ft, kias, verticalSpeed, etc.)
        // aircraft.angleOfAttackDeg         → AoA in degrees
        // aircraft.groundSpeed              → m/s  (NOT knots — use animationValue.kias for IAS)
        // aircraft.totalThrust              → total thrust (N)
        // aircraft.engine                   → {rpm, rpmLeft, rpmRight, on, …}
        // aircraft.engines[0/1]             → per-engine objects
        // aircraft.stalling / .overspeed    → booleans

        const aircraft = globalThis.geofs?.aircraft?.instance || null;
        const av       = aircraft?.animationValue || {};   // primary data source
        const htr      = aircraft?.htr;                   // [hdg_deg, pitch_deg, roll_deg]
        const engine   = aircraft?.engine  || {};
        const engines  = aircraft?.engines || [];

        // No aircraft loaded yet — send heartbeat so Pico knows we're alive
        if (!aircraft || !Array.isArray(htr) || htr.length < 3) {
            return [0,0,0,0,0,0,0,0,1.0,1013.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0].join(',') + '\n';
        }

        // ── Attitude ─────────────────────────────────────────────────────────
        // htr is [heading, pitch, roll] all in DEGREES.
        // GeoFS pitch convention: nose-up = positive in htr[1] but we want
        // positive pitch = nose-up on the Pico display → keep sign as-is.
        const heading = normalizeHeading(htr[0]);
        const pitch   = Number(htr[1]) || 0;   // degrees, positive = nose up
        const roll    = Number(htr[2]) || 0;   // degrees, positive = right bank

        // ── Flight data (from animationValue) ────────────────────────────────
        // altitude: in feet directly
        const altitude = Number(av.altitude)       || 0;
        // kias: knots indicated airspeed
        const ias      = Number(av.kias)           || 0;
        // verticalSpeed: ft/min in animationValue
        const vs       = Number(av.verticalSpeed)  || 0;
        const mach     = Number(av.mach)           || 0;

        // ── AoA: aircraft.angleOfAttackDeg is the confirmed field ────────────
        const aoa      = Number(aircraft.angleOfAttackDeg) || getNumber(av, ['aoa','angleOfAttack'], 0);

        // ── G-load ────────────────────────────────────────────────────────────
        const gLoad    = getNumber(av, ['gForce', 'gLoad', 'loadFactor', 'g'], 1.0);

        // ── QNH / other ──────────────────────────────────────────────────────
        const qnh      = getNumber(av, ['QNH', 'altimeterSetting', 'qnh'], 1013.25);
        const turnRate = getNumber(av, ['turnRate', 'turnrate'], 0);

        // groundSpeed: aircraft.groundSpeed is m/s; convert to knots (×1.94384)
        const groundSpeedKt = (Number(aircraft.groundSpeed) || 0) * 1.94384;

        // ── Engine ─────────────────────────────────────────────────────────
        // aircraft.totalThrust confirmed field
        const totalThrust  = Number(aircraft.totalThrust)  || 0;
        // engine.rpmLeft / rpmRight confirmed from dump: engine:{rpm:1000,rpmLeft:1000,rpmRight:1000}
        const engineRpmLeft  = Number(engine.rpmLeft)  || Number(engine.rpm) || 0;
        const engineRpmRight = Number(engine.rpmRight) || Number(engine.rpm) || 0;

        // Per-engine thrust fallbacks
        const leftThrust  = engines[0] ? (Number(engines[0].thrust) || 0) : 0;
        const rightThrust = engines[1] ? (Number(engines[1].thrust) || 0) : 0;
        const engineRpm   = Number(engine.rpm) || 0;

        // ── Warnings ─────────────────────────────────────────────────────────
        // Real GeoFS boolean flags only — no client-side calculation
        const isStalling  = av.stalling     ? 1 : 0;
        const isOverspeed = av.overspeed    ? 1 : 0;
        const enginesOn   = av.enginesOn    ? 1 : 0;
        const parkingBrake = (av.brakes >= 1) ? 1 : 0;  // brakes=1 = parking brake set

        // AP status — read directly from geofs.autopilot
        const ap = globalThis.geofs && globalThis.geofs.autopilot;
        const apOn = ap ? (ap.on === true) : false;

        // GPWS: read channel 1 directly — exact alarm ID string from GeoFS
        // Possible values: sinkrate, whoopwhooppullup, pullup, terrainterrain,
        // terrainterrainwhoopwhooppullup, terrainterrainpullup, dontsink,
        // toolowgear, toolowterrain, toolowflaps, glideslope, glideslopeloud,
        // bankAngle, stall, overspeed, apdisconnect — or null
        const gpwsAlarm = (geofs.alarms && geofs.alarms.channels[1].toPlay) || '';

        const gearPos = Number(av.gearPosition) || 0;

        // Flaps: send raw step and max steps as integers so Pico shows exact step.
        // flapsSteps = max step index (e.g. 2 → steps 0/1/2 = 3 positions).
        // flapsTarget is commanded step; flapsValue is fallback.
        const flapsSteps  = Math.max(1, Number(av.flapsSteps) || 1);
        const flapsTarget = Number(av.flapsTarget) || Number(av.flapsValue) || 0;
        // Still send as 0-1 for wire compatibility, but encode precisely:
        // flapsPos encodes BOTH step and max: step + (max/100) e.g. step=1,max=2 → 1.02
        // Pico decodes: step = floor(val), max = round((val-floor(val))*100)
        const flapsPos = flapsTarget + (flapsSteps / 100.0);

        // Spoilers: 0=off, 0.5=armed, 1=deployed (any non-zero position)
        const airbrakesArmed    = Number(av.airbrakesArmed)    || 0;
        const airbrakesPosition = Number(av.airbrakesPosition) || 0;
        let spoilersPos = 0;
        if      (airbrakesPosition > 0) spoilersPos = 1;
        else if (airbrakesArmed)        spoilersPos = 0.5;

        // Wind: av.windSpeed is directly in knots (confirmed from geofs.animation.values dump).
        // av.relativeWind is degrees relative to aircraft nose; convert to absolute bearing.
        const windSpeedKt = Number(av.windSpeed) || 0;
        const relWind     = Number(av.relativeWind) || 0;
        const windDirDeg  = ((heading + relWind + 180) % 360 + 360) % 360;

        // Ground data
        const groundContact = (av.groundContact && av.groundContact > 0) ? 1 : 0;
        const haglFt        = Number(av.haglFeet) || 0;  // height above ground level

        // Field index map (0-based):
        // 0-9:   pitch,roll,hdg,alt,ias,vs,mach,aoa,gLoad,qnh
        // 10-18: turnRate,gsKt,isStall,isOver,engOn,prkBrk,0,0,0
        // 19:    gpwsAlarm(str)
        // 20-25: thrust,lThrust,rThrust,rpm,rpmL,rpmR
        // 26-28: gear,flaps,spoilers
        // 29-32: windSpd,windDir,haglFt,groundContact
        // 33:    apOn
        return [
            pitch, roll, heading, altitude, ias, vs, mach, aoa, gLoad, qnh,
            turnRate, groundSpeedKt, isStalling, isOverspeed, enginesOn, parkingBrake,
            0, 0, 0,
            gpwsAlarm,
            totalThrust, leftThrust, rightThrust, engineRpm, engineRpmLeft, engineRpmRight,
            gearPos.toFixed(2), flapsPos.toFixed(2), spoilersPos.toFixed(2),
            Math.round(windSpeedKt), Math.round(windDirDeg), Math.round(haglFt), groundContact, apOn ? 1 : 0
        ].join(',') + '\n';
    }

    async function connect() {
        try {
            if (!('serial' in navigator)) {
                return;
            }

            serialPort = await navigator.serial.requestPort();
            await serialPort.open({ baudRate: 115200 });
            writer = serialPort.writable.getWriter();
            // start reader to accept incoming SET commands from Pico
            try {
                if (serialPort.readable) {
                    const decoder = new TextDecoder();
                    let rxBuf = '';
                    const reader = serialPort.readable.getReader();
                    (async function readLoop() {
                        try {
                            while (true) {
                                const { value, done } = await reader.read();
                                if (done) break;
                                if (!value) continue;
                                rxBuf += decoder.decode(value, { stream: true });
                                let nl;
                                while ((nl = rxBuf.indexOf('\n')) >= 0) {
                                    const line = rxBuf.slice(0, nl).trim();
                                    rxBuf = rxBuf.slice(nl + 1);
                                    if (!line) continue;
                                    // Expect lines like: SET,HDG,123  or  SET,AP,TOGGLE
                                    if (line.startsWith('SET,')) {
                                        const parts = line.split(',');
                                        if (parts.length >= 3) {
                                            const what = parts[1].toUpperCase();
                                            const val = Number(parts[2]);
                                            try {
                                                const ap = globalThis.geofs && globalThis.geofs.autopilot;
                                                let applied = false;
                                                let ackPayload = null;
                                                if (ap) {
                                                    if (what === 'HDG' && Number.isFinite(val)) {
                                                        if (typeof ap.setCourse === 'function') { ap.setCourse(val); applied = true; }
                                                    } else if (what === 'ALT' && Number.isFinite(val)) {
                                                        if (typeof ap.setAltitude === 'function') { ap.setAltitude(val); applied = true; }
                                                    } else if (what === 'IAS' && Number.isFinite(val)) {
                                                        if (typeof ap.setSpeed === 'function') { ap.setSpeed(val); applied = true; }
                                                    } else if (what === 'AP' && parts[2].toUpperCase() === 'TOGGLE') {
                                                        // Toggle autopilot: engage if off, disengage if on
                                                        // ap.on is confirmed boolean, ap.toggle() confirmed function
                                                        if (ap.on) {
                                                            ap.turnOff();
                                                            ackPayload = 'ACK,AP,OFF\n';
                                                        } else {
                                                            ap.turnOn();
                                                            ackPayload = 'ACK,AP,ON\n';
                                                        }
                                                        applied = true;
                                                    }
                                                }

                                                // Send ACK back to Pico
                                                if (applied && writer) {
                                                    try {
                                                        const msg = ackPayload || `ACK,${what},${val}\n`;
                                                        await writer.write(new TextEncoder().encode(msg));
                                                    } catch (e) { console.warn('Failed to send ACK', e); }
                                                }
                                            } catch (err) {
                                                console.error('Error applying SET', err);
                                            }
                                        }
                                    }
                                }
                            }
                        } catch (err) {
                            console.error('ReadLoop error', err);
                        } finally {
                            try { await reader.releaseLock(); } catch (_) { }
                        }
                    })();
                }
            } catch (e) {
                console.warn('Reader not started', e);
            }
            connected = true;
            btn.textContent = 'Disconnect Pico';
            btn.style.background = '#2e7d32';
            syncButtonVisibility();
        } catch (err) {
            console.error('Connect error:', err);
            await disconnect();
        }
    }

    async function disconnect() {
        connected = false;
        try {
            if (writer) {
                try { await writer.close(); } catch (_) { }
                try { writer.releaseLock(); } catch (_) { }
            }
            // reader will be released when port closes
        } finally {
            writer = null;
        }
        try {
            if (serialPort) {
                await serialPort.close();
            }
        } catch (_) {
        } finally {
            serialPort = null;
        }

        if (btn) {
            btn.textContent = 'Connect';
            btn.style.background = '#1976d2';
        }
        syncButtonVisibility();
    }

    async function transmitLoop() {
        if (!connected || !writer) return;
        try {
            const line = buildTelemetryLine();
            await writer.write(new TextEncoder().encode(line));
        } catch (err) {
            console.error('Send error:', err);
            await disconnect();
        }
    }

    function start() {
        uiMount();
        setInterval(() => {
            void transmitLoop();
        }, 50);
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', start, { once: true });
    } else {
        start();
    }
})();