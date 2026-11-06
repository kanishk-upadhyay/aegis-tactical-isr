/**
 * AEGIS-TACTICAL-ISR: C2 Tactical Dashboard Client Application
 * ============================================================
 * WebSocket live map track rendering, Leaflet.js MIL-STD-2525 symbology,
 * dynamic sensor ground swath polygons, and simulated sensor PiP HUD.
 */

// Global State & CARTO API Configuration
const CARTO_API_KEY = "cb1_53kv_1_538e2a794fe27d3ea2627d69";

let map = null;
let ws = null;
let audioEnabled = false;
let audioCtx = null;

// Map & Basemap Layers
let currentBasemap = 'dark';
let basemapLayers = {};
let uavMarker = null;
let swathPolygon = null;
let tracksGroup = null;
let trailsGroup = null;
let drCirclesGroup = null;
let adAssetGroup = null;
let wezRingsGroup = null;
let vectorArrowsGroup = null;

// Clocks & UI
function updateClocks() {
  const now = new Date();
  const utcString = now.toISOString().substring(11, 19) + 'Z';
  const istString = now.toLocaleTimeString('en-IN', { timeZone: 'Asia/Kolkata', hour12: false });
  
  const elUtc = document.getElementById('clock-utc');
  const elIst = document.getElementById('clock-ist');
  if (elUtc) elUtc.textContent = utcString;
  if (elIst) elIst.textContent = istString + ' IST';
}
setInterval(updateClocks, 1000);
updateClocks();

// Web Audio API Tactical Radar Ping
function playRadarPing(frequency = 880, duration = 0.12) {
  if (!audioEnabled) return;
  try {
    if (!audioCtx) {
      audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    }
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(frequency, audioCtx.currentTime);
    osc.frequency.exponentialRampToValueAtTime(frequency * 1.5, audioCtx.currentTime + duration);

    gain.gain.setValueAtTime(0.15, audioCtx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + duration);

    osc.connect(gain);
    gain.connect(audioCtx.destination);
    osc.start();
    osc.stop(audioCtx.currentTime + duration);
  } catch (e) {
    console.warn("Audio ping error:", e);
  }
}

// Audio Toggle Button
const btnAudio = document.getElementById('btn-audio-toggle');
if (btnAudio) {
  btnAudio.addEventListener('click', () => {
    audioEnabled = !audioEnabled;
    const icon = document.getElementById('audio-icon');
    if (audioEnabled) {
      icon.textContent = '🔊';
      btnAudio.style.borderColor = 'var(--color-cyan)';
      playRadarPing(1000, 0.1);
      logEvent('Tactical audio alerts enabled.');
    } else {
      icon.textContent = '🔇';
      btnAudio.style.borderColor = 'var(--border-tactical)';
      logEvent('Tactical audio alerts muted.');
    }
  });
}

// Log Event to Bottom Footer
function logEvent(text) {
  const stream = document.getElementById('event-log-stream');
  if (!stream) return;
  const now = new Date();
  const timeStr = now.toTimeString().substring(0, 8);
  const entry = document.createElement('div');
  entry.className = 'log-entry';
  entry.textContent = `[${timeStr}] ${text}`;
  stream.prepend(entry);
  while (stream.children.length > 5) {
    stream.removeChild(stream.lastChild);
  }
}

// Initialize Leaflet Tactical Map with Authenticated CARTO Basemaps
function initMap() {
  // Center on Northern Tactical Theatre (Ladakh sector)
  map = L.map('map', {
    center: [34.1526, 77.5771],
    zoom: 14,
    zoomControl: false,
    attributionControl: false,
  });

  // Authenticated CARTO & Satellite Basemap Layers
  basemapLayers = {
    dark: L.tileLayer(`https://{s}.basemaps.cartocdn.com/rastertiles/dark_all/{z}/{x}/{y}{r}.png?key=${CARTO_API_KEY}`, {
      maxZoom: 19,
      subdomains: 'abcd',
      attribution: '&copy; CARTO &copy; OpenStreetMap',
    }),
    voyager: L.tileLayer(`https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png?key=${CARTO_API_KEY}`, {
      maxZoom: 19,
      subdomains: 'abcd',
      attribution: '&copy; CARTO &copy; OpenStreetMap',
    }),
    satellite: L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 19,
      attribution: '&copy; Esri &copy; Maxar, Earthstar Geographics',
    }),
  };

  // Add default tactical dark basemap
  basemapLayers['dark'].addTo(map);

  // Tactical Layer Groups
  tracksGroup = L.layerGroup().addTo(map);
  trailsGroup = L.layerGroup().addTo(map);
  drCirclesGroup = L.layerGroup().addTo(map);
  adAssetGroup = L.layerGroup().addTo(map);
  wezRingsGroup = L.layerGroup().addTo(map);
  vectorArrowsGroup = L.layerGroup().addTo(map);

  // Zoom control top right
  L.control.zoom({ position: 'topright' }).addTo(map);
}

// Switch Basemap Layer (CARTO Dark, CARTO Voyager, Satellite)
function switchBasemap(mode) {
  if (!map || !basemapLayers[mode]) return;

  if (basemapLayers[currentBasemap]) {
    map.removeLayer(basemapLayers[currentBasemap]);
  }
  basemapLayers[mode].addTo(map);
  basemapLayers[mode].bringToBack();
  currentBasemap = mode;

  document.querySelectorAll('.btn-bm').forEach(b => b.classList.remove('active'));
  const activeBtn = document.getElementById(`btn-bm-${mode === 'satellite' ? 'sat' : mode}`);
  if (activeBtn) activeBtn.classList.add('active');

  logEvent(`Basemap switched to: ${mode.toUpperCase()} (CARTO Key Authenticated)`);
}
window.switchBasemap = switchBasemap;

// Custom Marker Icons
function createUavIcon(headingDeg) {
  return L.divIcon({
    className: 'uav-custom-marker',
    html: `<div style="transform: rotate(${headingDeg}deg); width:28px; height:28px; display:flex; align-items:center; justify-content:center; color:#00f3ff; font-size:24px; filter:drop-shadow(0 0 8px #00f3ff);">▲</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  });
}

function createTargetIcon(className, iffStatus, isOccluded) {
  let color = '#ff2a2a'; // Default Red
  if (iffStatus === 'FRIENDLY') color = '#00ff66';
  else if (iffStatus === 'SUSPECT') color = '#ffb700';
  else if (isOccluded) color = '#ff55aa';

  const shapeClass = className.includes('DRONE') ? 'rotate(45deg)' : 'rotate(0deg)';
  const borderStyle = isOccluded ? 'dashed' : 'solid';

  return L.divIcon({
    className: 'target-custom-marker',
    html: `
      <div style="position:relative; width:22px; height:22px; display:flex; align-items:center; justify-content:center;">
        <div style="width:14px; height:14px; background:${color}; transform:${shapeClass}; border:1px ${borderStyle} #ffffff; box-shadow:0 0 10px ${color};"></div>
        <div style="position:absolute; top:-16px; font-size:9px; font-weight:bold; color:#fff; background:rgba(0,0,0,0.85); padding:1px 3px; border-radius:2px; border:1px solid ${color}; white-space:nowrap;">
          ${className.substring(0, 5)}
        </div>
      </div>
    `,
    iconSize: [22, 22],
    iconAnchor: [11, 11],
  });
}

function createAdAssetIcon(name) {
  return L.divIcon({
    className: 'ad-asset-custom-marker',
    html: `
      <div style="position:relative; display:flex; flex-direction:column; align-items:center;">
        <div style="width:26px; height:26px; background:rgba(0, 243, 255, 0.25); border:2px solid #00f3ff; border-radius:4px; display:flex; align-items:center; justify-content:center; box-shadow:0 0 12px #00f3ff;">
          <span style="color:#00f3ff; font-size:14px; font-weight:bold;">▲</span>
        </div>
        <div style="margin-top:2px; font-size:9px; font-weight:bold; color:#00f3ff; background:rgba(0,10,20,0.92); padding:1px 4px; border:1px solid #00f3ff; border-radius:2px; white-space:nowrap; letter-spacing:0.5px;">
          ${name}
        </div>
      </div>
    `,
    iconSize: [30, 44],
    iconAnchor: [15, 22],
  });
}

// WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

  logEvent(`Connecting C2 Telemetry Stream to ${wsUrl}...`);
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    logEvent('C2 WebSocket connection established. Receiving STANAG 4609 GeoJSON feed.');
  };

  ws.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      renderTacticalPicture(payload);
    } catch (e) {
      console.error('Failed to parse telemetry packet:', e);
    }
  };

  ws.onclose = () => {
    logEvent('C2 Link lost. Reconnecting in 3s...');
    setTimeout(connectWebSocket, 3000);
  };

  ws.onerror = (err) => {
    console.error('WebSocket Error:', err);
  };
}

// Slew Gimbal Command over WebSocket
function slewGimbal(pitch, yaw) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      action: 'SLEW_GIMBAL',
      pitch: pitch,
      yaw: yaw,
    }));
    logEvent(`Gimbal slew command sent: Pitch=${pitch}°, Yaw=${yaw}°`);
  }
}

// Switch Sensor Mode
function switchSensorMode(mode) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      action: 'SWITCH_SENSOR_MODE',
      mode: mode,
    }));
    logEvent(`Payload sensor switched to: ${mode}`);
  }
}

// Wire Mode Buttons
document.getElementById('btn-sensor-rgb')?.addEventListener('click', () => {
  document.getElementById('btn-sensor-rgb').classList.add('active');
  document.getElementById('btn-sensor-flir').classList.remove('active');
  document.getElementById('text-spectral-band').textContent = 'Visible High-Res EO (0.4-0.7 µm)';
  document.getElementById('pip-mode-badge').textContent = 'RGB 1080p';
  document.getElementById('pip-mode-badge').className = 'mode-badge rgb';
  switchSensorMode('RGB');
});

document.getElementById('btn-sensor-flir')?.addEventListener('click', () => {
  document.getElementById('btn-sensor-flir').classList.add('active');
  document.getElementById('btn-sensor-rgb').classList.remove('active');
  document.getElementById('text-spectral-band').textContent = 'LWIR Uncooled Microbolometer (8-14 µm)';
  document.getElementById('pip-mode-badge').textContent = 'FLIR LWIR';
  document.getElementById('pip-mode-badge').className = 'mode-badge flir';
  switchSensorMode('FLIR');
});

// Transmit to Army Air Defence Battery Handover
document.getElementById('btn-air-defence-handover')?.addEventListener('click', () => {
  playRadarPing(1200, 0.25);
  logEvent('▲ TACTICAL HANDOVER: Track packet transmitted to Akash / L-70 Air Defence battery.');
  alert('AIR DEFENCE ENGAGEMENT HANDOVER INITIATED:\nTarget telemetry coordinates locked & streamed to DG AAD Battery Command.');
});

// Render Tactical Picture
let previousTrackCount = 0;

function renderTacticalPicture(payload) {
  const uav = payload.uav_telemetry;
  const tracks = payload.active_tracks;
  const footprint = payload.camera_footprint_geojson;
  const threatSummary = payload.threat_summary;

  // 1. Update Telemetry Panel Numbers
  document.getElementById('val-alt-msl').textContent = `${Math.round(uav.alt_msl_m).toLocaleString()} m`;
  document.getElementById('val-alt-agl').textContent = `${Math.round(uav.alt_agl_m)} m`;
  document.getElementById('val-speed').textContent = `${Math.round(uav.groundspeed_kmh)} km/h`;
  document.getElementById('val-heading').textContent = `${String(Math.round(uav.heading_deg)).padStart(3, '0')}°`;

  document.getElementById('val-uav-lat').textContent = `${uav.lat.toFixed(6)}° N`;
  document.getElementById('val-uav-lon').textContent = `${uav.lon.toFixed(6)}° E`;

  // Bars
  document.getElementById('val-roll').textContent = `${uav.roll_deg > 0 ? '+' : ''}${uav.roll_deg.toFixed(1)}°`;
  document.getElementById('val-pitch').textContent = `${uav.pitch_deg > 0 ? '+' : ''}${uav.pitch_deg.toFixed(1)}°`;
  document.getElementById('val-gimbal-tilt').textContent = `${uav.gimbal_pitch_deg.toFixed(1)}°`;
  document.getElementById('val-gimbal-pan').textContent = `${uav.gimbal_yaw_deg.toFixed(1)}°`;

  const rollPct = Math.min(100, Math.max(0, 50 + (uav.roll_deg / 45.0) * 50));
  const pitchPct = Math.min(100, Math.max(0, 50 + (uav.pitch_deg / 45.0) * 50));
  const tiltPct = Math.min(100, Math.max(0, 100 - (Math.abs(uav.gimbal_pitch_deg) / 90.0) * 100));
  const panPct = Math.min(100, Math.max(0, 50 + (uav.gimbal_yaw_deg / 180.0) * 50));

  document.getElementById('bar-roll').style.width = `${rollPct}%`;
  document.getElementById('bar-pitch').style.width = `${pitchPct}%`;
  document.getElementById('bar-gimbal-tilt').style.width = `${tiltPct}%`;
  document.getElementById('bar-gimbal-pan').style.width = `${panPct}%`;

  // Air Defence Asset & Threat Summary
  const adAsset = payload.air_defence_asset;
  const warningsActive = payload.intercept_warnings_active || 0;

  // Threat Barometer & WEZ Alert Pills
  document.getElementById('count-critical').textContent = threatSummary.CRITICAL || 0;
  document.getElementById('count-high').textContent = threatSummary.HIGH || 0;
  document.getElementById('count-medium').textContent = threatSummary.MEDIUM || 0;
  const drCount = tracks.filter(t => t.status === 'DEAD_RECKONING').length;
  document.getElementById('count-dr').textContent = drCount;

  const alertPill = document.getElementById('hud-intercept-alert');
  const alertCount = document.getElementById('intercept-alert-count');
  if (alertPill && alertCount) {
    if (warningsActive > 0) {
      alertPill.style.display = 'flex';
      alertCount.textContent = `${warningsActive} BREACH${warningsActive > 1 ? 'ES' : ''}`;
    } else {
      alertPill.style.display = 'none';
    }
  }

  // 2. Render UAV Marker
  const uavLatLng = [uav.lat, uav.lon];
  if (!uavMarker) {
    uavMarker = L.marker(uavLatLng, { icon: createUavIcon(uav.heading_deg) }).addTo(map);
  } else {
    uavMarker.setLatLng(uavLatLng);
    uavMarker.setIcon(createUavIcon(uav.heading_deg));
  }

  // 3. Render Camera FOV Swath
  if (footprint && footprint.geometry && footprint.geometry.coordinates) {
    // GeoJSON is [lon, lat], Leaflet is [lat, lon]
    const polygonLatLngs = footprint.geometry.coordinates[0].map(pt => [pt[1], pt[0]]);
    if (!swathPolygon) {
      swathPolygon = L.polygon(polygonLatLngs, {
        color: '#00f3ff',
        weight: 1.5,
        fillColor: '#00f3ff',
        fillOpacity: 0.12,
        dashArray: '4, 4',
      }).addTo(map);
    } else {
      swathPolygon.setLatLngs(polygonLatLngs);
    }
  }

  // 4. Render Air Defence Battery Asset & Weapon Engagement Zones (WEZ)
  adAssetGroup.clearLayers();
  wezRingsGroup.clearLayers();

  if (adAsset && adAsset.lat && adAsset.lon) {
    const adLatLng = [adAsset.lat, adAsset.lon];
    const assetCallsign = adAsset.callsign || adAsset.name || 'AKASH-BATTERY-01';
    const mezRadius = adAsset.mez_radius_m || adAsset.engagement_radius_m || 2500;
    const ewzRadius = adAsset.ewz_radius_m || adAsset.warning_radius_m || 5000;

    // Air Defence Battery Marker
    const adMarker = L.marker(adLatLng, {
      icon: createAdAssetIcon(assetCallsign),
    }).addTo(adAssetGroup);

    adMarker.bindPopup(`
      <div style="font-family:'Share Tech Mono',monospace; font-size:11px; color:#e1edf7; background:#0a1018; padding:8px; border:1px solid #00f3ff; min-width:260px;">
        <div style="font-weight:bold; color:#00f3ff; margin-bottom:4px;">AIR DEFENCE BATTERY: ${assetCallsign}</div>
        <div>SECTOR: NORTHERN / LADAKH THEATRE</div>
        <div>POSITION: ${adAsset.lat.toFixed(6)}°N, ${adAsset.lon.toFixed(6)}°E</div>
        <div>ALTITUDE: ${adAsset.alt_m ? Math.round(adAsset.alt_m) : 3950} m MSL</div>
        <div style="margin-top:6px; padding-top:6px; border-top:1px dashed #1e3347;">
          <div>MISSILE ENGAGEMENT ZONE (MEZ): <span style="color:#ff2a2a; font-weight:bold;">${mezRadius} m</span></div>
          <div>EARLY WARNING PERIMETER: <span style="color:#00f3ff;">${ewzRadius} m</span></div>
          <div>FIRE CONTROL STATUS: <span style="color:#00ff66; font-weight:bold;">ARMED & INTERFACE READY</span></div>
        </div>
      </div>
    `);

    // Outer Early Warning Zone (5,000m)
    L.circle(adLatLng, {
      radius: ewzRadius,
      color: '#00e5ff',
      weight: 1.5,
      dashArray: '6, 6',
      fillColor: '#00e5ff',
      fillOpacity: 0.04,
    }).addTo(wezRingsGroup);

    // Inner Weapon Engagement Zone / MEZ (2,500m)
    const isPerimeterBreached = warningsActive > 0;
    L.circle(adLatLng, {
      radius: mezRadius,
      color: isPerimeterBreached ? '#ff2a2a' : '#ffb700',
      weight: isPerimeterBreached ? 2.5 : 2.0,
      dashArray: '8, 4',
      fillColor: isPerimeterBreached ? '#ff2a2a' : '#ffb700',
      fillOpacity: isPerimeterBreached ? 0.15 : 0.06,
      className: isPerimeterBreached ? 'wez-pulsing-svg' : '',
    }).addTo(wezRingsGroup);
  }

  // 5. Render Active Tracks, Vector Arrows & Trajectory Trails
  tracksGroup.clearLayers();
  trailsGroup.clearLayers();
  drCirclesGroup.clearLayers();
  vectorArrowsGroup.clearLayers();

  const targetQueue = document.getElementById('target-queue');
  if (targetQueue) targetQueue.innerHTML = '';

  tracks.forEach((trk) => {
    const isOccluded = trk.status === 'DEAD_RECKONING';
    const trkLatLng = [trk.lat, trk.lon];
    const isInterceptWarning = trk.intercept_warning || trk.warning_level === 'ENGAGEMENT_CRITICAL';

    // Marker
    const marker = L.marker(trkLatLng, {
      icon: createTargetIcon(trk.class_name, trk.iff_status, isOccluded),
    }).addTo(tracksGroup);

    // 30-Second Kinematic Vector Arrow Projection
    if (trk.projected_30s_pos && trk.projected_30s_pos.length >= 2) {
      const projLatLng = [trk.projected_30s_pos[0], trk.projected_30s_pos[1]];
      const vecColor = isInterceptWarning ? '#ff2a2a' : (trk.iff_status === 'HOSTILE' ? '#ff9900' : '#00f3ff');

      // Projected flight path line
      L.polyline([trkLatLng, projLatLng], {
        color: vecColor,
        weight: isInterceptWarning ? 2.5 : 1.8,
        dashArray: isInterceptWarning ? '6, 3' : '4, 4',
        opacity: 0.9,
      }).addTo(vectorArrowsGroup);

      // Terminal 30s projection point marker
      L.circleMarker(projLatLng, {
        radius: isInterceptWarning ? 4 : 3,
        color: vecColor,
        fillColor: vecColor,
        fillOpacity: 0.85,
        weight: 1,
      }).addTo(vectorArrowsGroup).bindTooltip(`+30s Proj: ${trk.speed_kmh} km/h`, {
        permanent: false,
        direction: 'top',
        className: 'tactical-tooltip',
      });
    }

    // Terrain & CPA Formatting
    const elevText = trk.terrain_elevation_msl !== null && trk.terrain_elevation_msl !== undefined
      ? `${Math.round(trk.terrain_elevation_msl)} m MSL (Slope: ${trk.terrain_slope_deg ? trk.terrain_slope_deg.toFixed(1) : '0.0'}°)`
      : 'N/A';

    const cpaText = trk.cpa_distance_m !== null && trk.cpa_distance_m !== undefined
      ? `${Math.round(trk.cpa_distance_m)} m ${trk.time_to_cpa_sec !== null && trk.time_to_cpa_sec >= 0 ? '(in ' + Math.round(trk.time_to_cpa_sec) + 's)' : '(DIVERGING)'}`
      : 'N/A';

    const distToAdText = trk.distance_to_asset_m !== null && trk.distance_to_asset_m !== undefined
      ? `${Math.round(trk.distance_to_asset_m)} m (BRG: ${trk.bearing_to_asset_deg !== null ? Math.round(trk.bearing_to_asset_deg) : '0'}°)`
      : 'N/A';

    let alertBadgeHtml = '';
    if (isInterceptWarning) {
      alertBadgeHtml = `
        <div style="margin-top:6px; padding:4px 6px; background:rgba(255, 42, 42, 0.25); border:1px solid #ff2a2a; border-radius:2px; color:#ff2a2a; font-weight:bold; text-align:center;">
          ⚠ DEFENSIVE PERIMETER BREACH (${trk.warning_level})
        </div>
      `;
    }

    // Popup
    marker.bindPopup(`
      <div style="font-family:'Share Tech Mono',monospace; font-size:11px; color:#e1edf7; background:#0a1018; padding:8px; border:1px solid ${isInterceptWarning ? '#ff2a2a' : '#1e3347'}; min-width:250px;">
        <div style="font-weight:bold; color:#00f3ff; margin-bottom:4px;">TARGET TRACK #${trk.track_id} // ${trk.class_name}</div>
        <div>IFF STATUS: <span style="color:${trk.iff_status === 'HOSTILE' ? '#ff2a2a' : '#00ff66'}">${trk.iff_status}</span></div>
        <div>THREAT PRIORITY: <span style="color:#ffb700">${trk.threat_priority} (${trk.threat_score}/100)</span></div>
        <div>GROUNDSPEED: ${trk.speed_kmh} km/h | HEADING: ${trk.heading_deg}°</div>
        <div>COORDINATES: ${trk.lat.toFixed(5)}°N, ${trk.lon.toFixed(5)}°E</div>
        <div style="margin-top:4px; padding-top:4px; border-top:1px dashed #1e3347;">
          <div>TERRAIN ELEVATION: <span style="color:#00f3ff">${elevText}</span></div>
          <div>CPA TO AD BATTERY: <span style="color:${isInterceptWarning ? '#ff2a2a' : '#ffb700'}; font-weight:bold;">${cpaText}</span></div>
          <div>BATTERY RANGE: ${distToAdText}</div>
          <div>WARNING LEVEL: <span style="color:${trk.warning_level === 'ENGAGEMENT_CRITICAL' ? '#ff2a2a' : (trk.warning_level === 'ADVISORY' ? '#ffb700' : '#00ff66')}">${trk.warning_level || 'NOMINAL'}</span></div>
          <div>TRACK STATUS: <span style="color:#00f3ff">${trk.status}</span></div>
        </div>
        ${alertBadgeHtml}
      </div>
    `);

    // Dead Reckoning expanding uncertainty circle
    if (isOccluded || trk.uncertainty_m > 8.0) {
      L.circle(trkLatLng, {
        radius: trk.uncertainty_m,
        color: '#ff55aa',
        weight: 1,
        dashArray: '3, 3',
        fillColor: '#ff55aa',
        fillOpacity: 0.15,
      }).addTo(drCirclesGroup);
    }

    // Trajectory trails
    if (trk.trajectory && trk.trajectory.length > 1) {
      L.polyline(trk.trajectory, {
        color: trk.iff_status === 'HOSTILE' ? '#ff2a2a' : '#ffb700',
        weight: 1.5,
        opacity: 0.6,
      }).addTo(trailsGroup);
    }

    // Target Queue Card in Sidebar
    if (targetQueue) {
      const elevCardStr = trk.terrain_elevation_msl ? `${Math.round(trk.terrain_elevation_msl)}m` : '--';
      const slopeCardStr = trk.terrain_slope_deg ? `${Math.round(trk.terrain_slope_deg)}°` : '0°';
      const cpaCardStr = trk.cpa_distance_m !== null && trk.cpa_distance_m !== undefined
        ? `${Math.round(trk.cpa_distance_m)}m`
        : 'DIV';
      const timeCpaCardStr = trk.time_to_cpa_sec !== null && trk.time_to_cpa_sec >= 0
        ? `(${Math.round(trk.time_to_cpa_sec)}s)`
        : '';

      const card = document.createElement('div');
      const warningClass = isInterceptWarning ? 'intercept-breach' : '';
      card.className = `target-card ${trk.threat_priority.toLowerCase()} ${isOccluded ? 'dead-reckon' : ''} ${warningClass}`;
      card.innerHTML = `
        <div class="target-card-top">
          <span class="target-id-tag">#${trk.track_id} ${trk.class_name}</span>
          <span class="target-iff-badge ${isOccluded ? 'dr' : trk.iff_status.toLowerCase()}">${isOccluded ? 'DEAD RECKON' : trk.iff_status}</span>
        </div>
        <div class="target-card-mid">
          <span>SPD: ${trk.speed_kmh} km/h</span>
          <span>HDG: ${trk.heading_deg}°</span>
          <span>CPA: ${cpaCardStr} ${timeCpaCardStr}</span>
        </div>
        <div class="target-card-mid terrain-info">
          <span>ELEV: ${elevCardStr} MSL</span>
          <span>SLOPE: ${slopeCardStr}</span>
          <span>THREAT: ${trk.threat_score}</span>
        </div>
        <div class="target-card-bot">
          <span>${trk.lat.toFixed(4)}°N, ${trk.lon.toFixed(4)}°E</span>
          <span>UNCERT: ±${trk.uncertainty_m}m</span>
        </div>
        ${isInterceptWarning ? '<div class="card-intercept-banner">⚠ WEZ INTERCEPT ALERT</div>' : ''}
      `;
      card.addEventListener('click', () => {
        map.panTo(trkLatLng);
        marker.openPopup();
      });
      targetQueue.appendChild(card);
    }
  });

  // Sound ping on new high/critical threat or intercept warning
  if (warningsActive > 0) {
    playRadarPing(1400, 0.2);
  } else if (tracks.length > previousTrackCount && tracks.some(t => t.threat_priority === 'CRITICAL')) {
    playRadarPing(920, 0.15);
  }
  previousTrackCount = tracks.length;

  // 5. Draw Simulated Sensor Canvas PiP
  renderPipCanvas(tracks, uav.sensor_mode);
}

// Render Sensor Canvas
function renderPipCanvas(tracks, mode) {
  const canvas = document.getElementById('pip-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  // Clear background
  if (mode === 'FLIR') {
    // Thermal gradient
    ctx.fillStyle = '#06131d';
    ctx.fillRect(0, 0, w, h);
    // Add thermal scan lines
    ctx.strokeStyle = 'rgba(0, 200, 255, 0.05)';
    for (let y = 0; y < h; y += 4) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
  } else {
    // Daylight dark aerial terrain simulation
    ctx.fillStyle = '#121e14';
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
    for (let x = 0; x < w; x += 15) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
  }

  // Draw simulated targets on PiP
  tracks.forEach((trk, idx) => {
    // Synthetic normalized placement on canvas
    const x = ((trk.lon * 1000) % 200) + 60;
    const y = ((trk.lat * 1000) % 100) + 40;

    if (mode === 'FLIR') {
      // Hot spot bloom
      const grad = ctx.createRadialGradient(x, y, 2, x, y, 14);
      grad.addColorStop(0, '#ffffff');
      grad.addColorStop(0.4, '#ffdd44');
      grad.addColorStop(1, 'rgba(255, 100, 0, 0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(x, y, 14, 0, Math.PI * 2);
      ctx.fill();
    }

    // Bounding Box
    ctx.strokeStyle = trk.status === 'DEAD_RECKONING' ? '#ff55aa' : (trk.iff_status === 'HOSTILE' ? '#ff2a2a' : '#00ff66');
    ctx.lineWidth = 1.5;
    ctx.strokeRect(x - 12, y - 10, 24, 20);

    // Label
    ctx.fillStyle = '#ffffff';
    ctx.font = '8px monospace';
    ctx.fillText(`ID:${trk.track_id} ${trk.class_name.substring(0, 4)}`, x - 12, y - 12);
  });
}

// Window Onload
window.addEventListener('DOMContentLoaded', () => {
  initMap();
  connectWebSocket();
});
