// Pharma Rover Dashboard Controller

let isPTTActive = false;
let lastLogCount = 0;
let pttRecorder = null;
let pttStream = null;
let pttChunks = [];
let fetchRequestPending = false;
let lastBrowserSpeechText = '';
let startPoseInitialized = false;

// Tab switcher
function switchTab(tabId) {
  document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
  document.querySelectorAll('.tab-pane').forEach(pane => pane.classList.remove('active'));

  event.currentTarget.classList.add('active');
  const target = document.getElementById(tabId);
  if (target) {
    target.classList.add('active');
  }
}

// Global Emergency Stop
function triggerEmergencyStop() {
  fetch('/api/car/stop', { method: 'POST' })
    .then(r => r.json())
    .then(data => {
      alert("EMERGENCY STOP SENT: Car halted, arm aborted, grab deauthorised.");
      updateStatus();
    })
    .catch(err => console.error(err));
}

// Fetch Medicine Mission
function fetchMedicine(medId) {
  if (fetchRequestPending) return;
  fetchRequestPending = true;
  const buttons = document.querySelectorAll('.fetch-medicine-btn');
  buttons.forEach(button => { button.disabled = true; });
  const status = document.getElementById('fetch-status');
  status.textContent = 'Starting medicine mission...';

  fetch('/api/mission/fetch', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ medicine_id: medId })
  })
  .then(response => response.json().then(data => ({ response, data })))
  .then(data => {
    if (!data.response.ok || !data.data.ok) {
      throw new Error(data.data.error || data.data.message || 'Mission was refused.');
    }
    status.textContent = `Mission started: ${data.data.medicine}.`;
    updateStatus();
  })
  .catch(error => { status.textContent = `Could not start mission: ${error.message}`; })
  .finally(() => {
    fetchRequestPending = false;
    buttons.forEach(button => { button.disabled = false; });
  });
}

// Authorise Grab Gate Override
function authoriseGrab(medId) {
  fetch('/api/arm/authorise', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mission_id: "MANUAL_OPERATOR", medicine_id: medId })
  })
  .then(r => r.json())
  .then(data => {
    alert(data.message || "Authorised");
    updateStatus();
  });
}

// Manual Car Step
function carStep(direction) {
  const dist = parseFloat(document.getElementById('step-dist').value) || 200;
  const deg = parseFloat(document.getElementById('step-deg').value) || 45;
  const status = document.getElementById('drive-status');
  status.textContent = 'Sending movement command...';

  fetch('/api/car/step', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ dir: direction, dist_mm: dist, deg: deg })
  })
  .then(response => response.json().then(data => ({ response, data })))
  .then(data => {
    if (!data.response.ok || !data.data.ok) {
      throw new Error(data.data.error || 'Move was refused.');
    }
    status.textContent = data.data.message || 'Movement command accepted.';
    updateStatus();
  })
  .catch(error => { status.textContent = `Move failed: ${error.message}`; });
}

// Jog Arm Joint
function jogJoint(jointName, val) {
  fetch('/api/arm/jog', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ axis: jointName, val: parseFloat(val) })
  })
  .then(r => r.json())
  .then(data => {
    if (!data.ok) {
      alert("Arm move refused: " + data.error);
    }
  });
}

// Arm Presets
function armPreset(name) {
  fetch('/api/arm/preset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ preset: name })
  })
  .then(response => response.json())
  .then(data => {
    const status = document.getElementById('arm-action-status');
    status.textContent = data.ok ? `${name.toUpperCase()} preset applied.` : `Arm move refused: ${data.error}`;
    updateStatus();
  })
  .catch(error => { document.getElementById('arm-action-status').textContent = `Arm command failed: ${error.message}`; });
}

// Toggle Gripper
function toggleGripper() {
  fetch('/api/arm/gripper', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: "TOGGLE" })
  })
  .then(r => r.json())
  .then(data => {
    if (!data.ok) {
      alert("Gripper action refused: " + (data.error || "Requires grab authorisation gate"));
    }
  });
}

// Display Controls
function setDisplayState(state) {
  fetch('/api/display/state', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: state })
  });
}

function showDisplayCard(medId) {
  fetch('/api/display/card', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ medicine_id: medId })
  });
}

// Typed Command
function sendCommandText(text) {
  return fetch('/api/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ text: text })
  })
  .then(response => {
    if (!response.ok) throw new Error(`Server returned ${response.status}`);
    return response.json();
  })
  .then(data => {
    if (!data.ok) {
      const reason = data.result?.error || data.result?.message || data.error || 'Command was rejected.';
      throw new Error(`${data.intent || 'Command'}: ${reason}`);
    }
    updateStatus();
    return data;
  });
}

function sendTypedCommand() {
  const input = document.getElementById('input-command-text');
  const text = input.value.trim();
  if (!text) return;

  sendCommandText(text)
    .then(() => { input.value = ''; })
    .catch(error => alert(`Command could not be sent: ${error.message}`));
}

function sendQuickCommand(text) {
  sendCommandText(text).catch(error => alert(`Command failed: ${error.message}`));
}

function setStartPose() {
  const pose = {
    x_mm: Number(document.getElementById('start-x').value),
    y_mm: Number(document.getElementById('start-y').value),
    heading_deg: Number(document.getElementById('start-heading').value)
  };

  fetch('/api/start_pose', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(pose)
  })
  .then(response => response.json().then(data => ({ response, data })))
  .then(({ response, data }) => {
    if (!response.ok || !data.ok) throw new Error(data.error || 'Start location was rejected.');
    document.getElementById('start-status').textContent = `Start set to (${data.start_pose[0]}, ${data.start_pose[1]}) mm.`;
    reloadStream('overhead-stream');
  })
  .catch(error => { document.getElementById('start-status').textContent = `Could not set start: ${error.message}`; });
}

function speakBrowserText(text, id) {
  const key = (id !== undefined && id !== null) ? 'id' + id : text;   // same sentence twice still speaks
  if (!text || key === lastBrowserSpeechText) return;
  lastBrowserSpeechText = key;
  if (!window.speechSynthesis || !window.SpeechSynthesisUtterance) return;
  window.speechSynthesis.cancel();
  window.speechSynthesis.speak(new SpeechSynthesisUtterance(text));
}

function fillCommand(text) {
  const input = document.getElementById('input-command-text');
  input.value = text;
  input.focus();
}

// Push to talk
function setPTTStatus(message) {
  document.getElementById('ptt-status').textContent = message;
}

function updatePTTButton(active, label) {
  const button = document.getElementById('btn-ptt');
  button.style.background = active ? '#f85149' : '#2f81f7';
  button.innerText = label || (active ? 'LISTENING... RELEASE TO SEND' : '🎤 HOLD TO SPEAK (PTT)');
}

async function startPTT(event) {
  if (isPTTActive || pttRecorder) return;
  if (!navigator.mediaDevices?.getUserMedia || !window.MediaRecorder) {
    setPTTStatus('Audio recording is unavailable. Open this page on localhost or over HTTPS, or type the command.');
    return;
  }

  isPTTActive = true;
  updatePTTButton(true);
  setPTTStatus('Requesting microphone access...');

  try {
    if (event && event.pointerId !== undefined) {
      event.currentTarget.setPointerCapture(event.pointerId);
    }
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: { echoCancellation: true, noiseSuppression: true }
    });
    if (!isPTTActive) {
      stream.getTracks().forEach(track => track.stop());
      return;
    }

    pttStream = stream;
    pttChunks = [];
    const mimeType = ['audio/webm;codecs=opus', 'audio/mp4', 'audio/webm']
      .find(type => MediaRecorder.isTypeSupported(type));
    pttRecorder = mimeType ? new MediaRecorder(stream, { mimeType }) : new MediaRecorder(stream);

    pttRecorder.ondataavailable = recorderEvent => {
      if (recorderEvent.data.size > 0) pttChunks.push(recorderEvent.data);
    };
    pttRecorder.onerror = () => {
      setPTTStatus('Microphone recording failed. Check the browser microphone permission and try again.');
      stopPTT();
    };
    pttRecorder.onstop = async () => {
      const recording = new Blob(pttChunks, { type: pttRecorder.mimeType || 'audio/webm' });
      pttChunks = [];
      pttStream?.getTracks().forEach(track => track.stop());
      pttStream = null;

      if (!recording.size) {
        pttRecorder = null;
        updatePTTButton(false);
        setPTTStatus('No audio was recorded. Hold the button while speaking, then release it.');
        return;
      }

      setPTTStatus('Sending audio to the laptop for local transcription...');
      try {
        const response = await fetch('/api/audio/transcribe', {
          method: 'POST',
          headers: { 'Content-Type': recording.type || 'audio/webm' },
          body: recording
        });
        const result = await response.json();
        if (!response.ok || !result.ok) throw new Error(result.error || 'No speech was recognized.');

        const transcript = result.transcript.trim();
        document.getElementById('input-command-text').value = transcript;
        setPTTStatus(`Heard: ${transcript}. Sending command...`);
        const command = await sendCommandText(transcript);
        setPTTStatus(`Executed ${command.intent}: ${command.result.message || transcript}`);
      } catch (error) {
        setPTTStatus(`Voice command failed: ${error.message}`);
      } finally {
        pttRecorder = null;
        updatePTTButton(false);
      }
    };

    pttRecorder.start(250);
    updatePTTButton(true, 'LISTENING... RELEASE TO SEND');
    setPTTStatus('Listening. Speak clearly, then release the button. Audio is transcribed on this laptop.');
  } catch (error) {
    isPTTActive = false;
    pttStream?.getTracks().forEach(track => track.stop());
    pttStream = null;
    pttRecorder = null;
    updatePTTButton(false);
    const message = error.name === 'NotAllowedError'
      ? 'Microphone access was denied. Allow access for this site and try again.'
      : `Could not start microphone recording: ${error.message}`;
    setPTTStatus(message);
  }
}

function stopPTT() {
  if (!isPTTActive) return;
  isPTTActive = false;
  updatePTTButton(false, 'PROCESSING TRANSCRIPT...');
  if (pttRecorder && pttRecorder.state === 'recording') {
    pttRecorder.stop();
  } else {
    setPTTStatus('Stopping microphone capture...');
  }
}

// Refresh Stream Helper
function reloadStream(id) {
  // Camera views refresh by themselves (one picture at a time) - just grab a fresh one now.
  const el = document.getElementById(id);
  if (el && el.dataset.snap) el.src = el.dataset.snap + '?once=1&t=' + Date.now();
}

// Doctor Diagnostics
function loadDoctorDiag() {
  fetch('/api/doctor')
    .then(r => r.json())
    .then(data => {
      const tbody = document.getElementById('diag-body');
      tbody.innerHTML = '';
      data.checks.forEach(c => {
        const tr = document.createElement('tr');
        const badgeClass = c.status === 'PASS' ? 'blue' : (c.status === 'WARN' ? 'orange' : 'red');
        tr.innerHTML = `
          <td><span class="badge ${badgeClass}">${c.status}</span></td>
          <td><strong>${c.subsystem}</strong></td>
          <td>${c.name}</td>
          <td>${c.details}</td>
        `;
        tbody.appendChild(tr);
      });
    });
}

// Status Poller
function updateStatus() {
  fetch('/api/status')
    .then(r => r.json())
    .then(s => {
      try { updateLivePanel(s); } catch (e) { console.warn('live panel', e); }
      try { updateAiPanel(s); } catch (e) { console.warn('ai panel', e); }
      // 1. Header & Global State
      const stateBadge = document.getElementById('global-state-badge');
      const stateStr = s.mission.state || "IDLE";
      stateBadge.innerText = stateStr;
      stateBadge.className = 'pill-value ' + (stateStr === 'IDLE' ? 'state-idle' : (stateStr === 'ERROR' || stateStr === 'STOPPED' ? 'state-error' : 'state-active'));

      document.getElementById('battery-voltage').innerText = (s.car.battery_v || 7.4).toFixed(1) + 'V';

      // 2. Metrics Ribbon
      if (s.car.pose) {
        document.getElementById('car-pose-val').innerText = `X: ${Math.round(s.car.pose.x)} | Y: ${Math.round(s.car.pose.y)}`;
        document.getElementById('car-heading-val').innerText = `Heading: ${s.car.pose.heading.toFixed(1)}° • ${Math.round(s.car.v)} mm/s`;
      }

      if (s.start_pose && !startPoseInitialized) {
        document.getElementById('start-x').value = s.start_pose.x;
        document.getElementById('start-y').value = s.start_pose.y;
        document.getElementById('start-heading').value = s.start_pose.heading;
        startPoseInitialized = true;
      }

      if (s.vision) {
        document.getElementById('phone-camera-status').textContent = `${s.vision.overhead_status} • ${s.vision.overhead_fps} FPS`;
        document.getElementById('laptop-camera-status').textContent = s.vision.laptop_enabled
          ? `${s.vision.laptop_status} • ${s.vision.laptop_fps} FPS`
          : 'Disabled in camera configuration';
      }

      if (s.arm.tip_xyz_mm) {
        document.getElementById('arm-tip-val').innerText = `Tip Z: ${s.arm.tip_xyz_mm[2]}mm (${s.arm.gripper})`;
      }

      const torque = s.arm.torque || {};
      const shUtil = torque.shoulder_util_pct || 0;
      const wrUtil = torque.wrist_pitch_util_pct || 0;
      document.getElementById('arm-torque-val').innerText = `Shoulder: ${shUtil}% • Wrist: ${wrUtil}%`;

      // Hand Verdict
      const vBadge = document.getElementById('hand-verdict-badge');
      const auth = s.interlocks.grab_authorized;
      if (auth) {
        vBadge.className = 'verdict-tag match';
        vBadge.innerText = 'MATCH (AUTHORIZED)';
      } else {
        vBadge.className = 'verdict-tag uncertain';
        vBadge.innerText = 'UNCERTAIN';
      }

      // TFT State
      document.getElementById('display-state-val').innerText = `${(s.display.state || 'IDLE').toUpperCase()} FACE`;
      document.getElementById('display-card-val').innerText = `Card: ${s.display.showing_card ? 'ON' : 'OFF'} • Progress: ${s.display.progress}%`;

      // Active Mission
      document.getElementById('mission-name-val').innerText = s.mission.carried_box ? `Carrying: ${s.mission.carried_box}` : (s.mission.mission_id ? `Active (#${s.mission.mission_id})` : "NONE");
      document.getElementById('mission-progress-bar').style.width = (s.display.progress || 0) + '%';

      // 3. Arm Joint sliders & meters
      if (s.arm.q_cmd) {
        document.getElementById('val-yaw').innerText = Math.round(s.arm.q_cmd[0]) + '°';
        document.getElementById('val-shoulder').innerText = Math.round(s.arm.q_cmd[1]) + '°';
        document.getElementById('val-elbow').innerText = Math.round(s.arm.q_cmd[2]) + '°';
        document.getElementById('val-wrist').innerText = Math.round(s.arm.q_cmd[3]) + '°';
      }

      // Torque Meters
      const mSh = document.getElementById('meter-shoulder');
      const mEl = document.getElementById('meter-elbow');
      const mWr = document.getElementById('meter-wrist');

      if (mSh) {
        mSh.style.width = Math.min(100, shUtil) + '%';
        mSh.className = 'meter-fill ' + (shUtil > 100 ? 'warn' : '');
        document.getElementById('stat-shoulder').innerText = `${torque.shoulder_kg_cm || 0} kg.cm (${shUtil}% Stall)`;
      }
      if (mEl) {
        const elUtil = torque.elbow_util_pct || 0;
        mEl.style.width = Math.min(100, elUtil) + '%';
        document.getElementById('stat-elbow').innerText = `${torque.elbow_kg_cm || 0} kg.cm (${elUtil}% Stall)`;
      }
      if (mWr) {
        mWr.style.width = Math.min(100, wrUtil) + '%';
        mWr.className = 'meter-fill ' + (wrUtil > 100 ? 'warn' : '');
        document.getElementById('stat-wrist').innerText = `${torque.wrist_pitch_kg_cm || 0} kg.cm (${wrUtil}% Stall)`;
      }

      // 4. Audio speech bubble
      if (s.audio && s.audio.last_spoken) {
        document.getElementById('speech-bubble').innerText = `“${s.audio.last_spoken}”`;
        if (s.audio.tts_backend === 'web_speech') speakBrowserText(s.audio.last_spoken, s.audio.speech_id);
      }
      document.getElementById('mute-status').innerText = s.audio.muted ? "MUTED (SPEAKING)" : "UNMUTED";

      // 5. Timeline highlight
      updateTimelineHighlight(stateStr);

      // 6. Terminal logs
      if (s.logs && s.logs.length !== lastLogCount) {
        const term = document.getElementById('terminal-logs');
        term.innerHTML = '';
        s.logs.forEach(l => {
          const div = document.createElement('div');
          div.className = 'log-entry';
          div.innerHTML = `<span class="log-time">[${l.time_str}]</span> <span class="log-src">[${l.source}]</span> <span class="log-type">${l.type}</span>: ${JSON.stringify(l.data)}`;
          term.appendChild(div);
        });
        term.scrollTop = term.scrollHeight;
        lastLogCount = s.logs.length;
      }
    })
    .catch(err => {
      document.getElementById('link-status').innerText = 'OFFLINE';
      document.getElementById('link-status').className = 'pill-value offline';
    });
}

function updateTimelineHighlight(state) {
  const steps = document.querySelectorAll('.timeline-step');
  steps.forEach(st => st.classList.remove('active'));

  const map = {
    'IDLE': 0,
    'PLANNING': 1,
    'NAVIGATING': 2,
    'DOCKING': 3,
    'ARM_SEARCH': 4,
    'ARM_VERIFY': 4,
    'ARM_APPROACH': 5,
    'ARM_GRIP': 5,
    'ARM_LIFT': 5,
    'ARM_CARRY': 6,
    'NAVIGATING_RETURN': 6,
    'ARM_DELIVER': 7,
    'SUCCESS': 7
  };

  const idx = map[state] !== undefined ? map[state] : 0;
  if (steps[idx]) {
    steps[idx].classList.add('active');
  }
}

// ---------------------------------------------------------------------------
// Live Robot & Teach tab
// ---------------------------------------------------------------------------
function postJson(url, body, statusId) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(r => r.json())
    .then(res => {
      if (statusId) {
        const el = document.getElementById(statusId);
        if (el) el.innerText = res.ok ? ('OK ' + (res.message || '')) : ('Error: ' + (res.error || 'failed'));
      }
      return res;
    });
}

function nudge(delta) {
  postJson('/api/arm/nudge', delta, 'teach-status');
}

function teachSave() {
  const med = parseInt(document.getElementById('teach-med').value);
  const slot = document.getElementById('teach-slot').checked;
  postJson('/api/arm/teach', { medicine_id: med, update_slot: slot }).then(res => {
    const el = document.getElementById('teach-status');
    el.innerText = res.ok
      ? `Saved ${res.medicine}: grasp point ${JSON.stringify(res.grasp_point_mm)} mm, slot ${res.slot_u_mm} mm, tilt ${res.phi_deg} deg` + (res.saved_to_file ? ' (written to medicines.json)' : '')
      : 'Error: ' + res.error;
  });
}

function testGrasp(stage) {
  const med = parseInt(document.getElementById('teach-med').value);
  postJson('/api/arm/test_grasp', { medicine_id: med, stage: stage }, 'teach-status');
}

function setLive(id, text, good) {
  const el = document.getElementById(id);
  if (!el) return;
  el.innerText = text;
  el.className = good === true ? 'live-ok' : (good === false ? 'live-bad' : '');
}

function updateLivePanel(s) {
  if (!document.getElementById('tab-live')) return;
  const car = s.car || {}, arm = s.arm || {}, vis = s.vision || {}, dev = s.devices || {};

  if (dev.car_sim) setLive('live-esp32', 'simulation (car_sim: true)', null);
  else setLive('live-esp32', car.connected ? ('connected ' + (car.esp32_ip || '')) : ('NOT connected ' + (car.link_error || '')), !!car.connected);
  setLive('live-cam', `${vis.hand_status || '-'} • ${(vis.hand_fps || 0).toFixed(1)} fps`, vis.hand_status === 'CONNECTED' ? true : null);
  setLive('live-detector', `${vis.detector || '-'} ${vis.yolo_status ? '(' + vis.yolo_status + ')' : ''}`, null);
  const disc = dev.discovered || {};
  setLive('live-discovered', Object.keys(disc).length ? Object.entries(disc).map(([n, d]) => `${n}: ${d.ip}`).join(', ') : 'none yet', null);
  if (arm.pca_ok) setLive('live-pca', arm.pca_ok.map(ok => ok ? 'OK' : 'MISSING').join(' / '), arm.pca_ok.every(x => x));

  const d = car.distances_cm || {};
  const box = document.getElementById('live-distances');
  if (box) {
    box.innerHTML = ['front', 'back', 'left', 'right'].map(k => {
      const cm = d[k] === undefined ? 999 : d[k];
      const pct = Math.max(0, Math.min(100, 100 - cm));          // closer = fuller bar
      const cls = cm < 15 ? 'meter-fill warn' : 'meter-fill';
      const label = cm >= 999 ? 'clear' : cm.toFixed(0) + ' cm';
      return `<div class="dist-row"><span class="dist-name">${k}</span>` +
             `<div class="meter-bar-container"><div class="${cls}" style="width:${pct}%"></div></div>` +
             `<span class="dist-val">${label}</span></div>`;
    }).join('');
  }
  setLive('live-obstacles', `Obstacle side: ${car.obstacle_side || 'none'} • remembered on map: ${car.dynamic_obstacles || 0}`, null);

  const v = arm.last_verify || {};
  if (dev.arm_sim) setLive('live-verify', 'simulation', null);
  else setLive('live-verify', v.ok === undefined ? '-' : (v.ok ? `confirmed (max error ${v.max_error_deg} deg)` : v.error), v.ok === undefined ? null : !!v.ok);
  setLive('live-gripsensor', arm.grip_sensor_present ? (arm.grip_sensor_box ? 'BOX DETECTED' : 'empty') : 'not fitted (camera check used)', arm.grip_sensor_present ? !!arm.grip_sensor_box : null);
  const g = s.grip || {};
  setLive('live-grip', g.held === undefined ? '-' : `${g.held === true ? 'HELD' : (g.held === false ? 'EMPTY' : 'UNSURE')} via ${g.method} • ${g.reason} (${g.stage})`,
          g.held === undefined || g.held === null ? null : !!g.held);
  setLive('live-carry', (s.mission && s.mission.carried_box) || 'nothing', null);
  const sc = vis.last_scan || {};
  if (sc.slots) {
    const names = { 1: 'Paracetamol', 2: 'Cetirizine', 3: 'Omeprazole' };
    const txt = Object.entries(sc.slots).map(([u, id]) => `${Math.round(u)} mm: ${names[id] || id}`).join(' • ');
    setLive('live-scan', `${sc.medicine}: ${sc.found_slot !== null && sc.found_slot !== undefined ? 'found at ' + sc.found_slot + ' mm' : 'NOT found'} (${txt})`, sc.found_slot !== null && sc.found_slot !== undefined);
  }
  if (arm.tip) setLive('teach-tip', `tip x ${arm.tip.x} y ${arm.tip.y} z ${arm.tip.z} mm • tilt ${arm.tip.phi}°`, null);
}

function updateAiPanel(s) {
  const L = s.learning;
  if (!L || !document.getElementById('ai-kalman')) return;
  const k = L.kalman || {}, d = L.drive || {}, st = L.stats || {};
  const names = { 1: 'Paracetamol', 2: 'Cetirizine', 3: 'Omeprazole' };
  setLive('ai-kalman', k.active ? `active • avg correction ${k.avg_correction_mm} mm • ±${k.std_pos_mm} mm` : 'waiting for the phone camera (sim: off)', k.active ? true : null);
  setLive('ai-gains', `speed ×${d.speed_gain} ${d.speed_ready ? '(in use)' : '(learning)'} • turn ×${d.turn_gain} ${d.turn_ready ? '(in use)' : '(learning)'}`, d.speed_ready && d.turn_ready ? true : null);
  setLive('ai-drift', `${d.drift_rad_per_m} rad per m • turn bias ${d.turn_bias_rad_s} rad/s`, null);
  setLive('ai-samples', `${d.speed_samples} speed, ${d.turn_samples} turn (needs 40 each)`, null);
  const sv = L.visual_servo || {};
  setLive('ai-servo', sv.aligned === undefined ? '-' : (sv.aligned ? `aligned (dy ${sv.dy} mm, dz ${sv.dz} mm)` : (sv.reason || 'not aligned')),
          sv.aligned === undefined || sv.aligned === null ? null : !!sv.aligned);
  const off = L.grasp_offsets || {};
  setLive('ai-offsets', Object.keys(off).length ? Object.entries(off).map(([k, o]) => `${k} mm: dy ${o.dy}, dz ${o.dz} (${o.n} grips)`).join(' • ') : 'nothing learned yet', null);
  const sh = L.shelf_memory || {};
  setLive('ai-shelf', Object.keys(sh).length ? Object.entries(sh).map(([id, m]) => `${names[id] || id} @ ${Math.round(m.slot_u)} mm`).join(' • ') : 'empty', null);
  setLive('ai-stats', st.missions ? `${st.missions} • ${(st.success_rate * 100).toFixed(0)}% (last 10: ${(st.recent_success_rate * 100).toFixed(0)}%)` : 'no missions yet', st.missions ? st.success_rate >= 0.8 : null);
  setLive('ai-avg', st.avg_duration_s ? `${st.avg_duration_s} s • ${st.avg_grip_attempts} grabs` : '-', null);
  setLive('ai-fail', st.last_failure || 'none', st.last_failure ? false : null);
}

// ---------------------------------------------------------------------------
// Robot self-test
// ---------------------------------------------------------------------------
let selfTestTimer = null;
function runSelfTest() {
  const move = document.getElementById('selftest-move').checked;
  if (move && !confirm('The arm will move and the car will drive 10 cm forward and back. Is the area clear?')) return;
  postJson('/api/selftest/run', { allow_motion: move }).then(res => {
    if (!res.ok) { document.getElementById('selftest-summary').innerText = 'Error: ' + res.error; return; }
    document.getElementById('btn-selftest').disabled = true;
    clearInterval(selfTestTimer);
    selfTestTimer = setInterval(updateSelfTest, 700);
    updateSelfTest();
  });
}
function updateSelfTest() {
  fetch('/api/selftest').then(r => r.json()).then(rep => {
    const colors = { PASS: '#3fb950', WARN: '#d29922', FAIL: '#f85149', SKIP: '#8b949e', LOOK: '#58a6ff' };
    const icons = { PASS: '\u2714', WARN: '\u26a0', FAIL: '\u2716', SKIP: '\u2013', LOOK: '\ud83d\udc41' };
    document.getElementById('selftest-table').innerHTML = rep.results.map(r =>
      `<tr><td><span style="color:${colors[r.status]};font-weight:700">${icons[r.status]} ${r.status}</span> ${r.name}</td>` +
      `<td>${r.detail}${r.fix ? `<br><span style="color:${colors[r.status]}">\u2192 ${r.fix}</span>` : ''}</td></tr>`).join('');
    const c = rep.counts;
    document.getElementById('selftest-summary').innerText = rep.running
      ? 'Running... (' + rep.results.length + ' checks done)'
      : `Done in ${rep.duration_s} s: ${c.PASS} passed, ${c.WARN} warnings, ${c.FAIL} failed, ${c.LOOK} to look at, ${c.SKIP} skipped.`;
    if (!rep.running) { clearInterval(selfTestTimer); document.getElementById('btn-selftest').disabled = false; }
  });
}

// Initial boot
window.addEventListener('load', () => {
  loadDoctorDiag();
  setInterval(updateStatus, 500); // 2 Hz telemetry poll
});

// ---------------------------------------------------------------------------
// Camera views: fetch ONE picture at a time (~6 per second) and only for views that
// are visible. Endless video streams would use up the browser's ~6 connections to
// the server and the buttons would stop responding.
// ---------------------------------------------------------------------------
function startSnapshotViews() {
  document.querySelectorAll('img[data-snap]').forEach(img => {
    let busy = false;
    const next = () => setTimeout(tick, 160);
    function tick() {
      if (busy) return;
      if (img.offsetParent === null || document.hidden) { setTimeout(tick, 500); return; }  // hidden tab
      busy = true;
      const loader = new Image();
      loader.onload = () => { img.src = loader.src; busy = false; next(); };
      loader.onerror = () => { busy = false; setTimeout(tick, 1000); };
      loader.src = img.dataset.snap + '?once=1&t=' + Date.now();
    }
    tick();
  });
}
window.addEventListener('load', startSnapshotViews);
