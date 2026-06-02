/* ============================================================
   Lynx Control Panel  --  Joint target/actual position UI
   ============================================================ */

import { countSelfCollisions, lowestArmZ, TABLE_TOP_Z } from './mjcf-generator.js';

export class ControlPanel {
  /**
   * @param {HTMLElement} containerElement
   * @param {object} viewer  — MuJoCoViewer instance (see viewer.js API)
   */
  constructor(containerElement, viewer) {
    this._container = containerElement;
    this._viewer = viewer;
    this._rafId = null;
    this._monitoring = false;

    // Per-joint cached DOM refs (populated by updateForModel)
    this._rows = []; // { slider, targetVal, actualVal, errorDot, progressFill, lastTarget, lastActual }

    // Advanced gain values (defaults)
    this._kp = 500;
    this._kd = 50;
    this._maxVel = 2.0;
    this._maxAcc = 10.0;

    this._build();
  }

  /* ---------------------------------------------------------- */
  /*  Public API                                                 */
  /* ---------------------------------------------------------- */

  /** Rebuild the slider rows for the current model. */
  updateForModel() {
    this._rows = [];
    const count = this._viewer.getJointCount();
    this._jointList.innerHTML = '';

    if (count === 0) {
      this._jointList.innerHTML = '';
      const empty = document.createElement('div');
      empty.className = 'cp-empty';
      empty.textContent = 'No joints \u2014 generate a model first';
      this._jointList.appendChild(empty);
      return;
    }

    const names = this._viewer.getJointNames();
    const ranges = this._viewer.getJointRanges();

    for (let i = 0; i < count; i++) {
      const name = names[i] || `Joint ${i}`;
      const [rMin, rMax] = ranges[i] || [-3.14, 3.14];
      this._createJointRow(i, name, rMin, rMax);
    }
  }

  /** Start the requestAnimationFrame monitoring loop. */
  startMonitoring() {
    if (this._monitoring) return;
    this._monitoring = true;
    this._tick();
  }

  /** Stop the monitoring loop. */
  stopMonitoring() {
    this._monitoring = false;
    if (this._rafId !== null) {
      cancelAnimationFrame(this._rafId);
      this._rafId = null;
    }
  }

  /* ---------------------------------------------------------- */
  /*  Internal — skeleton build                                  */
  /* ---------------------------------------------------------- */

  _build() {
    this._container.innerHTML = '';
    this._container.classList.add('lynx-control-panel');

    // ---- Toolbar ----
    const toolbar = _el('div', 'cp-toolbar');

    const title = _el('span', 'cp-toolbar-title');
    title.textContent = 'Joint Control';
    toolbar.appendChild(title);

    this._btnReset = this._makeBtn('Reset All', 'cp-btn-reset', () => this._onReset());
    this._btnRandom = this._makeBtn('Random Pose', 'cp-btn-random', () => this._onRandom());

    toolbar.appendChild(this._btnReset);
    toolbar.appendChild(this._btnRandom);
    this._container.appendChild(toolbar);

    // ---- Joint list ----
    this._jointList = _el('div', 'cp-joint-list');
    this._container.appendChild(this._jointList);

    // Empty state initially
    const empty = _el('div', 'cp-empty');
    empty.textContent = 'No joints \u2014 generate a model first';
    this._jointList.appendChild(empty);

    // ---- Advanced section ----
    this._buildAdvanced();
  }

  _makeBtn(label, cls, onClick) {
    const btn = _el('button', `cp-btn ${cls}`);
    btn.textContent = label;
    btn.addEventListener('click', onClick);
    return btn;
  }

  /* ---------------------------------------------------------- */
  /*  Internal — joint row creation                              */
  /* ---------------------------------------------------------- */

  _createJointRow(index, name, min, max) {
    const row = _el('div', 'cp-joint-row');

    // ---- Main line ----
    const main = _el('div', 'cp-joint-main');

    const label = _el('span', 'cp-joint-label');
    label.textContent = name;
    label.title = name;
    main.appendChild(label);

    // Target group
    const tg = _el('div', 'cp-target-group');

    const tLabel = _el('span', 'cp-target-label');
    tLabel.textContent = 'Target:';
    tg.appendChild(tLabel);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'cp-slider';
    slider.min = min;
    slider.max = max;
    slider.step = 0.01;
    slider.value = 0;
    tg.appendChild(slider);

    const targetVal = _el('span', 'cp-value');
    targetVal.textContent = '0.00 rad';
    tg.appendChild(targetVal);

    main.appendChild(tg);

    // Actual group
    const ag = _el('div', 'cp-actual-group');

    const aLabel = _el('span', 'cp-actual-label');
    aLabel.textContent = 'Actual:';
    ag.appendChild(aLabel);

    const actualVal = _el('span', 'cp-actual-value');
    actualVal.textContent = '0.00 rad';
    ag.appendChild(actualVal);

    const errorDot = _el('span', 'cp-error-dot cp-err-green');
    ag.appendChild(errorDot);

    main.appendChild(ag);
    row.appendChild(main);

    // ---- Progress bar ----
    const progressWrap = _el('div', 'cp-progress-wrap');
    const progressFill = _el('div', 'cp-progress-fill');
    progressFill.style.width = '0%';
    progressFill.style.background = _progressGradient(0);
    progressWrap.appendChild(progressFill);
    row.appendChild(progressWrap);

    this._jointList.appendChild(row);

    // Slider event
    slider.addEventListener('input', () => {
      const val = parseFloat(slider.value);
      this._viewer.setTargetPosition(index, val);
      targetVal.textContent = val.toFixed(2) + ' rad';
    });

    // Cache references
    this._rows.push({
      slider,
      targetVal,
      actualVal,
      errorDot,
      progressFill,
      min,
      max,
      lastTarget: NaN,
      lastActual: NaN,
    });
  }

  /* ---------------------------------------------------------- */
  /*  Internal — advanced section                                */
  /* ---------------------------------------------------------- */

  _buildAdvanced() {
    // Toggle header
    const toggle = _el('div', 'cp-advanced-toggle');
    this._advArrow = _el('span', 'cp-advanced-arrow');
    this._advArrow.textContent = '\u25B6';
    toggle.appendChild(this._advArrow);
    const tText = document.createTextNode(' Advanced Settings');
    toggle.appendChild(tText);
    toggle.addEventListener('click', () => this._toggleAdvanced());
    this._container.appendChild(toggle);

    // Body
    this._advBody = _el('div', 'cp-advanced-body');

    const kpRow = this._advSliderRow('Kp (proportional)', 100, 2000, 1, this._kp, (v) => { this._kp = v; });
    const kdRow = this._advSliderRow('Kd (derivative)', 10, 200, 1, this._kd, (v) => { this._kd = v; });
    const velRow = this._advSliderRow('Max velocity (rad/s)', 0.5, 5.0, 0.1, this._maxVel, (v) => { this._maxVel = v; });
    const accRow = this._advSliderRow('Max accel (rad/s\u00B2)', 2.0, 20.0, 0.5, this._maxAcc, (v) => { this._maxAcc = v; });

    this._advBody.appendChild(kpRow);
    this._advBody.appendChild(kdRow);
    this._advBody.appendChild(velRow);
    this._advBody.appendChild(accRow);

    const applyBtn = _el('button', 'cp-adv-apply');
    applyBtn.textContent = 'Apply';
    applyBtn.addEventListener('click', () => this._applyAdvanced());
    this._advBody.appendChild(applyBtn);

    this._container.appendChild(this._advBody);
  }

  _advSliderRow(label, min, max, step, initial, onChange) {
    const row = _el('div', 'cp-adv-row');

    const lbl = _el('span', 'cp-adv-label');
    lbl.textContent = label;
    row.appendChild(lbl);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'cp-adv-slider';
    slider.min = min;
    slider.max = max;
    slider.step = step;
    slider.value = initial;
    row.appendChild(slider);

    const valSpan = _el('span', 'cp-adv-value');
    valSpan.textContent = Number(initial).toFixed(step < 1 ? 1 : 0);
    row.appendChild(valSpan);

    slider.addEventListener('input', () => {
      const v = parseFloat(slider.value);
      valSpan.textContent = v.toFixed(step < 1 ? 1 : 0);
      onChange(v);
    });

    return row;
  }

  _toggleAdvanced() {
    const open = this._advBody.classList.toggle('cp-visible');
    this._advArrow.classList.toggle('cp-expanded', open);
  }

  _applyAdvanced() {
    const count = this._viewer.getJointCount();
    if (count === 0) return;

    const kpArr = new Float64Array(count).fill(this._kp);
    const kdArr = new Float64Array(count).fill(this._kd);
    this._viewer.setPDGains(kpArr, kdArr);

    const effortArr = new Float64Array(count).fill(1000); // large default
    this._viewer.setLimits(effortArr, this._maxVel, this._maxAcc);
  }

  /* ---------------------------------------------------------- */
  /*  Internal — button handlers                                 */
  /* ---------------------------------------------------------- */

  _onReset() {
    this._viewer.reset();
    // Update sliders to 0
    for (const r of this._rows) {
      r.slider.value = 0;
      r.targetVal.textContent = '0.00 rad';
    }
  }

  _onTogglePause() {
    const running = this._viewer.isRunning();
    this._viewer.setPaused(running);
    this._btnPause.textContent = running ? 'Resume' : 'Pause';
    this._btnPause.classList.toggle('cp-btn-pause', !running);
  }

  _onRandom() {
    const count = this._viewer.getJointCount();
    if (count === 0) return;

    const ranges = this._viewer.getJointRanges();
    const model = this._viewer.model;                  // parsed MJCF (for collision FK)
    const start = this._viewer.getCurrentPositions();  // current (collision-free) pose
    const MAX_TRIES = 20;
    const PATH_SAMPLES = 5;                            // checks along the swept motion

    const sampleTargets = () => {
      const t = new Float64Array(count);
      for (let i = 0; i < count; i++) {
        const [rMin, rMax] = ranges[i] || [-3.14, 3.14];
        t[i] = rMin + Math.random() * (rMax - rMin);
      }
      return t;
    };

    // Collisions swept from the current pose to `targets` (self + table floor),
    // sampled at a few way-points so the *whole motion* stays clean — not just
    // the endpoint. Pure FK + capsule maths, run only on click (never per frame).
    const pathViolations = (targets) => {
      if (!model) return 0;
      let v = 0;
      for (let s = 1; s <= PATH_SAMPLES; s++) {
        const a = s / PATH_SAMPLES;
        const pose = new Array(count);
        for (let i = 0; i < count; i++) {
          const s0 = (start && start[i] != null) ? start[i] : 0;
          pose[i] = s0 + (targets[i] - s0) * a;
        }
        try {
          v += countSelfCollisions(model, { jointAngles: pose });
          if (lowestArmZ(model, { jointAngles: pose }) < TABLE_TOP_Z) v += 1;
        } catch (e) { /* never block motion on a check failure */ }
      }
      return v;
    };

    let targets = null, best = Infinity;
    for (let t = 0; t < MAX_TRIES; t++) {
      const cand = sampleTargets();
      const v = pathViolations(cand);
      if (v < best) { best = v; targets = cand; }
      if (v === 0) break;
    }
    if (!targets) targets = sampleTargets();

    for (let i = 0; i < count; i++) {
      if (this._rows[i]) {
        this._rows[i].slider.value = targets[i];
        this._rows[i].targetVal.textContent = targets[i].toFixed(2) + ' rad';
      }
    }
    this._viewer.setAllTargetPositions(targets);
  }

  /* ---------------------------------------------------------- */
  /*  Internal — monitoring loop                                 */
  /* ---------------------------------------------------------- */

  _tick() {
    if (!this._monitoring) return;

    const count = this._viewer.getJointCount();
    if (count > 0 && this._rows.length === count) {
      const actuals = this._viewer.getCurrentPositions();
      const targets = this._viewer.getTargetPositions();

      for (let i = 0; i < count; i++) {
        const r = this._rows[i];
        const actual = actuals[i];
        const target = targets[i];

        // Only touch DOM when value changed beyond threshold
        const THRESH = 0.001;

        if (Math.abs(actual - r.lastActual) > THRESH) {
          r.lastActual = actual;
          r.actualVal.textContent = actual.toFixed(2) + ' rad';

          // Progress bar: actual position within [min, max]
          const range = r.max - r.min;
          const pct = range > 0 ? ((actual - r.min) / range) * 100 : 50;
          r.progressFill.style.width = Math.max(0, Math.min(100, pct)).toFixed(1) + '%';
        }

        // Error dot + progress color
        const err = Math.abs(target - actual);
        if (Math.abs(target - r.lastTarget) > THRESH || Math.abs(actual - r.lastActual) < THRESH + 0.001) {
          r.lastTarget = target;

          let errClass;
          if (err < 0.05) errClass = 'cp-err-green';
          else if (err < 0.2) errClass = 'cp-err-yellow';
          else errClass = 'cp-err-red';

          // Only update class when it changes
          const current = r.errorDot.className;
          const desired = 'cp-error-dot ' + errClass;
          if (current !== desired) {
            r.errorDot.className = desired;
          }

          // Update progress fill color
          r.progressFill.style.background = _progressGradient(err);
        }

        // Sync slider if target was changed externally
        if (Math.abs(target - parseFloat(r.slider.value)) > THRESH) {
          r.slider.value = target;
          r.targetVal.textContent = target.toFixed(2) + ' rad';
        }
      }
    }

    this._rafId = requestAnimationFrame(() => this._tick());
  }
}

/* ============================================================
   Helpers
   ============================================================ */

function _el(tag, cls) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  return e;
}

/**
 * Returns a CSS gradient string for the progress fill.
 * Low error -> blue-green, high error -> blue-red.
 */
function _progressGradient(error) {
  if (error < 0.05) return 'linear-gradient(90deg, #2e8b57, #4cdf6b)';
  if (error < 0.2) return 'linear-gradient(90deg, #2e8b57, #f0c040)';
  return 'linear-gradient(90deg, #2e8b57, #ff5050)';
}
