/**
 * main.js — Integration layer for Lynx Morphology Viewer
 *
 * Wires together:
 *   - GraphEditor (global, from graph-editor.js)
 *   - MuJoCoViewer (global, from viewer.js)
 *   - ControlPanel (ES module, from control-panel.js)
 *   - generateMJCF / getDefaultPDParams (ES module, from mjcf-generator.js)
 */

import { ControlPanel } from './control-panel.js';
import { generateMJCF, getDefaultPDParams } from './mjcf-generator.js';

// ---------------------------------------------------------------------------
// Debug logger
// ---------------------------------------------------------------------------
const logContent = document.getElementById('log-content');
const logPanel = document.getElementById('log-panel');
const logToggle = document.getElementById('log-toggle');

function log(msg, level = 'info') {
  const ts = new Date().toLocaleTimeString();
  const line = document.createElement('div');
  line.className = `log-line log-${level}`;
  line.textContent = `[${ts}] ${msg}`;
  logContent.appendChild(line);
  logContent.scrollTop = logContent.scrollHeight;
  if (level === 'error') console.error(msg);
  else console.log(msg);
}

logToggle.addEventListener('click', () => {
  logPanel.classList.toggle('hidden');
  logToggle.classList.toggle('active');
});

// ---------------------------------------------------------------------------
// Status bar
// ---------------------------------------------------------------------------
const statusJoints = document.getElementById('status-joints');
const statusFps = document.getElementById('status-fps');
const statusModel = document.getElementById('status-model');

let frameCount = 0;
let lastFpsTime = performance.now();

function updateFps() {
  frameCount++;
  const now = performance.now();
  if (now - lastFpsTime >= 1000) {
    const fps = Math.round(frameCount * 1000 / (now - lastFpsTime));
    statusFps.textContent = `FPS: ${fps}`;
    frameCount = 0;
    lastFpsTime = now;
  }
  requestAnimationFrame(updateFps);
}
requestAnimationFrame(updateFps);

// ---------------------------------------------------------------------------
// Generate button states (initialised lazily after GraphEditor creates the DOM)
// ---------------------------------------------------------------------------
let generateBtn, generateStatus, generateLabel, generateIcon;

function initGenerateBtn() {
  generateBtn = document.getElementById('generate-btn');
  generateStatus = document.getElementById('generate-status');
  if (generateBtn) {
    generateLabel = generateBtn.querySelector('.generate-label');
    generateIcon = generateBtn.querySelector('.generate-icon');
  }
}

function setGenerateState(state, message) {
  if (!generateBtn) initGenerateBtn();
  if (!generateBtn) return;
  generateBtn.disabled = state === 'loading';
  generateBtn.classList.remove('state-loading', 'state-success', 'state-error');

  switch (state) {
    case 'loading':
      generateBtn.classList.add('state-loading');
      generateLabel.textContent = 'Generating...';
      generateIcon.innerHTML = '<span class="btn-spinner"></span>';
      generateStatus.textContent = '';
      generateStatus.className = 'generate-status';
      break;
    case 'success':
      generateBtn.classList.add('state-success');
      generateLabel.textContent = 'Model Loaded!';
      generateIcon.textContent = '\u2714';
      generateStatus.textContent = message || '';
      generateStatus.className = 'generate-status status-success';
      setTimeout(() => {
        generateBtn.classList.remove('state-success');
        generateLabel.textContent = 'Generate Model';
        generateIcon.textContent = '';
      }, 1500);
      break;
    case 'error':
      generateLabel.textContent = 'Generate Model';
      generateIcon.textContent = '';
      generateStatus.textContent = message || 'Error';
      generateStatus.className = 'generate-status status-error';
      break;
    default:
      generateLabel.textContent = 'Generate Model';
      generateIcon.textContent = '';
      generateStatus.textContent = '';
      generateStatus.className = 'generate-status';
  }
}

// ---------------------------------------------------------------------------
// Main initialisation
// ---------------------------------------------------------------------------
async function main() {
  log('Initialising viewer...');

  // 1. Initialise the 3D viewer
  const viewer = new MuJoCoViewer(document.getElementById('viewer-container'));
  await viewer.init();
  log('Three.js viewer ready');

  // 2. Initialise graph editor (this creates generate-btn in the DOM)
  const editor = new GraphEditor(document.getElementById('graph-container'));
  log('Graph editor ready');

  // 3. Init generate button refs (now that editor has created the DOM)
  initGenerateBtn();

  // 4. Initialise control panel
  const panel = new ControlPanel(document.getElementById('panel-container'), viewer);
  log('Control panel ready');

  // 5. Generate model handler
  async function generateModel() {
    try {
      setGenerateState('loading');
      log('Generating MJCF from morphology...');

      const morphology = editor.getMorphology();
      const mjcf = generateMJCF(morphology);
      const pdParams = getDefaultPDParams(morphology);

      await viewer.loadModel(mjcf, pdParams);

      panel.updateForModel();
      panel.startMonitoring();

      const jointCount = viewer.getJointCount();
      statusJoints.textContent = `Joints: ${jointCount}`;
      statusModel.textContent = 'Model: loaded';

      setGenerateState('success', `${jointCount} joints loaded`);
      log(`Model generated successfully with ${jointCount} joints`);
    } catch (err) {
      setGenerateState('error', err.message);
      log(`Model generation failed: ${err.message}`, 'error');
    }
  }

  if (generateBtn) generateBtn.addEventListener('click', generateModel);

  // 5b. Reset model handler
  const resetModelBtn = document.getElementById('reset-model-btn');
  if (resetModelBtn) {
    resetModelBtn.addEventListener('click', () => {
      log('Resetting to default morphology...');
      editor.resetMorphology();
      generateModel();
    });
  }

  // 5c. Random model handler — sample a morphology from the full editable space
  const randomModelBtn = document.getElementById('random-model-btn');
  if (randomModelBtn) {
    randomModelBtn.addEventListener('click', () => {
      log('Sampling a random morphology...');
      editor.randomMorphology();
      generateModel();
    });
  }

  // 5. Hide loading overlay
  window.__morphbenchReady = true;
  const overlay = document.getElementById('loading-overlay');
  overlay.classList.add('fade-out');
  setTimeout(() => overlay.remove(), 400);

  // 6. Auto-generate on load with default config
  log('Auto-generating default model...');
  setTimeout(() => generateModel(), 300);
}

main().catch((err) => {
  console.error('Fatal init error:', err);
  window.__morphbenchReady = true;
  const ov = document.getElementById('loading-overlay');
  if (ov) {
    ov.innerHTML =
      '<div style="text-align:center;max-width:480px;padding:0 24px">' +
        '<div style="font-size:28px;margin-bottom:12px">\u26A0\uFE0F</div>' +
        '<div style="font-size:15px;font-weight:600;color:#333;margin-bottom:8px">Initialisation error</div>' +
        '<div style="font-size:13px;color:#888;line-height:1.5">' + err.message + '</div>' +
        '<button onclick="location.reload()" style="margin-top:16px;padding:8px 20px;border:1px solid #ccc;border-radius:6px;background:#fff;cursor:pointer;font-size:13px">Retry</button>' +
      '</div>';
  }
});
