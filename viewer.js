/**
 * Lynx Robot 3D Viewer with PD Control
 *
 * Three.js-based MJCF viewer that parses MuJoCo XML, builds a matching
 * scene graph, and runs a PD controller with velocity/acceleration limits
 * identical to test_push_pd_limited.py.
 *
 * Dependencies (loaded from CDN by the host page):
 *   - Three.js r128  https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js
 *   - OrbitControls   https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js
 */

// ---------------------------------------------------------------------------
// MJCF Parser
// ---------------------------------------------------------------------------

class MJCFParser {
  /**
   * Parse an MJCF XML string and return a structured model description.
   * @param {string} xmlString
   * @returns {object} parsed model
   */
  static parse(xmlString) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(xmlString, 'text/xml');
    const mujoco = doc.querySelector('mujoco');
    if (!mujoco) throw new Error('No <mujoco> element found in XML');

    // Compiler settings
    const compiler = mujoco.querySelector('compiler');
    const angleUnit = compiler ? (compiler.getAttribute('angle') || 'degree') : 'degree';
    const useRadians = angleUnit === 'radian';

    // Option settings
    const option = mujoco.querySelector('option');
    const timestep = option ? parseFloat(option.getAttribute('timestep') || '0.002') : 0.002;

    // Parse worldbody hierarchy
    const worldbody = mujoco.querySelector('worldbody');
    const bodies = [];
    const joints = [];
    const geoms = [];

    if (worldbody) {
      MJCFParser._parseBody(worldbody, null, bodies, joints, geoms, useRadians);
    }

    // Parse actuators
    const actuators = [];
    const actuatorEl = mujoco.querySelector('actuator');
    if (actuatorEl) {
      for (const child of actuatorEl.children) {
        actuators.push({
          type: child.tagName,
          name: child.getAttribute('name') || '',
          joint: child.getAttribute('joint') || '',
          kp: parseFloat(child.getAttribute('kp') || '0'),
          kv: parseFloat(child.getAttribute('kv') || '0'),
          ctrlrange: MJCFParser._parseFloats(child.getAttribute('ctrlrange')),
          forcerange: MJCFParser._parseFloats(child.getAttribute('forcerange')),
        });
      }
    }

    return { bodies, joints, geoms, actuators, timestep, useRadians };
  }

  static _parseBody(el, parentName, bodies, joints, geoms, useRadians) {
    // Gather geoms that are direct children of this element (worldbody or body)
    for (const geomEl of el.children) {
      if (geomEl.tagName !== 'geom') continue;
      geoms.push(MJCFParser._parseGeom(geomEl, el.getAttribute('name') || '__worldbody__', useRadians));
    }

    for (const bodyEl of el.children) {
      if (bodyEl.tagName !== 'body') continue;

      const bodyName = bodyEl.getAttribute('name') || `body_${bodies.length}`;
      const pos = MJCFParser._parseVec3(bodyEl.getAttribute('pos'));
      const euler = MJCFParser._parseEuler(bodyEl.getAttribute('euler'), useRadians);
      const quat = MJCFParser._parseQuat(bodyEl.getAttribute('quat'));

      bodies.push({ name: bodyName, parent: parentName || '__worldbody__', pos, euler, quat });

      // Joints in this body
      for (const jEl of bodyEl.children) {
        if (jEl.tagName !== 'joint') continue;
        const jName = jEl.getAttribute('name') || `joint_${joints.length}`;
        const axis = MJCFParser._parseVec3(jEl.getAttribute('axis')) || [0, 0, 1];
        const range = MJCFParser._parseFloats(jEl.getAttribute('range'));
        const armature = parseFloat(jEl.getAttribute('armature') || '1.0');
        const damping = parseFloat(jEl.getAttribute('damping') || '0');
        const frictionloss = parseFloat(jEl.getAttribute('frictionloss') || '0');
        const jPos = MJCFParser._parseVec3(jEl.getAttribute('pos'));
        const jType = jEl.getAttribute('type') || 'hinge';

        // Skip freejoints
        if (jEl.tagName === 'freejoint' || jType === 'free') continue;

        joints.push({
          name: jName,
          body: bodyName,
          type: jType,
          axis,
          range: range && range.length === 2 ? range : [-Math.PI, Math.PI],
          armature,
          damping,
          frictionloss,
          pos: jPos,
        });
      }

      // Handle freejoints separately (skip them)
      for (const jEl of bodyEl.children) {
        if (jEl.tagName === 'freejoint') {
          // Mark body as free — we will still render it but not control it
          bodies[bodies.length - 1].isFree = true;
        }
      }

      // Geoms in this body
      for (const gEl of bodyEl.children) {
        if (gEl.tagName !== 'geom') continue;
        geoms.push(MJCFParser._parseGeom(gEl, bodyName, useRadians));
      }

      // Recurse
      MJCFParser._parseBody(bodyEl, bodyName, bodies, joints, geoms, useRadians);
    }
  }

  static _parseGeom(el, bodyName, useRadians) {
    const type = el.getAttribute('type') || 'sphere';
    const size = MJCFParser._parseFloats(el.getAttribute('size')) || [0.01];
    const pos = MJCFParser._parseVec3(el.getAttribute('pos')) || [0, 0, 0];
    const euler = MJCFParser._parseEuler(el.getAttribute('euler'), useRadians);
    const quat = MJCFParser._parseQuat(el.getAttribute('quat'));
    const rgba = MJCFParser._parseFloats(el.getAttribute('rgba')) || [0.5, 0.5, 0.5, 1];
    const name = el.getAttribute('name') || '';
    const group = parseInt(el.getAttribute('group') || '0', 10);
    const fromto = MJCFParser._parseFloats(el.getAttribute('fromto'));

    return { type, size, pos, euler, quat, rgba, name, body: bodyName, group, fromto };
  }

  static _parseVec3(s) {
    if (!s) return null;
    const parts = s.trim().split(/\s+/).map(Number);
    return parts.length >= 3 ? parts.slice(0, 3) : null;
  }

  static _parseEuler(s, useRadians) {
    if (!s) return null;
    const parts = s.trim().split(/\s+/).map(Number);
    if (parts.length < 3) return null;
    if (!useRadians) {
      return parts.map(v => v * Math.PI / 180);
    }
    return parts;
  }

  static _parseQuat(s) {
    if (!s) return null;
    const parts = s.trim().split(/\s+/).map(Number);
    return parts.length >= 4 ? parts.slice(0, 4) : null;
  }

  static _parseFloats(s) {
    if (!s) return null;
    return s.trim().split(/\s+/).map(Number);
  }
}

// ---------------------------------------------------------------------------
// PD Controller
// ---------------------------------------------------------------------------

class PDController {
  /**
   * @param {number} nJoints
   * @param {object} params  { kp, kd, effort_limit, max_velocity, max_acceleration }
   * @param {Array<object>} jointDefs  parsed joint definitions
   */
  constructor(nJoints, params, jointDefs) {
    this.n = nJoints;
    this.dt = params.dt || 0.002;

    this.kp = new Float64Array(nJoints);
    this.kd = new Float64Array(nJoints);
    this.effortLimit = new Float64Array(nJoints);
    this.armature = new Float64Array(nJoints);
    this.damping = new Float64Array(nJoints);
    this.jointRange = [];

    this.maxVelocity = params.max_velocity || 2.0;
    this.maxAcceleration = params.max_acceleration || 10.0;

    for (let i = 0; i < nJoints; i++) {
      this.kp[i] = params.kp ? params.kp[i] : 800;
      this.kd[i] = params.kd ? params.kd[i] : 75;
      this.effortLimit[i] = params.effort_limit ? params.effort_limit[i] : 100;
      this.armature[i] = jointDefs[i] ? (jointDefs[i].armature || 1.0) : 1.0;
      this.damping[i] = jointDefs[i] ? (jointDefs[i].damping || 0.0) : 0.0;
      this.jointRange.push(
        jointDefs[i] ? jointDefs[i].range : [-Math.PI, Math.PI]
      );
    }

    // State arrays
    this.qpos = new Float64Array(nJoints);
    this.qvel = new Float64Array(nJoints);
    this.target = new Float64Array(nJoints);
  }

  /**
   * Run one physics step — matches test_push_pd_limited.py exactly.
   */
  step() {
    const dt = this.dt;
    for (let i = 0; i < this.n; i++) {
      const q = this.qpos[i];
      const v = this.qvel[i];

      // 1. PD torque
      let torque = this.kp[i] * (this.target[i] - q) + this.kd[i] * (0.0 - v);

      // 2. Effort limiting
      torque = Math.max(-this.effortLimit[i], Math.min(this.effortLimit[i], torque));

      // 3. Compute acceleration (simplified: torque / armature)
      const acceleration = torque / this.armature[i];

      // 4. Update velocity
      let newVel = v + acceleration * dt;

      // 5. Velocity limiting
      newVel = Math.max(-this.maxVelocity, Math.min(this.maxVelocity, newVel));

      // 6. Acceleration limiting
      let acc = (newVel - v) / dt;
      acc = Math.max(-this.maxAcceleration, Math.min(this.maxAcceleration, acc));
      newVel = v + acc * dt;

      // 7. Update state
      this.qpos[i] = q + newVel * dt;
      this.qvel[i] = newVel;

      // 8. Apply damping
      this.qvel[i] *= (1.0 - this.damping[i] * dt);

      // 9. Clamp to joint range
      this.qpos[i] = Math.max(this.jointRange[i][0], Math.min(this.jointRange[i][1], this.qpos[i]));
    }
  }

  reset() {
    this.qpos.fill(0);
    this.qvel.fill(0);
    this.target.fill(0);
  }
}

// ---------------------------------------------------------------------------
// MuJoCoViewer — Three.js 3D viewer
// ---------------------------------------------------------------------------

class MuJoCoViewer {
  /**
   * @param {HTMLElement} containerElement
   */
  constructor(containerElement) {
    this.container = containerElement;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.animFrameId = null;
    this.paused = false;

    // Model data
    this.model = null;       // parsed MJCF
    this.pd = null;          // PDController
    this.bodyGroups = {};    // name -> THREE.Group
    this.jointDefs = [];     // filtered hinge joints
    this.jointBodyGroups = []; // THREE.Group for each joint's body
    this._disposed = false;
    this._meshes = [];
    this._geometries = [];
    this._materials = [];
  }

  // ---------- Initialisation ----------

  async init() {
    if (typeof THREE === 'undefined') {
      throw new Error('Three.js is not loaded. Include three.min.js before viewer.js');
    }

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    this._updateSize();
    this.container.appendChild(this.renderer.domElement);

    // Scene — dark neutral background
    this.scene = new THREE.Scene();
    const bgColor = new THREE.Color(0.08, 0.08, 0.08);
    this.scene.background = bgColor;

    // Fog — same color as background, hides ground plane edges
    this.scene.fog = new THREE.Fog(bgColor, 3, 8);

    // Camera (Z-up: we keep MuJoCo convention and rotate camera)
    this.camera = new THREE.PerspectiveCamera(45, this.container.clientWidth / this.container.clientHeight, 0.01, 100);
    this.camera.up.set(0, 0, 1); // Z-up
    this.camera.position.set(1.44, -1.44, 2.56);
    this.camera.lookAt(0, 0, 0.9);

    // Controls
    if (THREE.OrbitControls) {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.target.set(0, 0, 0.9);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.12;
      this.controls.minDistance = 0.15;
      this.controls.maxDistance = 5;
      this.controls.update();
    }

    // Lighting
    this._setupLights();

    // Ground plane with checker
    this._addGround();

    // Resize handling
    this._resizeObserver = new ResizeObserver(() => this._onResize());
    this._resizeObserver.observe(this.container);

    // Start render loop
    this._startLoop();
  }

  // ---------- Model Loading ----------

  /**
   * Load an MJCF model and set up PD controller.
   * @param {string} mjcfString  raw XML
   * @param {object} pdParams    { kp: [], kd: [], effort_limit: [], max_velocity, max_acceleration }
   */
  async loadModel(mjcfString, pdParams) {
    // Dispose previous model
    this._disposeModel();

    // Parse MJCF
    this.model = MJCFParser.parse(mjcfString);

    // Filter to hinge joints only (no free joints)
    this.jointDefs = this.model.joints.filter(j => j.type === 'hinge');

    const n = this.jointDefs.length;
    const params = Object.assign({ dt: this.model.timestep }, pdParams || {});

    // Provide defaults if pdParams doesn't have arrays
    if (!params.kp || params.kp.length === 0) {
      params.kp = new Array(n).fill(800);
    }
    if (!params.kd || params.kd.length === 0) {
      params.kd = new Array(n).fill(75);
    }
    if (!params.effort_limit || params.effort_limit.length === 0) {
      params.effort_limit = new Array(n).fill(100);
    }

    // Try to extract kp/kd from actuators if present and not provided
    if (this.model.actuators.length > 0 && pdParams && !pdParams.kp) {
      for (let i = 0; i < n && i < this.model.actuators.length; i++) {
        const act = this.model.actuators[i];
        if (act.kp > 0) params.kp[i] = act.kp;
        if (act.kv > 0) params.kd[i] = act.kv;
        if (act.forcerange && act.forcerange.length >= 2) {
          params.effort_limit[i] = act.forcerange[1];
        }
      }
    }

    this.pd = new PDController(n, params, this.jointDefs);

    // Build Three.js scene graph
    this._buildSceneGraph();
  }

  // ---------- Control Interface ----------

  setTargetPosition(jointIndex, target) {
    if (!this.pd || jointIndex < 0 || jointIndex >= this.pd.n) return;
    this.pd.target[jointIndex] = target;
  }

  setAllTargetPositions(targetArray) {
    if (!this.pd) return;
    for (let i = 0; i < this.pd.n && i < targetArray.length; i++) {
      this.pd.target[i] = targetArray[i];
    }
  }

  getTargetPositions() {
    return this.pd ? Array.from(this.pd.target) : [];
  }

  getCurrentPositions() {
    return this.pd ? Array.from(this.pd.qpos) : [];
  }

  getCurrentVelocities() {
    return this.pd ? Array.from(this.pd.qvel) : [];
  }

  // ---------- Model Info ----------

  getJointCount() {
    return this.jointDefs ? this.jointDefs.length : 0;
  }

  getJointNames() {
    return this.jointDefs ? this.jointDefs.map(j => j.name) : [];
  }

  getJointRanges() {
    return this.jointDefs ? this.jointDefs.map(j => [...j.range]) : [];
  }

  // ---------- Simulation Control ----------

  reset() {
    if (this.pd) this.pd.reset();
    this._applyJointAngles();
  }

  setPaused(p) { this.paused = !!p; }
  isRunning() { return !this.paused; }

  // ---------- PD Parameter Adjustment ----------

  setPDGains(kpArray, kdArray) {
    if (!this.pd) return;
    for (let i = 0; i < this.pd.n; i++) {
      if (kpArray && i < kpArray.length) this.pd.kp[i] = kpArray[i];
      if (kdArray && i < kdArray.length) this.pd.kd[i] = kdArray[i];
    }
  }

  setLimits(effortLimit, maxVel, maxAcc) {
    if (!this.pd) return;
    if (effortLimit) {
      for (let i = 0; i < this.pd.n && i < effortLimit.length; i++) {
        this.pd.effortLimit[i] = effortLimit[i];
      }
    }
    if (maxVel !== undefined) this.pd.maxVelocity = maxVel;
    if (maxAcc !== undefined) this.pd.maxAcceleration = maxAcc;
  }

  // ---------- Cleanup ----------

  dispose() {
    this._disposed = true;
    if (this.animFrameId) cancelAnimationFrame(this.animFrameId);
    if (this._resizeObserver) this._resizeObserver.disconnect();
    this._disposeModel();
    if (this.renderer) {
      this.renderer.dispose();
      if (this.renderer.domElement.parentNode) {
        this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
      }
    }
  }

  // =====================================================================
  // Private methods
  // =====================================================================

  _updateSize() {
    const w = this.container.clientWidth || 300;
    const h = this.container.clientHeight || 300;
    this.renderer.setSize(w, h);
    if (this.camera) {
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
    }
  }

  _onResize() {
    this._updateSize();
  }

  // ---------- Lights ----------

  _setupLights() {
    // Ambient — slightly brighter for dark scene
    const ambient = new THREE.AmbientLight(0xffffff, 0.25);
    this.scene.add(ambient);

    // Hemisphere — cool neutral above, dark below
    const hemi = new THREE.HemisphereLight(0xcccccc, 0x222222, 0.4);
    this.scene.add(hemi);

    // Key light — strong directional with shadows
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(2, -2, 4);
    dir.castShadow = true;
    dir.shadow.mapSize.set(1024, 1024);
    dir.shadow.camera.near = 0.1;
    dir.shadow.camera.far = 10;
    dir.shadow.camera.left = -2;
    dir.shadow.camera.right = 2;
    dir.shadow.camera.top = 2;
    dir.shadow.camera.bottom = -2;
    this.scene.add(dir);

    // Fill light — softer, neutral
    const fill = new THREE.DirectionalLight(0xffffff, 0.3);
    fill.position.set(-1, 2, 2);
    this.scene.add(fill);
  }

  // ---------- Ground ----------

  _addGround() {
    // Yellow-tinted checker texture (matching MuJoCo sim floor)
    const size = 512;
    const canvas = document.createElement('canvas');
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d');
    const tileSize = size / 32;
    for (let r = 0; r < 32; r++) {
      for (let c = 0; c < 32; c++) {
        ctx.fillStyle = (r + c) % 2 === 0 ? '#c9b873' : '#a89650';
        ctx.fillRect(c * tileSize, r * tileSize, tileSize, tileSize);
      }
    }
    const tex = new THREE.CanvasTexture(canvas);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(8, 8);

    // Large ground plane — fog hides the edges
    const groundGeo = new THREE.PlaneGeometry(30, 30);
    const groundMat = new THREE.MeshStandardMaterial({
      map: tex, roughness: 0.85, metalness: 0.05,
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.receiveShadow = true;
    this.scene.add(ground);

    // Track ground objects separately so they survive model reloads
    this._groundObjects = { meshes: [ground], geometries: [groundGeo], materials: [groundMat] };
  }

  // ---------- Scene graph from MJCF ----------

  _buildSceneGraph() {
    const { bodies, geoms } = this.model;

    // Root group for the whole model
    this._modelRoot = new THREE.Group();
    this.scene.add(this._modelRoot);

    // Create a group for every body (and __worldbody__)
    this.bodyGroups = {};
    this.bodyGroups['__worldbody__'] = this._modelRoot;

    for (const body of bodies) {
      const grp = new THREE.Group();
      grp.name = body.name;

      // Apply body position
      if (body.pos) {
        grp.position.set(body.pos[0], body.pos[1], body.pos[2]);
      }

      // Apply body orientation
      if (body.quat) {
        // MuJoCo quat = [w, x, y, z], Three.js Quaternion(x, y, z, w)
        grp.quaternion.set(body.quat[1], body.quat[2], body.quat[3], body.quat[0]);
      } else if (body.euler) {
        // XYZ euler in radians
        grp.rotation.set(body.euler[0], body.euler[1], body.euler[2], 'XYZ');
      }

      this.bodyGroups[body.name] = grp;

      // Parent group
      const parentGroup = this.bodyGroups[body.parent] || this._modelRoot;
      parentGroup.add(grp);
    }

    // Map joints to body groups
    this.jointBodyGroups = [];
    for (const jd of this.jointDefs) {
      this.jointBodyGroups.push(this.bodyGroups[jd.body] || null);
    }

    // Create meshes for geoms
    for (const g of geoms) {
      const mesh = this._createGeomMesh(g);
      if (!mesh) continue;
      const parent = this.bodyGroups[g.body] || this._modelRoot;
      parent.add(mesh);
      this._meshes.push(mesh);
    }
  }

  _createGeomMesh(g) {
    let geometry = null;
    let material = null;

    const color = new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]);
    const opacity = g.rgba[3];
    const transparent = opacity < 1.0;

    material = new THREE.MeshStandardMaterial({
      color,
      opacity,
      transparent,
      roughness: 0.55,
      metalness: 0.15,
      side: transparent ? THREE.DoubleSide : THREE.FrontSide,
    });

    switch (g.type) {
      case 'plane': {
        // Ground plane from MJCF (skip if already have ground)
        // We skip MJCF planes since we add our own
        return null;
      }

      case 'box': {
        const sx = g.size[0] * 2;
        const sy = g.size[1] * 2;
        const sz = g.size.length >= 3 ? g.size[2] * 2 : sx;
        geometry = new THREE.BoxGeometry(sx, sy, sz);
        break;
      }

      case 'sphere': {
        const r = g.size[0];
        geometry = new THREE.SphereGeometry(r, 24, 16);
        break;
      }

      case 'cylinder': {
        // MuJoCo: size = [radius, half_length]
        const radius = g.size[0];
        const halfLen = g.size.length >= 2 ? g.size[1] : 0.05;
        geometry = new THREE.CylinderGeometry(radius, radius, halfLen * 2, 24);
        // Three.js CylinderGeometry axis is Y. MuJoCo cylinder axis is Z.
        // Rotate geometry so its axis aligns with Z.
        geometry.rotateX(Math.PI / 2);
        break;
      }

      case 'capsule': {
        const radius = g.size[0];
        let halfLen;

        if (g.fromto && g.fromto.length === 6) {
          // fromto capsule
          const p0 = new THREE.Vector3(g.fromto[0], g.fromto[1], g.fromto[2]);
          const p1 = new THREE.Vector3(g.fromto[3], g.fromto[4], g.fromto[5]);
          const dir = p1.clone().sub(p0);
          const len = dir.length();
          halfLen = len / 2;

          // Build capsule along Z
          geometry = this._capsuleGeometry(radius, halfLen);

          const mesh = new THREE.Mesh(geometry, material);
          this._geometries.push(geometry);
          this._materials.push(material);

          // Position at midpoint
          const mid = p0.clone().add(p1).multiplyScalar(0.5);
          mesh.position.set(mid.x, mid.y, mid.z);

          // Orient: default axis is Z, need to rotate to match dir
          if (len > 1e-8) {
            const zAxis = new THREE.Vector3(0, 0, 1);
            const q = new THREE.Quaternion().setFromUnitVectors(zAxis, dir.clone().normalize());
            mesh.quaternion.copy(q);
          }

          mesh.castShadow = true;
          mesh.receiveShadow = true;
          return mesh;
        } else {
          halfLen = g.size.length >= 2 ? g.size[1] : 0.05;
          geometry = this._capsuleGeometry(radius, halfLen);
        }
        break;
      }

      default:
        // Fallback: small sphere
        geometry = new THREE.SphereGeometry(0.01, 12, 8);
        break;
    }

    if (!geometry) return null;

    this._geometries.push(geometry);
    this._materials.push(material);

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // Apply geom local position
    if (g.pos) {
      mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
    }

    // Apply geom local orientation
    if (g.quat) {
      mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
    } else if (g.euler) {
      mesh.rotation.set(g.euler[0], g.euler[1], g.euler[2], 'XYZ');
    }

    return mesh;
  }

  /**
   * Build a capsule geometry (cylinder + two hemisphere caps) aligned with Z axis.
   */
  _capsuleGeometry(radius, halfLen) {
    const segments = 20;

    // Cylinder part
    let cylGeo = new THREE.CylinderGeometry(radius, radius, halfLen * 2, segments);
    cylGeo.rotateX(Math.PI / 2); // align Y->Z

    // Top hemisphere
    let topSphere = new THREE.SphereGeometry(radius, segments, segments / 2, 0, Math.PI * 2, 0, Math.PI / 2);
    topSphere.rotateX(Math.PI / 2);
    topSphere.translate(0, 0, halfLen);

    // Bottom hemisphere
    let botSphere = new THREE.SphereGeometry(radius, segments, segments / 2, 0, Math.PI * 2, 0, Math.PI / 2);
    botSphere.rotateX(-Math.PI / 2);
    botSphere.translate(0, 0, -halfLen);

    // Convert indexed geometries to non-indexed so we can merge vertex buffers.
    // Without this, copying position arrays from indexed geometry produces garbled
    // triangles because the index buffer is lost.
    const geosSrc = [cylGeo, topSphere, botSphere];
    const geos = geosSrc.map(g => {
      const ni = g.toNonIndexed();
      g.dispose();
      return ni;
    });

    const merged = new THREE.BufferGeometry();
    const positions = [];
    const normals = [];
    for (const geo of geos) {
      const pos = geo.getAttribute('position');
      const nrm = geo.getAttribute('normal');
      for (let i = 0; i < pos.count; i++) {
        positions.push(pos.getX(i), pos.getY(i), pos.getZ(i));
        normals.push(nrm.getX(i), nrm.getY(i), nrm.getZ(i));
      }
      geo.dispose();
    }

    merged.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    merged.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));

    return merged;
  }

  // ---------- Joint angle application ----------

  _applyJointAngles() {
    if (!this.pd) return;

    for (let i = 0; i < this.jointDefs.length; i++) {
      const jd = this.jointDefs[i];
      const grp = this.jointBodyGroups[i];
      if (!grp) continue;

      const angle = this.pd.qpos[i];
      const ax = jd.axis;

      // Reset rotation to identity first, then apply body orientation + joint rotation
      // We store the original body orientation and compose with joint rotation.
      if (!grp.userData._origQuat) {
        grp.userData._origQuat = grp.quaternion.clone();
      }

      // Joint rotation axis in body-local frame
      const axisVec = new THREE.Vector3(ax[0], ax[1], ax[2]).normalize();
      const jointQuat = new THREE.Quaternion().setFromAxisAngle(axisVec, angle);

      // Compose: original body orientation * joint rotation
      grp.quaternion.copy(grp.userData._origQuat).multiply(jointQuat);
    }
  }

  // ---------- Animation loop ----------

  _startLoop() {
    let lastTime = performance.now();

    const loop = (now) => {
      if (this._disposed) return;
      this.animFrameId = requestAnimationFrame(loop);

      const dt = (now - lastTime) / 1000;
      lastTime = now;

      // Physics steps
      if (!this.paused && this.pd) {
        const stepsPerFrame = Math.round(Math.min(dt, 1 / 30) / this.pd.dt);
        for (let s = 0; s < stepsPerFrame; s++) {
          this.pd.step();
        }
        this._applyJointAngles();
      }

      // Render
      if (this.controls) this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };

    this.animFrameId = requestAnimationFrame(loop);
  }

  // ---------- Cleanup helpers ----------

  _disposeModel() {
    // Remove meshes
    for (const m of this._meshes) {
      if (m.parent) m.parent.remove(m);
    }
    for (const g of this._geometries) g.dispose();
    for (const m of this._materials) m.dispose();
    this._meshes = [];
    this._geometries = [];
    this._materials = [];

    if (this._modelRoot && this._modelRoot.parent) {
      this._modelRoot.parent.remove(this._modelRoot);
    }
    this._modelRoot = null;
    this.bodyGroups = {};
    this.jointBodyGroups = [];
    this.jointDefs = [];
    this.pd = null;
    this.model = null;
  }
}

// ---------------------------------------------------------------------------
// Export for ES modules or global scope
// ---------------------------------------------------------------------------
if (typeof window !== 'undefined') {
  window.MuJoCoViewer = MuJoCoViewer;
  window.MJCFParser = MJCFParser;
  window.PDController = PDController;
}
