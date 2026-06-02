/**
 * Lynx Morphology Graph Editor
 *
 * A cytoscape.js-based visual editor for serial-chain robot arm morphology.
 * Features: drag-and-drop from parts palette, snap-to-insert animations,
 * right-click delete, enhanced property panels.
 *
 * Pure vanilla JS, no build tools.
 *
 * Usage:
 *   const editor = new GraphEditor(containerElement);
 *   editor.onMorphologyChange(callback);
 *   editor.getMorphology();
 *   editor.setMorphology(configJSON);
 */

/* global cytoscape */

const DEFAULT_MORPHOLOGY = {
  num_joints: 6,
  genotype_tube: [0, 1, 0, 1, 0],
  genotype_joints: [1, 0, 0, 0, 0],
  joint_types: ["inline", "inline", "orthogonal", "orthogonal", "orthogonal", "orthogonal"],
  rotation_angles: [180, 0, 0, -180, 0, 0],
  l1_end_point_pos: [0.0, 0.0, 0.2805],
  l1_end_point_theta: 0.0,
  l1_dual_point_distance: 0.05,
  l2_end_point_pos: [0.0, 0.0, 0.2805],
  l2_end_point_theta: 0.0,
  l2_dual_point_distance: 0.05,
  l3_end_point_pos: [0.0, 0.0, 0.2],
  l3_end_point_theta: 0.0,
  l3_dual_point_distance: 0.05,
  l4_end_point_pos: [0.0, 0.0, 0.2],
  l4_end_point_theta: 0.0,
  l4_dual_point_distance: 0.05,
  l5_end_point_pos: [0.0, 0.0, 0.2],
  l5_end_point_theta: 0.0,
  l5_dual_point_distance: 0.05,
};

const TUBE_DEFAULT_POS = [0.0, 0.0, 0.2];
const TUBE_DEFAULT_THETA = 0.0;
const TUBE_DEFAULT_DUAL_DIST = 0.05;
const MAX_JOINTS = 10;
// "Random Model" samples num_joints uniformly in [RANDOM_JOINTS_MIN, RANDOM_JOINTS_MAX].
// Set RANDOM_JOINTS_MAX = 6 to restrict sampling to the 4J/5J/6J benchmark taxonomy.
const RANDOM_JOINTS_MIN = 2;
const RANDOM_JOINTS_MAX = MAX_JOINTS;
const SPACING = 80;
const FIXED_GAP = 30;
const ANIM_DURATION = 300;


class GraphEditor {
  constructor(container) {
    this._container = container;
    this._callbacks = [];
    this._morphology = JSON.parse(JSON.stringify(DEFAULT_MORPHOLOGY));
    this._selectedElement = null;
    this._highlightedEdge = null;
    this._isDraggingJoint = false;
    this._ghostEl = null;
    this._insertIndicator = null;

    this._buildDOM();
    this._initCytoscape();
    this._buildGraph();
    this._bindEvents();
    this._updatePalette();

    // Cytoscape needs a non-zero container to render. On mobile or when the
    // container isn't fully laid out yet, force a resize after paint.
    requestAnimationFrame(() => {
      if (this._cy) {
        this._cy.resize();
        this._cy.fit(undefined, 45);
        const z = this._cy.zoom();
        this._cy.zoom(z * 1.32);
        this._cy.center();
      }
    });
  }

  // ------------------------------------------------------------------ Public API

  onMorphologyChange(cb) {
    if (typeof cb === "function") this._callbacks.push(cb);
  }

  getMorphology() {
    const m = JSON.parse(JSON.stringify(this._morphology));
    // Ensure joint_types is present
    if (!Array.isArray(m.joint_types)) m.joint_types = [];
    while (m.joint_types.length < m.num_joints) m.joint_types.push("orthogonal");
    m.joint_types.length = m.num_joints;
    m.joint_types[0] = "inline";
    // Compute genotype_joints as binary array for j2-j6
    m.genotype_joints = m.joint_types.slice(1).map(t => t === "inline" ? 1 : 0);
    return m;
  }

  resetMorphology() {
    this.setMorphology(JSON.parse(JSON.stringify(DEFAULT_MORPHOLOGY)));
  }

  /**
   * Sample a morphology by uniformly drawing EVERY dimension the editor can
   * change and RETURN it (no side effects — caller decides whether to load it,
   * e.g. after a collision check). Sampled space:
   *   - num_joints                     [RANDOM_JOINTS_MIN .. RANDOM_JOINTS_MAX]
   *   - each joint axis type (J1 fixed) inline | orthogonal
   *   - each joint rotation angle       -180..180 deg (slider step 1)
   *   - each link type                  direct | tube
   *   - each TUBE link geometry         end pos (x,y,z), theta, dual-point dist
   * Tube geometry is only editable when the link is a tube, so we sample it
   * only for tube links — matching the property panel exactly.
   */
  sampleRandomMorphology() {
    const ri = (lo, hi) => lo + Math.floor(Math.random() * (hi - lo + 1)); // int [lo,hi]
    const rf = (lo, hi) => lo + Math.random() * (hi - lo);                  // float [lo,hi)
    const snap = (v, step) => Math.round(v / step) * step;

    const n = ri(RANDOM_JOINTS_MIN, RANDOM_JOINTS_MAX);

    const joint_types = ["inline"]; // J1 is always inline
    for (let i = 1; i < n; i++) joint_types.push(Math.random() < 0.5 ? "inline" : "orthogonal");

    const rotation_angles = [];
    for (let i = 0; i < n; i++) rotation_angles.push(ri(-180, 180));

    const genotype_tube = [];
    for (let i = 0; i < n - 1; i++) genotype_tube.push(Math.random() < 0.5 ? 1 : 0);

    const cfg = { num_joints: n, joint_types, rotation_angles, genotype_tube };

    for (let i = 0; i < n - 1; i++) {
      if (genotype_tube[i] !== 1) continue; // direct link → no editable geometry
      const t = i + 1;
      cfg[`l${t}_end_point_pos`] = [
        snap(rf(-0.2, 0.2), 0.01),  // X (UI unbounded; bound like Y for sanity)
        snap(rf(-0.2, 0.2), 0.01),  // Y  [-0.2, 0.2]
        snap(rf(0.1, 0.36), 0.01),  // Z  [0.1, 0.36]
      ];
      cfg[`l${t}_end_point_theta`] = ri(-180, 180);                  // [-180, 180]
      cfg[`l${t}_dual_point_distance`] = snap(rf(0.01, 0.2), 0.005); // [0.01, 0.2]
    }

    return cfg;
  }

  /** Sample a random morphology and load it immediately (no collision check). */
  randomMorphology() {
    this.setMorphology(this.sampleRandomMorphology());
  }

  setMorphology(cfg) {
    const m = JSON.parse(JSON.stringify(cfg));
    m.num_joints = Math.max(1, Math.min(MAX_JOINTS, m.num_joints || 1));
    if (!Array.isArray(m.genotype_tube)) m.genotype_tube = [];
    while (m.genotype_tube.length < m.num_joints - 1) m.genotype_tube.push(0);
    m.genotype_tube.length = Math.max(0, m.num_joints - 1);
    // Build joint_types array
    if (Array.isArray(m.joint_types)) {
      // Use provided joint_types, pad/trim to num_joints
      while (m.joint_types.length < m.num_joints) m.joint_types.push("orthogonal");
      m.joint_types.length = m.num_joints;
    } else if (Array.isArray(m.genotype_joints)) {
      // Compute from binary array genotype_joints [1,0,1,...] for j2-j6
      m.joint_types = ["inline"]; // J1 always inline
      for (let i = 0; i < m.num_joints - 1; i++) {
        m.joint_types.push(i < m.genotype_joints.length && m.genotype_joints[i] ? "inline" : "orthogonal");
      }
    } else {
      // Legacy: compute from integer genotype_joints (cutoff)
      const gj = Math.max(0, Math.min(m.num_joints - 1, m.genotype_joints || 0));
      m.joint_types = ["inline"]; // J1 always inline
      for (let i = 0; i < m.num_joints - 1; i++) {
        m.joint_types.push(i < gj ? "inline" : "orthogonal");
      }
    }
    // J1 is always inline
    m.joint_types[0] = "inline";
    // Compute genotype_joints as binary array for j2-j6
    m.genotype_joints = m.joint_types.slice(1).map(t => t === "inline" ? 1 : 0);
    if (!Array.isArray(m.rotation_angles)) m.rotation_angles = [];
    while (m.rotation_angles.length < m.num_joints) m.rotation_angles.push(0);
    m.rotation_angles.length = m.num_joints;
    // Ensure tube properties exist for indices 1..num_joints-1
    for (let i = 1; i <= Math.max(m.num_joints - 1, 5); i++) {
      const pk = `l${i}_end_point_pos`;
      const tk = `l${i}_end_point_theta`;
      const dk = `l${i}_dual_point_distance`;
      if (!Array.isArray(m[pk]) || m[pk].length !== 3) m[pk] = [...TUBE_DEFAULT_POS];
      if (typeof m[tk] !== "number") m[tk] = TUBE_DEFAULT_THETA;
      if (typeof m[dk] !== "number") m[dk] = TUBE_DEFAULT_DUAL_DIST;
    }
    this._morphology = m;
    this._buildGraph();
    this._updatePalette();
    this._hidePanel();
    this._hideContextMenu();
    this._notify();
  }

  // ------------------------------------------------------------------ DOM

  _buildDOM() {
    this._container.classList.add("lynx-graph-editor");
    this._container.innerHTML = "";

    // Parts palette (left sidebar)
    this._paletteEl = document.createElement("div");
    this._paletteEl.className = "lynx-palette";
    this._paletteEl.innerHTML = `
      <div class="lynx-joint-chip" id="lynx-joint-chip" title="Drag to add a joint">
        <div class="lynx-chip-icons">
          <span class="lynx-legend-dot inline"></span>
          <span class="lynx-legend-dot orthogonal"></span>
        </div>
        <span class="lynx-chip-label">Drag to<br>Add Joint</span>
        <span class="lynx-chip-arrow">&#x27A1;</span>
      </div>
    `;
    this._container.appendChild(this._paletteEl);

    // Property panel (inline flex item, between palette and canvas)
    this._panelEl = document.createElement("div");
    this._panelEl.className = "lynx-prop-panel hidden";
    this._container.appendChild(this._panelEl);

    // Canvas (cytoscape)
    this._canvasEl = document.createElement("div");
    this._canvasEl.className = "lynx-graph-canvas";
    this._container.appendChild(this._canvasEl);

    // Legend (floating top-right)
    this._legendEl = document.createElement("div");
    this._legendEl.className = "lynx-legend";
    this._legendEl.innerHTML = `
      <div class="lynx-legend-item"><span class="lynx-legend-dot inline"></span> Inline</div>
      <div class="lynx-legend-item"><span class="lynx-legend-dot orthogonal"></span> Orthogonal</div>
      <div class="lynx-legend-item"><span class="lynx-legend-swatch tube"></span> Tube</div>
      <div class="lynx-legend-item"><span class="lynx-legend-swatch direct"></span> Direct</div>
    `;
    this._container.appendChild(this._legendEl);

    // Generate button (floating bottom-right)
    this._generateWrap = document.createElement("div");
    this._generateWrap.className = "lynx-generate-wrap";
    this._generateWrap.innerHTML = `
      <span id="generate-status" class="generate-status"></span>
      <button id="generate-btn" class="generate-btn"><span class="generate-icon"></span><span class="generate-label">Generate Model</span></button>
      <div class="model-btn-row">
        <button id="reset-model-btn" class="generate-btn reset-model-btn"><span class="reset-label">Reset Model</span></button>
        <button id="random-model-btn" class="generate-btn random-model-btn"><span class="random-label">Random Model</span></button>
      </div>
    `;
    this._container.appendChild(this._generateWrap);

    // Context menu (overlay)
    this._ctxMenuEl = document.createElement("div");
    this._ctxMenuEl.className = "lynx-ctx-menu hidden";
    this._container.appendChild(this._ctxMenuEl);

    // Ghost element for drag (overlay)
    this._ghostEl = document.createElement("div");
    this._ghostEl.className = "lynx-drag-ghost hidden";
    this._ghostEl.innerHTML = '<span class="lynx-joint-chip-icon"></span><span>Joint</span>';
    this._container.appendChild(this._ghostEl);

    // Insert indicator (+ circle shown on highlighted edge, overlay)
    this._insertIndicator = document.createElement("div");
    this._insertIndicator.className = "lynx-insert-indicator hidden";
    this._insertIndicator.textContent = "+";
    this._container.appendChild(this._insertIndicator);
  }

  // ------------------------------------------------------------------ Cytoscape

  _initCytoscape() {
    this._cy = cytoscape({
      container: this._canvasEl,
      style: [
        {
          selector: 'node[type="base"]',
          style: {
            label: "data(label)", "background-color": "#2a2a2a",
            width: 60, height: 40, shape: "round-rectangle", "corner-radius": 6,
            "font-size": 11, color: "#aaaaaa", "text-valign": "center", "text-halign": "center",
            "text-margin-x": -4,
            "font-weight": 700, "border-width": 1, "border-color": "#444444",
          },
        },
        {
          selector: 'node[type="ee"]',
          style: {
            label: "data(label)", "background-color": "#2a2a2a",
            width: 30, height: 20, shape: "round-rectangle", "corner-radius": 4,
            "font-size": 11, color: "#aaaaaa", "text-valign": "center", "text-halign": "center",
            "font-weight": 700, "border-width": 1, "border-color": "#444444",
          },
        },
        {
          selector: 'node[type="joint"][axisType="inline"]',
          style: {
            label: "data(label)", "background-color": "#2e8b57",
            width: 36, height: 36, shape: "ellipse", "font-size": 11, color: "#ffffff",
            "text-valign": "center", "text-halign": "center", "font-weight": 700,
            "border-width": 2, "border-color": "#1e6b3e",
          },
        },
        {
          selector: 'node[type="joint"][axisType="orthogonal"]',
          style: {
            label: "data(label)", "background-color": "#ff8c42",
            width: 36, height: 36, shape: "ellipse", "font-size": 11, color: "#ffffff",
            "text-valign": "center", "text-halign": "center", "font-weight": 700,
            "border-width": 2, "border-color": "#df6c22",
          },
        },
        {
          selector: 'edge[linkType="tube"]',
          style: {
            "line-color": "#2e8b57", width: 5, "line-style": "solid",
            "curve-style": "straight", "target-arrow-shape": "none",
          },
        },
        {
          selector: 'edge[linkType="direct"]',
          style: {
            "line-color": "#666666", width: 2, "line-style": "dashed",
            "line-dash-pattern": [6, 4], "curve-style": "straight", "target-arrow-shape": "none",
          },
        },
        {
          selector: 'edge[linkType="fixed"]',
          style: {
            "line-color": "#2a2a2a", width: 1, "line-style": "solid",
            "curve-style": "straight", "target-arrow-shape": "none",
            opacity: 0.5,
          },
        },
        {
          selector: "edge.highlight-insert",
          style: {
            "line-color": "#ffffff", width: 4, "line-style": "solid",
            opacity: 1, "z-index": 999,
          },
        },
        {
          selector: "node:selected",
          style: { "border-width": 3, "border-color": "#ffffff", "overlay-opacity": 0 },
        },
        {
          selector: "edge:selected",
          style: { "overlay-opacity": 0.08, "overlay-color": "#ffffff" },
        },
      ],
      layout: { name: "preset" },
      userZoomingEnabled: true,
      userPanningEnabled: true,
      boxSelectionEnabled: false,
      autoungrabify: false,
      minZoom: 0.3,
      maxZoom: 3,
    });
  }

  // ------------------------------------------------------------------ Graph build

  _getNodePositions() {
    const m = this._morphology;
    const n = m.num_joints;
    const cy = 100; // vertical center of the canvas
    // Horizontal layout: Base at left, EE at right
    const baseX = FIXED_GAP;
    const positions = { base: { x: baseX, y: cy } };
    for (let i = 1; i <= n; i++) {
      const x = baseX + FIXED_GAP + (i - 1) * SPACING;
      positions[`joint-${i}`] = { x, y: cy };
    }
    const lastJointX = baseX + FIXED_GAP + (n - 1) * SPACING;
    positions.ee = { x: lastJointX + FIXED_GAP, y: cy };
    return positions;
  }

  _buildGraph(animateFrom) {
    const m = this._morphology;
    const n = m.num_joints;
    const positions = this._getNodePositions();

    this._cy.elements().remove();

    // Base node
    this._cy.add({
      group: "nodes",
      data: { id: "base", label: "Base", type: "base" },
      position: { ...((animateFrom && animateFrom.base) || positions.base) },
      locked: true, grabbable: false,
    });

    // Joint nodes
    for (let i = 1; i <= n; i++) {
      const isInline = this._isJointInline(i);
      const id = `joint-${i}`;
      this._cy.add({
        group: "nodes",
        data: { id, label: `J${i}`, type: "joint", jointIndex: i, axisType: isInline ? "inline" : "orthogonal" },
        position: { ...((animateFrom && animateFrom[id]) || positions[id]) },
        grabbable: false,
      });
    }

    // EE node
    this._cy.add({
      group: "nodes",
      data: { id: "ee", label: "EE", type: "ee" },
      position: { ...((animateFrom && animateFrom.ee) || positions.ee) },
      locked: true, grabbable: false,
    });

    // Edges
    this._cy.add({ group: "edges", data: { id: "edge-base-j1", source: "base", target: "joint-1", linkType: "fixed", edgeIndex: -1 } });
    for (let i = 1; i < n; i++) {
      const isTube = m.genotype_tube[i - 1] === 1;
      this._cy.add({
        group: "edges",
        data: { id: `edge-j${i}-j${i + 1}`, source: `joint-${i}`, target: `joint-${i + 1}`, linkType: isTube ? "tube" : "direct", edgeIndex: i - 1 },
      });
    }
    this._cy.add({ group: "edges", data: { id: `edge-j${n}-ee`, source: `joint-${n}`, target: "ee", linkType: "fixed", edgeIndex: -1 } });

    // Animate to correct positions if animateFrom was provided
    if (animateFrom) {
      this._animateToPositions(positions);
    }

    this._cy.fit(undefined, 45);
    const z = this._cy.zoom();
    this._cy.zoom(z * 1.32);
    this._cy.center();
  }

  _animateToPositions(positions) {
    for (const [id, pos] of Object.entries(positions)) {
      const node = this._cy.getElementById(id);
      if (node.length) {
        node.unlock();
        node.animate({ position: pos }, { duration: ANIM_DURATION, easing: "ease-out", complete: () => {
          if (id === "base" || id === "ee") node.lock();
        }});
      }
    }
  }

  _isJointInline(jointIndex) {
    if (jointIndex === 1) return true;
    const types = this._morphology.joint_types;
    if (types && types[jointIndex - 1]) return types[jointIndex - 1] === "inline";
    return false;
  }

  // ------------------------------------------------------------------ Ensure tube properties

  _ensureTubeProperties() {
    const m = this._morphology;
    for (let i = 1; i <= Math.max(m.num_joints - 1, 0); i++) {
      const pk = `l${i}_end_point_pos`;
      const tk = `l${i}_end_point_theta`;
      const dk = `l${i}_dual_point_distance`;
      if (!Array.isArray(m[pk]) || m[pk].length !== 3) m[pk] = [...TUBE_DEFAULT_POS];
      if (typeof m[tk] !== "number") m[tk] = TUBE_DEFAULT_THETA;
      if (typeof m[dk] !== "number") m[dk] = TUBE_DEFAULT_DUAL_DIST;
    }
  }

  // ------------------------------------------------------------------ Events

  _bindEvents() {
    // Click on joint node -> show joint panel
    this._cy.on("tap", 'node[type="joint"]', (evt) => {
      const node = evt.target;
      this._cy.elements().unselect();
      node.select();
      this._selectedElement = node;
      this._showJointPanel(node.data("jointIndex"));
    });

    // Click on edge -> show link panel
    this._cy.on("tap", "edge", (evt) => {
      const edge = evt.target;
      const idx = edge.data("edgeIndex");
      if (idx < 0) return;
      this._cy.elements().unselect();
      edge.select();
      this._selectedElement = edge;
      this._showLinkPanel(idx);
    });

    // Click on background -> deselect & hide panel
    this._cy.on("tap", (evt) => {
      if (evt.target === this._cy) {
        this._cy.elements().unselect();
        this._selectedElement = null;
        this._hidePanel();
      }
    });

    // Right-click context menu on joints
    this._cy.on("cxttap", 'node[type="joint"]', (evt) => {
      evt.originalEvent.preventDefault();
      const node = evt.target;
      const jointIndex = node.data("jointIndex");
      const renderedPos = node.renderedPosition();
      const canvasRect = this._canvasEl.getBoundingClientRect();
      const containerRect = this._container.getBoundingClientRect();
      const x = canvasRect.left - containerRect.left + renderedPos.x + 20;
      const y = canvasRect.top - containerRect.top + renderedPos.y;
      this._showContextMenu(x, y, jointIndex);
    });

    // Prevent default context menu on canvas
    this._canvasEl.addEventListener("contextmenu", (e) => e.preventDefault());

    // Hide context menu on click elsewhere
    document.addEventListener("pointerdown", (e) => {
      if (!this._ctxMenuEl.contains(e.target)) {
        this._hideContextMenu();
      }
    });

    // Drag from palette
    this._initDragFromPalette();
  }

  // ------------------------------------------------------------------ Drag from Palette

  _initDragFromPalette() {
    const chip = this._container.querySelector("#lynx-joint-chip");
    let dragging = false;
    let startX = 0, startY = 0;

    const onPointerDown = (e) => {
      if (this._morphology.num_joints >= MAX_JOINTS) return;
      e.preventDefault();
      dragging = false;
      startX = e.clientX;
      startY = e.clientY;
      document.addEventListener("pointermove", onPointerMove);
      document.addEventListener("pointerup", onPointerUp);
    };

    const onPointerMove = (e) => {
      const dx = e.clientX - startX;
      const dy = e.clientY - startY;
      if (!dragging && (Math.abs(dx) > 5 || Math.abs(dy) > 5)) {
        dragging = true;
        this._isDraggingJoint = true;
        this._ghostEl.classList.remove("hidden");
        // Disable cy panning during drag
        this._cy.userPanningEnabled(false);
      }
      if (dragging) {
        const containerRect = this._container.getBoundingClientRect();
        const gx = e.clientX - containerRect.left - 30;
        const gy = e.clientY - containerRect.top - 15;
        this._ghostEl.style.left = gx + "px";
        this._ghostEl.style.top = gy + "px";
        this._updateInsertHighlight(e.clientX, e.clientY);
      }
    };

    const onPointerUp = (e) => {
      document.removeEventListener("pointermove", onPointerMove);
      document.removeEventListener("pointerup", onPointerUp);
      if (dragging) {
        this._ghostEl.classList.add("hidden");
        this._insertIndicator.classList.add("hidden");
        this._cy.userPanningEnabled(true);
        this._isDraggingJoint = false;
        if (this._highlightedEdge !== null) {
          this._clearEdgeHighlight();
          this._insertJointAtEdge(this._highlightedEdge);
          this._highlightedEdge = null;
        }
      }
      dragging = false;
    };

    chip.addEventListener("pointerdown", onPointerDown);
  }

  _updateInsertHighlight(clientX, clientY) {
    const canvasRect = this._canvasEl.getBoundingClientRect();
    // Check if cursor is over canvas
    if (clientX < canvasRect.left || clientX > canvasRect.right ||
        clientY < canvasRect.top || clientY > canvasRect.bottom) {
      this._clearEdgeHighlight();
      this._insertIndicator.classList.add("hidden");
      this._highlightedEdge = null;
      return;
    }

    // Find nearest insertable edge (non-fixed edges between joints)
    const m = this._morphology;
    const n = m.num_joints;
    // Cytoscape model position from rendered position
    const renderedX = clientX - canvasRect.left;
    const renderedY = clientY - canvasRect.top;
    const modelPos = this._cy.renderer().projectIntoViewport(clientX, clientY);

    let bestEdgeIdx = -1;
    let bestDist = Infinity;
    let bestMidRendered = null;

    // Check inter-joint edges
    for (let i = 0; i < n - 1; i++) {
      const edgeId = `edge-j${i + 1}-j${i + 2}`;
      const edge = this._cy.getElementById(edgeId);
      if (!edge.length) continue;
      const srcPos = edge.source().renderedPosition();
      const tgtPos = edge.target().renderedPosition();
      const midX = (srcPos.x + tgtPos.x) / 2;
      const midY = (srcPos.y + tgtPos.y) / 2;
      const dist = Math.sqrt((renderedX - midX) ** 2 + (renderedY - midY) ** 2);
      if (dist < bestDist) {
        bestDist = dist;
        bestEdgeIdx = i;
        bestMidRendered = { x: midX, y: midY };
      }
    }

    // Also consider the fixed edges for insertion at ends
    // Base->J1 edge: insert before J1 (insertAfter = -1 means insert as new J1)
    {
      const edge = this._cy.getElementById("edge-base-j1");
      if (edge.length) {
        const srcPos = edge.source().renderedPosition();
        const tgtPos = edge.target().renderedPosition();
        const midX = (srcPos.x + tgtPos.x) / 2;
        const midY = (srcPos.y + tgtPos.y) / 2;
        const dist = Math.sqrt((renderedX - midX) ** 2 + (renderedY - midY) ** 2);
        if (dist < bestDist) {
          bestDist = dist;
          bestEdgeIdx = -1; // special: insert at bottom (new J1)
          bestMidRendered = { x: midX, y: midY };
        }
      }
    }
    // JN->EE edge: insert after last joint
    {
      const edge = this._cy.getElementById(`edge-j${n}-ee`);
      if (edge.length) {
        const srcPos = edge.source().renderedPosition();
        const tgtPos = edge.target().renderedPosition();
        const midX = (srcPos.x + tgtPos.x) / 2;
        const midY = (srcPos.y + tgtPos.y) / 2;
        const dist = Math.sqrt((renderedX - midX) ** 2 + (renderedY - midY) ** 2);
        if (dist < bestDist) {
          bestDist = dist;
          bestEdgeIdx = n - 1; // insert after last joint
          bestMidRendered = { x: midX, y: midY };
        }
      }
    }

    if (bestDist < 120 && bestMidRendered) {
      // Highlight edge
      if (this._highlightedEdge !== bestEdgeIdx) {
        this._clearEdgeHighlight();
        this._highlightedEdge = bestEdgeIdx;
        // Add highlight class
        let edgeId;
        if (bestEdgeIdx === -1) {
          edgeId = "edge-base-j1";
        } else if (bestEdgeIdx === n - 1) {
          edgeId = `edge-j${n}-ee`;
        } else {
          edgeId = `edge-j${bestEdgeIdx + 1}-j${bestEdgeIdx + 2}`;
        }
        const edge = this._cy.getElementById(edgeId);
        if (edge.length) edge.addClass("highlight-insert");
      }

      // Show insert indicator
      const containerRect = this._container.getBoundingClientRect();
      const canvasOffset = {
        x: canvasRect.left - containerRect.left,
        y: canvasRect.top - containerRect.top,
      };
      this._insertIndicator.classList.remove("hidden");
      this._insertIndicator.style.left = (canvasOffset.x + bestMidRendered.x - 12) + "px";
      this._insertIndicator.style.top = (canvasOffset.y + bestMidRendered.y - 12) + "px";
    } else {
      this._clearEdgeHighlight();
      this._insertIndicator.classList.add("hidden");
      this._highlightedEdge = null;
    }
  }

  _clearEdgeHighlight() {
    this._cy.edges().removeClass("highlight-insert");
  }

  // ------------------------------------------------------------------ Insert Joint

  _insertJointAtEdge(edgeIdx) {
    const m = this._morphology;
    const n = m.num_joints;
    if (n >= MAX_JOINTS) return;

    // edgeIdx: -1 = insert before J1 (new J1), 0..n-2 = between J(edgeIdx+1) and J(edgeIdx+2), n-1 = after last joint
    // Convert to insertAfter (0-based joint index, -1 means before all joints)
    let insertAfterJointIdx; // 0-based index in the joints array
    if (edgeIdx === -1) {
      insertAfterJointIdx = -1; // insert before J1
    } else {
      insertAfterJointIdx = edgeIdx; // insert after joint at this 0-based index
    }

    // Save old positions for animation
    const oldPositions = {};
    this._cy.nodes().forEach(node => {
      oldPositions[node.id()] = { ...node.position() };
    });

    // Update morphology
    // insertAfterJointIdx is 0-based. -1 means insert as new first joint.
    const spliceIdx = insertAfterJointIdx + 1; // position in rotation_angles array

    m.num_joints++;

    // Ensure joint_types array exists
    if (!Array.isArray(m.joint_types)) m.joint_types = [];
    while (m.joint_types.length < n) m.joint_types.push("orthogonal");
    m.joint_types[0] = "inline";

    if (insertAfterJointIdx === -1) {
      // Inserting before J1: new link (new J1 -> old J1) = direct
      m.genotype_tube.splice(0, 0, 0);
      m.rotation_angles.splice(0, 0, 0);
      m.joint_types.splice(0, 0, "inline");
    } else if (insertAfterJointIdx === n - 1) {
      // Inserting after last joint: new link (old last -> new) = direct
      m.genotype_tube.splice(insertAfterJointIdx, 0, 0);
      m.rotation_angles.splice(insertAfterJointIdx + 1, 0, 0);
      m.joint_types.splice(insertAfterJointIdx + 1, 0, "inline");
    } else {
      // Inserting between joints: lower link keeps original, upper link = new direct
      m.genotype_tube.splice(insertAfterJointIdx + 1, 0, 0);
      m.rotation_angles.splice(insertAfterJointIdx + 1, 0, 0);
      m.joint_types.splice(insertAfterJointIdx + 1, 0, "inline");
    }

    // J1 is always inline
    m.joint_types[0] = "inline";

    this._ensureTubeProperties();

    // Build new positions
    const newPositions = this._getNodePositions();

    // Map old node IDs to new positions for animation
    // Old joints shift: joints after the insertion point get renumbered
    const animFrom = {};

    // Base stays
    animFrom.base = oldPositions.base || newPositions.base;

    // Map old joints to new IDs
    const newJointCount = m.num_joints;
    for (let newI = 1; newI <= newJointCount; newI++) {
      const newId = `joint-${newI}`;
      if (newI === spliceIdx + 1) {
        // This is the newly inserted joint — start from midpoint for animation
        const aboveId = newI > 1 ? `joint-${newI - 1}` : "base";
        const belowId = newI < newJointCount ? `joint-${newI + 1}` : "ee";
        const abovePos = newPositions[aboveId] || newPositions.base;
        const belowPos = newPositions[belowId] || newPositions.ee;
        animFrom[newId] = {
          x: (abovePos.x + belowPos.x) / 2,
          y: (abovePos.y + belowPos.y) / 2,
        };
      } else {
        // Map from old joint position
        let oldI;
        if (newI <= spliceIdx) {
          oldI = newI; // before insertion, same index
        } else {
          oldI = newI - 1; // after insertion, shifted
        }
        const oldId = `joint-${oldI}`;
        animFrom[newId] = oldPositions[oldId] || newPositions[newId];
      }
    }

    animFrom.ee = oldPositions.ee || newPositions.ee;

    this._buildGraph(animFrom);
    this._updatePalette();
    this._hidePanel();
    this._notify();
  }

  // ------------------------------------------------------------------ Delete Joint

  _deleteJoint(jointIndex) {
    const m = this._morphology;
    if (m.num_joints <= 1) return;

    const delIdx = jointIndex - 1; // 0-based

    // Save old positions
    const oldPositions = {};
    this._cy.nodes().forEach(node => {
      oldPositions[node.id()] = { ...node.position() };
    });

    // Update morphology arrays
    if (delIdx === 0) {
      // Deleting first joint: remove genotype_tube[0] (the link above it)
      if (m.genotype_tube.length > 0) m.genotype_tube.splice(0, 1);
    } else {
      // Remove the upper link (genotype_tube[delIdx]) — link between deleted joint and the one above
      if (delIdx < m.genotype_tube.length) {
        m.genotype_tube.splice(delIdx, 1);
      } else {
        // delIdx is last joint, remove the link below it
        m.genotype_tube.splice(delIdx - 1, 1);
      }
    }
    m.rotation_angles.splice(delIdx, 1);
    if (Array.isArray(m.joint_types)) {
      m.joint_types.splice(delIdx, 1);
      // Ensure J1 is always inline after deletion
      if (m.joint_types.length > 0) m.joint_types[0] = "inline";
    }
    m.num_joints--;

    // Shift tube properties: remove the properties for the deleted link index and shift down
    this._shiftTubePropertiesAfterDelete(delIdx);

    this._ensureTubeProperties();

    // Build animation from positions
    const newPositions = this._getNodePositions();
    const animFrom = {};
    animFrom.base = oldPositions.base || newPositions.base;

    const newN = m.num_joints;
    for (let newI = 1; newI <= newN; newI++) {
      const newId = `joint-${newI}`;
      // Map: new joint newI corresponds to old joint newI (if < delIdx+1) or newI+1 (if >= delIdx+1)
      let oldI;
      if (newI <= delIdx) {
        oldI = newI;
      } else {
        oldI = newI + 1;
      }
      const oldId = `joint-${oldI}`;
      animFrom[newId] = oldPositions[oldId] || newPositions[newId];
    }

    animFrom.ee = oldPositions.ee || newPositions.ee;

    this._buildGraph(animFrom);
    this._updatePalette();
    this._hidePanel();
    this._hideContextMenu();
    this._notify();
  }

  _shiftTubePropertiesAfterDelete(delIdx) {
    const m = this._morphology;
    // We need to shift l{i} properties down for indices above the deleted link
    // The link that was removed corresponds to tube index delIdx (for delIdx > 0) or 0 (for delIdx == 0)
    const removedTubeIdx = delIdx === 0 ? 0 : delIdx;

    // Collect all existing tube properties
    const tubeProps = [];
    for (let i = 1; i <= 20; i++) {
      const pk = `l${i}_end_point_pos`;
      if (m[pk]) {
        tubeProps.push({
          idx: i,
          pos: m[pk],
          theta: m[`l${i}_end_point_theta`],
          dual: m[`l${i}_dual_point_distance`],
        });
      }
    }

    // Remove the tube property at the removed index (1-based = removedTubeIdx + 1)
    // Then renumber remaining
    const removedTube1Based = removedTubeIdx + 1;

    // Clear all existing
    for (let i = 1; i <= 20; i++) {
      delete m[`l${i}_end_point_pos`];
      delete m[`l${i}_end_point_theta`];
      delete m[`l${i}_dual_point_distance`];
    }

    // Rewrite, skipping the removed one
    let newIdx = 1;
    for (const tp of tubeProps) {
      if (tp.idx === removedTube1Based) continue;
      m[`l${newIdx}_end_point_pos`] = tp.pos;
      m[`l${newIdx}_end_point_theta`] = tp.theta;
      m[`l${newIdx}_dual_point_distance`] = tp.dual;
      newIdx++;
    }
  }

  // ------------------------------------------------------------------ Context Menu

  _showContextMenu(x, y, jointIndex) {
    const m = this._morphology;
    const canDelete = m.num_joints > 1;
    this._ctxMenuEl.classList.remove("hidden");
    this._ctxMenuEl.innerHTML = `
      <button class="lynx-ctx-item${canDelete ? "" : " disabled"}" id="lynx-ctx-delete">
        Delete Joint ${jointIndex}
      </button>
    `;
    // Position within container bounds
    const containerRect = this._container.getBoundingClientRect();
    const menuW = 160;
    const menuH = 36;
    const posX = Math.min(x, containerRect.width - menuW - 4);
    const posY = Math.min(y, containerRect.height - menuH - 4);
    this._ctxMenuEl.style.left = posX + "px";
    this._ctxMenuEl.style.top = posY + "px";

    if (canDelete) {
      this._ctxMenuEl.querySelector("#lynx-ctx-delete").addEventListener("click", (e) => {
        e.stopPropagation();
        this._deleteJoint(jointIndex);
      });
    }
  }

  _hideContextMenu() {
    this._ctxMenuEl.classList.add("hidden");
    this._ctxMenuEl.innerHTML = "";
  }

  // ------------------------------------------------------------------ Palette

  _updatePalette() {
    const m = this._morphology;
    const chip = this._container.querySelector("#lynx-joint-chip");
    if (m.num_joints >= MAX_JOINTS) {
      chip.classList.add("disabled");
      chip.title = "Maximum 10 joints";
    } else {
      chip.classList.remove("disabled");
      chip.title = "Drag to add a joint";
    }
  }

  // ------------------------------------------------------------------ Panels

  _hidePanel() {
    this._panelEl.classList.add("hidden");
    this._panelEl.innerHTML = "";
  }

  _showJointPanel(jointIndex) {
    const m = this._morphology;
    const angle = m.rotation_angles[jointIndex - 1] || 0;
    const isInline = this._isJointInline(jointIndex);
    const canDelete = m.num_joints > 1;
    const isJ1 = jointIndex === 1;

    this._panelEl.classList.remove("hidden");
    this._panelEl.innerHTML = `
      <div class="lynx-prop-header">
        Joint ${jointIndex}
        <button class="lynx-close-btn" id="lynx-panel-close">&times;</button>
      </div>
      <div class="lynx-prop-body">
        <div class="lynx-form-group">
          <div class="lynx-toggle-row lynx-toggle-sm${isJ1 ? " lynx-toggle-disabled" : ""}">
            <button class="lynx-toggle-btn ${isInline ? "active" : ""}" id="lynx-axis-inline" ${isJ1 ? "disabled" : ""}>Inline</button>
            <button class="lynx-toggle-btn ${!isInline ? "active" : ""}" id="lynx-axis-ortho" ${isJ1 ? "disabled" : ""}>Orthogonal</button>
          </div>
        </div>
        <div class="lynx-form-group">
          <label class="lynx-label-sm">Rotation Angle (deg)</label>
          <div class="lynx-slider-row lynx-slider-compact">
            <input type="range" min="-180" max="180" step="1" value="${angle}" id="lynx-rot-slider" />
            <span class="lynx-slider-value" id="lynx-rot-value">${Math.round(angle)}&deg;</span>
          </div>
        </div>
        <button class="lynx-btn lynx-btn-delete lynx-btn-delete-sm${canDelete ? "" : " disabled"}" id="lynx-delete-joint"
          ${canDelete ? "" : 'disabled title="Cannot delete last joint"'}>
          Delete
        </button>
      </div>
    `;

    this._panelEl.querySelector("#lynx-panel-close").addEventListener("click", () => {
      this._cy.elements().unselect();
      this._hidePanel();
    });

    // Axis type toggle (Inline / Orthogonal)
    if (!isJ1) {
      const btnInline = this._panelEl.querySelector("#lynx-axis-inline");
      const btnOrtho = this._panelEl.querySelector("#lynx-axis-ortho");
      const setAxisType = (asInline) => {
        if (!Array.isArray(m.joint_types)) m.joint_types = [];
        while (m.joint_types.length < m.num_joints) m.joint_types.push("orthogonal");
        m.joint_types[jointIndex - 1] = asInline ? "inline" : "orthogonal";
        btnInline.classList.toggle("active", asInline);
        btnOrtho.classList.toggle("active", !asInline);
        // Update cytoscape node data
        const node = this._cy.getElementById(`joint-${jointIndex}`);
        if (node.length) node.data("axisType", asInline ? "inline" : "orthogonal");
        this._notify();
      };
      btnInline.addEventListener("click", () => setAxisType(true));
      btnOrtho.addEventListener("click", () => setAxisType(false));
    }

    const slider = this._panelEl.querySelector("#lynx-rot-slider");
    const valueDisp = this._panelEl.querySelector("#lynx-rot-value");
    const update = (val) => {
      val = Math.max(-180, Math.min(180, parseFloat(val) || 0));
      m.rotation_angles[jointIndex - 1] = val;
      slider.value = val;
      valueDisp.textContent = Math.round(val) + "\u00B0";
      this._notify();
    };
    slider.addEventListener("input", () => update(slider.value));

    if (canDelete) {
      this._panelEl.querySelector("#lynx-delete-joint").addEventListener("click", () => {
        this._deleteJoint(jointIndex);
      });
    }
  }

  _showLinkPanel(edgeIndex) {
    const m = this._morphology;
    const isTube = m.genotype_tube[edgeIndex] === 1;
    const tubeNum = edgeIndex + 1;
    const posKey = `l${tubeNum}_end_point_pos`;
    const thetaKey = `l${tubeNum}_end_point_theta`;
    const dualKey = `l${tubeNum}_dual_point_distance`;
    const pos = m[posKey] || [...TUBE_DEFAULT_POS];
    const theta = typeof m[thetaKey] === "number" ? m[thetaKey] : 0;
    const dualDist = typeof m[dualKey] === "number" ? m[dualKey] : TUBE_DEFAULT_DUAL_DIST;

    this._panelEl.classList.remove("hidden");
    this._panelEl.innerHTML = `
      <div class="lynx-prop-header">
        Link J${tubeNum}&rarr;J${tubeNum + 1}
        <button class="lynx-close-btn" id="lynx-panel-close">&times;</button>
      </div>
      <div class="lynx-prop-body">
        <div class="lynx-form-group">
          <div class="lynx-toggle-row lynx-toggle-sm">
            <button class="lynx-toggle-btn ${!isTube ? "active" : ""}" id="lynx-link-direct">Direct</button>
            <button class="lynx-toggle-btn ${isTube ? "active" : ""}" id="lynx-link-tube">Tube</button>
          </div>
        </div>
        <div id="lynx-tube-params" style="${isTube ? "" : "display:none"}">
          <div class="lynx-form-group">
            <label class="lynx-label-sm">End Position</label>
            <div class="lynx-vec3-labels"><span>X</span><span>Y</span><span>Z</span></div>
            <div class="lynx-vec3-row">
              <input type="number" class="lynx-input" step="0.01" value="${pos[0]}" id="lynx-pos-x" />
              <input type="number" class="lynx-input" step="0.01" min="-0.2" max="0.2" value="${pos[1]}" id="lynx-pos-y" />
              <input type="number" class="lynx-input" step="0.01" min="0.1" max="0.36" value="${pos[2]}" id="lynx-pos-z" />
            </div>
          </div>
          <div class="lynx-form-group">
            <label class="lynx-label-sm">Theta (deg)</label>
            <div class="lynx-slider-row lynx-slider-compact">
              <input type="range" min="-180" max="180" step="1" value="${theta}" id="lynx-theta-slider" />
              <span class="lynx-slider-value" id="lynx-theta-value">${Math.round(theta)}&deg;</span>
            </div>
          </div>
          <div class="lynx-form-group">
            <label class="lynx-label-sm">Dual Point Dist</label>
            <div class="lynx-slider-row lynx-slider-compact">
              <input type="range" min="0.01" max="0.2" step="0.005" value="${dualDist}" id="lynx-dual-slider" />
              <span class="lynx-slider-value" id="lynx-dual-value">${dualDist.toFixed(3)}</span>
            </div>
          </div>
        </div>
      </div>
    `;

    const self = this;
    const edge = this._selectedElement;
    const tubeParamsDiv = this._panelEl.querySelector("#lynx-tube-params");
    const btnDirect = this._panelEl.querySelector("#lynx-link-direct");
    const btnTube = this._panelEl.querySelector("#lynx-link-tube");

    this._panelEl.querySelector("#lynx-panel-close").addEventListener("click", () => {
      this._cy.elements().unselect();
      this._hidePanel();
    });

    const setLinkType = (asTube) => {
      m.genotype_tube[edgeIndex] = asTube ? 1 : 0;
      if (edge && edge.length) edge.data("linkType", asTube ? "tube" : "direct");
      btnDirect.classList.toggle("active", !asTube);
      btnTube.classList.toggle("active", asTube);
      tubeParamsDiv.style.display = asTube ? "" : "none";
      self._notify();
    };

    btnDirect.addEventListener("click", () => setLinkType(false));
    btnTube.addEventListener("click", () => setLinkType(true));

    const posX = this._panelEl.querySelector("#lynx-pos-x");
    const posY = this._panelEl.querySelector("#lynx-pos-y");
    const posZ = this._panelEl.querySelector("#lynx-pos-z");
    const thetaSlider = this._panelEl.querySelector("#lynx-theta-slider");
    const thetaDisp = this._panelEl.querySelector("#lynx-theta-value");
    const dualSlider = this._panelEl.querySelector("#lynx-dual-slider");
    const dualDisp = this._panelEl.querySelector("#lynx-dual-value");

    const updatePos = () => {
      if (!m[posKey]) m[posKey] = [...TUBE_DEFAULT_POS];
      m[posKey][0] = parseFloat(posX.value) || 0;
      m[posKey][1] = Math.max(-0.2, Math.min(0.2, parseFloat(posY.value) || 0));
      m[posKey][2] = Math.max(0.1, Math.min(0.36, parseFloat(posZ.value) || 0.2));
      posY.value = m[posKey][1];
      posZ.value = m[posKey][2];
      this._notify();
    };
    const updateTheta = (val) => {
      val = Math.max(-180, Math.min(180, parseFloat(val) || 0));
      m[thetaKey] = val;
      thetaSlider.value = val;
      thetaDisp.textContent = Math.round(val) + "\u00B0";
      this._notify();
    };
    const updateDual = (val) => {
      val = Math.max(0.01, Math.min(0.2, parseFloat(val) || 0.05));
      m[dualKey] = val;
      dualSlider.value = val;
      dualDisp.textContent = val.toFixed(3);
      this._notify();
    };

    posX.addEventListener("change", updatePos);
    posY.addEventListener("change", updatePos);
    posZ.addEventListener("change", updatePos);
    thetaSlider.addEventListener("input", () => updateTheta(thetaSlider.value));
    dualSlider.addEventListener("input", () => updateDual(dualSlider.value));
  }

  // ------------------------------------------------------------------ Notification

  _notify() {
    const cfg = this.getMorphology();
    for (const cb of this._callbacks) {
      try { cb(cfg); } catch (e) { console.error("GraphEditor callback error:", e); }
    }
  }
}

// ---- Export ----
if (typeof module !== "undefined" && module.exports) {
  module.exports = { GraphEditor };
}
if (typeof window !== "undefined") {
  window.GraphEditor = GraphEditor;
}
