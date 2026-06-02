/**
 * mjcf-generator.js
 *
 * Converts a Lynx morphology config JSON into a valid MuJoCo MJCF XML string.
 * Faithfully replicates the geometry from constructor.py, including:
 *   - JointInline / JointOrthogonal differences
 *   - Quaternion-based attachment chain
 *   - Correct cyl0/cyl1/cyl2 placement and orientation
 *   - Tube (BSpline simplified as capsule) with correct endpoints
 *
 * No external dependencies.
 */

// ---------------------------------------------------------------------------
// Motor specs
// ---------------------------------------------------------------------------
const MOTOR = {
  massive:  { armature: 10.0, damping: 0.1, frictionloss: 50.0, max_torque: 200.0, kp: 800, kd: 75, mass: 0.5 },
  standard: { armature: 2.0,  damping: 0.1, frictionloss: 15.0, max_torque: 75.0,  kp: 800, kd: 75, mass: 0.3 },
  lite:     { armature: 1.0,  damping: 0.05,frictionloss: 5.0,  max_torque: 20.0,  kp: 800, kd: 75, mass: 0.2 },
};

function motorFor(j1) { return j1 <= 2 ? MOTOR.massive : j1 <= 4 ? MOTOR.standard : MOTOR.lite; }

// ---------------------------------------------------------------------------
// Joint geometry dimensions (constructor kwargs — EXTERNAL naming)
// For Orthogonal joints, the constructor swaps cyl0↔cyl2 internally.
// ---------------------------------------------------------------------------
function extDims(j1) {
  // These are the kwargs passed to both JointInline and JointOrthogonal in constructor.py
  if (j1 <= 2) return { cyl0_l: 0.036, cyl0_r: 0.058, cyl1_l: 0.128, cyl1_r: 0.062, cyl2_l: 0.013, cyl2_r: 0.062 };
  if (j1 <= 4) return { cyl0_l: 0.029, cyl0_r: 0.04,  cyl1_l: 0.092, cyl1_r: 0.042, cyl2_l: 0.008, cyl2_r: 0.042 };
  return              { cyl0_l: 0.029, cyl0_r: 0.04,  cyl1_l: 0.096, cyl1_r: 0.042, cyl2_l: 0.008, cyl2_r: 0.042 };
}

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------
const PI = Math.PI;
function deg2rad(d) { return d * PI / 180; }
function fmt(v) { return Number(v.toFixed(7)).toString(); }
function fv(arr) { return arr.map(a => fmt(a)).join(' '); }
function indent(n) { return '  '.repeat(n); }

// Quaternion [w,x,y,z] — MuJoCo convention
function qId() { return [1, 0, 0, 0]; }

function qFromAA(axis, angle) {
  const ha = angle / 2, s = Math.sin(ha), c = Math.cos(ha);
  return [c, axis[0]*s, axis[1]*s, axis[2]*s];
}

function qMul(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}

function qConj(q) { return [q[0], -q[1], -q[2], -q[3]]; }

function qRot(q, v) {
  const vq = [0, v[0], v[1], v[2]];
  const r = qMul(qMul(q, vq), qConj(q));
  return [r[1], r[2], r[3]];
}

function v3add(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function v3sub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function v3scale(a, s) { return [a[0]*s, a[1]*s, a[2]*s]; }
function v3len(a) { return Math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]); }
function v3norm(a) { const l = v3len(a) || 1e-12; return v3scale(a, 1/l); }

// ---------------------------------------------------------------------------
// Clamped cubic B-spline evaluation (matches bspline_tube_with_clamps.py)
// ---------------------------------------------------------------------------
function bsplineBasis(knots, i, p, t) {
  if (p === 0) return (t >= knots[i] && t < knots[i+1]) ? 1.0 : 0.0;
  let c1 = 0, c2 = 0;
  const d1 = knots[i+p] - knots[i];
  const d2 = knots[i+p+1] - knots[i+1];
  if (Math.abs(d1) > 1e-12) c1 = ((t - knots[i]) / d1) * bsplineBasis(knots, i, p-1, t);
  if (Math.abs(d2) > 1e-12) c2 = ((knots[i+p+1] - t) / d2) * bsplineBasis(knots, i+1, p-1, t);
  return c1 + c2;
}

function evalBSpline(ctrlPts, knots, degree, t) {
  // Clamp t to avoid edge issues
  t = Math.max(knots[degree], Math.min(knots[knots.length - degree - 1] - 1e-10, t));
  let pt = [0, 0, 0];
  for (let i = 0; i < ctrlPts.length; i++) {
    const b = bsplineBasis(knots, i, degree, t);
    pt = v3add(pt, v3scale(ctrlPts[i], b));
  }
  return pt;
}

/**
 * Compute B-spline control points for a tube, matching bspline_tube_with_clamps.py.
 * @param {number[]} endPos  - endpoint position [x, y, z]
 * @param {number} thetaRad  - endpoint theta in radians (YZ plane rotation)
 * @param {number} dualDist  - dual_point_distance (default ~0.15 for massive, ~0.07 for standard/lite)
 * @param {number} mountStart - mounting_length_start (clamp length)
 * @param {number} mountEnd   - mounting_length_end (clamp length)
 * @returns {{ ctrlPts, knots, degree, actualEnd }}
 */
function tubeSpline(endPos, thetaRad, dualDist, mountStart, mountEnd) {
  const d = Math.max(1e-6, dualDist);
  // End tangent direction from theta in YZ plane
  const t = v3norm([0, Math.sin(thetaRad), Math.cos(thetaRad)]);

  const p0 = [0, 0, 0];
  const p1 = v3add(p0, v3scale([0, 0, 1], d));   // straight upward tangent at start

  // p3 = endPos - startOffset*Z - endOffset*t  (count_joint_volumes=False in constructor)
  const p3 = v3sub(v3sub(endPos, v3scale([0, 0, 1], mountStart)), v3scale(t, mountEnd));
  const p2 = v3sub(p3, v3scale(t, d));

  const ctrlPts = [p0, p1, p2, p3];
  const knots = [0, 0, 0, 0, 1, 1, 1, 1];
  const degree = 3;

  // Evaluate the actual endpoint (last segment direction for attachment)
  const lastT = 0.999;
  const almostEnd = evalBSpline(ctrlPts, knots, degree, lastT);
  const endPt = evalBSpline(ctrlPts, knots, degree, 1.0 - 1e-10);
  const endDir = v3norm(v3sub(endPt, almostEnd));

  // actual_end_position ~ p3 + last_direction * mountEnd
  const actualEnd = v3add(p3, v3scale(endDir, mountEnd));

  return { ctrlPts, knots, degree, actualEnd, endDir };
}

/**
 * Sample tube curve points for visualization.
 * Returns array of [x,y,z] points along the tube centerline.
 */
function sampleTubeCurve(endPos, thetaRad, dualDist, mountStart, mountEnd, nSegments) {
  const { ctrlPts, knots, degree, actualEnd, endDir } = tubeSpline(endPos, thetaRad, dualDist, mountStart, mountEnd);
  const pts = [];

  // Start clamp segment: from origin along Z
  pts.push([0, 0, 0]);
  pts.push([0, 0, mountStart]);

  // B-spline curve segments
  for (let i = 0; i <= nSegments; i++) {
    const t = i / nSegments;
    const p = evalBSpline(ctrlPts, knots, degree, t);
    // Offset by mountStart along Z (the spline starts after the start clamp)
    pts.push(v3add(p, [0, 0, mountStart]));
  }

  // End clamp segment: from spline end along endDir
  const splineEnd = v3add(evalBSpline(ctrlPts, knots, degree, 1.0 - 1e-10), [0, 0, mountStart]);
  pts.push(v3add(splineEnd, v3scale(endDir, mountEnd)));

  return pts;
}

// ---------------------------------------------------------------------------
// Tube rendering params (from constructor.py tube creation)
// ---------------------------------------------------------------------------
// Tube params depend on position in the chain (which joints it connects)
function tubeParams(tubeNumber) {
  if (tubeNumber <= 1) return { mountStart: 0.045, mountEnd: 0.045, preR: 0.062, nextR: 0.042 };
  if (tubeNumber <= 2) return { mountStart: 0.045, mountEnd: 0.045, preR: 0.042, nextR: 0.042 };
  return                      { mountStart: 0.0359, mountEnd: 0.0359, preR: 0.042, nextR: 0.042 };
}

/**
 * Write tube geoms into the current MJCF body.
 * Uses B-spline curve points to create multiple capsule segments,
 * with larger cylinder "clamps" at start and end.
 *
 * @param {Function} L - line writer L(depth, text)
 * @param {number} depth - current indentation
 * @param {number[]} ep - end_point_pos [x,y,z]
 * @param {number} etRad - end_point_theta in radians
 * @param {number} tubeNum - 1-based tube number
 * @param {number} tubeR - tube radius
 * @param {number} dualDist - dual_point_distance from config
 * @returns {{ actualEnd: number[], endQuat: number[] }}
 */
function writeTubeGeoms(L, depth, ep, etRad, tubeNum, tubeR, dualDist) {
  const tp = tubeParams(tubeNum);
  const N_SEG = 50; // match constructor.py num_segments

  const TUBE_COLOR = '0.04 0.18 0.14 1'; // dark ink-green
  const pts = sampleTubeCurve(ep, etRad, dualDist, tp.mountStart, tp.mountEnd, N_SEG);
  const massPerSeg = 0.05 / Math.max(1, pts.length - 1);

  // Create capsule segment for each consecutive pair — uniform radius
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = pts[i];
    const p1 = pts[i + 1];
    if (v3len(v3sub(p1, p0)) < 1e-6) continue;
    L(depth, `<geom type="capsule" fromto="${fv(p0)} ${fv(p1)}" size="${fmt(tubeR)}" rgba="${TUBE_COLOR}" contype="0" conaffinity="0" mass="${fmt(massPerSeg)}"/>`);
  }

  // Compute attachment at the tube's actual end
  const lastPt = pts[pts.length - 1];
  const prevPt = pts[pts.length - 2];
  const endDir = v3norm(v3sub(lastPt, prevPt));

  // Compute end quaternion: rotation that maps Z to endDir
  // Use axis-angle: axis = normalize(Z × endDir), angle = acos(Z·endDir)
  const zAxis = [0, 0, 1];
  const dot = endDir[2]; // Z · endDir
  let endQuat;
  if (dot > 0.9999) {
    endQuat = qId(); // nearly parallel to Z
  } else if (dot < -0.9999) {
    endQuat = qFromAA([1, 0, 0], PI); // anti-parallel
  } else {
    const cross = [
      zAxis[1]*endDir[2] - zAxis[2]*endDir[1],
      zAxis[2]*endDir[0] - zAxis[0]*endDir[2],
      zAxis[0]*endDir[1] - zAxis[1]*endDir[0],
    ];
    const crossLen = v3len(cross);
    const axis = v3scale(cross, 1/crossLen);
    const angle = Math.acos(Math.max(-1, Math.min(1, dot)));
    endQuat = qFromAA(axis, angle);
  }

  return { actualEnd: lastPt, endQuat };
}

// ---------------------------------------------------------------------------
// Build joint type array: true = inline, false = orthogonal
// ---------------------------------------------------------------------------
function jointTypes(numJ, gj, jtArray) {
  // If joint_types array provided, use it directly
  if (Array.isArray(jtArray) && jtArray.length >= numJ) {
    return jtArray.slice(0, numJ).map(t => t === "inline");
  }
  // Fallback: compute from genotype_joints
  // Joint 1 is always inline.
  if (Array.isArray(gj)) {
    // genotype_joints is a binary array [1,0,1,...] for j2-j6
    const r = [true]; // j1
    for (let i = 0; i < numJ - 1; i++) r.push(i < gj.length ? !!gj[i] : false);
    return r;
  }
  // Legacy: integer cutoff, first gj of j2..jN are inline, rest orthogonal.
  const r = [true]; // j1
  for (let i = 0; i < numJ - 1; i++) r.push(i < gj);
  return r;
}

// ---------------------------------------------------------------------------
// Main MJCF generator
// ---------------------------------------------------------------------------
export function generateMJCF(cfg) {
  const numJ = cfg.num_joints || 6;
  const gTube = (cfg.genotype_tube || [0,1,0,1,0]).slice(0, numJ - 1);
  const gJoints = cfg.genotype_joints !== undefined ? cfg.genotype_joints : 1;
  const rotAngles = (cfg.rotation_angles || [180,0,0,-180,0,0]).slice(0, numJ);
  const jTypes = jointTypes(numJ, gJoints, cfg.joint_types);

  // Tube params
  const tubeEndPos = [], tubeEndTheta = [], tubeDualDist = [];
  for (let i = 1; i <= 5; i++) {
    tubeEndPos.push(cfg[`l${i}_end_point_pos`] || [0, 0, 0.2]);
    tubeEndTheta.push(cfg[`l${i}_end_point_theta`] || 0.0);
    tubeDualDist.push(cfg[`l${i}_dual_point_distance`] !== undefined ? cfg[`l${i}_dual_point_distance`] : 0.05);
  }

  const TUBE_R = 0.0396;
  const lines = [];
  const actuators = [];
  let depth = 0;
  let openBodies = 0;

  function L(d, t) { lines.push(indent(d) + t); }
  function openBody(d, attrs) { L(d, `<body ${attrs}>`); openBodies++; }

  // ---- Header ----
  L(0, '<mujoco model="lynx_arm">');
  L(1, '<compiler angle="radian" inertiafromgeom="true"/>');
  L(1, '<option timestep="0.002" integrator="implicitfast" solver="Newton" iterations="50" tolerance="1e-10" gravity="0 0 -9.81"/>');
  L(0, '');
  L(1, '<default>');
  L(2, '<geom rgba="0.8 0.8 0.8 1" contype="1" conaffinity="1"/>');
  L(2, '<joint limited="true" range="-3.14159 3.14159" damping="0.1"/>');
  L(1, '</default>');
  L(0, '');
  L(1, '<asset>');
  L(2, '<texture type="skybox" builtin="gradient" rgb1="0.25 0.25 0.25" rgb2="0.5 0.5 0.5" width="512" height="512"/>');
  L(2, '<texture type="2d" name="texplane" builtin="checker" rgb1="0.79 0.72 0.45" rgb2="0.65 0.58 0.35" width="512" height="512"/>');
  L(2, '<material name="matplane" texture="texplane" texrepeat="5 5" reflectance="0.2"/>');
  L(1, '</asset>');
  L(0, '');

  // ---- Worldbody ----
  L(1, '<worldbody>');
  L(2, '<light directional="true" diffuse="0.6 0.6 0.6" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>');
  L(2, '<light directional="true" diffuse="0.4 0.4 0.4" specular="0.1 0.1 0.1" pos="2 2 3" dir="-1 -1 -1"/>');
  L(2, '<geom name="floor" type="plane" size="2 2 0.1" rgba="0.79 0.72 0.45 1" material="matplane"/>');
  L(0, '');
  L(2, '<!-- Table: width=1.0 depth=0.8 height=0.6 thickness=0.018 leg_width=0.045 -->');
  L(2, '<geom name="table_top" type="box" size="0.5 0.4 0.009" pos="0 0 0.609" rgba="0.95 0.88 0.7 1" mass="100"/>');
  L(2, '<geom name="table_leg_1" type="box" size="0.0225 0.0225 0.3" pos="0.4775 0.3775 0.3" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>');
  L(2, '<geom name="table_leg_2" type="box" size="0.0225 0.0225 0.3" pos="-0.4775 0.3775 0.3" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>');
  L(2, '<geom name="table_leg_3" type="box" size="0.0225 0.0225 0.3" pos="0.4775 -0.3775 0.3" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>');
  L(2, '<geom name="table_leg_4" type="box" size="0.0225 0.0225 0.3" pos="-0.4775 -0.3775 0.3" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>');

  L(0, '');

  // ---- Base body ----
  depth = 2;
  openBody(depth, 'name="base" pos="0 0 0.609"');
  depth++;
  // Base geoms (from base.py: base_length1=0.001, base_length2=0.018)
  L(depth, '<geom type="cylinder" size="0.08 0.0005" rgba="0.08 0.08 0.08 1" pos="0 0 0.0005" mass="0.01"/>');
  L(depth, '<geom type="cylinder" size="0.08 0.009" rgba="0.08 0.08 0.08 1" pos="0 0 0.01" mass="0.01"/>');

  // Base attachment: [0, 0, 0.019] (base_length1 + base_length2 = 0.001 + 0.018)
  // with identity quaternion
  let attachPos = [0, 0, 0.019];
  let attachQuat = qId();

  // Slot-indexed tube mapping: slot i (between joint i+1 and joint i+2) uses l{i+1} params

  // ---- Build kinematic chain ----
  for (let ji = 0; ji < numJ; ji++) {
    const jNum = ji + 1;
    const isInline = jTypes[ji];
    const motor = motorFor(jNum);
    const ext = extDims(jNum);
    const angleRad = deg2rad(rotAngles[ji]);

    // Compute the relative quaternion for this joint
    // Inline:     relQuat = Rx(π/2) * Rz(angle)    — Python: rx * twist
    // Orthogonal: relQuat = Rz(angle) * Rx(π/2)    — Python: twist * rx
    const rx = qFromAA([1, 0, 0], PI / 2);
    const rz = qFromAA([0, 0, 1], angleRad);
    const relQuat = isInline ? qMul(rx, rz) : qMul(rz, rx);

    // Colors — all black except specific small ellipse parts (silver-gray)
    const blackColor  = '0.08 0.08 0.08 1';   // pure black
    const silverColor = '0.35 0.37 0.40 1';    // dark metallic gray

    if (isInline) {
      // ============================================================
      // INLINE JOINT
      // ============================================================
      // External dims used directly (no swap)
      const c0l = ext.cyl0_l, c0r = ext.cyl0_r;
      const c1l = ext.cyl1_l, c1r = ext.cyl1_r;
      const c2l = ext.cyl2_l, c2r = ext.cyl2_r;

      // Pivot = cyl0_l + cyl1_l/2 (center of main motor)
      const pivot = c0l + c1l / 2;

      // --- Fixed body ---
      L(depth, `<!-- Joint ${jNum} (Inline) -->`);
      openBody(depth, `name="j${jNum}_fixed" pos="${fv(attachPos)}" quat="${fv(attachQuat)}"`);
      depth++;
      // cyl0: first small ellipse — SILVER (Inline signature part)
      L(depth, `<geom type="cylinder" size="${fmt(c0r)} ${fmt(c0l/2)}" rgba="${silverColor}" pos="0 0 ${fmt(c0l/2)}" contype="0" conaffinity="0" mass="0.01"/>`);

      // --- Rotating body at pivot ---
      openBody(depth, `name="j${jNum}" pos="0 0 ${fmt(pivot)}"`);
      depth++;

      // Joint at origin (pivot = body origin)
      const jName = `joint_${ji}`;
      L(depth, `<joint name="${jName}" type="hinge" axis="0 0 -1" range="-3.14159 3.14159" armature="${fmt(motor.armature)}" damping="${fmt(motor.damping)}" frictionloss="${fmt(motor.frictionloss)}"/>`);

      // cyl1: main motor body — BLACK
      L(depth, `<geom type="cylinder" size="${fmt(c1r)} ${fmt(c1l/2)}" rgba="${blackColor}" pos="0 0 0" mass="${fmt(motor.mass)}"/>`);

      // cyl2 bridge — BLACK
      const bridgePos = qRot(relQuat, [0, 0, c1r / 2]);
      L(depth, `<geom type="cylinder" size="${fmt(c2r)} ${fmt(c1r/2)}" rgba="${blackColor}" pos="${fv(bridgePos)}" quat="${fv(relQuat)}" contype="0" conaffinity="0" mass="0.001"/>`);

      // cyl2 cap — BLACK
      const capPos = qRot(relQuat, [0, 0, c1r + c2l / 2]);
      L(depth, `<geom type="cylinder" size="${fmt(c2r)} ${fmt(c2l/2)}" rgba="${blackColor}" pos="${fv(capPos)}" quat="${fv(relQuat)}" contype="0" conaffinity="0" mass="0.001"/>`);

      // Attachment for next module (relative to pivot = body origin)
      const nextOff = qRot(relQuat, [0, 0, c1r + c2l]);
      const nextPos = nextOff;   // relative to this rotating body
      const nextQuat = relQuat;

      // Record actuator
      actuators.push({ name: `motor_${ji}`, joint: jName, kp: motor.kp, kd: motor.kd, ft: motor.max_torque });

      // Handle tube or direct connection to next joint
      if (ji < numJ - 1) {
        const hasTube = ji < gTube.length && gTube[ji] === 1;
        if (hasTube) {
          const ep = tubeEndPos[ji] || [0, 0, 0.2];
          const et = deg2rad(tubeEndTheta[ji] || 0);
          const dd = tubeDualDist[ji] || 0.05;
          const tubeSlotNum = ji + 1;

          // Tube body at the attachment point
          openBody(depth, `name="tube_${tubeSlotNum}" pos="${fv(nextPos)}" quat="${fv(nextQuat)}"`);
          depth++;

          // B-spline tube segments (multiple capsules approximating the curve)
          const tubeResult = writeTubeGeoms(L, depth, ep, et, tubeSlotNum, TUBE_R, dd);

          // Update attachment for next joint
          attachPos = tubeResult.actualEnd;
          attachQuat = tubeResult.endQuat;
        } else {
          // Direct connection: next joint attaches at nextPos with nextQuat
          attachPos = nextPos;
          attachQuat = nextQuat;
        }
      } else {
        // Last joint: end effector
        const eeOff = qRot(relQuat, [0, 0, c1r + c2l]);
        openBody(depth, `name="end_effector" pos="${fv(eeOff)}" quat="${fv(relQuat)}"`);
        depth++;
        L(depth, `<geom type="cylinder" size="0.01 0.035" rgba="0.08 0.08 0.08 1" pos="0 0 0.035" mass="0.005"/>`);
        L(depth, `<site name="ee_site" pos="0 0 0.07" size="0.005" rgba="1 0 0 1"/>`);
      }

    } else {
      // ============================================================
      // ORTHOGONAL JOINT
      // ============================================================
      // Constructor.py swaps cyl0↔cyl2 internally for Orthogonal:
      //   internal cyl0 = external cyl2 (small cap)
      //   internal cyl1 = external cyl1 (same)
      //   internal cyl2 = external cyl0 (big housing)
      const ic0l = ext.cyl2_l, ic0r = ext.cyl2_r;  // internal cyl0 = ext cyl2
      const ic1l = ext.cyl1_l, ic1r = ext.cyl1_r;  // internal cyl1 = ext cyl1
      const ic2l = ext.cyl0_l, ic2r = ext.cyl0_r;  // internal cyl2 = ext cyl0

      // Pivot = internal_cyl0_l + cyl_radius1 (center of shell)
      const pivot = ic0l + ic1r;

      // --- Fixed body ---
      L(depth, `<!-- Joint ${jNum} (Orthogonal) -->`);
      openBody(depth, `name="j${jNum}_fixed" pos="${fv(attachPos)}" quat="${fv(attachQuat)}"`);
      depth++;

      // cyl0 (internal): small cap at bottom — BLACK
      L(depth, `<geom type="cylinder" size="${fmt(ic0r)} ${fmt(ic0l/2)}" rgba="${blackColor}" pos="0 0 ${fmt(ic0l/2)}" contype="0" conaffinity="0" mass="0.001"/>`);

      // Structural bridge — BLACK
      L(depth, `<geom type="cylinder" size="${fmt(ic0r)} ${fmt(ic1r/2)}" rgba="${blackColor}" pos="0 0 ${fmt(ic0l + ic1r/2)}" contype="0" conaffinity="0" mass="0.001"/>`);

      // Main motor shell — BLACK
      L(depth, `<geom type="cylinder" size="${fmt(ic1r)} ${fmt(ic1l/2)}" rgba="${blackColor}" pos="0 0 ${fmt(pivot)}" quat="${fv(relQuat)}" mass="${fmt(motor.mass)}"/>`);

      // --- Rotating body at pivot ---
      openBody(depth, `name="j${jNum}" pos="0 0 ${fmt(pivot)}"`);
      depth++;

      // Joint at origin; axis = relQuat * [0,0,-1]
      const jName = `joint_${ji}`;
      const axis = qRot(relQuat, [0, 0, -1]);
      L(depth, `<joint name="${jName}" type="hinge" axis="${fv(axis)}" range="-3.14159 3.14159" armature="${fmt(motor.armature)}" damping="${fmt(motor.damping)}" frictionloss="${fmt(motor.frictionloss)}"/>`);

      // cyl2 (internal): last small ellipse, rotating arm — SILVER (Orthogonal signature part)
      const cyl2Pos = qRot(relQuat, [0, 0, ic1l / 2 + ic2l / 2]);
      L(depth, `<geom type="cylinder" size="${fmt(ic2r)} ${fmt(ic2l/2)}" rgba="${silverColor}" pos="${fv(cyl2Pos)}" quat="${fv(relQuat)}" contype="0" conaffinity="0" mass="0.001"/>`);

      // Attachment for next module (relative to pivot)
      const nextOff = qRot(relQuat, [0, 0, ic1l / 2 + ic2l]);
      const nextPos = nextOff;
      const nextQuat = relQuat;

      // Record actuator
      actuators.push({ name: `motor_${ji}`, joint: jName, kp: motor.kp, kd: motor.kd, ft: motor.max_torque });

      // Handle tube or direct connection
      if (ji < numJ - 1) {
        const hasTube = ji < gTube.length && gTube[ji] === 1;
        if (hasTube) {
          const ep = tubeEndPos[ji] || [0, 0, 0.2];
          const et = deg2rad(tubeEndTheta[ji] || 0);
          const dd = tubeDualDist[ji] || 0.05;
          const tubeSlotNum = ji + 1;

          openBody(depth, `name="tube_${tubeSlotNum}" pos="${fv(nextPos)}" quat="${fv(nextQuat)}"`);
          depth++;

          // B-spline tube segments
          const tubeResult = writeTubeGeoms(L, depth, ep, et, tubeSlotNum, TUBE_R, dd);

          attachPos = tubeResult.actualEnd;
          attachQuat = tubeResult.endQuat;
        } else {
          attachPos = nextPos;
          attachQuat = nextQuat;
        }
      } else {
        // Last joint: end effector
        openBody(depth, `name="end_effector" pos="${fv(nextOff)}" quat="${fv(nextQuat)}"`);
        depth++;
        L(depth, `<geom type="cylinder" size="0.01 0.035" rgba="0.08 0.08 0.08 1" pos="0 0 0.035" mass="0.005"/>`);
        L(depth, `<site name="ee_site" pos="0 0 0.07" size="0.005" rgba="1 0 0 1"/>`);
      }
    }
  }

  // Close all open bodies (serial chain nesting)
  while (openBodies > 0) {
    depth--;
    L(depth, '</body>');
    openBodies--;
  }

  L(0, '');
  L(1, '</worldbody>');
  L(0, '');

  // ---- Actuators ----
  L(1, '<actuator>');
  for (const a of actuators) {
    L(2, `<general name="${a.name}" joint="${a.joint}" gainprm="${a.kp} 0 0" biasprm="0 -${a.kp} -${a.kd}" ctrlrange="-3.14159 3.14159" forcerange="-${a.ft} ${a.ft}"/>`);
  }
  L(1, '</actuator>');
  L(0, '');

  // ---- Sensors ----
  L(1, '<sensor>');
  for (let ji = 0; ji < numJ; ji++) {
    L(2, `<jointpos name="jpos_${ji}" joint="joint_${ji}"/>`);
    L(2, `<jointvel name="jvel_${ji}" joint="joint_${ji}"/>`);
  }
  L(1, '</sensor>');

  L(0, '</mujoco>');
  return lines.join('\n');
}

// ---------------------------------------------------------------------------
// Default PD parameters
// ---------------------------------------------------------------------------
export function getDefaultPDParams(cfg) {
  const numJ = cfg.num_joints || 6;
  const kp = [], kd = [], effortLimit = [];
  for (let j = 1; j <= numJ; j++) {
    const m = motorFor(j);
    kp.push(m.kp);
    kd.push(m.kd);
    effortLimit.push(m.max_torque);
  }
  return { kp, kd, effort_limit: effortLimit, max_velocity: 2.0, max_acceleration: 10.0 };
}

// ---------------------------------------------------------------------------
// Rest-pose self-collision detection
//
// Works on a model already parsed by viewer.js's MJCFParser
// ({ bodies:[{name,parent,pos,quat,euler}], geoms:[{type,body,fromto,size}] }).
// We forward-kinematic the body tree at the rest pose (all joints = 0, so each
// body contributes only its fixed pos/quat), world-transform every tube capsule
// segment, and test capsule overlap between tube links that are >= MIN_LINK_SEP
// indices apart (adjacent tubes naturally touch at their shared joint, so they
// are skipped). Quaternions are MuJoCo wxyz. This covers link-vs-link
// self-collision (the dominant failure for a serial arm); joint housings, base,
// and table are intentionally NOT checked in this first version.
// ---------------------------------------------------------------------------
function _qmul(a, b) {
  const aw = a[0], ax = a[1], ay = a[2], az = a[3];
  const bw = b[0], bx = b[1], by = b[2], bz = b[3];
  return [
    aw*bw - ax*bx - ay*by - az*bz,
    aw*bx + ax*bw + ay*bz - az*by,
    aw*by - ax*bz + ay*bw + az*bx,
    aw*bz + ax*by - ay*bx + az*bw,
  ];
}
function _qrot(q, v) {
  const w = q[0], x = q[1], y = q[2], z = q[3];
  const tx = 2*(y*v[2] - z*v[1]);
  const ty = 2*(z*v[0] - x*v[2]);
  const tz = 2*(x*v[1] - y*v[0]);
  return [
    v[0] + w*tx + (y*tz - z*ty),
    v[1] + w*ty + (z*tx - x*tz),
    v[2] + w*tz + (x*ty - y*tx),
  ];
}
function _eulerToQuat(e) { // xyz intrinsic, radians (fallback; generator uses quat)
  let q = [Math.cos(e[0]/2), Math.sin(e[0]/2), 0, 0];
  q = _qmul(q, [Math.cos(e[1]/2), 0, Math.sin(e[1]/2), 0]);
  q = _qmul(q, [Math.cos(e[2]/2), 0, 0, Math.sin(e[2]/2)]);
  return q;
}
// Minimum distance between segments p1-q1 and p2-q2 (Ericson, Real-Time CD).
function _segSegDist(p1, q1, p2, q2) {
  const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
  const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  const EPS = 1e-12;
  let s, t;
  if (a <= EPS && e <= EPS) return Math.sqrt(dot(r, r));
  if (a <= EPS) { s = 0; t = Math.max(0, Math.min(1, f/e)); }
  else {
    const c = dot(d1, r);
    if (e <= EPS) { t = 0; s = Math.max(0, Math.min(1, -c/a)); }
    else {
      const b = dot(d1, d2), denom = a*e - b*b;
      s = denom > EPS ? Math.max(0, Math.min(1, (b*f - c*e)/denom)) : 0;
      t = (b*s + f)/e;
      if (t < 0) { t = 0; s = Math.max(0, Math.min(1, -c/a)); }
      else if (t > 1) { t = 1; s = Math.max(0, Math.min(1, (b - c)/a)); }
    }
  }
  const c1 = [p1[0]+d1[0]*s, p1[1]+d1[1]*s, p1[2]+d1[2]*s];
  const c2 = [p2[0]+d2[0]*t, p2[1]+d2[1]*t, p2[2]+d2[2]*t];
  const dd = sub(c1, c2);
  return Math.sqrt(dot(dd, dd));
}

function _quatFromAxisAngle(axis, angle) {
  let x = axis[0], y = axis[1], z = axis[2];
  const n = Math.hypot(x, y, z) || 1;
  x /= n; y /= n; z /= n;
  const s = Math.sin(angle / 2);
  return [Math.cos(angle / 2), x*s, y*s, z*s];
}

// Forward kinematics: returns world(name) -> {p:[x,y,z], q:[w,x,y,z]}.
// jointAngles (optional, aligned with model.joints) applies each hinge's rotation
// — joints sit at their body origin in this model, so the same routine yields the
// rest pose (no angles) or any commanded pose (for in-motion collision checks).
function _buildWorldFn(model, jointAngles) {
  const bodies = model && model.bodies ? model.bodies : [];
  const joints = model && model.joints ? model.joints : [];
  const byName = {};
  for (const b of bodies) byName[b.name] = b;
  const jointByBody = {};
  if (jointAngles) {
    for (let i = 0; i < joints.length; i++) {
      const a = jointAngles[i];
      if (a) jointByBody[joints[i].body] = { axis: joints[i].axis || [0,0,1], angle: a };
    }
  }
  const cache = {};
  function world(name) {
    if (name == null || name === '__worldbody__') return { p: [0,0,0], q: [1,0,0,0] };
    if (cache[name]) return cache[name];
    const b = byName[name];
    if (!b) return { p: [0,0,0], q: [1,0,0,0] };
    const par = world(b.parent);
    const lq = b.quat ? b.quat : (b.euler ? _eulerToQuat(b.euler) : [1,0,0,0]);
    const lp = b.pos || [0,0,0];
    const rp = _qrot(par.q, lp);
    let q = _qmul(par.q, lq);
    const jb = jointByBody[name];
    if (jb) q = _qmul(q, _quatFromAxisAngle(jb.axis, jb.angle));
    cache[name] = { p: [par.p[0]+rp[0], par.p[1]+rp[1], par.p[2]+rp[2]], q };
    return cache[name];
  }
  return world;
}

/**
 * Count colliding tube-link pairs (every two tubes, including consecutive ones).
 * Consecutive tubes share exactly one joint and legitimately meet there, so we
 * ignore overlaps within `junctionR` of that shared joint; everything else —
 * including a tube bending back into its neighbour — is flagged.
 * @param {object} model  parsed MJCF ({ bodies, geoms, joints })
 * @param {object} [opts] { margin=1.0, maxSegPerTube=16, junctionR=0.08, jointAngles }
 * @returns {number} number of distinct tube pairs whose capsules overlap
 */
export function countSelfCollisions(model, opts = {}) {
  const margin = opts.margin != null ? opts.margin : 1.0;      // collide if dist < (r1+r2)*margin
  const maxSeg = opts.maxSegPerTube != null ? opts.maxSegPerTube : 16;
  const junctionR = opts.junctionR != null ? opts.junctionR : 0.08; // shared-joint exclusion radius

  const geoms = model && model.geoms ? model.geoms : [];
  const world = _buildWorldFn(model, opts.jointAngles);

  // World-space capsule segments grouped by tube index.
  const tubes = {};
  for (const g of geoms) {
    if (g.type !== 'capsule' || !g.fromto) continue;
    const m = /^tube_(\d+)$/.exec(g.body || '');
    if (!m) continue;
    const idx = parseInt(m[1], 10);
    const wt = world(g.body);
    const ra = _qrot(wt.q, [g.fromto[0], g.fromto[1], g.fromto[2]]);
    const rb = _qrot(wt.q, [g.fromto[3], g.fromto[4], g.fromto[5]]);
    const seg = {
      a: [wt.p[0]+ra[0], wt.p[1]+ra[1], wt.p[2]+ra[2]],
      b: [wt.p[0]+rb[0], wt.p[1]+rb[1], wt.p[2]+rb[2]],
      r: (g.size && g.size[0]) || 0.0396,
    };
    (tubes[idx] = tubes[idx] || []).push(seg);
  }

  // Stride-subsample long tubes for speed (smooth curves → coarse is enough).
  const idxs = Object.keys(tubes).map(Number).sort((x, y) => x - y);
  for (const i of idxs) {
    const segs = tubes[i];
    if (segs.length > maxSeg) {
      const stride = Math.ceil(segs.length / maxSeg);
      tubes[i] = segs.filter((_, k) => k % stride === 0 || k === segs.length - 1);
    }
  }

  let collisions = 0;
  for (let i = 0; i < idxs.length; i++) {
    for (let j = i + 1; j < idxs.length; j++) {
      // Consecutive tube slots share one joint (body "j{slot}") and meet there
      // legitimately — exclude overlaps near it. Non-consecutive tubes share
      // nothing and are checked in full.
      let excl = null;
      if (idxs[j] - idxs[i] === 1) excl = world('j' + idxs[j]).p;
      const A = tubes[idxs[i]], B = tubes[idxs[j]];
      let hit = false;
      for (let x = 0; x < A.length && !hit; x++) {
        for (let y = 0; y < B.length; y++) {
          if (excl) {
            const amx = (A[x].a[0]+A[x].b[0])/2, amy = (A[x].a[1]+A[x].b[1])/2, amz = (A[x].a[2]+A[x].b[2])/2;
            const bmx = (B[y].a[0]+B[y].b[0])/2, bmy = (B[y].a[1]+B[y].b[1])/2, bmz = (B[y].a[2]+B[y].b[2])/2;
            if (Math.hypot(amx-excl[0], amy-excl[1], amz-excl[2]) < junctionR ||
                Math.hypot(bmx-excl[0], bmy-excl[1], bmz-excl[2]) < junctionR) continue;
          }
          if (_segSegDist(A[x].a, A[x].b, B[y].a, B[y].b) < (A[x].r + B[y].r) * margin) { hit = true; break; }
        }
      }
      if (hit) collisions++;
    }
  }
  return collisions;
}

// Table top surface z: table_top is a box, half-z 0.009 atop center z=0.609.
export const TABLE_TOP_Z = 0.618;

/**
 * Lowest surface z the arm's tube links + end-effector reach at the rest pose.
 * Cheap "just the z height" table-collision proxy: the arm intersects the table
 * top iff lowestArmZ(model) < TABLE_TOP_Z. The table x/y footprint and the
 * short joint housings near the mount are intentionally ignored (conservative).
 */
export function lowestArmZ(model, opts = {}) {
  const tubeR = opts.tubeR != null ? opts.tubeR : 0.0396;
  const includeJoints = opts.includeJoints !== false;
  const jointClear = opts.jointClearance != null ? opts.jointClearance : 0.005;
  const world = _buildWorldFn(model, opts.jointAngles);
  const bodies = model && model.bodies ? model.bodies : [];
  const geoms = model && model.geoms ? model.geoms : [];
  let minZ = Infinity;
  // Tube link capsules: lowest endpoint minus the tube radius.
  for (const g of geoms) {
    if (g.type !== 'capsule' || !g.fromto || !/^tube_\d+$/.test(g.body || '')) continue;
    const wt = world(g.body);
    const az = wt.p[2] + _qrot(wt.q, [g.fromto[0], g.fromto[1], g.fromto[2]])[2];
    const bz = wt.p[2] + _qrot(wt.q, [g.fromto[3], g.fromto[4], g.fromto[5]])[2];
    const r = (g.size && g.size[0]) || tubeR;
    minZ = Math.min(minZ, az - r, bz - r);
  }
  // Joint housings — including the distal ones that swing below the table when
  // joints are numerous and the links between them are "direct" (no tube to
  // sample). Body-origin centerline minus a small clearance. The first joint
  // sits ~1 cm above the table and never moves, so jointClear (0.005) keeps it
  // from false-positiving while still catching joints that dip under.
  if (includeJoints) {
    for (const b of bodies) {
      if (/^j\d+(_fixed)?$/.test(b.name)) minZ = Math.min(minZ, world(b.name).p[2] - jointClear);
    }
  }
  // End-effector tip (local +z 0.07) and origin, minus its small geom radius.
  const ee = bodies.find(b => b.name === 'end_effector');
  if (ee) {
    const w = world('end_effector');
    const tipZ = w.p[2] + _qrot(w.q, [0, 0, 0.07])[2];
    minZ = Math.min(minZ, w.p[2] - 0.01, tipZ - 0.01);
  }
  return minZ;
}
