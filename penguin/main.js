const CONFIG = {
  imageSrc: "ass/penguin.jpg", // update this path if your image lives elsewhere

  boxSize: 90,           // px — long side of the collision box
  gravity: 1800,         // px/s^2

  restitution: 0.55,     // bounciness against walls/floor (0-1)
  boxRestitution: 0.5,   // bounciness between penguins (0-1)

  friction: 0.35,        // tangential friction between penguins
  wallFriction: 0.45,    // tangential friction against walls/floor

  linearDamping: 0.999,  // slight air resistance
  angularDamping: 0.985, // spin bleeds off over time instead of forever

  maxAngularVel: 25,     // rad/s clamp, keeps things from spinning like a fan
  solverIterations: 4,   // more iterations = more stable stacking
  maxPenguins: 150,
};

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
}
window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// Load penguin image once, share across all instances.
const penguinImg = new Image();
let imgLoaded = false;
let imgAspect = 1; // width / height, computed once the image loads
penguinImg.onload = () => {
  imgLoaded = true;
  imgAspect = penguinImg.naturalWidth / penguinImg.naturalHeight || 1;
};
penguinImg.onerror = () => console.warn("Could not load", CONFIG.imageSrc);
penguinImg.src = CONFIG.imageSrc;

// ---------------------------------------------------------------------------
// Penguin rigid body (oriented rectangle)
// ---------------------------------------------------------------------------
class Penguin {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.vx = (Math.random() - 0.5) * 100;
    this.vy = 0;
    this.angle = 0;
    this.angularVel = 0;
    this.dragging = false;

    // size the box to match the image's real proportions
    if (imgAspect >= 1) {
      this.w = CONFIG.boxSize;
      this.h = CONFIG.boxSize / imgAspect;
    } else {
      this.h = CONFIG.boxSize;
      this.w = CONFIG.boxSize * imgAspect;
    }

    const mass = Math.max((this.w * this.h) / 1000, 0.1);
    this.mass = mass;
    this.inertia = (mass * (this.w * this.w + this.h * this.h)) / 12;
    this.invMass = 1 / mass;
    this.invInertia = 1 / this.inertia;
  }

  // lock/unlock physics response while being dragged (acts like infinite mass)
  setDragging(isDragging) {
    this.dragging = isDragging;
    this.invMass = isDragging ? 0 : 1 / this.mass;
    this.invInertia = isDragging ? 0 : 1 / this.inertia;
  }

  getVertices() {
    const hw = this.w / 2;
    const hh = this.h / 2;
    const c = Math.cos(this.angle);
    const s = Math.sin(this.angle);
    const local = [
      { x: -hw, y: -hh },
      { x: hw, y: -hh },
      { x: hw, y: hh },
      { x: -hw, y: hh },
    ];
    return local.map((p) => ({
      x: this.x + p.x * c - p.y * s,
      y: this.y + p.x * s + p.y * c,
    }));
  }

  getAxes() {
    const c = Math.cos(this.angle);
    const s = Math.sin(this.angle);
    return [
      { x: c, y: s }, // local x axis in world space
      { x: -s, y: c }, // local y axis in world space
    ];
  }

  integrate(dt) {
    if (this.dragging) return; // position driven by mouse while dragging

    this.vy += CONFIG.gravity * dt;
    this.vx *= CONFIG.linearDamping;
    this.vy *= CONFIG.linearDamping;
    this.angularVel *= CONFIG.angularDamping;
    this.angularVel = Math.max(
      -CONFIG.maxAngularVel,
      Math.min(CONFIG.maxAngularVel, this.angularVel)
    );

    this.x += this.vx * dt;
    this.y += this.vy * dt;
    this.angle += this.angularVel * dt;
  }

  draw(ctx) {
    ctx.save();
    ctx.translate(this.x, this.y);
    ctx.rotate(this.angle);

    if (imgLoaded) {
      ctx.drawImage(penguinImg, -this.w / 2, -this.h / 2, this.w, this.h);
    } else {
      ctx.fillStyle = "#222";
      ctx.fillRect(-this.w / 2, -this.h / 2, this.w, this.h);
    }

    ctx.restore();
  }

  containsPoint(px, py) {
    const dx = px - this.x;
    const dy = py - this.y;
    const c = Math.cos(this.angle);
    const s = Math.sin(this.angle);
    // rotate the point into the box's local space
    const lx = dx * c + dy * s;
    const ly = -dx * s + dy * c;
    return Math.abs(lx) <= this.w / 2 && Math.abs(ly) <= this.h / 2;
  }
}

// ---------------------------------------------------------------------------
// World state
// ---------------------------------------------------------------------------
const penguins = [];

function spawnPenguin(x, y) {
  if (penguins.length >= CONFIG.maxPenguins) penguins.shift();
  penguins.push(new Penguin(x, y));
}

// ---------------------------------------------------------------------------
// SAT collision test between two oriented boxes.
// Returns { normal, depth, contact } (normal points from a -> b) or null.
// ---------------------------------------------------------------------------
function project(vertices, axis) {
  let min = Infinity;
  let max = -Infinity;
  for (const v of vertices) {
    const p = v.x * axis.x + v.y * axis.y;
    if (p < min) min = p;
    if (p > max) max = p;
  }
  return { min, max };
}

function testSAT(a, b) {
  const axes = [...a.getAxes(), ...b.getAxes()];
  const vertsA = a.getVertices();
  const vertsB = b.getVertices();

  let minOverlap = Infinity;
  let smallestAxis = null;

  for (const axis of axes) {
    const pa = project(vertsA, axis);
    const pb = project(vertsB, axis);
    const overlap = Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min);
    if (overlap <= 0) return null; // found a separating axis — no collision
    if (overlap < minOverlap) {
      minOverlap = overlap;
      smallestAxis = axis;
    }
  }

  // orient the normal so it points from a toward b
  let normal = smallestAxis;
  const d = { x: b.x - a.x, y: b.y - a.y };
  if (d.x * normal.x + d.y * normal.y < 0) {
    normal = { x: -normal.x, y: -normal.y };
  }

  // approximate contact point: midpoint between the vertex of A pushed
  // furthest toward B, and the vertex of B pushed furthest toward A.
  // (a lightweight stand-in for full polygon clipping — good enough
  // for snappy, stable-looking contact response here)
  let bestA = vertsA[0];
  let bestAd = -Infinity;
  for (const v of vertsA) {
    const dp = v.x * normal.x + v.y * normal.y;
    if (dp > bestAd) {
      bestAd = dp;
      bestA = v;
    }
  }
  let bestB = vertsB[0];
  let bestBd = Infinity;
  for (const v of vertsB) {
    const dp = v.x * normal.x + v.y * normal.y;
    if (dp < bestBd) {
      bestBd = dp;
      bestB = v;
    }
  }
  const contact = { x: (bestA.x + bestB.x) / 2, y: (bestA.y + bestB.y) / 2 };

  return { normal, depth: minOverlap, contact };
}

// Positional correction so overlapping boxes don't sink into each other.
function positionalCorrection(a, b, manifold) {
  const percent = 0.8;
  const slop = 0.01;
  const invMassSum = a.invMass + b.invMass;
  if (invMassSum === 0) return;
  const correction =
    (Math.max(manifold.depth - slop, 0) / invMassSum) * percent;
  a.x -= manifold.normal.x * correction * a.invMass;
  a.y -= manifold.normal.y * correction * a.invMass;
  b.x += manifold.normal.x * correction * b.invMass;
  b.y += manifold.normal.y * correction * b.invMass;
}

// Rigid-body impulse resolution (normal + friction) at the contact point.
function resolveImpulse(a, b, manifold) {
  const invMassSum0 = a.invMass + b.invMass;
  if (invMassSum0 === 0) return;

  const rA = { x: manifold.contact.x - a.x, y: manifold.contact.y - a.y };
  const rB = { x: manifold.contact.x - b.x, y: manifold.contact.y - b.y };

  const velA = { x: a.vx - a.angularVel * rA.y, y: a.vy + a.angularVel * rA.x };
  const velB = { x: b.vx - b.angularVel * rB.y, y: b.vy + b.angularVel * rB.x };
  const rv = { x: velB.x - velA.x, y: velB.y - velA.y };

  const n = manifold.normal;
  const velAlongNormal = rv.x * n.x + rv.y * n.y;
  if (velAlongNormal > 0) return; // already separating

  const raCrossN = rA.x * n.y - rA.y * n.x;
  const rbCrossN = rB.x * n.y - rB.y * n.x;
  const invMassSum =
    a.invMass +
    b.invMass +
    raCrossN * raCrossN * a.invInertia +
    rbCrossN * rbCrossN * b.invInertia;

  const e = CONFIG.boxRestitution;
  const j = (-(1 + e) * velAlongNormal) / invMassSum;

  const impulse = { x: j * n.x, y: j * n.y };
  a.vx -= impulse.x * a.invMass;
  a.vy -= impulse.y * a.invMass;
  a.angularVel -= raCrossN * j * a.invInertia;
  b.vx += impulse.x * b.invMass;
  b.vy += impulse.y * b.invMass;
  b.angularVel += rbCrossN * j * b.invInertia;

  // friction impulse, tangent to the collision normal
  const rv2 = {
    x: b.vx - b.angularVel * rB.y - (a.vx - a.angularVel * rA.y),
    y: b.vy + b.angularVel * rB.x - (a.vy + a.angularVel * rA.x),
  };
  let tangent = {
    x: rv2.x - (rv2.x * n.x + rv2.y * n.y) * n.x,
    y: rv2.y - (rv2.x * n.x + rv2.y * n.y) * n.y,
  };
  const tLen = Math.hypot(tangent.x, tangent.y);
  if (tLen > 0.0001) {
    tangent.x /= tLen;
    tangent.y /= tLen;

    const raCrossT = rA.x * tangent.y - rA.y * tangent.x;
    const rbCrossT = rB.x * tangent.y - rB.y * tangent.x;
    const invMassSumT =
      a.invMass +
      b.invMass +
      raCrossT * raCrossT * a.invInertia +
      rbCrossT * rbCrossT * b.invInertia;

    let jt = -(rv2.x * tangent.x + rv2.y * tangent.y) / invMassSumT;
    const maxFriction = CONFIG.friction * Math.abs(j);
    jt = Math.max(-maxFriction, Math.min(maxFriction, jt));

    const frictionImpulse = { x: jt * tangent.x, y: jt * tangent.y };
    a.vx -= frictionImpulse.x * a.invMass;
    a.vy -= frictionImpulse.y * a.invMass;
    a.angularVel -= raCrossT * jt * a.invInertia;
    b.vx += frictionImpulse.x * b.invMass;
    b.vy += frictionImpulse.y * b.invMass;
    b.angularVel += rbCrossT * jt * b.invInertia;
  }
}

function resolveCollisions() {
  for (let i = 0; i < penguins.length; i++) {
    for (let j = i + 1; j < penguins.length; j++) {
      const a = penguins[i];
      const b = penguins[j];
      const manifold = testSAT(a, b);
      if (manifold) {
        resolveImpulse(a, b, manifold);
        positionalCorrection(a, b, manifold);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Wall / floor collisions — treated as a static (infinite-mass) body, so we
// reuse the same normal-impulse + friction math against a fixed plane.
// ---------------------------------------------------------------------------
function resolveStaticContact(body, contact, normal, depth) {
  if (body.dragging) return;

  // positional correction — push fully out, body has all the invMass
  body.x += normal.x * depth;
  body.y += normal.y * depth;

  const r = { x: contact.x - body.x, y: contact.y - body.y };
  const vel = {
    x: body.vx - body.angularVel * r.y,
    y: body.vy + body.angularVel * r.x,
  };
  const velAlongNormal = vel.x * normal.x + vel.y * normal.y;

  if (velAlongNormal < 0) {
    const rCrossN = r.x * normal.y - r.y * normal.x;
    const invMassSum = body.invMass + rCrossN * rCrossN * body.invInertia;
    const e = CONFIG.restitution;
    const j = (-(1 + e) * velAlongNormal) / invMassSum;
    body.vx += j * normal.x * body.invMass;
    body.vy += j * normal.y * body.invMass;
    body.angularVel += rCrossN * j * body.invInertia;
  }

  // friction against the surface
  const tangent = { x: -normal.y, y: normal.x };
  const relVelTangent = vel.x * tangent.x + vel.y * tangent.y;
  const rCrossT = r.x * tangent.y - r.y * tangent.x;
  const invMassSumT = body.invMass + rCrossT * rCrossT * body.invInertia;
  const jt = (-relVelTangent * CONFIG.wallFriction) / invMassSumT;
  body.vx += jt * tangent.x * body.invMass;
  body.vy += jt * tangent.y * body.invMass;
  body.angularVel += rCrossT * jt * body.invInertia;
}

function resolveWalls(body) {
  if (body.dragging) return;
  const verts = body.getVertices();

  let maxY = -Infinity, floorVert = null;
  let minY = Infinity, ceilVert = null;
  let minX = Infinity, leftVert = null;
  let maxX = -Infinity, rightVert = null;

  for (const v of verts) {
    if (v.y > maxY) { maxY = v.y; floorVert = v; }
    if (v.y < minY) { minY = v.y; ceilVert = v; }
    if (v.x < minX) { minX = v.x; leftVert = v; }
    if (v.x > maxX) { maxX = v.x; rightVert = v; }
  }

  if (maxY > canvas.height) {
    resolveStaticContact(body, floorVert, { x: 0, y: -1 }, maxY - canvas.height);
  }
  if (minY < 0) {
    resolveStaticContact(body, ceilVert, { x: 0, y: 1 }, -minY);
  }
  if (minX < 0) {
    resolveStaticContact(body, leftVert, { x: 1, y: 0 }, -minX);
  }
  if (maxX > canvas.width) {
    resolveStaticContact(body, rightVert, { x: -1, y: 0 }, maxX - canvas.width);
  }
}

// ---------------------------------------------------------------------------
// Mouse / touch interaction — click empty space to spawn, drag to move & throw
// ---------------------------------------------------------------------------
let dragTarget = null;
let dragOffsetX = 0;
let dragOffsetY = 0;
let lastMouseX = 0;
let lastMouseY = 0;
let lastPointerTime = performance.now();
let pointerDownPos = null;
const CLICK_MOVE_THRESHOLD = 6; // px — below this, treat mouseup as a "click"
const MAX_DRAG_SPEED = 4500; // px/s clamp, keeps a jumpy mouse event from launching things to orbit

function getPointerPos(e) {
  const rect = canvas.getBoundingClientRect();
  const src = e.touches ? e.touches[0] : e;
  return { x: src.clientX - rect.left, y: src.clientY - rect.top };
}

function onPointerDown(e) {
  const { x, y } = getPointerPos(e);
  pointerDownPos = { x, y };

  // find topmost penguin under pointer (iterate reverse = most recent on top)
  for (let i = penguins.length - 1; i >= 0; i--) {
    if (penguins[i].containsPoint(x, y)) {
      dragTarget = penguins[i];
      dragTarget.setDragging(true);
      dragTarget.vx = 0;
      dragTarget.vy = 0;
      dragTarget.angularVel = 0;
      dragOffsetX = x - dragTarget.x;
      dragOffsetY = y - dragTarget.y;
      lastMouseX = x;
      lastMouseY = y;
      lastPointerTime = performance.now();
      break;
    }
  }
}

function onPointerMove(e) {
  const { x, y } = getPointerPos(e);
  const now = performance.now();

  if (dragTarget) {
    // Give the dragged penguin a real velocity (px/s), computed from how
    // far it actually moved since the last event. Collision resolution
    // reads this velocity every frame, so a fast drag genuinely flings
    // anything it plows through — a slow drag barely nudges things.
    const dt = Math.max((now - lastPointerTime) / 1000, 1 / 1000);
    const newX = x - dragOffsetX;
    const newY = y - dragOffsetY;

    let vx = (newX - dragTarget.x) / dt;
    let vy = (newY - dragTarget.y) / dt;
    const speed = Math.hypot(vx, vy);
    if (speed > MAX_DRAG_SPEED) {
      const scale = MAX_DRAG_SPEED / speed;
      vx *= scale;
      vy *= scale;
    }

    dragTarget.vx = vx;
    dragTarget.vy = vy;
    dragTarget.angularVel = vx * 0.01;

    dragTarget.x = newX;
    dragTarget.y = newY;
  }

  lastMouseX = x;
  lastMouseY = y;
  lastPointerTime = now;
}

function onPointerUp(e) {
  const { x, y } = getPointerPos(e);

  if (dragTarget) {
    // vx/vy already hold real drag velocity from the last onPointerMove,
    // so releasing just lets normal physics take over from here (throw).
    dragTarget.setDragging(false);
    dragTarget = null;
  } else if (pointerDownPos) {
    const moved = Math.hypot(x - pointerDownPos.x, y - pointerDownPos.y);
    if (moved < CLICK_MOVE_THRESHOLD) {
      spawnPenguin(x, y);
    }
  }
  pointerDownPos = null;
}

canvas.addEventListener("mousedown", onPointerDown);
canvas.addEventListener("mousemove", onPointerMove);
window.addEventListener("mouseup", onPointerUp);

canvas.addEventListener("touchstart", (e) => { e.preventDefault(); onPointerDown(e); }, { passive: false });
canvas.addEventListener("touchmove", (e) => { e.preventDefault(); onPointerMove(e); }, { passive: false });
canvas.addEventListener("touchend", (e) => { e.preventDefault(); onPointerUp(e); }, { passive: false });

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
let lastTime = performance.now();

function loop(now) {
  let dt = (now - lastTime) / 1000;
  dt = Math.min(dt, 1 / 30); // clamp so tab-switch pauses don't cause a jump
  lastTime = now;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (const p of penguins) p.integrate(dt);

  // multiple solver passes = more stable stacking/contact behavior
  for (let iter = 0; iter < CONFIG.solverIterations; iter++) {
    resolveCollisions();
    for (const p of penguins) resolveWalls(p);
  }

  for (const p of penguins) p.draw(ctx);

  if (penguins.length === 0) {
    ctx.fillStyle = "rgba(0,0,0,0.5)";
    ctx.font = "16px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Click anywhere to spawn a penguin", canvas.width / 2, canvas.height / 2);
  }

  requestAnimationFrame(loop);
}

requestAnimationFrame(loop);
