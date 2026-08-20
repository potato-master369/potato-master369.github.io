
const CONFIG = {
  imageSrc: "ass/penguin.jpg", // update this path if your image lives elsewhere
  penguinRadius: 40,           // collision radius (image is drawn at 2x this)
  gravity: 1800,               // px/s^2
  restitution: 0.55,           // bounciness on walls/floor (0-1)
  penguinRestitution: 0.3,     // bounciness between penguins
  airDamping: 0.999,           // slight air resistance
  groundFriction: 0.86,        // horizontal damping when resting on floor
  maxPenguins: 200,
};

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight - document.querySelector('h1').offsetHeight;
}
window.addEventListener("resize", resizeCanvas);
resizeCanvas();

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
// Penguin entity
// ---------------------------------------------------------------------------
class Penguin {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.vx = (Math.random() - 0.5) * 100;
    this.vy = 0;
    this.r = CONFIG.penguinRadius;
    this.angle = 0;
    this.angularVel = 0;
    this.dragging = false;
  }

  applyPhysics(dt) {
    if (this.dragging) return; // position driven by mouse while dragging

    this.vy += CONFIG.gravity * dt;
    this.vx *= CONFIG.airDamping;
    this.vy *= CONFIG.airDamping;

    this.x += this.vx * dt;
    this.y += this.vy * dt;

    this.angle += this.angularVel * dt;
    this.angularVel *= 0.98;

    this.resolveWalls();
  }

  resolveWalls() {
    const r = this.r;

    // Floor
    if (this.y + r > canvas.height) {
      this.y = canvas.height - r;
      if (this.vy > 0) this.vy = -this.vy * CONFIG.restitution;
      this.vx *= CONFIG.groundFriction;
      // stop tiny jitter
      if (Math.abs(this.vy) < 30) this.vy = 0;
    }
    // Ceiling
    if (this.y - r < 0) {
      this.y = r;
      if (this.vy < 0) this.vy = -this.vy * CONFIG.restitution;
    }
    // Left wall
    if (this.x - r < 0) {
      this.x = r;
      if (this.vx < 0) this.vx = -this.vx * CONFIG.restitution;
    }
    // Right wall
    if (this.x + r > canvas.width) {
      this.x = canvas.width - r;
      if (this.vx > 0) this.vx = -this.vx * CONFIG.restitution;
    }
  }

  draw(ctx) {
    ctx.save();
    ctx.translate(this.x, this.y);
    ctx.rotate(this.angle);

    if (imgLoaded) {
      const d = this.r * 2;
      ctx.drawImage(penguinImg, -this.r, -this.r, d, d);
    } else {
      // fallback so you see *something* before/without the image
      ctx.beginPath();
      ctx.arc(0, 0, this.r, 0, Math.PI * 2);
      ctx.fillStyle = "#222";
      ctx.fill();
    }

    ctx.restore();
  }

  containsPoint(px, py) {
    const dx = px - this.x;
    const dy = py - this.y;
    return dx * dx + dy * dy <= this.r * this.r;
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

// Circle-circle collision resolution (equal-mass elastic-ish response).
function resolveCollisions() {
  for (let i = 0; i < penguins.length; i++) {
    for (let j = i + 1; j < penguins.length; j++) {
      const a = penguins[i];
      const b = penguins[j];

      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const dist = Math.hypot(dx, dy) || 0.0001;
      const minDist = a.r + b.r;

      if (dist < minDist) {
        const nx = dx / dist;
        const ny = dy / dist;
        const overlap = minDist - dist;

        // Positional correction — push apart, weighted so dragged
        // penguins don't get shoved by whoever you're holding.
        const aStatic = a.dragging;
        const bStatic = b.dragging;

        if (!aStatic && !bStatic) {
          a.x -= nx * overlap * 0.5;
          a.y -= ny * overlap * 0.5;
          b.x += nx * overlap * 0.5;
          b.y += ny * overlap * 0.5;
        } else if (aStatic && !bStatic) {
          b.x += nx * overlap;
          b.y += ny * overlap;
        } else if (!aStatic && bStatic) {
          a.x -= nx * overlap;
          a.y -= ny * overlap;
        }

        // Velocity response along the collision normal.
        if (!aStatic || !bStatic) {
          const rvx = b.vx - a.vx;
          const rvy = b.vy - a.vy;
          const velAlongNormal = rvx * nx + rvy * ny;

          if (velAlongNormal < 0) {
            const restitution = CONFIG.penguinRestitution;
            const impulse = -(1 + restitution) * velAlongNormal / 2;

            if (!aStatic) {
              a.vx -= impulse * nx;
              a.vy -= impulse * ny;
              a.angularVel -= impulse * 0.01;
            }
            if (!bStatic) {
              b.vx += impulse * nx;
              b.vy += impulse * ny;
              b.angularVel += impulse * 0.01;
            }
          }
        }
      }
    }
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
let mouseVX = 0;
let mouseVY = 0;
let pointerDownPos = null;
const CLICK_MOVE_THRESHOLD = 6; // px — below this, treat mouseup as a "click"

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
      dragTarget.dragging = true;
      dragTarget.vx = 0;
      dragTarget.vy = 0;
      dragOffsetX = x - dragTarget.x;
      dragOffsetY = y - dragTarget.y;
      lastMouseX = x;
      lastMouseY = y;
      break;
    }
  }
}

function onPointerMove(e) {
  const { x, y } = getPointerPos(e);
  mouseVX = x - lastMouseX;
  mouseVY = y - lastMouseY;
  lastMouseX = x;
  lastMouseY = y;

  if (dragTarget) {
    dragTarget.x = x - dragOffsetX;
    dragTarget.y = y - dragOffsetY;
  }
}

function onPointerUp(e) {
  const { x, y } = getPointerPos(e);

  if (dragTarget) {
    dragTarget.dragging = false;
    // throw: convert recent mouse motion into velocity (60fps-normalized)
    dragTarget.vx = mouseVX * 60;
    dragTarget.vy = mouseVY * 60;
    dragTarget.angularVel = mouseVX * 0.05;
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

  for (const p of penguins) p.applyPhysics(dt);
  resolveCollisions();
  // re-clamp to walls after collision pushes, so nothing pokes through edges
  for (const p of penguins) p.resolveWalls();

  for (const p of penguins) p.draw(ctx);

  // hint text when empty
  if (penguins.length === 0) {
    ctx.fillStyle = "rgba(0,0,0,0.5)";
    ctx.font = "16px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Click anywhere to spawn a penguin", canvas.width / 2, canvas.height / 2);
  }

  requestAnimationFrame(loop);
}

requestAnimationFrame(loop);
