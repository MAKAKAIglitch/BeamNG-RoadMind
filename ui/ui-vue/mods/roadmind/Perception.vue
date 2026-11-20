<template>
  <div class="perception">
    <header class="header">
      <div>
        <div class="kicker">ROADMIND / PERCEPTION</div>
        <h2>{{ t("title") }}</h2>
        <p>{{ t("subtitle") }}</p>
      </div>
      <div class="switcher">
        <button :class="{ active: mode === 'top' }" @click="mode = 'top'">{{ t("top") }}</button>
        <button :class="{ active: mode === 'follow' }" @click="mode = 'follow'">{{ t("follow") }}</button>
      </div>
    </header>

    <div class="scene">
      <svg viewBox="0 0 480 520" preserveAspectRatio="xMidYMid meet" role="img" :aria-label="t('scene')">
        <defs>
          <linearGradient id="roadmind-perception-bg" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stop-color="#111923" />
            <stop offset="100%" stop-color="#080d12" />
          </linearGradient>
          <linearGradient id="roadmind-perception-road" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stop-color="#354452" stop-opacity="0.52" />
            <stop offset="100%" stop-color="#1b2630" stop-opacity="0.20" />
          </linearGradient>
          <pattern id="roadmind-perception-grid" width="30" height="30" patternUnits="userSpaceOnUse">
            <path d="M 30 0 L 0 0 0 30" fill="none" stroke="#263440" stroke-width="0.8" />
          </pattern>
        </defs>

        <rect width="480" height="520" fill="url(#roadmind-perception-bg)" />
        <rect width="480" height="520" fill="url(#roadmind-perception-grid)" opacity="0.34" />

        <g class="spatial-grid">
          <polyline v-for="line in gridLines" :key="line.id" :points="line.points" />
        </g>

        <polygon v-if="roadSurface" :points="roadSurface" class="road-surface" />
        <polyline v-if="roadPath" :points="roadPath" class="road-centreline" />
        <polyline v-if="routePath" :points="routePath" class="route-line" />

        <g class="mesh">
          <polygon
            v-for="face in meshFaces"
            :key="face.id"
            :points="face.points"
            :fill-opacity="face.opacity"
            :stroke-opacity="face.opacity * 2.4"
            class="mesh-face"
          />
        </g>

        <g class="vehicle-boxes">
          <g v-for="vehicle in vehicles" :key="vehicle.id">
            <line
              v-for="(edge, index) in vehicle.edges"
              :key="vehicle.id + '-' + index"
              :x1="edge.x1" :y1="edge.y1" :x2="edge.x2" :y2="edge.y2"
              :class="vehicle.edgeClass"
            />
            <polygon v-if="vehicle.top" :points="vehicle.top" :class="vehicle.topClass" />
          </g>
        </g>

        <g class="obstacle-markers">
          <g v-for="obstacle in obstacles" :key="obstacle.id">
            <circle :cx="obstacle.x" :cy="obstacle.y" r="3.2" />
            <path :d="'M ' + (obstacle.x - 5) + ' ' + obstacle.y + ' L ' + (obstacle.x + 5) + ' ' + obstacle.y" />
          </g>
        </g>

        <g class="cloud-points">
          <circle
            v-for="point in meshPoints"
            :key="point.id"
            :cx="point.x"
            :cy="point.y"
            :r="point.radius"
            :fill-opacity="point.opacity"
          />
        </g>

        <g v-if="egoGeometry" class="ego-car">
          <polygon :points="egoGeometry.side" class="ego-side" />
          <polygon :points="egoGeometry.top" class="ego-top" />
          <polyline :points="egoGeometry.outline" class="ego-outline" />
          <polyline :points="egoGeometry.roof" class="ego-roof" />
        </g>

        <text x="15" y="22" class="overlay-label">{{ t("view") }}: {{ mode === "follow" ? "3D FOLLOW" : "3D TOP" }}</text>
        <text x="465" y="22" text-anchor="end" class="overlay-label">{{ t("road") }}: {{ roadQuality }}</text>
        <text x="15" y="505" class="overlay-label">{{ t("cloud") }}: {{ meshPoints.length }} / {{ pointSourceLabel }}</text>
        <text x="465" y="505" text-anchor="end" class="overlay-label">{{ t("ego") }}</text>
      </svg>
      <div v-if="!snapshot.vehicle" class="waiting">{{ t("waiting") }}</div>
      <div v-else-if="!meshPoints.length" class="no-cloud">{{ t("noCloud") }}</div>
    </div>

    <div class="legend">
      <span><i class="mesh-dot"></i>{{ t("mesh") }}: {{ meshFaces.length }}</span>
      <span><i class="point-dot"></i>{{ t("points") }}: {{ meshPoints.length }}</span>
      <span><i class="car-dot"></i>{{ t("vehicles") }}: {{ vehicles.length }}</span>
      <span><i class="route-dot"></i>{{ t("route") }}</span>
      <span class="age">{{ t("age") }}: {{ frameAge }}</span>
    </div>

    <div class="objects">
      <div class="list-title">{{ t("detected") }}</div>
      <div v-if="!vehicles.length && !obstacles.length" class="empty">{{ t("empty") }}</div>
      <div v-for="car in vehicles.slice(0, 8)" :key="car.id" class="object-row">
        <span class="vehicle-icon"></span>
        <div class="object-main">
          <strong>{{ car.id }}</strong>
          <span>{{ car.source }}</span>
        </div>
        <div class="object-info">
          <strong>{{ car.state }}</strong>
          <span>{{ car.distance }} m</span>
        </div>
      </div>
      <div v-for="obstacle in obstacles.slice(0, 4)" :key="obstacle.id" class="object-row">
        <span class="obstacle-icon"></span>
        <div class="object-main">
          <strong>{{ t("obstacle") }}</strong>
          <span>{{ obstacle.source }}</span>
        </div>
        <div class="object-info">
          <strong>{{ t("measured") }}</strong>
          <span>{{ obstacle.distance }} m</span>
        </div>
      </div>
    </div>

    <footer>
      <span>{{ t("disclaimer") }}</span>
      <button @click="refresh">{{ t("refresh") }}</button>
    </footer>
  </div>
</template>

<script setup>
import { computed, onMounted, onBeforeUnmount, ref } from "vue"
import { runRaw } from "@/bridge/libs/Lua.js"

const EN = {
  title: "3D environment model",
  subtitle: "LiDAR point cloud · triangulated spatial view",
  scene: "3D triangulated perception map",
  top: "Top view",
  follow: "Follow",
  ego: "EGO",
  view: "Camera",
  road: "NavGraph",
  cloud: "Cloud",
  mesh: "Mesh faces",
  points: "Points",
  route: "Planned route",
  age: "Frame age",
  detected: "TRACKED OBJECTS",
  obstacle: "Obstacle cluster",
  measured: "MEASURED",
  empty: "No current objects in the World Model.",
  waiting: "Waiting for RoadMind sensor data",
  noCloud: "Waiting for LiDAR points · enable point accumulation for a persistent map",
  visible: "visible",
  predicted: "predicted",
  stale: "stale",
  carCamera: "Camera visibility model",
  carSim: "BeamNG vehicle bounds",
  refresh: "Refresh",
  disclaimer: "Triangles are a render-only surface estimate between nearby LiDAR returns. They are not used by the planner."
}
const RU = {
  title: "3D-модель окружения",
  subtitle: "Облако LiDAR · триангулированное пространство",
  scene: "Трёхмерная карта восприятия",
  top: "Сверху",
  follow: "Сзади",
  ego: "АВТО",
  view: "Камера",
  road: "NavGraph",
  cloud: "Облако",
  mesh: "Треугольники",
  points: "Точки",
  route: "Маршрут",
  age: "Возраст кадра",
  detected: "ОТСЛЕЖИВАЕМЫЕ ОБЪЕКТЫ",
  obstacle: "Кластер препятствий",
  measured: "ИЗМЕРЕНО",
  empty: "В World Model пока нет объектов.",
  waiting: "Ожидание данных RoadMind",
  noCloud: "Ожидание точек LiDAR · включи накопление облака для постоянной карты",
  visible: "виден",
  predicted: "прогноз",
  stale: "устарел",
  carCamera: "Модель видимости камер",
  carSim: "Габариты машины BeamNG",
  refresh: "Обновить",
  disclaimer: "Треугольники — визуальная аппроксимация поверхности между близкими точками LiDAR. Планировщик их не использует."
}

const locale = ref("en-US")
const mode = ref("follow")
const loading = ref(false)
const snapshot = ref({ vehicle: null, targets: [], obstacles: [], lidarPoints: [], road: {}, route: [] })
let timer = null

const BOX_EDGES = [
  [0, 1], [1, 2], [2, 3], [3, 0],
  [4, 5], [5, 6], [6, 7], [7, 4],
  [0, 4], [1, 5], [2, 6], [3, 7]
]

function t(key) {
  const dict = String(locale.value).toLowerCase().startsWith("ru") ? RU : EN
  return dict[key] || EN[key] || key
}

const roadQuality = computed(() => {
  const q = snapshot.value.road && snapshot.value.road.quality
  return q && q !== "unavailable" ? String(q).toUpperCase() : "—"
})
const frameAge = computed(() => {
  const age = Number(snapshot.value.frameAge)
  return Number.isFinite(age) ? age.toFixed(2) + " s" : "—"
})
const pointSourceLabel = computed(() => {
  const source = snapshot.value.lidarPointSource
  if (source === "accumulated") return String(locale.value).toLowerCase().startsWith("ru") ? "накоплено" : "accumulated"
  if (source === "live") return String(locale.value).toLowerCase().startsWith("ru") ? "скан" : "live scan"
  return "—"
})

function normalize3(v, fallback) {
  if (!v) return fallback
  const x = Number(v.x) || 0
  const y = Number(v.y) || 0
  const z = Number(v.z) || 0
  const length = Math.sqrt(x * x + y * y + z * z)
  if (!Number.isFinite(length) || length < 1e-6) return fallback
  return { x: x / length, y: y / length, z: z / length }
}

function dot3(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z
}

function worldToLocal(point) {
  const vehicle = snapshot.value.vehicle
  if (!vehicle || !point || !vehicle.position || !vehicle.forward || !vehicle.right) return null
  const dx = Number(point.x) - Number(vehicle.position.x)
  const dy = Number(point.y) - Number(vehicle.position.y)
  const dz = Number(point.z || 0) - Number(vehicle.position.z || 0)
  if (![dx, dy, dz].every(Number.isFinite)) return null
  const forward = normalize3(vehicle.forward, { x: 0, y: 1, z: 0 })
  const right = normalize3(vehicle.right, { x: 1, y: 0, z: 0 })
  const up = normalize3(vehicle.up, { x: 0, y: 0, z: 1 })
  return {
    r: dx * right.x + dy * right.y + dz * right.z,
    f: dx * forward.x + dy * forward.y + dz * forward.z,
    u: dx * up.x + dy * up.y + dz * up.z
  }
}

const camera = computed(() => {
  if (mode.value === "top") {
    return { r: 0, f: -10, u: 32, targetR: 0, targetF: 20, targetU: 0, focal: 300, horizon: 280 }
  }
  // Slightly farther back and above the ego car, while keeping it prominent.
  return { r: 0, f: -10, u: 5, targetR: 0, targetF: 26, targetU: 0, focal: 330, horizon: 300 }
})

function projectLocal(point) {
  const c = camera.value
  if (!point) return null
  const direction = normalize3({
    x: c.targetR - c.r,
    y: c.targetF - c.f,
    z: c.targetU - c.u
  }, { x: 0, y: 1, z: 0 })
  const worldUp = { x: 0, y: 0, z: 1 }
  let right = {
    x: direction.y * worldUp.z - direction.z * worldUp.y,
    y: direction.z * worldUp.x - direction.x * worldUp.z,
    z: direction.x * worldUp.y - direction.y * worldUp.x
  }
  right = normalize3(right, { x: 1, y: 0, z: 0 })
  const up = normalize3({
    x: right.y * direction.z - right.z * direction.y,
    y: right.z * direction.x - right.x * direction.z,
    z: right.x * direction.y - right.y * direction.x
  }, worldUp)

  const delta = { x: point.r - c.r, y: point.f - c.f, z: point.u - c.u }
  const depth = dot3(delta, direction)
  if (!Number.isFinite(depth) || depth < 1.2) return null
  const cameraX = dot3(delta, right)
  const cameraY = dot3(delta, up)
  const scale = c.focal / depth
  const x = 240 + cameraX * scale
  const y = c.horizon - cameraY * scale
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null
  return { x, y, depth }
}

function projectWorld(point) {
  const local = worldToLocal(point)
  return local ? projectLocal(local) : null
}

function pathFromWorld(points) {
  const projected = (points || []).map(projectWorld).filter(Boolean)
  if (projected.length < 2) return ""
  return projected.map((p, i) => (i ? "L " : "M ") + p.x.toFixed(1) + " " + p.y.toFixed(1)).join(" ")
}

const roadPath = computed(() => pathFromWorld(snapshot.value.road && snapshot.value.road.corridor || []))
const routePath = computed(() => pathFromWorld(snapshot.value.route || []))

const roadSurface = computed(() => {
  const points = (snapshot.value.road && snapshot.value.road.corridor || []).map(worldToLocal).filter(Boolean)
  if (points.length < 2) return ""
  const left = []
  const right = []
  const halfWidth = 3.7
  for (let i = 0; i < points.length; i++) {
    const previous = points[Math.max(0, i - 1)]
    const next = points[Math.min(points.length - 1, i + 1)]
    const dr = next.r - previous.r
    const df = next.f - previous.f
    const length = Math.sqrt(dr * dr + df * df) || 1
    const sideR = -df / length
    const sideF = dr / length
    const a = projectLocal({ r: points[i].r + sideR * halfWidth, f: points[i].f + sideF * halfWidth, u: points[i].u - 0.08 })
    const b = projectLocal({ r: points[i].r - sideR * halfWidth, f: points[i].f - sideF * halfWidth, u: points[i].u - 0.08 })
    if (a && b) {
      left.push(a.x.toFixed(1) + "," + a.y.toFixed(1))
      right.push(b.x.toFixed(1) + "," + b.y.toFixed(1))
    }
  }
  if (left.length < 2 || right.length < 2) return ""
  return left.concat(right.reverse()).join(" ")
})

const gridLines = computed(() => {
  const result = []
  let id = 0
  for (let f = -10; f <= 90; f += 10) {
    const a = projectLocal({ r: -28, f, u: -0.12 })
    const b = projectLocal({ r: 28, f, u: -0.12 })
    if (a && b) result.push({ id: "cross-" + id++, points: a.x.toFixed(1) + "," + a.y.toFixed(1) + " " + b.x.toFixed(1) + "," + b.y.toFixed(1) })
  }
  for (let r = -28; r <= 28; r += 4) {
    const a = projectLocal({ r, f: -10, u: -0.12 })
    const b = projectLocal({ r, f: 90, u: -0.12 })
    if (a && b) result.push({ id: "long-" + id++, points: a.x.toFixed(1) + "," + a.y.toFixed(1) + " " + b.x.toFixed(1) + "," + b.y.toFixed(1) })
  }
  return result
})

function circumcircleContains(point, triangle, points) {
  const a = points[triangle.a]
  const b = points[triangle.b]
  const c = points[triangle.c]
  const d = 2 * (a.r * (b.f - c.f) + b.r * (c.f - a.f) + c.r * (a.f - b.f))
  if (Math.abs(d) < 1e-9) return false
  const aa = a.r * a.r + a.f * a.f
  const bb = b.r * b.r + b.f * b.f
  const cc = c.r * c.r + c.f * c.f
  const centerR = (aa * (b.f - c.f) + bb * (c.f - a.f) + cc * (a.f - b.f)) / d
  const centerF = (aa * (c.r - b.r) + bb * (a.r - c.r) + cc * (b.r - a.r)) / d
  const dr = point.r - centerR
  const df = point.f - centerF
  const ar = a.r - centerR
  const af = a.f - centerF
  const radiusSquared = ar * ar + af * af
  return dr * dr + df * df <= radiusSquared + 1e-7
}

function triangulateDelaunay(input) {
  const count = input.length
  if (count < 3) return []
  let minR = Infinity
  let maxR = -Infinity
  let minF = Infinity
  let maxF = -Infinity
  for (const p of input) {
    minR = Math.min(minR, p.r)
    maxR = Math.max(maxR, p.r)
    minF = Math.min(minF, p.f)
    maxF = Math.max(maxF, p.f)
  }
  const span = Math.max(maxR - minR, maxF - minF, 1)
  const midR = (minR + maxR) * 0.5
  const midF = (minF + maxF) * 0.5
  const points = input.slice()
  points.push(
    { r: midR - span * 24, f: midF - span * 3, u: 0 },
    { r: midR, f: midF + span * 24, u: 0 },
    { r: midR + span * 24, f: midF - span * 3, u: 0 }
  )
  let triangles = [{ a: count, b: count + 1, c: count + 2 }]

  for (let index = 0; index < count; index++) {
    const bad = new Set()
    const edges = new Map()
    for (let i = 0; i < triangles.length; i++) {
      const triangle = triangles[i]
      if (!circumcircleContains(points[index], triangle, points)) continue
      bad.add(i)
      const pairs = [[triangle.a, triangle.b], [triangle.b, triangle.c], [triangle.c, triangle.a]]
      for (const pair of pairs) {
        const low = Math.min(pair[0], pair[1])
        const high = Math.max(pair[0], pair[1])
        const key = low + ":" + high
        const edge = edges.get(key)
        if (edge) edge.count++
        else edges.set(key, { a: pair[0], b: pair[1], count: 1 })
      }
    }
    if (!bad.size) continue
    triangles = triangles.filter((triangle, i) => !bad.has(i))
    for (const edge of edges.values()) {
      if (edge.count !== 1) continue
      const a = points[edge.a]
      const b = points[edge.b]
      const area = (b.r - a.r) * (points[index].f - a.f) - (b.f - a.f) * (points[index].r - a.r)
      if (Math.abs(area) > 1e-6) triangles.push({ a: edge.a, b: edge.b, c: index })
    }
  }
  return triangles.filter(triangle => triangle.a < count && triangle.b < count && triangle.c < count)
}

const localCloudPoints = computed(() => {
  const source = snapshot.value.lidarPoints || []
  const cells = new Map()
  for (const worldPoint of source) {
    const p = worldToLocal(worldPoint)
    if (!p || p.f < -18 || p.f > 88 || Math.abs(p.r) > 30 || p.u < -6 || p.u > 12) continue
    const cell = Math.floor(p.r / 0.72) + ":" + Math.floor(p.f / 0.72)
    const existing = cells.get(cell)
    if (!existing || Math.abs(p.u + 0.7) < Math.abs(existing.u + 0.7)) cells.set(cell, p)
  }
  const points = Array.from(cells.values()).sort((a, b) => a.f - b.f || a.r - b.r)
  const maxPoints = 420
  const stride = Math.max(1, Math.ceil(points.length / maxPoints))
  const sampled = []
  for (let i = 0; i < points.length; i += stride) sampled.push(points[i])
  return sampled
})

const meshFaces = computed(() => {
  const points = localCloudPoints.value
  if (points.length < 3) return []
  const triangles = triangulateDelaunay(points)
  const faces = []
  let serial = 0
  for (const triangle of triangles) {
    const a = points[triangle.a]
    const b = points[triangle.b]
    const c = points[triangle.c]
    const pairs = [[a, b], [b, c], [c, a]]
    let maxEdge = 0
    for (const pair of pairs) {
      const dr = pair[0].r - pair[1].r
      const df = pair[0].f - pair[1].f
      const du = pair[0].u - pair[1].u
      maxEdge = Math.max(maxEdge, Math.sqrt(dr * dr + df * df + du * du))
    }
    const heightSpan = Math.max(a.u, b.u, c.u) - Math.min(a.u, b.u, c.u)
    const horizontalArea = Math.abs((b.r - a.r) * (c.f - a.f) - (b.f - a.f) * (c.r - a.r)) * 0.5
    if (maxEdge > 4.2 || heightSpan > 2.4 || horizontalArea < 0.08 || horizontalArea > 8.5) continue
    const pa = projectLocal(a)
    const pb = projectLocal(b)
    const pc = projectLocal(c)
    if (!pa || !pb || !pc) continue
    if ([pa, pb, pc].every(p => p.x < -20 || p.x > 500 || p.y < -20 || p.y > 540)) continue
    const depth = (pa.depth + pb.depth + pc.depth) / 3
    faces.push({
      id: "face-" + serial++,
      points: [pa, pb, pc].map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" "),
      depth,
      opacity: Math.max(0.045, Math.min(0.19, 0.19 - depth * 0.00145))
    })
  }
  return faces.sort((a, b) => b.depth - a.depth)
})

const meshPoints = computed(() => {
  return localCloudPoints.value.map((p, index) => {
    const projected = projectLocal(p)
    if (!projected || projected.x < -15 || projected.x > 495 || projected.y < -15 || projected.y > 535) return null
    return {
      id: "point-" + index,
      x: projected.x,
      y: projected.y,
      radius: Math.max(1, Math.min(2.2, 2.3 - projected.depth * 0.012)),
      opacity: Math.max(0.25, Math.min(0.9, 0.92 - projected.depth * 0.008))
    }
  }).filter(Boolean)
})

function worldVectorToLocalBasis(vector) {
  const vehicle = snapshot.value.vehicle
  if (!vehicle || !vector) return { r: 1, f: 0, u: 0 }
  const forward = normalize3(vehicle.forward, { x: 0, y: 1, z: 0 })
  const right = normalize3(vehicle.right, { x: 1, y: 0, z: 0 })
  const up = normalize3(vehicle.up, { x: 0, y: 0, z: 1 })
  return { r: dot3(vector, right), f: dot3(vector, forward), u: dot3(vector, up) }
}

function buildBox(centerWorld, dimensions, orientation) {
  const vehicle = snapshot.value.vehicle
  if (!vehicle || !centerWorld) return null
  const basis = orientation || {}
  const forward = worldVectorToLocalBasis(basis.forward || vehicle.forward)
  const right = worldVectorToLocalBasis(basis.right || vehicle.right)
  const up = worldVectorToLocalBasis(basis.up || vehicle.up)
  const width = Math.max(0.8, Number(dimensions && dimensions.width) || 1.8)
  const length = Math.max(1.4, Number(dimensions && dimensions.length) || 4.4)
  const height = Math.max(0.6, Number(dimensions && dimensions.height) || 1.5)
  const center = worldToLocal(centerWorld)
  if (!center) return null
  const signs = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
  ]
  const corners = signs.map(sign => ({
    r: center.r + right.r * sign[0] * width * 0.5 + forward.r * sign[1] * length * 0.5 + up.r * sign[2] * height * 0.5,
    f: center.f + right.f * sign[0] * width * 0.5 + forward.f * sign[1] * length * 0.5 + up.f * sign[2] * height * 0.5,
    u: center.u + right.u * sign[0] * width * 0.5 + forward.u * sign[1] * length * 0.5 + up.u * sign[2] * height * 0.5
  }))
  const projected = corners.map(projectLocal)
  const edges = BOX_EDGES.map(pair => {
    const a = projected[pair[0]]
    const b = projected[pair[1]]
    if (!a || !b) return null
    return { x1: a.x, y1: a.y, x2: b.x, y2: b.y }
  }).filter(Boolean)
  const top = [projected[4], projected[5], projected[6], projected[7]]
  const topPoints = top.every(Boolean) ? top.map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ") : ""
  const side = [projected[0], projected[1], projected[5], projected[4]]
  const sidePoints = side.every(Boolean) ? side.map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ") : ""
  return { edges, top: topPoints, side: sidePoints }
}

const vehicles = computed(() => (snapshot.value.targets || []).map(track => {
  const center = track.predictedPosition || track.position
  const projected = projectWorld(center)
  if (!projected || projected.x < -40 || projected.x > 520 || projected.y < -40 || projected.y > 560) return null
  const age = Number(track.dataAge || 0)
  const state = track.status === "confirmed" && age < 0.55 ? "visible"
    : (track.status === "stale" || age > 1 ? "stale" : "predicted")
  const sources = track.sourceSensors || []
  const source = sources.some(s => s.sensor === "camera") ? t("carCamera") : t("carSim")
  const box = buildBox(center, track.dimensions, track.orientation)
  const delta = worldToLocal(center)
  const distance = delta ? Math.sqrt(delta.f * delta.f + delta.r * delta.r) : 0
  return {
    id: track.trackId || "vehicle",
    edges: box ? box.edges : [],
    top: box ? box.top : "",
    state: t(state),
    source,
    edgeClass: state === "visible" ? "vehicle-edge-visible" : state === "stale" ? "vehicle-edge-stale" : "vehicle-edge-predicted",
    topClass: state === "visible" ? "vehicle-top-visible" : "vehicle-top-predicted",
    distance: distance.toFixed(1)
  }
}).filter(Boolean))

const obstacles = computed(() => (snapshot.value.obstacles || []).map(obstacle => {
  const p = projectWorld(obstacle.position)
  if (!p || p.x < -30 || p.x > 510 || p.y < -30 || p.y > 550) return null
  const sensors = (obstacle.sourceSensors || []).map(s => s.sensor).filter(Boolean)
  const local = worldToLocal(obstacle.position)
  return {
    id: obstacle.obstacleId || p.x.toFixed(0) + "-" + p.y.toFixed(0),
    x: p.x,
    y: p.y,
    source: [...new Set(sensors)].join(" / ") || "LiDAR",
    distance: local ? Math.sqrt(local.f * local.f + local.r * local.r).toFixed(1) : "—"
  }
}).filter(Boolean))

const egoGeometry = computed(() => {
  const vehicle = snapshot.value.vehicle
  if (!vehicle) return null
  const dimensions = vehicle.dimensions || {}
  const width = Math.max(1.5, Number(dimensions.width) || 1.9)
  const length = Math.max(3.2, Number(dimensions.length) || 4.5)
  const height = Math.max(1.0, Number(dimensions.height) || 1.4)
  const halfW = width * 0.5
  const halfL = length * 0.5
  const localCorners = [
    { r: -halfW, f: -halfL, u: 0.05 },
    { r: halfW, f: -halfL, u: 0.05 },
    { r: halfW, f: halfL, u: 0.05 },
    { r: -halfW, f: halfL, u: 0.05 },
    { r: -halfW, f: -halfL, u: height * 0.72 },
    { r: halfW, f: -halfL, u: height * 0.72 },
    { r: halfW, f: halfL, u: height * 0.72 },
    { r: -halfW, f: halfL, u: height * 0.72 }
  ]
  const p = localCorners.map(projectLocal)
  if (!p.every(Boolean)) return null
  const top = [p[4], p[5], p[6], p[7]]
  const topPoints = top.map(v => v.x.toFixed(1) + "," + v.y.toFixed(1)).join(" ")
  const side = [p[0], p[1], p[5], p[4]].map(v => v.x.toFixed(1) + "," + v.y.toFixed(1)).join(" ")
  const outline = [p[4], p[5], p[6], p[7], p[4]].map(v => v.x.toFixed(1) + "," + v.y.toFixed(1)).join(" ")
  const centerX = top.reduce((sum, v) => sum + v.x, 0) / top.length
  const centerY = top.reduce((sum, v) => sum + v.y, 0) / top.length
  const roof = top.map(v => (centerX + (v.x - centerX) * 0.55).toFixed(1) + "," + (centerY + (v.y - centerY) * 0.58).toFixed(1)).join(" ")
  return { top: topPoints, side, outline, roof }
})

async function refresh() {
  if (loading.value) return
  loading.value = true
  try {
    const cmd = "(function() extensions.load('roadmind'); return extensions.roadmind.getPerceptionSnapshot() end)()"
    const result = await runRaw(cmd)
    if (result && result.ok === false) throw new Error(result.error || "RoadMind error")
    if (result && result.language) locale.value = result.language
    if (result) snapshot.value = result
  } catch (e) {
    console.warn("RoadMind perception snapshot unavailable", e)
  } finally {
    loading.value = false
  }
}

onMounted(() => {
  refresh()
  timer = setInterval(refresh, 500)
})
onBeforeUnmount(() => {
  if (timer) clearInterval(timer)
})
</script>

<style scoped lang="scss">
.perception {
  height: 78vh;
  min-height: 460px;
  display: flex;
  flex-direction: column;
  gap: 0.55rem;
  padding: 0.7rem;
  overflow: hidden;
  color: #d8e1e9;
  background: #10161d;
  border-radius: 15px;
}
.header { display: flex; align-items: center; justify-content: space-between; gap: 0.8rem; }
.kicker { color: #78cbbb; font-size: 0.65rem; letter-spacing: 0.16em; font-weight: 700; }
.header h2 { margin: 0.1rem 0; font-size: 1.05rem; font-weight: 650; }
.header p { margin: 0; color: #81909e; font-size: 0.72rem; }
.switcher { display: flex; gap: 0.35rem; }
.switcher button, footer button {
  border: 1px solid #33414d;
  background: #18212a;
  color: #bfcbd4;
  border-radius: 8px;
  padding: 0.35rem 0.6rem;
  font: inherit;
  font-size: 0.72rem;
}
.switcher button.active { border-color: #66c5b4; background: #1c3b3a; color: #c8fff3; }
.scene {
  position: relative;
  flex: 1 1 auto;
  min-height: 260px;
  overflow: hidden;
  border: 1px solid #344653;
  border-radius: 19px;
  background: #0b1118;
  box-shadow: inset 0 0 0 1px rgba(190, 220, 235, 0.025), 0 8px 24px rgba(0, 0, 0, 0.18);
}
.scene svg { display: block; width: 100%; height: 100%; }
.spatial-grid polyline { fill: none; stroke: #32414e; stroke-width: 0.85; stroke-opacity: 0.48; }
.road-surface { fill: url(#roadmind-perception-road); stroke: #617181; stroke-width: 0.8; stroke-opacity: 0.3; }
.road-centreline { fill: none; stroke: #8496a6; stroke-width: 1.1; stroke-dasharray: 3 7; stroke-opacity: 0.55; }
.route-line { fill: none; stroke: #4aafff; stroke-width: 3.2; stroke-linecap: round; stroke-linejoin: round; filter: drop-shadow(0 0 4px rgba(61, 160, 255, 0.56)); }
.mesh-face { fill: #318e91; stroke: #67c6c3; stroke-width: 0.75; stroke-linejoin: round; }
.cloud-points circle { fill: #d5f2ee; stroke: #78cfc5; stroke-width: 0.35; }
.obstacle-markers circle { fill: #d9e3e9; fill-opacity: 0.12; stroke: #d2dee7; stroke-width: 1.1; }
.obstacle-markers path { fill: none; stroke: #d2dee7; stroke-width: 0.8; stroke-opacity: 0.75; }
.vehicle-edge-visible { stroke: #70d7c3; stroke-width: 1.5; }
.vehicle-edge-predicted { stroke: #e6c36b; stroke-width: 1.15; stroke-dasharray: 3 2; }
.vehicle-edge-stale { stroke: #8795a2; stroke-width: 1; stroke-dasharray: 2 3; }
.vehicle-top-visible { fill: #24645e; fill-opacity: 0.30; stroke: none; }
.vehicle-top-predicted { fill: #80682e; fill-opacity: 0.24; stroke: none; }
.ego-side { fill: #30404d; fill-opacity: 0.92; stroke: #d7e5ee; stroke-width: 1.2; }
.ego-top { fill: #d5e0e8; fill-opacity: 0.92; stroke: #f5fbff; stroke-width: 1.5; }
.ego-outline { fill: none; stroke: #f5fbff; stroke-width: 1.6; }
.ego-roof { fill: none; stroke: #6f8494; stroke-width: 1.1; }
.overlay-label { fill: #9aafbf; font-size: 8.5px; font-family: sans-serif; letter-spacing: 0.04em; }
.waiting, .no-cloud {
  position: absolute;
  left: 12px;
  right: 12px;
  bottom: 34px;
  display: grid;
  place-items: center;
  color: #a7b6c2;
  font-size: 0.74rem;
  text-align: center;
  pointer-events: none;
  text-shadow: 0 1px 3px #000;
}
.waiting { inset: 0; background: rgba(9, 14, 19, 0.76); }
.no-cloud { color: #8096a7; }
.legend { display: flex; flex-wrap: wrap; gap: 0.6rem 1rem; color: #9aabb9; font-size: 0.7rem; }
.legend span { display: inline-flex; align-items: center; gap: 0.35rem; }
.legend i { display: inline-block; width: 8px; height: 8px; border-radius: 2px; }
.mesh-dot { background: #56b7b3; }
.point-dot { background: #d5f2ee; border-radius: 50% !important; }
.car-dot { background: #70d7c3; }
.route-dot { background: #4aafff; }
.age { margin-left: auto; font-variant-numeric: tabular-nums; }
.objects { display: flex; flex-direction: column; gap: 0.25rem; max-height: 145px; overflow-y: auto; }
.list-title { color: #718493; font-size: 0.65rem; letter-spacing: 0.14em; font-weight: 700; padding: 0.25rem 0; }
.object-row { display: flex; align-items: center; gap: 0.55rem; min-height: 29px; padding: 0.25rem 0.45rem; border: 1px solid #26333e; border-radius: 8px; background: #151e27; }
.vehicle-icon, .obstacle-icon { width: 13px; height: 20px; border: 1px solid #75cfc0; border-radius: 4px; background: #245d59; }
.obstacle-icon { width: 11px; height: 11px; border-color: #9caab7; border-radius: 3px; background: #687787; }
.object-main { display: flex; flex-direction: column; flex: 1; min-width: 0; }
.object-main strong { color: #d9e4eb; font-size: 0.72rem; }
.object-main span, .object-info span { color: #8293a2; font-size: 0.65rem; }
.object-info { display: flex; flex-direction: column; align-items: flex-end; }
.object-info strong { color: #83d3c2; font-size: 0.65rem; }
.empty { padding: 0.5rem; color: #8293a2; font-size: 0.75rem; }
footer { display: flex; align-items: center; justify-content: space-between; gap: 0.6rem; color: #7d8e9d; font-size: 0.64rem; }
footer button { flex-shrink: 0; }
@media (max-width: 600px) {
  .perception { height: 76vh; padding: 0.35rem; }
  .header p { max-width: 13rem; }
  .scene { border-radius: 16px; }
}
</style>
