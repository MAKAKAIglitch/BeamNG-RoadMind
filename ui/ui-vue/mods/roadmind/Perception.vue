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

        <g class="vehicle-models">
          <g v-for="vehicle in vehicles" :key="vehicle.id">
            <polygon
              v-for="(face, index) in vehicle.faces"
              :key="vehicle.id + '-face-' + index"
              :points="face.points"
              :class="face.className"
            />
            <polyline
              v-if="vehicle.outline"
              :points="vehicle.outline"
              :class="vehicle.outlineClass"
            />
            <polyline
              v-if="vehicle.heading"
              :points="vehicle.heading"
              :class="vehicle.headingClass"
            />
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
        <text x="15" y="39" class="overlay-label">{{ diagnosticLine }}</text>
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
  unverified: "unverified",
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
  unverified: "не подтверждён",
  refresh: "Обновить",
  disclaimer: "Треугольники — визуальная аппроксимация поверхности между близкими точками LiDAR. Планировщик их не использует."
}

const locale = ref("en-US")
const mode = ref("follow")
const loading = ref(false)
const snapshot = ref({ vehicle: null, targets: [], candidates: [], obstacles: [], lidarPoints: [], road: {}, route: [] })
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
    // A high vertical camera shows the full 360-degree sensor neighbourhood.
    return { r: 0, f: 0, u: 48, targetR: 0, targetF: 0, targetU: 0, focal: 190, horizon: 260 }
  }
  // Pull the camera back and slightly higher to expose more surrounding geometry.
  return { r: 0, f: -21.5, u: 9.2, targetR: 0, targetF: 27, targetU: 0, focal: 320, horizon: 302 }
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
  const cellR = 0.82
  const cellF = 0.82
  const cellU = 0.72

  for (const worldPoint of source) {
    const p = worldToLocal(worldPoint)
    if (!p || p.f < -18 || p.f > 88 || Math.abs(p.r) > 30 || p.u < -10 || p.u > 14) continue

    // A 3D voxel key retains raised and vertical surfaces instead of reducing
    // all returns at the same ground-plane location to a single sample.
    const ir = Math.floor(p.r / cellR)
    const iff = Math.floor(p.f / cellF)
    const iu = Math.floor(p.u / cellU)
    const key = ir + ":" + iff + ":" + iu
    const centerR = (ir + 0.5) * cellR
    const centerF = (iff + 0.5) * cellF
    const centerU = (iu + 0.5) * cellU
    const score = Math.pow(p.r - centerR, 2)
      + Math.pow(p.f - centerF, 2) + Math.pow(p.u - centerU, 2)
    const existing = cells.get(key)
    if (!existing || score < existing.score) cells.set(key, { point: p, score })
  }

  const points = Array.from(cells.values(), item => item.point)
    .sort((a, b) => a.f - b.f || a.r - b.r || a.u - b.u)
  const maxPoints = 600
  if (points.length <= maxPoints) return points

  // Preserve coverage across the full sorted cloud when applying the cap.
  const stride = points.length / maxPoints
  const sampled = []
  for (let i = 0; i < maxPoints; i++) sampled.push(points[Math.floor(i * stride)])
  return sampled
})

function buildMeshSlices(points, axis) {
  const groups = new Map()
  const sliceSize = 0.78
  for (const point of points) {
    let sliceCoordinate
    let planeR
    let planeF
    if (axis === "height") {
      sliceCoordinate = point.u
      planeR = point.r
      planeF = point.f
    } else if (axis === "lateral") {
      sliceCoordinate = point.r
      planeR = point.f
      planeF = point.u
    } else {
      sliceCoordinate = point.f
      planeR = point.r
      planeF = point.u
    }

    const slice = Math.floor(sliceCoordinate / sliceSize)
    let group = groups.get(slice)
    if (!group) {
      group = []
      groups.set(slice, group)
    }
    group.push({ r: planeR, f: planeF, u: point.u, source: point })
  }
  return Array.from(groups.values()).filter(group => group.length >= 3)
}

const meshFaces = computed(() => {
  const points = localCloudPoints.value
  if (points.length < 3) return []

  const faces = []
  const axes = ["height", "lateral", "longitudinal"]
  let serial = 0
  for (const axis of axes) {
    const slices = buildMeshSlices(points, axis)
    for (const slice of slices) {
      const triangles = triangulateDelaunay(slice)
      for (const triangle of triangles) {
        const a = slice[triangle.a].source
        const b = slice[triangle.b].source
        const c = slice[triangle.c].source
        const pairs = [[a, b], [b, c], [c, a]]
        let maxEdge = 0
        for (const pair of pairs) {
          const dr = pair[0].r - pair[1].r
          const df = pair[0].f - pair[1].f
          const du = pair[0].u - pair[1].u
          maxEdge = Math.max(maxEdge, Math.sqrt(dr * dr + df * df + du * du))
        }

        let planeArea
        if (axis === "height") {
          planeArea = Math.abs((b.r - a.r) * (c.f - a.f) - (b.f - a.f) * (c.r - a.r)) * 0.5
        } else if (axis === "lateral") {
          planeArea = Math.abs((b.f - a.f) * (c.u - a.u) - (b.u - a.u) * (c.f - a.f)) * 0.5
        } else {
          planeArea = Math.abs((b.r - a.r) * (c.u - a.u) - (b.u - a.u) * (c.r - a.r)) * 0.5
        }
        if (maxEdge > 4.4 || planeArea < 0.045 || planeArea > 9.5) continue

        const pa = projectLocal(a)
        const pb = projectLocal(b)
        const pc = projectLocal(c)
        if (!pa || !pb || !pc) continue
        if ([pa, pb, pc].every(p => p.x < -20 || p.x > 500 || p.y < -20 || p.y > 540)) continue

        const depth = (pa.depth + pb.depth + pc.depth) / 3
        faces.push({
          id: axis + "-face-" + serial++,
          points: [pa, pb, pc].map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" "),
          depth,
          opacity: Math.max(0.045, Math.min(0.19, 0.19 - depth * 0.00145))
        })
        if (faces.length >= 1500) break
      }
      if (faces.length >= 1500) break
    }
    if (faces.length >= 1500) break
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

function buildCarModel(centerWorld, dimensions, orientation, status) {
  const vehicle = snapshot.value.vehicle
  if (!vehicle || !centerWorld) return null

  const basis = orientation || {}
  const forward = worldVectorToLocalBasis(basis.forward || vehicle.forward)
  const right = worldVectorToLocalBasis(basis.right || vehicle.right)
  const up = worldVectorToLocalBasis(basis.up || vehicle.up)
  const width = Math.max(0.9, Number(dimensions && dimensions.width) || 1.8)
  const length = Math.max(1.6, Number(dimensions && dimensions.length) || 4.4)
  const height = Math.max(0.75, Number(dimensions && dimensions.height) || 1.5)
  const halfW = width * 0.5
  const halfL = length * 0.5
  const halfH = height * 0.5
  const center = worldToLocal(centerWorld)
  if (!center) return null

  const point = (r, f, u) => ({
    r: center.r + right.r * r + forward.r * f + up.r * u,
    f: center.f + right.f * r + forward.f * f + up.f * u,
    u: center.u + right.u * r + forward.u * f + up.u * u
  })

  const suffix = status === "visible" ? ""
    : status === "unverified" ? "-unverified"
      : status === "stale" ? "-stale" : "-predicted"
  const faces = []
  const addFace = (vertices, material) => {
    const projected = vertices.map(v => projectLocal(point(v[0], v[1], v[2])))
    if (!projected.every(Boolean)) return
    const depth = projected.reduce((sum, p) => sum + p.depth, 0) / projected.length
    faces.push({
      points: projected.map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" "),
      className: "vehicle-" + material + suffix,
      depth
    })
  }

  const bodyBottom = -halfH * 0.78
  const bodyTop = -halfH * 0.02
  const bodyHalfW = halfW * 0.94
  const bodyHalfL = halfL * 0.94
  const cabinBottom = bodyTop + height * 0.08
  const cabinTop = halfH * 0.78
  const cabinHalfW = halfW * 0.70
  const cabinFront = halfL * 0.38
  const cabinRear = -halfL * 0.42
  const roofFront = halfL * 0.20
  const roofRear = -halfL * 0.29

  // Main body shell.
  addFace([
    [-bodyHalfW, -bodyHalfL, bodyTop], [bodyHalfW, -bodyHalfL, bodyTop],
    [bodyHalfW, bodyHalfL, bodyTop], [-bodyHalfW, bodyHalfL, bodyTop]
  ], "body")
  addFace([
    [-bodyHalfW, -bodyHalfL, bodyBottom], [-bodyHalfW, bodyHalfL, bodyBottom],
    [-bodyHalfW, bodyHalfL, bodyTop], [-bodyHalfW, -bodyHalfL, bodyTop]
  ], "body")
  addFace([
    [bodyHalfW, -bodyHalfL, bodyBottom], [bodyHalfW, bodyHalfL, bodyBottom],
    [bodyHalfW, bodyHalfL, bodyTop], [bodyHalfW, -bodyHalfL, bodyTop]
  ], "body")
  addFace([
    [-bodyHalfW, bodyHalfL, bodyBottom], [bodyHalfW, bodyHalfL, bodyBottom],
    [bodyHalfW, bodyHalfL, bodyTop], [-bodyHalfW, bodyHalfL, bodyTop]
  ], "body")
  addFace([
    [-bodyHalfW, -bodyHalfL, bodyBottom], [bodyHalfW, -bodyHalfL, bodyBottom],
    [bodyHalfW, -bodyHalfL, bodyTop], [-bodyHalfW, -bodyHalfL, bodyTop]
  ], "body")

  // Raised cabin with sloped front/rear glass and a roof plane.
  addFace([
    [-cabinHalfW, cabinRear, cabinBottom], [-cabinHalfW, cabinFront, cabinBottom],
    [-cabinHalfW * 0.82, roofFront, cabinTop], [-cabinHalfW * 0.82, roofRear, cabinTop]
  ], "cabin")
  addFace([
    [cabinHalfW, cabinRear, cabinBottom], [cabinHalfW, cabinFront, cabinBottom],
    [cabinHalfW * 0.82, roofFront, cabinTop], [cabinHalfW * 0.82, roofRear, cabinTop]
  ], "cabin")
  addFace([
    [-cabinHalfW, cabinFront, cabinBottom], [cabinHalfW, cabinFront, cabinBottom],
    [cabinHalfW * 0.82, roofFront, cabinTop], [-cabinHalfW * 0.82, roofFront, cabinTop]
  ], "glass")
  addFace([
    [-cabinHalfW, cabinRear, cabinBottom], [cabinHalfW, cabinRear, cabinBottom],
    [cabinHalfW * 0.82, roofRear, cabinTop], [-cabinHalfW * 0.82, roofRear, cabinTop]
  ], "glass")
  const roofVertices = [
    [-cabinHalfW * 0.82, roofRear, cabinTop],
    [cabinHalfW * 0.82, roofRear, cabinTop],
    [cabinHalfW * 0.82, roofFront, cabinTop],
    [-cabinHalfW * 0.82, roofFront, cabinTop]
  ]
  addFace(roofVertices, "roof")

  // Compact wheel blocks give the silhouette depth at the road plane.
  const wheelHalfWidth = Math.max(0.07, width * 0.055)
  const wheelHalfLength = Math.max(0.14, length * 0.075)
  const wheelU = bodyBottom + height * 0.14
  for (const side of [-1, 1]) {
    for (const axle of [-1, 1]) {
      const wheelR = side * (halfW - wheelHalfWidth * 0.45)
      const wheelF = axle * halfL * 0.62
      addFace([
        [wheelR - wheelHalfWidth, wheelF - wheelHalfLength, wheelU - 0.04],
        [wheelR + wheelHalfWidth, wheelF - wheelHalfLength, wheelU - 0.04],
        [wheelR + wheelHalfWidth, wheelF + wheelHalfLength, wheelU + 0.12],
        [wheelR - wheelHalfWidth, wheelF + wheelHalfLength, wheelU + 0.12]
      ], "wheel")
    }
  }

  const roofPoints = roofVertices.map(v => projectLocal(point(v[0], v[1], v[2])))
  const outline = roofPoints.every(Boolean)
    ? [...roofPoints, roofPoints[0]].map(p => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ")
    : ""
  const headingStart = projectLocal(point(0, halfL * 0.68, bodyTop + 0.02))
  const headingEnd = projectLocal(point(0, halfL * 0.98, bodyTop + 0.02))
  const heading = headingStart && headingEnd
    ? headingStart.x.toFixed(1) + "," + headingStart.y.toFixed(1)
      + " " + headingEnd.x.toFixed(1) + "," + headingEnd.y.toFixed(1)
    : ""

  faces.sort((a, b) => b.depth - a.depth)
  return {
    faces,
    outline,
    heading,
    outlineClass: "vehicle-outline" + suffix,
    headingClass: "vehicle-heading" + suffix
  }
}

const vehicles = computed(() => {
  const all = [
    ...(snapshot.value.targets || []).map(track => ({ ...track, candidate: false })),
    ...(snapshot.value.candidates || []).map(candidate => ({ ...candidate, candidate: true }))
  ]
  const seen = new Set()
  const result = []

  for (const track of all) {
    const key = track.sourceVehicleId != null
      ? String(track.sourceVehicleId)
      : String(track.trackId || track.candidateId || "")
    if (!key || seen.has(key)) continue
    seen.add(key)

    const center = track.predictedPosition || track.position
    const projected = projectWorld(center)
    if (!projected || projected.x < -40 || projected.x > 520 || projected.y < -40 || projected.y > 560) continue

    const age = Number(track.dataAge || 0)
    const status = track.candidate && track.visibility !== "visible" ? "unverified"
      : (track.status === "confirmed" && age < 0.75 ? "visible"
        : (track.status === "stale" || age > 1 ? "stale" : "predicted"))
    const sources = track.sourceSensors || []
    const source = sources.some(s => s.sensor === "camera") ? t("carCamera") : t("carSim")
    const model = buildCarModel(center, track.dimensions, track.orientation, status)
    if (!model) continue

    const delta = worldToLocal(center)
    const distance = delta ? Math.sqrt(delta.f * delta.f + delta.r * delta.r) : 0
    result.push({
      id: key,
      faces: model.faces,
      outline: model.outline,
      heading: model.heading,
      outlineClass: model.outlineClass,
      headingClass: model.headingClass,
      state: status === "unverified" ? t("unverified") : t(status),
      source,
      distance: distance.toFixed(1)
    })
  }
  return result.sort((a, b) => Number(b.distance) - Number(a.distance))
})

const diagnosticLine = computed(() => {
  const data = snapshot.value.diagnostics || {}
  const value = key => Number(data[key]) || 0
  return "REG " + value("registryCount")
    + " · Z " + value("cameraZoneCount")
    + " · CAM " + value("cameraCandidateCount")
    + " · TRK " + value("trackedTargetCount")
    + " · DRAW " + vehicles.value.length
})

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
  border-radius: 18px;
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
  border-radius: 24px;
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
.vehicle-body { fill: #278176; fill-opacity: 0.92; stroke: #9aeee0; stroke-width: 0.9; stroke-linejoin: round; }
.vehicle-cabin { fill: #3a9b90; fill-opacity: 0.96; stroke: #b6f5ea; stroke-width: 0.8; stroke-linejoin: round; }
.vehicle-glass { fill: #173b48; fill-opacity: 0.96; stroke: #75c7c5; stroke-width: 0.65; stroke-linejoin: round; }
.vehicle-roof { fill: #4baea2; fill-opacity: 0.96; stroke: #c1fff3; stroke-width: 0.8; stroke-linejoin: round; }
.vehicle-wheel { fill: #111a22; fill-opacity: 0.98; stroke: #70818e; stroke-width: 0.45; }
.vehicle-body-unverified { fill: #9c7834; fill-opacity: 0.52; stroke: #e6c36b; stroke-width: 0.9; stroke-dasharray: 2.5 1.8; stroke-linejoin: round; }
.vehicle-cabin-unverified { fill: #9a793e; fill-opacity: 0.58; stroke: #f0d38a; stroke-width: 0.8; stroke-dasharray: 2.5 1.8; }
.vehicle-glass-unverified { fill: #51452b; fill-opacity: 0.72; stroke: #dfbd70; stroke-width: 0.65; }
.vehicle-roof-unverified { fill: #c29b4e; fill-opacity: 0.62; stroke: #ffe3a0; stroke-width: 0.8; stroke-dasharray: 2.5 1.8; }
.vehicle-wheel-unverified { fill: #261f15; fill-opacity: 0.88; stroke: #d8b86f; stroke-width: 0.45; }
.vehicle-body-predicted, .vehicle-body-stale { fill: #5c6872; fill-opacity: 0.60; stroke: #a6b0b8; stroke-width: 0.8; stroke-dasharray: 2 2; }
.vehicle-cabin-predicted, .vehicle-cabin-stale { fill: #687987; fill-opacity: 0.68; stroke: #b1c0ca; stroke-width: 0.7; stroke-dasharray: 2 2; }
.vehicle-glass-predicted, .vehicle-glass-stale { fill: #263642; fill-opacity: 0.82; stroke: #8295a3; stroke-width: 0.6; }
.vehicle-roof-predicted, .vehicle-roof-stale { fill: #8495a2; fill-opacity: 0.64; stroke: #c1cbd1; stroke-width: 0.7; stroke-dasharray: 2 2; }
.vehicle-wheel-predicted, .vehicle-wheel-stale { fill: #111a22; stroke: #8795a2; stroke-width: 0.45; }
.vehicle-outline { fill: none; stroke: #d4fff5; stroke-width: 0.85; stroke-linejoin: round; }
.vehicle-outline-unverified { fill: none; stroke: #f5dda0; stroke-width: 0.8; stroke-dasharray: 2 2; }
.vehicle-outline-predicted, .vehicle-outline-stale { fill: none; stroke: #aab7c0; stroke-width: 0.8; stroke-dasharray: 2 2; }
.vehicle-heading { fill: none; stroke: #f4fff9; stroke-width: 1.25; stroke-linecap: round; }
.vehicle-heading-unverified { fill: none; stroke: #ffe8ae; stroke-width: 1.1; stroke-dasharray: 2 1.5; }
.vehicle-heading-predicted, .vehicle-heading-stale { fill: none; stroke: #b2c0c8; stroke-width: 1; stroke-dasharray: 2 1.5; }
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
