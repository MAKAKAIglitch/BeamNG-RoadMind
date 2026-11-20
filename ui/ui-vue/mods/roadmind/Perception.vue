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
      <svg viewBox="0 0 480 520" role="img" :aria-label="t('scene')">
        <defs>
          <pattern id="roadmind-perception-grid" width="32" height="32" patternUnits="userSpaceOnUse">
            <path d="M 32 0 L 0 0 0 32" fill="none" stroke="#202a35" stroke-width="1" />
          </pattern>
        </defs>
        <rect width="480" height="520" fill="url(#roadmind-perception-grid)" />
        <path v-if="roadPath" :d="roadPath" fill="none" stroke="#526273" stroke-width="62" opacity="0.28" stroke-linecap="round" />
        <path v-if="roadPath" :d="roadPath" fill="none" stroke="#9aa8b5" stroke-width="1.5" stroke-dasharray="5 7" />
        <path v-if="routePath" :d="routePath" fill="none" stroke="#65cdbb" stroke-width="2.2" stroke-dasharray="7 5" />
        <path v-if="predictionPath" :d="predictionPath" fill="none" stroke="#62dec2"
          stroke-width="2.8" stroke-linecap="round" stroke-dasharray="5 5" />
        <circle v-if="brakeMarker" :cx="brakeMarker.x" :cy="brakeMarker.y" r="7"
          :fill="guidance.status === 'brake_now' ? '#ff655f' : '#f2bb61'"
          stroke="#111a22" stroke-width="2" />
        <g v-for="o in obstacles" :key="o.id">
          <circle :cx="o.x" :cy="o.y" r="3.5" fill="#8997a5" fill-opacity="0.35" stroke="#aab6c0" stroke-width="1" />
        </g>
        <g v-for="car in vehicles" :key="car.id"
          :transform="'translate(' + car.x + ' ' + car.y + ') rotate(' + car.angle + ')'">
          <rect :x="-car.w / 2" :y="-car.l / 2" :width="car.w" :height="car.l"
            rx="3" :fill="car.fill" :stroke="car.stroke" stroke-width="1.4" />
          <path d="M -4 -5 L 0 -11 L 4 -5" fill="none" stroke="#e6edf5" stroke-width="1" />
          <text x="0" :y="car.l / 2 + 11" text-anchor="middle" class="object-label">{{ car.label }}</text>
        </g>
        <g transform="translate(240 430)">
          <rect x="-12" y="-25" width="24" height="50" rx="7" fill="#d9e6ef" stroke="#f4f8fb" stroke-width="1.6" />
          <path d="M -8 -13 L 8 -13 M -8 13 L 8 13" stroke="#5f7487" stroke-width="2" />
        </g>
        <text x="240" y="474" text-anchor="middle" class="ego-label">{{ t("ego") }}</text>
        <text x="14" y="22" class="overlay-label">{{ t("scale") }}: {{ mode.toUpperCase() }}</text>
        <text x="466" y="22" text-anchor="end" class="overlay-label">{{ t("road") }}: {{ roadQuality }}</text>
      </svg>
      <div v-if="!snapshot.vehicle" class="waiting">{{ t("waiting") }}</div>
    </div>

    <div class="telemetry" v-if="snapshot.motion">
      <div class="telemetry-main">
        <strong>{{ speedLabel }}</strong>
        <span>{{ t("acceleration") }}: {{ accelerationLabel }}</span>
        <span>{{ t("lateral") }}: {{ lateralLabel }}</span>
      </div>
      <div class="guidance" :class="guidance.status">
        <strong>{{ guidanceLabel }}</strong>
        <span v-if="guidance.turnDirection">{{ guidance.turnDirection === 'left' ? t("leftTurn") : t("rightTurn") }}</span>
        <span v-if="guidance.targetSpeedMps">{{ t("targetSpeed") }}: {{ Math.round(guidance.targetSpeedMps * 3.6) }} km/h</span>
        <span v-if="guidance.distanceToBrakePointM !== undefined">{{ t("brakeIn") }}: {{ guidance.distanceToBrakePointM.toFixed(0) }} m</span>
        <span v-if="snapshot.motion.warning">{{ t("warning") }}: {{ snapshot.motion.warning.toUpperCase() }}</span>
      </div>
    </div>

    <div class="legend">
      <span><i class="car-dot"></i>{{ t("vehicles") }}: {{ vehicles.length }}</span>
      <span><i class="obstacle-dot"></i>{{ t("obstacles") }}: {{ obstacles.length }}</span>
      <span><i class="road-dot"></i>{{ t("navgraph") }}</span>
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
      <div v-for="o in obstacles.slice(0, 4)" :key="o.id" class="object-row">
        <span class="obstacle-icon"></span>
        <div class="object-main">
          <strong>{{ t("obstacle") }}</strong>
          <span>{{ o.source }}</span>
        </div>
        <div class="object-info">
          <strong>{{ t("measured") }}</strong>
          <span>{{ o.distance }} m</span>
        </div>
      </div>
    </div>

    <footer>
      <span>{{ lastError ? t("reconnecting") : t("disclaimer") }}</span>
      <button @click="refresh">{{ t("refresh") }}</button>
    </footer>
  </div>
</template>

<script setup>
import { computed, onMounted, onBeforeUnmount, ref } from "vue"
import { runRaw } from "@/bridge/libs/Lua.js"

const EN = {
  title: "Environment model",
  subtitle: "Cached world model · lightweight 2D display",
  scene: "Top-down perception view",
  top: "Top view",
  follow: "Follow",
  ego: "EGO",
  scale: "View",
  road: "NavGraph",
  vehicles: "Vehicles",
  obstacles: "Obstacles",
  navgraph: "Road centreline",
  age: "Frame age",
  detected: "TRACKED OBJECTS",
  obstacle: "Obstacle cluster",
  measured: "MEASURED",
  empty: "No current objects in the World Model.",
  waiting: "Waiting for RoadMind vehicle data",
  acceleration: "Acceleration",
  lateral: "Lateral load",
  targetSpeed: "Curve speed",
  brakeIn: "Brake point",
  warning: "ADAS",
  guidanceBrake: "BRAKE NOW",
  guidancePrepare: "SLOW FOR CURVE",
  guidanceClear: "ROAD PREVIEW CLEAR",
  guidanceWaiting: "ROAD DATA UNAVAILABLE",
  leftTurn: "LEFT TURN",
  rightTurn: "RIGHT TURN",
  reconnecting: "RoadMind connection unavailable · keeping the last valid view",
  visible: "VISIBLE",
  predicted: "PREDICTED",
  stale: "STALE",
  carCamera: "Camera visibility model",
  carSim: "BeamNG vehicle bounds",
  refresh: "Refresh",
  disclaimer: "Drive mode uses simulator-assisted vehicle tracks. NavGraph is a road centreline, not painted lane markings."
}
const RU = {
  title: "Модель окружения",
  subtitle: "Кэш модели мира · лёгкая 2D-визуализация",
  scene: "Вид сверху на обнаруженные объекты",
  top: "Сверху",
  follow: "Следование",
  ego: "АВТО",
  scale: "Режим",
  road: "NavGraph",
  vehicles: "Машины",
  obstacles: "Препятствия",
  navgraph: "Ось дороги",
  age: "Возраст кадра",
  detected: "ОТСЛЕЖИВАЕМЫЕ ОБЪЕКТЫ",
  obstacle: "Кластер препятствий",
  measured: "ИЗМЕРЕНО",
  empty: "В World Model пока нет объектов.",
  waiting: "Ожидание данных автомобиля RoadMind",
  acceleration: "Ускорение",
  lateral: "Боковая нагрузка",
  targetSpeed: "Скорость в повороте",
  brakeIn: "Точка торможения",
  warning: "ADAS",
  guidanceBrake: "ПОРА ТОРМОЗИТЬ",
  guidancePrepare: "СНИЗИТЬ СКОРОСТЬ К ПОВОРОТУ",
  guidanceClear: "ТРАЕКТОРИЯ СВОБОДНА",
  guidanceWaiting: "НЕТ ДАННЫХ О ДОРОГЕ",
  leftTurn: "ПОВОРОТ НАЛЕВО",
  rightTurn: "ПОВОРОТ НАПРАВО",
  reconnecting: "Нет связи с RoadMind · показаны последние данные",
  visible: "ВИДЕН",
  predicted: "ПРОГНОЗ",
  stale: "УСТАРЕЛ",
  carCamera: "Модель видимости камер",
  carSim: "Габариты машины BeamNG",
  refresh: "Обновить",
  disclaimer: "В Drive машины отслеживаются по данным симулятора. NavGraph показывает ось дороги, а не разметку полос."
}

const locale = ref("en-US")
const mode = ref("top")
const loading = ref(false)
const snapshot = ref({ vehicle: null, targets: [], obstacles: [], road: {}, route: [], motion: null })
const lastError = ref("")
let timer = null
let disposed = false

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

function project(p) {
  const v = snapshot.value.vehicle
  if (!v || !p || !v.position || !v.forward || !v.right) return null
  const dx = Number(p.x) - Number(v.position.x)
  const dy = Number(p.y) - Number(v.position.y)
  const dz = Number(p.z || 0) - Number(v.position.z || 0)
  const f = dx * v.forward.x + dy * v.forward.y + dz * (v.forward.z || 0)
  const r = dx * v.right.x + dy * v.right.y + dz * (v.right.z || 0)
  const k = mode.value === "follow" ? Math.max(1.7, 5.2 - Math.max(0, f) * 0.035) : 4.0
  return { x: 240 + r * k, y: 430 - f * k, f, r, k }
}
function toPath(points) {
  const pts = (points || []).map(project).filter(Boolean)
    .filter(p => p.x >= -40 && p.x <= 520 && p.y >= -40 && p.y <= 560)
  if (pts.length < 2) return ""
  return pts.map((p, i) => (i ? "L " : "M ") + p.x.toFixed(1) + " " + p.y.toFixed(1)).join(" ")
}
const roadPath = computed(() => toPath(snapshot.value.road && snapshot.value.road.corridor || []))
const routePath = computed(() => toPath(snapshot.value.route || []))
const predictionPath = computed(() => toPath(
  snapshot.value.motion && snapshot.value.motion.predictedPath || []
))
const guidance = computed(() => snapshot.value.motion && snapshot.value.motion.guidance || {})
const brakeMarker = computed(() =>
  (guidance.value.status === "prepare" || guidance.value.status === "brake_now")
    && guidance.value.brakePoint ? project(guidance.value.brakePoint) : null
)
const motion = computed(() => snapshot.value.motion || {})
const speedLabel = computed(() => Number.isFinite(Number(motion.value.speedKph))
  ? Math.round(Number(motion.value.speedKph)) + " km/h" : "—")
const accelerationLabel = computed(() => Number.isFinite(Number(motion.value.accelerationMps2))
  ? (Number(motion.value.accelerationMps2) > 0 ? "+" : "") +
    Number(motion.value.accelerationMps2).toFixed(1) + " m/s²" : "—")
const lateralLabel = computed(() => Number.isFinite(Number(motion.value.lateralG))
  ? Number(motion.value.lateralG).toFixed(2) + " g" : "—")
const guidanceLabel = computed(() => {
  const status = guidance.value.status
  if (status === "brake_now") return t("guidanceBrake")
  if (status === "prepare") return t("guidancePrepare")
  if (status === "clear") return t("guidanceClear")
  return t("guidanceWaiting")
})
const vehicles = computed(() => (snapshot.value.targets || []).map(track => {
  const p = project(track.predictedPosition || track.position)
  if (!p || p.x < -35 || p.x > 515 || p.y < -35 || p.y > 555) return null
  const d = track.dimensions || {}
  const w = Math.max(8, Math.min(25, Number(d.width || 1.8) * p.k))
  const l = Math.max(14, Math.min(42, Number(d.length || 4.4) * p.k))
  const heading = track.orientation && track.orientation.forward
  let angle = 0
  if (heading && snapshot.value.vehicle) {
    const v = snapshot.value.vehicle
    angle = Math.atan2(heading.x * v.right.x + heading.y * v.right.y,
      heading.x * v.forward.x + heading.y * v.forward.y) * 180 / Math.PI
  }
  const age = Number(track.dataAge || 0)
  const state = track.status === "confirmed" && age < 0.55 ? "visible"
    : (track.status === "stale" || age > 1 ? "stale" : "predicted")
  const sources = track.sourceSensors || []
  const source = sources.some(s => s.sensor === "camera") ? t("carCamera") : t("carSim")
  return {
    id: track.trackId || "vehicle",
    x: p.x, y: p.y, w, l, angle,
    label: state === "visible" ? "CAR" : state === "stale" ? "STALE" : "PRED",
    state: t(state),
    source,
    fill: state === "visible" ? "#245d59" : state === "stale" ? "#535d69" : "#65552f",
    stroke: state === "visible" ? "#71d8c5" : state === "stale" ? "#8b98a5" : "#e6c36b",
    distance: Math.sqrt(p.f * p.f + p.r * p.r).toFixed(1)
  }
}).filter(Boolean))
const obstacles = computed(() => (snapshot.value.obstacles || []).map(o => {
  const p = project(o.position)
  if (!p || p.x < -30 || p.x > 510 || p.y < -30 || p.y > 550) return null
  const sensors = (o.sourceSensors || []).map(s => s.sensor).filter(Boolean)
  return {
    id: o.obstacleId || p.x.toFixed(0) + "-" + p.y.toFixed(0),
    x: p.x, y: p.y,
    source: [...new Set(sensors)].join(" / ") || "LiDAR / scene",
    distance: Math.sqrt(p.f * p.f + p.r * p.r).toFixed(1)
  }
}).filter(Boolean))

async function setPerceptionView(enabled) {
  const cmd = "(function() " +
    "local ext = extensions and extensions.roadmind; " +
    "if type(ext) ~= 'table' then " +
      "local ok = pcall(function() extensions.load('roadmind') end); " +
      "if not ok then return {ok=false,error='RoadMind extension load failed'} end; " +
      "ext = extensions and extensions.roadmind; " +
    "end; " +
    "if type(ext) ~= 'table' or type(ext.setPerceptionViewEnabled) ~= 'function' then " +
      "return {ok=false,error='RoadMind perception API unavailable'} end; " +
    "local ok, result = pcall(function() return ext.setPerceptionViewEnabled(" +
      (enabled ? "true" : "false") + ") end); " +
    "if not ok then return {ok=false,error=tostring(result)} end; " +
    "return result " +
  "end)()"
  try {
    const result = await runRaw(cmd)
    if (result && result.ok === false) throw new Error(result.error || "RoadMind error")
  } catch (error) {
    if (enabled) lastError.value = String(error && error.message || error)
  }
}

async function refresh() {
  if (loading.value || disposed) return
  loading.value = true
  try {
    const cmd = "(function() " +
      "local ext = extensions and extensions.roadmind; " +
      "if type(ext) ~= 'table' then " +
        "local ok = pcall(function() extensions.load('roadmind') end); " +
        "if not ok then return nil end; ext = extensions and extensions.roadmind; " +
      "end; " +
      "if type(ext) ~= 'table' or type(ext.getPerceptionSnapshot) ~= 'function' then return nil end; " +
      "local ok, value = pcall(function() return ext.getPerceptionSnapshot() end); " +
      "if not ok then return nil end; return value " +
    "end)()"
    let result = await runRaw(cmd)
    if (typeof result === "string") {
      try { result = JSON.parse(result) } catch (_) { result = null }
    }
    if (result && result.result !== undefined && result.schemaVersion === undefined) {
      result = result.result
      if (typeof result === "string") {
        try { result = JSON.parse(result) } catch (_) { result = null }
      }
    }
    if (!result || result.schemaVersion !== 1 || !result.vehicle) {
      throw new Error("No valid vehicle perception snapshot")
    }
    if (disposed) return
    if (result.language) locale.value = result.language
    snapshot.value = result
    lastError.value = ""
  } catch (error) {
    if (!disposed) {
      lastError.value = String(error && error.message || error)
      console.warn("RoadMind perception snapshot unavailable; keeping previous view", error)
    }
  } finally {
    loading.value = false
  }
}
onMounted(async () => {
  await setPerceptionView(true)
  if (disposed) return
  await refresh()
  timer = setInterval(refresh, 250)
})
onBeforeUnmount(() => {
  disposed = true
  if (timer) clearInterval(timer)
  setPerceptionView(false)
})
</script>

<style scoped lang="scss">
.perception { height: 78vh; min-height: 460px; display: flex; flex-direction: column; gap: 0.55rem; padding: 0.7rem; overflow: hidden; color: #d8e1e9; background: #10161d; }
.header { display: flex; align-items: center; justify-content: space-between; gap: 0.8rem; }
.kicker { color: #78cbbb; font-size: 0.65rem; letter-spacing: 0.16em; font-weight: 700; }
.header h2 { margin: 0.1rem 0; font-size: 1.05rem; font-weight: 650; }
.header p { margin: 0; color: #81909e; font-size: 0.72rem; }
.switcher { display: flex; gap: 0.35rem; }
.switcher button, footer button { border: 1px solid #33414d; background: #18212a; color: #bfcbd4; border-radius: 4px; padding: 0.35rem 0.6rem; font: inherit; font-size: 0.72rem; }
.switcher button.active { border-color: #66c5b4; background: #1c3b3a; color: #c8fff3; }
.scene { position: relative; flex: 1 1 auto; min-height: 260px; overflow: hidden; border: 1px solid #293642; border-radius: 6px; background: #111a22; }
.scene svg { display: block; width: 100%; height: 100%; }
.overlay-label { fill: #8093a3; font-size: 9px; font-family: sans-serif; }
.ego-label { fill: #a9bac8; font-size: 9px; font-weight: 700; }
.object-label { fill: #cad5de; font-size: 7px; font-weight: 600; }
.waiting { position: absolute; inset: 0; display: grid; place-items: center; color: #9aaab8; font-size: 0.85rem; background: rgba(13,19,25,0.72); }
.telemetry { display: flex; flex-wrap: wrap; align-items: stretch; gap: 0.45rem; }
.telemetry-main, .guidance { display: flex; align-items: center; flex-wrap: wrap; gap: 0.35rem 0.75rem; min-height: 34px; padding: 0.4rem 0.55rem; border: 1px solid #2b3d48; border-radius: 5px; background: #141f28; color: #9fb1bd; font-size: 0.68rem; font-variant-numeric: tabular-nums; }
.telemetry-main strong { color: #e1edf2; font-size: 1rem; }
.guidance { flex: 1; border-color: #385047; }
.guidance strong { color: #83d7c5; letter-spacing: 0.04em; }
.guidance.prepare strong { color: #f2bb61; }
.guidance.brake_now { border-color: #74443e; background: #2a1b1c; }
.guidance.brake_now strong { color: #ff8178; }
.guidance.unavailable strong, .guidance.waiting strong { color: #98a5af; }
.legend { display: flex; flex-wrap: wrap; gap: 0.6rem 1rem; color: #9aabb9; font-size: 0.7rem; }
.legend span { display: inline-flex; align-items: center; gap: 0.35rem; }
.legend i { display: inline-block; width: 8px; height: 8px; border-radius: 2px; }
.car-dot { background: #70d7c3; }
.obstacle-dot { background: #9caab7; border-radius: 50% !important; }
.road-dot { background: #657687; }
.age { margin-left: auto; font-variant-numeric: tabular-nums; }
.objects { display: flex; flex-direction: column; gap: 0.25rem; max-height: 145px; overflow-y: auto; }
.list-title { color: #718493; font-size: 0.65rem; letter-spacing: 0.14em; font-weight: 700; padding: 0.25rem 0; }
.object-row { display: flex; align-items: center; gap: 0.55rem; min-height: 29px; padding: 0.25rem 0.45rem; border: 1px solid #26333e; border-radius: 4px; background: #151e27; }
.vehicle-icon, .obstacle-icon { width: 13px; height: 20px; border: 1px solid #75cfc0; border-radius: 4px; background: #245d59; }
.obstacle-icon { width: 11px; height: 11px; border-color: #9caab7; border-radius: 2px; background: #687787; }
.object-main { display: flex; flex-direction: column; flex: 1; min-width: 0; }
.object-main strong { color: #d9e4eb; font-size: 0.72rem; }
.object-main span, .object-info span { color: #8293a2; font-size: 0.65rem; }
.object-info { display: flex; flex-direction: column; align-items: flex-end; }
.object-info strong { color: #83d3c2; font-size: 0.65rem; }
.empty { padding: 0.5rem; color: #8293a2; font-size: 0.75rem; }
footer { display: flex; align-items: center; justify-content: space-between; gap: 0.6rem; color: #7d8e9d; font-size: 0.64rem; }
footer button { flex-shrink: 0; }
@media (max-width: 600px) { .perception { height: 76vh; padding: 0.35rem; } .header p { max-width: 13rem; } }
</style>
