<template>
  <div class="roadmind-adas">
    <BngCardHeading>RoadMind ADAS</BngCardHeading>
    <p class="note">{{ t("intro") }}</p>

    <section class="section">
      <BngCardHeading>{{ t("assistance") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("master") }}</template>
        <BngSwitch :modelValue="settings.adasEnabled" :disabled="busy"
          @update:modelValue="v => setSetting('adasEnabled', v)" />
      </BngRow>
      <BngRow v-for="item in options" :key="item.key">
        <template #label>{{ t(item.label) }}</template>
        <BngSwitch :modelValue="settings[item.key]" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="v => setSetting(item.key, v)" />
      </BngRow>
      <p class="note">{{ t("masterNote") }}</p>
      <div v-if="settings.adasEnabled" class="status">
        <strong>{{ t("status") }}: {{ t("status_" + ((status.adasStatus && status.adasStatus.status) || "standby")) }}</strong>
        <span v-if="status.adasStatus && status.adasStatus.fcw" class="alert">{{ t("fcw") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.aeb" class="alert">{{ t("aeb") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.leftBlindSpot" class="note">{{ t("left") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.rightBlindSpot" class="note">{{ t("right") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.laneDeparture" class="alert">{{ t("lane") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.aebDistance !== undefined" class="metric">
          {{ t("aebDistance") }}: {{ Number(status.adasStatus.aebDistance).toFixed(1) }} m
          / {{ t("aebThreshold") }}: {{ Number(status.adasStatus.aebThreshold).toFixed(1) }} m
        </span>
        <span v-if="status.adasStatus && status.adasStatus.aebTimeToCollision !== undefined" class="metric">
          {{ t("ttc") }}: {{ Number(status.adasStatus.aebTimeToCollision).toFixed(2) }} s
          · {{ t("closingSpeed") }}: {{ Number(status.adasStatus.aebClosingSpeed || 0).toFixed(1) }} m/s
        </span>
        <span v-if="status.adasStatus && status.adasStatus.aebHazardKind" class="metric">
          {{ t("hazardSource") }}: {{ status.adasStatus.aebHazardSource || status.adasStatus.aebHazardKind }}
        </span>
        <span class="metric">
          {{ t("forwardSource") }}: {{ status.adasStatus && status.adasStatus.cameraAvailable ? t("camera") : (status.adasStatus && status.adasStatus.sensorAvailable ? t("physicalSensors") : t("noForwardSource")) }}
        </span>
        <span v-if="status.adasStatus && status.adasStatus.leadDistance !== undefined" class="metric">
          {{ t("lead") }}: {{ Number(status.adasStatus.leadDistance).toFixed(1) }} m
        </span>
      </div>
    </section>

    <section class="section">
      <BngCardHeading>{{ t("autopilot") }}</BngCardHeading>
      <p class="note">{{ t("routeDescription") }}</p>
      <label for="roadmind-adas-route">{{ t("routePoints") }}</label>
      <textarea id="roadmind-adas-route" v-model="routeText" rows="3" class="route"
        :placeholder="t('routePlaceholder')" :disabled="busy"></textarea>
      <div class="actions">
        <BngButton :accent="ACCENTS.main" :disabled="busy" @click="applyRoute">{{ t("apply") }}</BngButton>
        <BngButton :accent="ACCENTS.outlined" :disabled="busy" @click="clearRoute">{{ t("clear") }}</BngButton>
      </div>
      <div class="runtime">
        <div>
          <strong>{{ status.routePilotEnabled ? (status.routeSafetyStopped ? t("paused") : t("running")) : t("stopped") }}</strong>
          <span class="metric">{{ t("progress") }}: {{ status.routeProgressPercent || 0 }}%</span>
          <span class="metric">{{ t("planner") }}: {{ status.plannerStatus || "idle" }}</span>
        </div>
        <BngButton :accent="status.routePilotEnabled ? ACCENTS.outlined : ACCENTS.main"
          :disabled="busy" @click="togglePilot">
          {{ status.routePilotEnabled ? t("stopPilot") : t("startPilot") }}
        </BngButton>
      </div>
      <p v-if="status.routePilotError" class="alert">{{ routeError(status.routePilotError) }}</p>
      <p class="note">{{ t("safetyNote") }}</p>
    </section>

    <section class="section">
      <BngCardHeading>{{ t("handover") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("takeover") }}</template>
        <BngSwitch :modelValue="settings.driverTakeoverEnabled !== false"
          :disabled="busy || !status.routePilotEnabled"
          @update:modelValue="v => setSetting('driverTakeoverEnabled', v)" />
      </BngRow>
      <p class="note">{{ t("takeoverNote") }}</p>
      <p class="note">{{ t("futureNote") }}</p>
    </section>

    <div class="footer">
      <span v-if="error" class="alert">{{ error }}</span>
      <span v-else class="note">{{ t("saved") }}</span>
      <BngButton :accent="ACCENTS.outlined" :disabled="busy" @click="refresh">{{ t("refresh") }}</BngButton>
    </div>
  </div>
</template>

<script setup>
import { onMounted, onBeforeUnmount, ref } from "vue"
import { runRaw, serialize } from "@/bridge/libs/Lua.js"
import { BngButton, BngSwitch, BngRow, BngCardHeading, ACCENTS } from "@/common/components/base"

const EN = {
  intro: "Advanced Driver Assistance Systems. Assistance and route-pilot controls are separate from core sensor settings.",
  assistance: "Driver assistance",
  master: "Enable ADAS",
  masterNote: "ADAS requires active scanning and fresh observations.",
  fcw: "Forward collision warning (FCW)",
  aeb: "Automatic emergency braking (AEB)",
  acc: "Adaptive cruise control (ACC — route pilot)",
  bsm: "Blind-spot monitoring (BSM)",
  parking: "Parking distance warning",
  ldw: "Lane-departure warning (LDW)",
  status: "ADAS status",
  status_disabled: "Off",
  status_standby: "Standby",
  status_waiting_for_sensors: "Waiting for sensor data",
  status_active: "Monitoring",
  status_fcw: "Collision warning",
  status_aeb: "Emergency braking",
  status_blind_spot: "Vehicle in blind spot",
  status_lane_departure: "Lane departure risk",
  left: "Left blind spot",
  right: "Right blind spot",
  lane: "Lane departure risk",
  aebDistance: "Hazard distance",
  aebThreshold: "Braking threshold",
  ttc: "Time to collision",
  closingSpeed: "Closing speed",
  hazardSource: "Detected by",
  forwardSource: "Forward perception",
  camera: "Cameras",
  physicalSensors: "LiDAR / radar / ultrasonic",
  noForwardSource: "No fresh source",
  lead: "Lead vehicle gap",
  autopilot: "Road-network autopilot (experimental)",
  routeDescription: "The route pilot follows a NavGraph route using RoadMind's Lua controller. Target speed is currently capped at 8 m/s (28.8 km/h).",
  routePoints: "Optional route points (world X, Y, Z; one per line)",
  routePlaceholder: "X, Y, Z\nX, Y, Z",
  apply: "Apply route",
  clear: "Clear route",
  startPilot: "Start autopilot",
  stopPilot: "Stop autopilot",
  running: "Autopilot running",
  paused: "Safety stop",
  stopped: "Autopilot off",
  progress: "Progress",
  planner: "Planner",
  safetyNote: "Experimental controller; use only in a controlled test area. This is not a certified collision-avoidance system.",
  handover: "Driver takeover",
  takeover: "Return control on steering, throttle or brake input",
  takeoverNote: "Enabled by default. Manual input releases RoadMind's actuator outputs and disengages route pilot. Camera/view movement is ignored.",
  futureNote: "Lane centering, automatic parking, ABS/ESC/TCS integration and manual-driving ACC are not active features yet.",
  saved: "Settings save automatically.",
  refresh: "Refresh",
  routeFormat: "Enter 1–12 lines with three numbers per line: X, Y, Z.",
  routeGeneric: "Could not start or continue the route pilot.",
  savedError: "Could not contact RoadMind."
}
const RU = {
  intro: "Системы помощи водителю. Настройки ADAS и автопилота отделены от основных настроек датчиков.",
  assistance: "Помощь водителю",
  master: "Включить ADAS",
  masterNote: "Для ADAS нужны активное сканирование и свежие данные.",
  fcw: "Предупреждение о столкновении (FCW)",
  aeb: "Автоматическое экстренное торможение (AEB)",
  acc: "Адаптивный круиз-контроль (ACC — автопилот)",
  bsm: "Контроль слепых зон (BSM)",
  parking: "Предупреждение при парковке",
  ldw: "Предупреждение о сходе с полосы (LDW)",
  status: "Состояние ADAS",
  status_disabled: "Выключено",
  status_standby: "Ожидание",
  status_waiting_for_sensors: "Ожидание данных датчиков",
  status_active: "Мониторинг",
  status_fcw: "Предупреждение о столкновении",
  status_aeb: "Экстренное торможение",
  status_blind_spot: "Машина в слепой зоне",
  status_lane_departure: "Риск ухода с полосы",
  left: "Слева слепая зона",
  right: "Справа слепая зона",
  lane: "Риск ухода с полосы",
  aebDistance: "Дистанция до цели",
  aebThreshold: "Порог торможения",
  ttc: "Время до столкновения",
  closingSpeed: "Скорость сближения",
  hazardSource: "Обнаружено через",
  forwardSource: "Переднее восприятие",
  camera: "Камеры",
  physicalSensors: "LiDAR / радар / ультразвук",
  noForwardSource: "Нет свежих данных",
  lead: "Дистанция до машины впереди",
  autopilot: "Автопилот по дорожной сети (эксперимент)",
  routeDescription: "Автопилот следует по NavGraph собственным Lua-контроллером RoadMind. Целевая скорость ограничена 8 м/с (28,8 км/ч).",
  routePoints: "Необязательные точки маршрута (мировые X, Y, Z; одна точка на строку)",
  routePlaceholder: "X, Y, Z\nX, Y, Z",
  apply: "Применить маршрут",
  clear: "Очистить маршрут",
  startPilot: "Включить автопилот",
  stopPilot: "Остановить автопилот",
  running: "Автопилот включён",
  paused: "Остановка защиты",
  stopped: "Автопилот выключен",
  progress: "Прогресс",
  planner: "Планировщик",
  safetyNote: "Экспериментальный контроллер. Проверяй его только в контролируемой зоне; это не сертифицированная система предотвращения столкновений.",
  handover: "Передача управления водителю",
  takeover: "Передавать управление при рулении, газе или тормозе",
  takeoverNote: "По умолчанию включено. При ручном вводе RoadMind отпускает свои команды и выключает автопилот. Управление камерой игнорируется.",
  futureNote: "Удержание по центру полосы, автопарковка, интеграция ABS/ESC/TCS и ручной ACC пока не реализованы.",
  saved: "Настройки сохраняются автоматически.",
  refresh: "Обновить",
  routeFormat: "Введи от 1 до 12 строк по три числа: X, Y, Z.",
  routeGeneric: "Не удалось запустить или продолжить автопилот.",
  savedError: "Не удалось связаться с RoadMind."
}

const options = [
  { key: "adasFCWEnabled", label: "fcw" },
  { key: "adasAEBEnabled", label: "aeb" },
  { key: "adasACCEnabled", label: "acc" },
  { key: "adasBSMEnabled", label: "bsm" },
  { key: "adasParkingEnabled", label: "parking" },
  { key: "adasLDWEnabled", label: "ldw" }
]
const settings = ref({
  adasEnabled: false,
  adasFCWEnabled: true,
  adasAEBEnabled: true,
  adasACCEnabled: false,
  adasBSMEnabled: true,
  adasParkingEnabled: true,
  adasLDWEnabled: true,
  driverTakeoverEnabled: true
})
const status = ref({
  adasStatus: { status: "disabled" },
  routePilotEnabled: false,
  routeSafetyStopped: false,
  routeProgressPercent: 0,
  plannerStatus: "idle"
})
const locale = ref("en-US")
const error = ref("")
const busy = ref(false)
const routeText = ref("")
const methods = new Set(["getSettings", "setSetting", "setRoutePilotEnabled", "setRoutePoints", "clearRoutePoints"])
let refreshTimer = null

function t(key) {
  const dict = String(locale.value).toLowerCase().startsWith("ru") ? RU : EN
  return dict[key] || EN[key] || key
}
function callRoadMind(method, args = []) {
  if (!methods.has(method)) return Promise.reject(new Error("Unsupported RoadMind method: " + method))
  const serializedArgs = args.map(value => serialize(value)).join(", ")
  const command = "(function() " +
    "local ext = extensions and extensions.roadmind; " +
    "if type(ext) ~= 'table' then " +
      "local loaded = pcall(function() extensions.load('roadmind') end); " +
      "if not loaded then return {ok=false,error='RoadMind extension load failed'} end; " +
      "ext = extensions and extensions.roadmind; " +
    "end; " +
    "if type(ext) ~= 'table' then return {ok=false,error='RoadMind extension unavailable'} end; " +
    "local fn = ext[" + JSON.stringify(method) + "]; " +
    "if type(fn) ~= 'function' then return {ok=false,error='RoadMind method unavailable'} end; " +
    "local ok, result = pcall(function() return fn(" + serializedArgs + ") end); " +
    "if not ok then return {ok=false,error=tostring(result)} end; " +
    "return result " +
  "end)()"
  return runRaw(command)
}
function accept(result) {
  if (!result || result.ok === false) {
    error.value = result && result.error ? routeError(result.error) : t("savedError")
    return false
  }
  if (result.settings) settings.value = { ...settings.value, ...result.settings }
  if (result.status) status.value = { ...status.value, ...result.status }
  if (result.language) locale.value = result.language
  error.value = ""
  return true
}
function routeError(message) {
  const value = String(message || "")
  if (!String(locale.value).toLowerCase().startsWith("ru")) return value || t("routeGeneric")
  if (value.startsWith("Local planner:")) return "Планировщик не подтвердил безопасную траекторию."
  if (value === "The active vehicle changed") return "Активный автомобиль изменился."
  if (value === "Route localization lost") return "Потеряна привязка к маршруту."
  if (value === "NavGraph route API is unavailable on this map") return "На этой карте недоступен NavGraph."
  return value || t("routeGeneric")
}
async function refresh() {
  busy.value = true
  try {
    accept(await callRoadMind("getSettings"))
  } catch (e) {
    error.value = t("savedError")
  } finally {
    busy.value = false
  }
}
async function setSetting(key, value) {
  try {
    accept(await callRoadMind("setSetting", [key, value]))
  } catch (e) {
    error.value = t("savedError")
  }
}
async function applyRoute() {
  const lines = routeText.value.split(/\r?\n/).map(line => line.trim()).filter(Boolean)
  if (lines.length < 1 || lines.length > 12) {
    error.value = t("routeFormat")
    return
  }
  const points = []
  for (const line of lines) {
    const nums = line.split(/[,\s;]+/).filter(Boolean).map(Number)
    if (nums.length !== 3 || nums.some(v => !Number.isFinite(v))) {
      error.value = t("routeFormat")
      return
    }
    points.push({ x: nums[0], y: nums[1], z: nums[2] })
  }
  busy.value = true
  try {
    accept(await callRoadMind("setRoutePoints", [JSON.stringify(points)]))
  } catch (e) {
    error.value = t("savedError")
  } finally {
    busy.value = false
  }
}
async function clearRoute() {
  busy.value = true
  try {
    accept(await callRoadMind("clearRoutePoints"))
    routeText.value = ""
  } catch (e) {
    error.value = t("savedError")
  } finally {
    busy.value = false
  }
}
async function togglePilot() {
  busy.value = true
  try {
    accept(await callRoadMind("setRoutePilotEnabled", [!status.value.routePilotEnabled]))
  } catch (e) {
    error.value = t("savedError")
  } finally {
    busy.value = false
  }
}
onMounted(() => {
  refresh()
  refreshTimer = setInterval(() => { if (!busy.value) refresh() }, 1500)
})
onBeforeUnmount(() => {
  if (refreshTimer) clearInterval(refreshTimer)
})
</script>

<style scoped lang="scss">
.roadmind-adas { height: 78vh; min-height: 440px; display: flex; flex-direction: column; gap: 0.65em; padding: 0.5em 0.75em 1em; overflow-y: auto; }
.section { display: flex; flex-direction: column; gap: 0.35em; padding: 0.55em 0.65em 0.8em; border-bottom: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.12)); }
.note { color: var(--bng-cool-gray-300, #bbb); font-size: 0.9em; line-height: 1.4; }
.status { display: flex; flex-direction: column; gap: 0.3em; margin-top: 0.5em; padding: 0.5em; border: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.12)); border-radius: 0.25em; }
.alert { color: var(--bng-add-red-400, #f88); font-size: 0.9em; }
.metric { color: var(--bng-cool-gray-300, #bbb); font-size: 0.84em; font-variant-numeric: tabular-nums; }
.route { box-sizing: border-box; width: 100%; min-height: 5.5em; padding: 0.55em; border: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.18)); border-radius: 0.25em; color: var(--bng-cool-gray-100, #eee); background: var(--bng-cool-gray-900, rgba(0,0,0,0.25)); font: inherit; font-family: monospace; resize: vertical; }
.actions { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.5em; }
.runtime { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 0.75em; margin-top: 0.5em; }
.runtime > div { display: flex; flex-direction: column; gap: 0.25em; }
.footer { display: flex; align-items: center; justify-content: space-between; gap: 0.75em; }
</style>
