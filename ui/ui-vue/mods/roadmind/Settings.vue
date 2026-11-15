<template>
  <div class="roadmind-settings">
    <BngCardHeading>{{ t("title") }}</BngCardHeading>
    <p class="roadmind-intro">{{ t("intro") }}</p>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("sensorSuite") }}</BngCardHeading>
      <div class="roadmind-runtime">
        <div>
          <span class="roadmind-status" :class="{ active: status.visualizationEnabled }">
            {{ status.visualizationEnabled ? t("running") : t("stopped") }}
          </span>
          <span class="roadmind-budget">
            {{ t("estimatedLoad") }}: {{ status.estimatedRaycastsPerSecond || 0 }} {{ t("queriesPerSecond") }}
          </span>
        </div>
        <BngButton
          :accent="status.visualizationEnabled ? ACCENTS.outlined : ACCENTS.main"
          :disabled="busy"
          @click="toggleRuntime"
        >{{ status.visualizationEnabled ? t("stop") : t("start") }}</BngButton>
      </div>
      <p class="roadmind-note">{{ t("runtimeNote") }}</p>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("performance") }}</BngCardHeading>
      <p class="roadmind-note">{{ t("performanceNote") }}</p>
      <div class="roadmind-presets">
        <BngButton
          v-for="preset in presets"
          :key="preset.id"
          class="roadmind-preset"
          :accent="settings.preset === preset.id ? ACCENTS.main : ACCENTS.outlined"
          :disabled="busy"
          @click="applyPreset(preset.id)"
        >{{ t(preset.label) }}</BngButton>
      </div>
      <div class="roadmind-profile-summary">
        <div><strong>{{ t("economy") }}</strong><span>≈ 26 {{ t("queriesPerSecond") }}</span></div>
        <div><strong>{{ t("balanced") }}</strong><span>≈ 74 {{ t("queriesPerSecond") }}</span></div>
        <div><strong>{{ t("high") }}</strong><span>≈ 310 {{ t("queriesPerSecond") }}</span></div>
      </div>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("lidar") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("enabled") }}</template>
        <BngSwitch :modelValue="settings.lidarEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('lidarEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("resolution") }}</template>
        <BngSelect :value="settings.lidarQuality" :options="lidarQualityOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('lidarQuality', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("scanRate") }}</template>
        <BngSelect :value="settings.lidarHz" :options="lidarRateOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('lidarHz', Number(value))" />
      </BngRow>
      <BngRow :vertical="true">
        <template #label>{{ t("range") }}: {{ settings.lidarRange }} m</template>
        <BngSlider :min="30" :max="160" :step="5" :modelValue="settings.lidarRange"
          :with-input="true" :disabled="busy"
          @update:modelValue="queueLidarRange" />
      </BngRow>
      <p class="roadmind-note">{{ t("lidarNote") }}</p>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("radar") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("enabled") }}</template>
        <BngSwitch :modelValue="settings.radarEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('radarEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("rayDensity") }}</template>
        <BngSelect :value="settings.radarDensity" :options="radarDensityOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('radarDensity', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("scanRate") }}</template>
        <BngSelect :value="settings.radarHz" :options="radarRateOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('radarHz', Number(value))" />
      </BngRow>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("ultrasonic") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("enabled") }}</template>
        <BngSwitch :modelValue="settings.ultrasonicEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('ultrasonicEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("beamWidth") }}</template>
        <BngSelect :value="settings.ultrasonicDensity" :options="ultrasonicDensityOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('ultrasonicDensity', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("scanRate") }}</template>
        <BngSelect :value="settings.ultrasonicHz" :options="ultrasonicRateOptions"
          :config="optionConfig" :disabled="busy"
          @valueChanged="value => setSetting('ultrasonicHz', Number(value))" />
      </BngRow>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("visualization") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("lidarDisplay") }}</template>
        <BngSwitch :modelValue="settings.showLidar" :disabled="busy"
          @update:modelValue="value => setSetting('showLidar', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("radarDisplay") }}</template>
        <BngSwitch :modelValue="settings.showRadar" :disabled="busy"
          @update:modelValue="value => setSetting('showRadar', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("ultrasonicDisplay") }}</template>
        <BngSwitch :modelValue="settings.showUltrasonic" :disabled="busy"
          @update:modelValue="value => setSetting('showUltrasonic', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("cameraFrustums") }}</template>
        <BngSwitch :modelValue="settings.showCameras" :disabled="busy"
          @update:modelValue="value => setSetting('showCameras', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("localizationMarkers") }}</template>
        <BngSwitch :modelValue="settings.showLocalization" :disabled="busy"
          @update:modelValue="value => setSetting('showLocalization', value)" />
      </BngRow>
      <p class="roadmind-note">{{ t("visualizationNote") }}</p>
    </section>

    <div class="roadmind-footer">
      <span v-if="error" class="roadmind-error">{{ error }}</span>
      <span v-else class="roadmind-note">{{ t("savedAutomatically") }}</span>
      <BngButton :accent="ACCENTS.outlined" :disabled="busy" @click="refresh">{{ t("refresh") }}</BngButton>
    </div>
  </div>
</template>

<script setup>
import { computed, onMounted, onBeforeUnmount, ref } from "vue"
import {
  BngButton,
  BngSwitch,
  BngSlider,
  BngSelect,
  BngRow,
  BngCardHeading,
  ACCENTS,
} from "@/common/components/base"

const EN = {
  title: "RoadMind settings",
  intro: "Configure the virtual sensors and their 3D display. Scanning stays off until you start it.",
  sensorSuite: "Sensor suite",
  running: "Scanning enabled",
  stopped: "Scanning stopped",
  estimatedLoad: "Estimated workload",
  queriesPerSecond: "queries/s",
  start: "Start scanning",
  stop: "Stop scanning",
  runtimeNote: "The estimate counts scheduled scene raycasts. It is not a measured FPS or CPU result.",
  performance: "Performance",
  performanceNote: "Choose a preset, then fine-tune the sensors below. Changing an individual setting switches the profile to Custom.",
  economy: "Economy",
  balanced: "Balanced",
  high: "High quality",
  lidar: "Forward LiDAR",
  enabled: "Enabled",
  resolution: "Resolution",
  scanRate: "Scan rate",
  range: "Maximum range",
  lidarNote: "Low: 9 rays per scan. Balanced: 26. High: 51. More rays, range and scan frequency increase CPU workload.",
  radar: "Automotive radar",
  rayDensity: "Ray density",
  ultrasonic: "Ultrasonic sensors",
  beamWidth: "Beam width",
  visualization: "3D visualization",
  lidarDisplay: "LiDAR rays and points",
  radarDisplay: "Radar rays and points",
  ultrasonicDisplay: "Ultrasonic rays and points",
  cameraFrustums: "Camera fields of view",
  localizationMarkers: "IMU and GNSS markers",
  visualizationNote: "Hiding the LiDAR, radar or ultrasonic overlay does not stop its scans. Use the sensor switches to reduce raycast work; the camera and localization switches also stop their non-raycast data updates.",
  savedAutomatically: "Settings are saved automatically.",
  refresh: "Refresh status",
  error: "Could not contact RoadMind.",
  noVehicle: "Enter a vehicle before starting the sensors.",
  lowResolution: "Low (9 × 1)",
  balancedResolution: "Balanced (13 × 2)",
  highResolution: "High (17 × 3)",
  lowDensity: "Low (1 ray per radar)",
  mediumDensity: "Balanced (up to 2 rays per radar)",
  highDensity: "High (full ray count)",
  singleBeam: "Single ray per sensor",
  wideBeam: "Three-ray beam",
}

const RU = {
  title: "Настройки RoadMind",
  intro: "Настройка виртуальных датчиков и их отображения в 3D-мире. Сканирование запускается вручную.",
  sensorSuite: "Комплект датчиков",
  running: "Сканирование включено",
  stopped: "Сканирование остановлено",
  estimatedLoad: "Расчётная нагрузка",
  queriesPerSecond: "запросов/с",
  start: "Запустить сканирование",
  stop: "Остановить сканирование",
  runtimeNote: "Показано расчётное число запросов к сцене. Это не замер FPS или загрузки процессора.",
  performance: "Производительность",
  performanceNote: "Выбери профиль, затем настрой датчики отдельно. Изменение параметра переключает профиль на «Свои настройки».",
  economy: "Экономный",
  balanced: "Сбалансированный",
  high: "Высокое качество",
  lidar: "Передний LiDAR",
  enabled: "Включён",
  resolution: "Разрешение",
  scanRate: "Частота сканирования",
  range: "Максимальная дальность",
  lidarNote: "Низкое: 9 лучей за скан. Сбалансированное: 26. Высокое: 51. Большее число лучей, дальность и частота повышают нагрузку на процессор.",
  radar: "Автомобильные радары",
  rayDensity: "Плотность лучей",
  ultrasonic: "Ультразвуковые датчики",
  beamWidth: "Ширина луча",
  visualization: "Отображение в 3D-мире",
  lidarDisplay: "Лучи и точки LiDAR",
  radarDisplay: "Лучи и точки радаров",
  ultrasonicDisplay: "Лучи и точки ультразвука",
  cameraFrustums: "Поля обзора камер",
  localizationMarkers: "Маркеры IMU и GNSS",
  visualizationNote: "Скрытие лучей LiDAR, радаров или ультразвука не останавливает сканирование. Для снижения числа запросов отключи сам датчик. Камеры и локализация также прекращают обновлять свои данные.",
  savedAutomatically: "Настройки сохраняются автоматически.",
  refresh: "Обновить статус",
  error: "Не удалось связаться с RoadMind.",
  noVehicle: "Сначала сядь в автомобиль.",
  lowResolution: "Низкое (9 × 1)",
  balancedResolution: "Среднее (13 × 2)",
  highResolution: "Высокое (17 × 3)",
  lowDensity: "Низкая (1 луч на радар)",
  mediumDensity: "Средняя (до 2 лучей на радар)",
  highDensity: "Высокая (полное число лучей)",
  singleBeam: "Один луч на датчик",
  wideBeam: "Три луча",
}

const locale = ref("en-US")

function luaLiteral(value) {
  if (typeof value === "string") return JSON.stringify(value)
  if (typeof value === "boolean") return value ? "true" : "false"
  if (typeof value === "number" && Number.isFinite(value)) return String(value)
  throw new TypeError("Unsupported RoadMind setting value")
}

function callRoadMind(method, args = []) {
  return new Promise((resolve, reject) => {
    const engineLua = window.bngApi && window.bngApi.engineLua
    if (typeof engineLua !== "function") {
      reject(new Error("BeamNG engineLua bridge is unavailable"))
      return
    }

    const command = "extensions.load('roadmind'); return extensions.roadmind."
      + method + "(" + args.map(luaLiteral).join(", ") + ")"

    try {
      engineLua(command, resolve)
    } catch (error) {
      reject(error)
    }
  })
}

function t(key) {
  const dictionary = String(locale.value).toLowerCase().startsWith("ru") ? RU : EN
  return dictionary[key] || EN[key] || key
}

const settings = ref({
  preset: "economy",
  lidarEnabled: true,
  lidarQuality: "low",
  lidarHz: 1,
  lidarRange: 45,
  radarEnabled: true,
  radarDensity: "low",
  radarHz: 1,
  ultrasonicEnabled: true,
  ultrasonicDensity: "single",
  ultrasonicHz: 1,
  poseHz: 2,
  showLidar: true,
  showRadar: false,
  showUltrasonic: false,
  showCameras: false,
  showLocalization: false,
})
const status = ref({ visualizationEnabled: false, activeVehicle: false, estimatedRaycastsPerSecond: 0 })
const error = ref("")
const busy = ref(false)
const optionConfig = { value: option => option.value, label: option => option.label }
const presets = [
  { id: "economy", label: "economy" },
  { id: "balanced", label: "balanced" },
  { id: "high", label: "high" },
]

const lidarQualityOptions = computed(() => [
  { value: "low", label: t("lowResolution") },
  { value: "balanced", label: t("balancedResolution") },
  { value: "high", label: t("highResolution") },
])
const radarDensityOptions = computed(() => [
  { value: "low", label: t("lowDensity") },
  { value: "medium", label: t("mediumDensity") },
  { value: "high", label: t("highDensity") },
])
const ultrasonicDensityOptions = computed(() => [
  { value: "single", label: t("singleBeam") },
  { value: "wide", label: t("wideBeam") },
])
const lidarRateOptions = [1, 2, 4].map(value => ({ value, label: value + " Hz" }))
const radarRateOptions = [1, 2].map(value => ({ value, label: value + " Hz" }))
const ultrasonicRateOptions = [1, 2].map(value => ({ value, label: value + " Hz" }))

function acceptResult(result) {
  if (!result || result.ok === false) {
    const message = result && result.error
    error.value = message === "No active player vehicle" ? t("noVehicle") : (message || t("error"))
    return false
  }

  if (result.settings) settings.value = { ...settings.value, ...result.settings }
  if (result.status) status.value = { ...status.value, ...result.status }
  if (result.language) locale.value = result.language
  error.value = ""
  return true
}

let lidarRangeTimer = null
let pendingLidarRange = null

function queueLidarRange(value) {
  pendingLidarRange = Number(value)
  settings.value.lidarRange = pendingLidarRange
  if (lidarRangeTimer) clearTimeout(lidarRangeTimer)
  lidarRangeTimer = setTimeout(() => {
    const nextValue = pendingLidarRange
    pendingLidarRange = null
    lidarRangeTimer = null
    setSetting("lidarRange", nextValue)
  }, 180)
}

onBeforeUnmount(() => {
  if (lidarRangeTimer) {
    clearTimeout(lidarRangeTimer)
    lidarRangeTimer = null
    if (pendingLidarRange !== null) setSetting("lidarRange", pendingLidarRange)
    pendingLidarRange = null
  }
})

async function refresh() {
  busy.value = true
  try {
    acceptResult(await callRoadMind('getSettings'))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not read settings", e)
  } finally {
    busy.value = false
  }
}

async function setSetting(key, value) {
  try {
    acceptResult(await callRoadMind('setSetting', [key, value]))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not set " + key, e)
  }
}

async function applyPreset(name) {
  busy.value = true
  try {
    acceptResult(await callRoadMind('setPreset', [name]))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not apply preset", e)
  } finally {
    busy.value = false
  }
}

async function toggleRuntime() {
  busy.value = true
  try {
    acceptResult(await callRoadMind('setVisualizationEnabled', [!status.value.visualizationEnabled]))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not toggle scanning", e)
  } finally {
    busy.value = false
  }
}

onMounted(refresh)
</script>

<style scoped lang="scss">
.roadmind-settings {
  display: flex;
  flex-direction: column;
  gap: 0.65em;
  max-height: 78vh;
  overflow-y: auto;
  padding: 0.5em 0.75em 1em;
}
.roadmind-intro,
.roadmind-note {
  color: var(--bng-cool-gray-300, #bbb);
  font-size: 0.9em;
  line-height: 1.4;
}
.roadmind-section {
  display: flex;
  flex-direction: column;
  gap: 0.2em;
  padding: 0.55em 0.65em 0.8em;
  border-bottom: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.12));
}
.roadmind-runtime {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  justify-content: space-between;
  gap: 0.75em;
}
.roadmind-runtime > div {
  display: flex;
  flex-direction: column;
  gap: 0.25em;
}
.roadmind-status {
  color: var(--bng-cool-gray-200, #ccc);
  font-weight: 600;
}
.roadmind-status.active {
  color: var(--bng-add-green-400, #8bd79b);
}
.roadmind-budget {
  color: var(--bng-cool-gray-300, #bbb);
  font-size: 0.9em;
  font-variant-numeric: tabular-nums;
}
.roadmind-presets {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 0.5em;
}
.roadmind-preset {
  min-width: 0;
}
.roadmind-profile-summary {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5em 1.2em;
  padding-top: 0.35em;
  font-size: 0.88em;
}
.roadmind-profile-summary > div {
  display: flex;
  flex-direction: column;
  gap: 0.1em;
}
.roadmind-profile-summary span {
  color: var(--bng-cool-gray-300, #bbb);
}
.roadmind-footer {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.75em;
}
.roadmind-error {
  color: var(--bng-add-red-400, #f88);
  font-size: 0.9em;
}
</style>
