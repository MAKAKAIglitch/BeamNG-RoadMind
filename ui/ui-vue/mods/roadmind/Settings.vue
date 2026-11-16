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
          <span class="roadmind-budget">
            {{ t("vehiclesDetected") }}: {{ status.visibleVehicleCount || status.detectedVehicleCount || 0 }}
          </span>
          <span class="roadmind-budget">
            {{ t("vehicleCandidates") }}: {{ status.vehicleCandidateCount || 0 }}
          </span>
          <span class="roadmind-budget">
            {{ t("registrySize") }}: {{ status.vehicleRegistryCount || 0 }}
            <span v-if="status.vehicleRegistryTruncated">({{ t("registryLimited") }})</span>
          </span>
        </div>
        <BngButton
          :accent="status.visualizationEnabled ? ACCENTS.outlined : ACCENTS.main"
          :disabled="busy"
          @click="toggleRuntime"
        >{{ status.visualizationEnabled ? t("stop") : t("start") }}</BngButton>
      </div>
      <p class="roadmind-note">{{ t("runtimeNote") }}</p>
      <p class="roadmind-note">{{ t("detectionNote") }}</p>
      <div v-if="status.trackedObjects && status.trackedObjects.length" class="roadmind-track-list">
        <strong>{{ t("trackedObjects") }} ({{ status.trackedVehicleCount || 0 }})</strong>
        <div v-for="track in status.trackedObjects" :key="track.trackId" class="roadmind-track">
          <div class="roadmind-track-heading">
            <strong>{{ track.trackId }}</strong>
            <span>{{ t("track" + track.status.charAt(0).toUpperCase() + track.status.slice(1)) }}</span>
          </div>
          <div class="roadmind-track-metrics">
            <span>{{ t("trackRange") }}: {{ track.distance === undefined ? "—" : track.distance.toFixed(1) + " m" }}</span>
            <span>{{ t("trackSpeed") }}: {{ track.estimatedSpeedKph === undefined ? "—" : track.estimatedSpeedKph.toFixed(1) + " km/h" }}</span>
            <span>{{ t("trackRelativeSpeed") }}: {{ track.estimatedRelativeSpeedKph === undefined ? "—" : track.estimatedRelativeSpeedKph.toFixed(1) + " km/h" }}</span>
            <span>{{ t("trackAge") }}: {{ track.dataAge.toFixed(2) }} s</span>
            <span>{{ t("trackConfidence") }}: {{ track.confidencePercent }}%</span>
          </div>
          <span class="roadmind-track-sensors">{{ (track.sensors || []).join(" · ") }}</span>
        </div>
      </div>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("vehicleGeometry") }}</BngCardHeading>
      <div class="roadmind-runtime">
        <div>
          <span class="roadmind-status" :class="{ active: status.vehicleCalibration && status.vehicleCalibration.calibrated }">
            {{ status.vehicleCalibration && status.vehicleCalibration.calibrated ? t("calibrated") : t("fallbackCalibration") }}
          </span>
          <span v-if="status.vehicleCalibration && status.vehicleCalibration.calibrated" class="roadmind-budget">
            {{ status.vehicleCalibration.length.toFixed(2) }} ×
            {{ status.vehicleCalibration.width.toFixed(2) }} ×
            {{ status.vehicleCalibration.height.toFixed(2) }} m
          </span>
          <span v-else class="roadmind-budget">
            {{ status.vehicleCalibration && status.vehicleCalibration.error || t("calibrationWaiting") }}
          </span>
        </div>
        <BngButton :accent="ACCENTS.outlined" :disabled="busy || !status.activeVehicle"
          @click="recalibrateVehicle">{{ t("recalibrate") }}</BngButton>
      </div>
      <p class="roadmind-note">{{ t("calibrationNote") }}</p>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("routePilot") }}</BngCardHeading>
      <p class="roadmind-note">{{ t("routeDescription") }}</p>
      <div class="roadmind-runtime">
        <div>
          <span class="roadmind-status" :class="{ active: status.routePilotEnabled && !status.routeSafetyStopped }">
            {{ status.routePilotEnabled ? (status.routeSafetyStopped ? t("routePaused") : t("routeRunning")) : t("routeStopped") }}
          </span>
          <span class="roadmind-budget">{{ t("routeSpeed") }}: 29 km/h</span>
          <span class="roadmind-budget">{{ t("routeWaypoints") }}: {{ status.routeWaypointCount || 0 }}</span>
        </div>
        <BngButton
          :accent="status.routePilotEnabled ? ACCENTS.outlined : ACCENTS.main"
          :disabled="busy"
          @click="toggleRoutePilot"
        >{{ status.routePilotEnabled ? t("stopRoute") : t("startRoute") }}</BngButton>
      </div>
      <p v-if="status.routeSafetyStopped && status.routePilotEnabled" class="roadmind-note">
        {{ t("routeSafetyStop") }}
      </p>
      <p v-if="status.routePilotError && status.routePilotEnabled" class="roadmind-error">
        {{ routeErrorText(status.routePilotError) }}
      </p>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("performance") }}</BngCardHeading>
      <p class="roadmind-note">{{ t("performanceNote") }}</p>
      <p class="roadmind-active-profile">{{ t("activeProfile") }}: <strong>{{ t(settings.preset) }}</strong></p>
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
        <div><strong>{{ t("economy") }}</strong><span>≈ 9 {{ t("queriesPerSecond") }}</span></div>
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
      <BngRow :vertical="true">
        <template #label>{{ t("pointCount") }}: {{ settings.lidarPointCount }} {{ t("pointsPerRow") }}</template>
        <BngSlider :min="9" :max="33" :step="4" :modelValue="settings.lidarPointCount"
          :with-input="true" :disabled="busy"
          @update:modelValue="queueLidarPointCount" />
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
      <BngRow>
        <template #label>{{ t("persistence") }}</template>
        <BngSwitch :modelValue="settings.lidarPersistenceEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('lidarPersistenceEnabled', value)" />
      </BngRow>
      <p class="roadmind-note">
        {{ t("persistenceNote") }} ({{ status.persistentLidarPointCount || 0 }} / 600)
      </p>
      <BngButton :accent="ACCENTS.outlined" :disabled="busy" @click="clearLidarMap">
        {{ t("clearCloud") }}
      </BngButton>
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
        <template #label>{{ t("detectedVehicles") }}</template>
        <BngSwitch :modelValue="settings.showDetectedVehicles" :disabled="busy"
          @update:modelValue="value => setSetting('showDetectedVehicles', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("cameraFrustums") }}</template>
        <BngSwitch :modelValue="settings.showCameras" :disabled="busy"
          @update:modelValue="value => setSetting('showCameras', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("calibrationOverlay") }}</template>
        <BngSwitch :modelValue="settings.showCalibration" :disabled="busy"
          @update:modelValue="value => setSetting('showCalibration', value)" />
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
import { useBridge } from "@/bridge"
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
  performanceNote: "Economy runs only the forward LiDAR. Radar and ultrasonic start off; Balanced enables all three sensor groups. Changing a setting switches to Custom.",
  activeProfile: "Active profile",
  custom: "Custom",
  economy: "Economy (LiDAR only)",
  balanced: "Balanced",
  high: "High quality",
  lidar: "Forward LiDAR",
  enabled: "Enabled",
  resolution: "Vertical layers",
  pointCount: "Points per horizontal row",
  pointsPerRow: "points",
  persistence: "Accumulate point cloud",
  persistenceNote: "Keeps up to 600 world-space points for this game session. Drawing the cloud can reduce FPS.",
  clearCloud: "Clear scanned point cloud",
  detectedVehicles: "Detected vehicles",
  vehiclesDetected: "Vehicles confirmed visible",
  vehicleCandidates: "Vehicle candidates in sensor zones",
  trackedObjects: "RoadMind object tracks",
  trackConfirmed: "Confirmed",
  trackCoasting: "Coasting",
  trackStale: "Stale",
  trackRange: "Range",
  trackSpeed: "Estimated speed",
  trackRelativeSpeed: "Relative speed",
  trackAge: "Data age",
  trackConfidence: "Confidence",
  registrySize: "Cached vehicle registry",
  registryLimited: "limit reached",
  detectionNote: "Vehicle presence is simulator-assisted: RoadMind checks vehicle OBBs against calibrated sensor zones and verifies visibility with a bounded number of occlusion raycasts. It is not a raw LiDAR/radar/camera detection stream.",
  vehicleGeometry: "Vehicle geometry and sensor calibration",
  calibrated: "Geometry calibrated",
  fallbackCalibration: "Using fallback sensor positions",
  calibrationWaiting: "Calibration has not been run yet",
  recalibrate: "Recalibrate sensors",
  calibrationNote: "RoadMind reads the vehicle oriented bounding box once per attach/reset and calculates all sensor mounts in vehicle-local coordinates. Turn on the calibration overlay to inspect the bounds, axes and sensor positions. Vehicle-class coverage still requires in-game validation.",
  calibrationOverlay: "Vehicle bounds, axes and all sensor mounts",
  routeWaypoints: "Road waypoints in active path",
  routePilot: "Road route pilot (experimental)",
  routeDescription: "Builds a forward route on the current road NavGraph and sends its waypoints to BeamNG vehicle AI (up to 29 km/h). LiDAR, radar, ultrasonic returns and nearby-vehicle detections guard the path: it stops when blocked or when sensor data is stale, then replans ahead when clear. It does not yet steer around a blocked lane or accept a chosen destination; camera frustums are view-only.",
  routeSpeed: "Speed limit",
  routeRunning: "Route pilot active",
  routePaused: "Paused by safety guard",
  routeStopped: "Route pilot off",
  startRoute: "Drive on road network",
  stopRoute: "Stop route pilot",
  routeSafetyStop: "The vehicle is stopping or waiting for fresh, clear sensor data.",

  scanRate: "Scan rate",
  range: "Maximum range",
  lidarNote: "The slider sets 9–33 horizontal samples; resolution adds 1, 2 or 3 vertical layers. More samples, range and scan frequency increase CPU workload.",
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
  lowResolution: "1 vertical layer",
  balancedResolution: "2 vertical layers",
  highResolution: "3 vertical layers",
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
  performanceNote: "В режиме «Экономный» работает только передний LiDAR. Радар и ультразвук выключены; «Сбалансированный» включает все три группы. Изменение параметра переключает профиль на свои настройки.",
  activeProfile: "Активный профиль",
  custom: "Свои настройки",
  economy: "Экономный (только LiDAR)",
  balanced: "Сбалансированный",
  high: "Высокое качество",
  lidar: "Передний LiDAR",
  enabled: "Включён",
  resolution: "Вертикальные слои",
  pointCount: "Точек по горизонтали",
  pointsPerRow: "шт.",
  persistence: "Накапливать облако точек",
  persistenceNote: "Хранит до 600 точек в мировых координатах до конца текущей игровой сессии. Отрисовка облака может снизить FPS.",
  clearCloud: "Очистить облако точек",
  detectedVehicles: "Обнаруженные автомобили",
  vehiclesDetected: "Подтверждённо видимых машин",
  vehicleCandidates: "Кандидатов в зонах датчиков",
  trackedObjects: "Треки RoadMind",
  trackConfirmed: "Подтверждён",
  trackCoasting: "Потеря видимости",
  trackStale: "Устарел",
  trackRange: "Дальность",
  trackSpeed: "Расчётная скорость",
  trackRelativeSpeed: "Относительная скорость",
  trackAge: "Возраст данных",
  trackConfidence: "Достоверность",
  registrySize: "Машин в общем реестре",
  registryLimited: "достигнут лимит",
  detectionNote: "Обнаружение машин использует данные симулятора: RoadMind проверяет OBB автомобилей в зонах датчиков и видимость ограниченным числом лучей. Это не поток физических измерений LiDAR, радара или камер.",
  vehicleGeometry: "Геометрия автомобиля и калибровка датчиков",
  calibrated: "Геометрия определена",
  fallbackCalibration: "Используются резервные позиции датчиков",
  calibrationWaiting: "Калибровка ещё не выполнялась",
  recalibrate: "Перекалибровать датчики",
  calibrationNote: "RoadMind один раз при подключении или сбросе считывает ориентированный габарит автомобиля и рассчитывает крепления датчиков в локальных координатах. Включи отладочную визуализацию, чтобы увидеть габариты, оси и датчики. Размещение на разных классах машин ещё требует проверки в игре.",
  calibrationOverlay: "Габариты, оси и все крепления датчиков",
  routeWaypoints: "Точек дорожного маршрута",
  routePilot: "Автопилот по дорожной сети (эксперимент)",
  routeDescription: "Строит путь вперёд по дорожному графу текущей карты и передаёт точки штатному AI BeamNG (до 29 км/ч). LiDAR, радары, ультразвук и обнаружение машин контролируют путь: при препятствии или устаревших данных машина останавливается, после освобождения пути строится следующий участок. Объезд препятствий и выбор конечной точки пока не реализованы; камеры показывают только поля обзора.",
  routeSpeed: "Ограничение скорости",
  routeRunning: "Автопилот включён",
  routePaused: "Остановка по датчикам",
  routeStopped: "Автопилот выключен",
  startRoute: "Ехать по дорожной сети",
  stopRoute: "Остановить автопилот",
  routeSafetyStop: "Автомобиль останавливается или ждёт новых данных без препятствий.",
  routeWaitingData: "Ожидание свежих данных датчиков",
  routeNoRoadNodes: "Не удалось найти дорожные узлы рядом с автомобилем и впереди него",
  routeNoPath: "Не удалось построить связный путь по дорожному графу",
  routeApiMissing: "На этой карте недоступен API дорожного графа",
  routeHeadingInvalid: "Не удалось определить направление автомобиля",
  routeVehicleChanged: "Активный автомобиль изменился",
  routeObstacle: "Препятствие ближе {distance} м — автопилот остановлен",
  routeGenericError: "Не удалось запустить или продолжить маршрут",

  scanRate: "Частота сканирования",
  range: "Максимальная дальность",
  lidarNote: "Ползунок задаёт 9–33 точки по горизонтали; разрешение добавляет 1, 2 или 3 вертикальных слоя. Большее число точек, дальность и частота повышают нагрузку на процессор.",
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
  lowResolution: "1 слой",
  balancedResolution: "2 слоя",
  highResolution: "3 слоя",
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

const { api } = useBridge()

function callRoadMind(method, args = []) {
  return new Promise((resolve, reject) => {
    if (!api || typeof api.engineLua !== "function") {
      reject(new Error("BeamNG Vue bridge is unavailable"))
      return
    }

    const command = "(function() extensions.load('roadmind'); return extensions.roadmind."
      + method + "(" + args.map(luaLiteral).join(", ") + ") end)()"

    try {
      // Keep the bridge API receiver intact: engineLua uses its own callback registry.
      api.engineLua(command, resolve)
    } catch (error) {
      reject(error)
    }
  })
}

function t(key) {
  const dictionary = String(locale.value).toLowerCase().startsWith("ru") ? RU : EN
  return dictionary[key] || EN[key] || key
}

function routeErrorText(message) {
  const value = String(message || "")
  if (!String(locale.value).toLowerCase().startsWith("ru")) return value

  if (value === "Waiting for fresh sensor data") return t("routeWaitingData")
  if (value === "Could not find connected road nodes near the vehicle and forward target") return t("routeNoRoadNodes")
  if (value === "No valid forward road path could be built from the current NavGraph") return t("routeNoPath")
  if (value === "NavGraph route API is unavailable on this map") return t("routeApiMissing")
  if (value === "Vehicle heading is invalid" || value === "Could not read vehicle position or heading") return t("routeHeadingInvalid")
  if (value === "The active vehicle changed") return t("routeVehicleChanged")

  const obstacle = value.match(/^Obstacle inside ([0-9.]+) m safety distance$/)
  if (obstacle) return t("routeObstacle").replace("{distance}", obstacle[1])
  return t("routeGenericError")
}

const settings = ref({
  preset: "economy",
  lidarEnabled: true,
  lidarQuality: "low",
  lidarPointCount: 9,
  lidarHz: 1,
  lidarRange: 45,
  radarEnabled: false,
  radarDensity: "low",
  radarHz: 1,
  ultrasonicEnabled: false,
  ultrasonicDensity: "single",
  ultrasonicHz: 1,
  poseHz: 2,
  lidarPersistenceEnabled: false,
  showLidar: true,
  showRadar: false,
  showUltrasonic: false,
  showCameras: false,
  showLocalization: false,
  showDetectedVehicles: true,
  showCalibration: false,
})
const status = ref({
  visualizationEnabled: false,
  activeVehicle: false,
  estimatedRaycastsPerSecond: 0,
  routePilotEnabled: false,
  routeSafetyStopped: false,
  persistentLidarPointCount: 0,
  detectedVehicleCount: 0,
  visibleVehicleCount: 0,
  vehicleCandidateCount: 0,
  vehicleRegistryCount: 0,
  vehicleRegistryTruncated: false,
  occlusionRaycastsPerSecond: 0,
  trackedVehicleCount: 0,
  trackedObjects: [],
  vehicleCalibration: { calibrated: false, error: "Calibration has not been run" },
})
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
let lidarPointCountTimer = null
let pendingLidarPointCount = null

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

function queueLidarPointCount(value) {
  pendingLidarPointCount = Number(value)
  settings.value.lidarPointCount = pendingLidarPointCount
  if (lidarPointCountTimer) clearTimeout(lidarPointCountTimer)
  lidarPointCountTimer = setTimeout(() => {
    const nextValue = pendingLidarPointCount
    pendingLidarPointCount = null
    lidarPointCountTimer = null
    setSetting("lidarPointCount", nextValue)
  }, 180)
}

onBeforeUnmount(() => {
  if (statusRefreshTimer) {
    clearInterval(statusRefreshTimer)
    statusRefreshTimer = null
  }
  if (lidarRangeTimer) {
    clearTimeout(lidarRangeTimer)
    lidarRangeTimer = null
    if (pendingLidarRange !== null) setSetting("lidarRange", pendingLidarRange)
    pendingLidarRange = null
  }
  if (lidarPointCountTimer) {
    clearTimeout(lidarPointCountTimer)
    lidarPointCountTimer = null
    if (pendingLidarPointCount !== null) setSetting("lidarPointCount", pendingLidarPointCount)
    pendingLidarPointCount = null
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

async function toggleRoutePilot() {
  busy.value = true
  try {
    acceptResult(await callRoadMind('setRoutePilotEnabled', [!status.value.routePilotEnabled]))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not toggle route pilot", e)
  } finally {
    busy.value = false
  }
}

async function recalibrateVehicle() {
  busy.value = true
  try {
    acceptResult(await callRoadMind('recalibrateVehicle'))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not recalibrate vehicle geometry", e)
  } finally {
    busy.value = false
  }
}

async function clearLidarMap() {
  busy.value = true
  try {
    acceptResult(await callRoadMind('clearLidarMap'))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not clear LiDAR map", e)
  } finally {
    busy.value = false
  }
}

let statusRefreshTimer = null

onMounted(() => {
  refresh()
  statusRefreshTimer = setInterval(async () => {
    if (busy.value) return
    try {
      const result = await callRoadMind("getSettings")
      if (result && result.status) status.value = { ...status.value, ...result.status }
    } catch (e) {
      console.warn("RoadMind: background status refresh failed", e)
    }
  }, 1500)
})
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
.roadmind-active-profile {
  font-size: 0.9em;
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
.roadmind-track-list {
  display: flex;
  flex-direction: column;
  gap: 0.4em;
  margin-top: 0.35em;
}
.roadmind-track {
  display: flex;
  flex-direction: column;
  gap: 0.2em;
  padding: 0.4em 0.5em;
  border: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.12));
  border-radius: 0.25em;
}
.roadmind-track-heading,
.roadmind-track-metrics {
  display: flex;
  flex-wrap: wrap;
  justify-content: space-between;
  gap: 0.25em 0.8em;
}
.roadmind-track-metrics,
.roadmind-track-sensors {
  color: var(--bng-cool-gray-300, #bbb);
  font-size: 0.82em;
  font-variant-numeric: tabular-nums;
}
.roadmind-track-sensors {
  overflow-wrap: anywhere;
}
.roadmind-error {
  color: var(--bng-add-red-400, #f88);
  font-size: 0.9em;
}
</style>
