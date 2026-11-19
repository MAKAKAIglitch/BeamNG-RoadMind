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
            {{ t("trafficUpdateRate") }}:
            {{ status.trafficUpdateHz ? status.trafficUpdateHz.toFixed(1) : "—" }} Hz
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
      <p v-if="status.currentFps !== undefined" class="roadmind-budget">
        {{ t("currentFps") }}: {{ status.currentFps.toFixed(0) }} / {{ settings.targetFps }} FPS
        · {{ t("sensorQuality") }}: {{ (status.adaptiveQualityTier || 0) + 1 }}/5
      </p>
      <p v-if="status.performanceLimitReached" class="roadmind-error">
        {{ t("targetFpsUnreachable") }}
      </p>
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
      <div class="roadmind-world-model">
        <div class="roadmind-track-heading">
          <strong>{{ t("worldModel") }} v{{ status.worldModelSchemaVersion || 2 }}</strong>
          <span>{{
            !status.worldModelRegistryEnabled
              ? t("sourceDisabled")
              : (status.worldModelRegistryFresh ? t("sourceFresh") : t("sourceStale"))
          }}</span>
        </div>
        <div class="roadmind-track-metrics">
          <span>{{ t("worldTargets") }}: {{ status.worldModelTargetCount || 0 }}</span>
          <span>{{ t("worldObstacles") }}: {{ status.worldModelObstacleCount || 0 }}</span>
          <span>{{ t("worldObservations") }}: {{ status.worldModelObservationCount || 0 }}</span>
          <span>{{ t("worldStaleSources") }}: {{ status.worldModelStaleSourceCount || 0 }}</span>
        </div>
        <span v-if="status.worldModelInvalidObservationCount || status.worldModelDroppedObservationCount || status.worldModelConflictingObservationCount"
          class="roadmind-track-sensors">
          {{ t("worldQuality") }}:
          {{ status.worldModelInvalidObservationCount || 0 }} / 
          {{ status.worldModelDroppedObservationCount || 0 }} /
          {{ status.worldModelConflictingObservationCount || 0 }}
        </span>
      </div>
    </section>

    <section class="roadmind-section">
      <BngCardHeading>{{ t("adas") }}</BngCardHeading>
      <BngRow>
        <template #label>{{ t("adasMaster") }}</template>
        <BngSwitch :modelValue="settings.adasEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('adasEnabled', value)" />
      </BngRow>
      <p class="roadmind-note">{{ t("adasNote") }}</p>
      <BngRow>
        <template #label>{{ t("adasFCW") }}</template>
        <BngSwitch :modelValue="settings.adasFCWEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasFCWEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("adasAEB") }}</template>
        <BngSwitch :modelValue="settings.adasAEBEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasAEBEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("adasACC") }}</template>
        <BngSwitch :modelValue="settings.adasACCEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasACCEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("adasBSM") }}</template>
        <BngSwitch :modelValue="settings.adasBSMEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasBSMEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("adasParking") }}</template>
        <BngSwitch :modelValue="settings.adasParkingEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasParkingEnabled', value)" />
      </BngRow>
      <BngRow>
        <template #label>{{ t("adasLDW") }}</template>
        <BngSwitch :modelValue="settings.adasLDWEnabled" :disabled="busy || !settings.adasEnabled"
          @update:modelValue="value => setSetting('adasLDWEnabled', value)" />
      </BngRow>
      <div v-if="settings.adasEnabled" class="roadmind-world-model">
        <strong>{{ t("adasState") }}: {{ t("adasStatus_" + ((status.adasStatus && status.adasStatus.status) || "standby")) }}</strong>
        <span v-if="status.adasStatus && status.adasStatus.fcw" class="roadmind-error">{{ t("adasAlertFCW") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.aeb" class="roadmind-error">{{ t("adasAlertAEB") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.leftBlindSpot" class="roadmind-note">{{ t("adasAlertLeft") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.rightBlindSpot" class="roadmind-note">{{ t("adasAlertRight") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.laneDeparture" class="roadmind-note">{{ t("adasAlertLane") }}</span>
        <span v-if="status.adasStatus && status.adasStatus.parkingDistance !== undefined" class="roadmind-track-metrics">
          {{ t("adasParkingDistance") }}: {{ status.adasStatus.parkingDistance.toFixed(1) }} m
        </span>
        <span v-if="status.adasStatus && status.adasStatus.leadDistance !== undefined" class="roadmind-track-metrics">
          {{ t("adasLeadDistance") }}: {{ status.adasStatus.leadDistance.toFixed(1) }} m
        </span>
        <p v-if="status.adasStatus && status.adasStatus.sensorAvailable === false" class="roadmind-note">
          {{ t("adasRequiresSensors") }}
        </p>
      </div>
    </section>

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
      <label class="roadmind-route-points-label" for="roadmind-route-points">{{ t("routePoints") }}</label>
      <textarea
        id="roadmind-route-points"
        v-model="routePointsText"
        class="roadmind-route-points"
        rows="3"
        :placeholder="t('routePointsPlaceholder')"
        :disabled="busy"
      ></textarea>
      <div class="roadmind-route-actions">
        <BngButton :accent="ACCENTS.main" :disabled="busy" @click="applyRoutePoints">
          {{ t("applyRoutePoints") }}
        </BngButton>
        <BngButton :accent="ACCENTS.outlined" :disabled="busy" @click="clearRoutePoints">
          {{ t("clearRoutePoints") }}
        </BngButton>
      </div>
      <p class="roadmind-note">
        {{ t("routeInputCount") }}: {{ status.routeInputPointCount || 0 }}
        · {{ t("routeProgress") }}: {{ status.routeProgressPercent || 0 }}%
      </p>
      <p class="roadmind-note">
        {{ t("roadLocalization") }}: {{ t("roadQuality_" + (status.roadLocalizationQuality || "unavailable")) }}
        <span v-if="status.roadDistanceToSegment !== undefined">
          · {{ status.roadDistanceToSegment.toFixed(1) }} m
        </span>
        <span v-if="status.routeDistanceToPath !== undefined">
          · {{ t("routePathOffset") }}: {{ status.routeDistanceToPath.toFixed(1) }} m
        </span>
        <span v-if="status.routeDestinationSnapDistance !== undefined">
          · {{ t("destinationSnap") }}: {{ status.routeDestinationSnapDistance.toFixed(1) }} m
        </span>
      </p>
      <p v-if="status.routeCompleted" class="roadmind-status active">{{ t("routeCompleted") }}</p>
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
      <div class="roadmind-world-model">
        <strong>{{ t("localPlanner") }}: {{ t("plannerStatus_" + (status.plannerStatus || "idle")) }}</strong>
        <span class="roadmind-track-metrics">
          {{ t("plannerCandidates") }}: {{ status.plannerCandidateCount || 0 }}
          · {{ t("plannerRejected") }}: {{ status.plannerRejectedCandidateCount || 0 }}
        </span>
        <span v-if="status.plannerSelectedScore !== undefined" class="roadmind-track-metrics">
          {{ t("plannerOffset") }}: {{ Number(status.plannerSelectedOffset || 0).toFixed(2) }} m
          · {{ t("plannerClearance") }}: {{ Number(status.plannerMinimumClearance || 0).toFixed(2) }} m
          · {{ t("plannerRoadMargin") }}: {{ Number(status.plannerMinimumRoadMargin || 0).toFixed(2) }} m
        </span>
        <span class="roadmind-note">{{ t("plannerReason_" + (status.plannerReasonCode || "inactive")) }}</span>
        <p class="roadmind-note">{{ t("plannerBoundaryNote") }}</p>
        <p class="roadmind-note">{{ t("plannerControlNote") }}</p>
      </div>
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
      <div class="roadmind-adaptive">
        <strong>{{ t("adaptivePerformance") }}</strong>
        <p class="roadmind-note">{{ t("adaptivePerformanceNote") }}</p>
        <BngRow>
          <template #label>{{ t("adaptiveFpsEnabled") }}</template>
          <BngSwitch :modelValue="settings.adaptiveFpsEnabled" :disabled="busy"
            @update:modelValue="value => setSetting('adaptiveFpsEnabled', value)" />
        </BngRow>
        <BngRow :vertical="true" v-if="settings.adaptiveFpsEnabled">
          <template #label>{{ t("targetFps") }}: {{ settings.targetFps }} FPS</template>
          <BngSlider :min="30" :max="165" :step="5" :modelValue="settings.targetFps"
            :with-input="true" :disabled="busy" @update:modelValue="queueTargetFps" />
        </BngRow>
        <BngRow>
          <template #label>{{ t("speedAdaptiveRate") }}</template>
          <BngSwitch :modelValue="settings.speedAdaptiveRateEnabled" :disabled="busy"
            @update:modelValue="value => setSetting('speedAdaptiveRateEnabled', value)" />
        </BngRow>
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
      <p class="roadmind-note">{{ t("lidarNote") }}</p>
      <BngRow>
        <template #label>{{ t("persistence") }}</template>
        <BngSwitch :modelValue="settings.lidarPersistenceEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('lidarPersistenceEnabled', value)" />
      </BngRow>
      <BngRow :vertical="true">
        <template #label>
          {{ t("pointCloudLimit") }}: {{ settings.lidarPointCloudLimit }} {{ t("points") }}
        </template>
        <BngSlider :min="500" :max="10000" :step="500"
          :modelValue="settings.lidarPointCloudLimit" :with-input="true" :disabled="busy"
          @update:modelValue="queuePointCloudLimit" />
      </BngRow>
      <p class="roadmind-note">
        {{ t("persistenceNote") }} ({{ status.persistentLidarPointCount || 0 }} /
        {{ status.persistentLidarPointLimit || settings.lidarPointCloudLimit }})
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
        <template #label>{{ t("visualizationInertia") }}</template>
        <BngSwitch :modelValue="settings.visualizationInertiaEnabled" :disabled="busy"
          @update:modelValue="value => setSetting('visualizationInertiaEnabled', value)" />
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
  adas: "ADAS driver assistance",
  adasMaster: "Enable ADAS",
  adasNote: "ADAS needs active scanning and fresh measurements. AEB can apply brakes while driving manually. ACC currently adjusts speed only while the RoadMind route pilot is active. Lane departure uses the NavGraph road centreline, not detected lane markings.",
  adasFCW: "Forward collision warning (FCW)",
  adasAEB: "Automatic emergency braking (AEB)",
  adasACC: "Adaptive cruise control (ACC, route pilot)",
  adasBSM: "Blind spot monitoring (BSM)",
  adasParking: "Parking distance warning",
  adasLDW: "Lane departure warning (LDW)",
  adasState: "ADAS status",
  adasStatus_disabled: "Off",
  adasStatus_standby: "Standby",
  adasStatus_waiting_for_sensors: "Waiting for fresh sensor data",
  adasStatus_vehicle_state_unavailable: "Vehicle data unavailable",
  adasStatus_active: "Monitoring",
  adasStatus_fcw: "Forward collision warning",
  adasStatus_aeb: "Emergency braking active",
  adasStatus_blind_spot: "Vehicle in blind spot",
  adasStatus_lane_departure: "Lane departure risk",
  adasAlertFCW: "Obstacle ahead",
  adasAlertAEB: "AEB braking",
  adasAlertLeft: "Left blind spot",
  adasAlertRight: "Right blind spot",
  adasAlertLane: "Lane departure risk",
  adasParkingDistance: "Nearest parking obstacle",
  adasLeadDistance: "Lead vehicle gap",
  adasRequiresSensors: "Enable the required sensor groups: LiDAR/radar for forward warnings, ultrasonic for parking, and scanning/vehicle tracking for blind spots.",
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
  adaptivePerformance: "Experimental load control",
  adaptivePerformanceNote: "RoadMind adjusts sensor ray count and scan rates to approach the target FPS. It cannot make the game reach an FPS that the rest of the scene, traffic or hardware cannot sustain.",
  adaptiveFpsEnabled: "Automatically adapt sensor quality",
  targetFps: "Desired FPS",
  speedAdaptiveRate: "Adjust sensor rate to vehicle speed",
  currentFps: "Measured FPS",
  sensorQuality: "Sensor quality",
  trafficUpdateRate: "Traffic position updates",
  targetFpsUnreachable: "Minimum sensor workload reached. RoadMind cannot force the rest of the game to reach the target FPS.",
  activeProfile: "Active profile",
  custom: "Custom",
  economy: "Economy (LiDAR only)",
  balanced: "Balanced",
  high: "High quality",
  lidar: "Forward LiDAR",
  enabled: "Enabled",
  resolution: "Vertical layers",
  persistence: "Accumulate point cloud",
  pointCloudLimit: "Accumulation limit",
  points: "points",
  persistenceNote: "Stores LiDAR returns in world coordinates for this game session, including vehicle surfaces. The map remains visible while paused; drawing more points can reduce FPS.",
  clearCloud: "Clear scanned point cloud",
  visualizationInertia: "Smooth inertial movement",
  detectedVehicles: "Detected vehicles",
  vehiclesDetected: "Vehicles confirmed visible",
  vehicleCandidates: "Vehicle candidates in sensor zones",
  trackedObjects: "RoadMind object tracks",
  worldModel: "Unified world model",
  worldTargets: "Targets",
  worldObstacles: "Obstacle clusters",
  worldObservations: "Observations",
  worldStaleSources: "Stale sources",
  worldQuality: "Invalid / dropped / conflicting",
  sourceFresh: "Registry fresh",
  sourceStale: "Registry stale",
  sourceDisabled: "Registry disabled",
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
  detectionNote: "Vehicle presence is simulator-assisted. Green OBB outlines are confirmed visible; yellow outlines are candidates still waiting for the limited occlusion checks. This is not raw LiDAR/radar/camera recognition.",
  vehicleGeometry: "Vehicle geometry and sensor calibration",
  calibrated: "Geometry calibrated",
  fallbackCalibration: "Using fallback sensor positions",
  calibrationWaiting: "Calibration has not been run yet",
  recalibrate: "Recalibrate sensors",
  calibrationNote: "RoadMind reads the vehicle oriented bounding box once per attach/reset and calculates all sensor mounts in vehicle-local coordinates. Turn on the calibration overlay to inspect the bounds, axes and sensor positions. Vehicle-class coverage still requires in-game validation.",
  calibrationOverlay: "Vehicle bounds, axes and all sensor mounts",
  routeWaypoints: "Road waypoints in active path",
  routePilot: "RoadMind autonomous pilot (experimental)",
  routeDescription: "Uses the current map NavGraph. Enter ordered world coordinates as X,Y,Z (one point per line). RoadMind’s own Lua controller follows the planner-approved trajectory and controls steering, throttle and brake at up to 28.8 km/h. Sensor and road checks are experimental; camera frustums are view-only.",
  routePoints: "Ordered route points (world coordinates, metres)",
  routePointsPlaceholder: "X, Y, Z\nX, Y, Z\nX, Y, Z",
  applyRoutePoints: "Set route points",
  clearRoutePoints: "Clear route",
  routeInputCount: "Input points",
  routeProgress: "Route progress",
  roadLocalization: "Road localization",
  routePathOffset: "Route offset",
  destinationSnap: "Road-node snap",
  routePointsFormat: "Enter 1–12 lines with three numeric values: X, Y, Z.",
  routePointsJson: "Route data could not be decoded.",
  routePointOffRoad: "One of the points is too far from a road on this map.",
  routeNoConnectedPath: "The road graph has no connected path between these points.",
  routePathLimit: "The route is too long for the current waypoint limit.",
  routeLocalizationLost: "Vehicle left the planned route; the pilot stopped and will try to replan.",
  routeCompleted: "Destination road node reached",
  roadQuality_high: "High",
  roadQuality_medium: "Medium",
  roadQuality_low: "Low",
  roadQuality_lost: "Lost",
  roadQuality_unavailable: "Unavailable",
  routeSpeed: "Speed limit",
  routeRunning: "Route pilot active",
  routePaused: "Paused by safety guard",
  routeStopped: "Route pilot off",
  startRoute: "Drive on road network",
  stopRoute: "Stop route pilot",
  routeSafetyStop: "The vehicle is stopping or waiting for fresh, clear sensor data.",
  localPlanner: "Local trajectory planner",
  plannerStatus_idle: "Inactive",
  plannerStatus_waiting: "Waiting for valid data",
  plannerStatus_ready: "Safe trajectory selected",
  plannerStatus_blocked: "No safe trajectory",
  plannerCandidates: "Candidates",
  plannerRejected: "Rejected",
  plannerOffset: "Selected lateral offset",
  plannerClearance: "Obstacle clearance",
  plannerRoadMargin: "Road-corridor margin",
  plannerReason_inactive: "The route pilot is inactive.",
  plannerReason_sensor_stale: "Waiting for a fresh enabled obstacle sensor.",
  plannerReason_road_unavailable: "Road localization is unavailable or uncertain.",
  plannerReason_vehicle_geometry: "The calibrated vehicle footprint is unavailable.",
  plannerReason_route_unavailable: "A valid connected route is unavailable.",
  plannerReason_selected: "The best-scoring safe trajectory is shown in green.",
  plannerReason_no_safe_candidate: "Every candidate failed at least one safety check.",
  plannerReason_obstacle: "All candidates intersect a scene obstacle or lack clearance.",
  plannerReason_vehicle: "Tracked traffic overlaps the candidate footprints.",
  plannerReason_road_edge: "The vehicle body does not fit inside the conservative road corridor.",
  plannerReason_turning_radius: "The route exceeds the estimated turning capability.",
  plannerBoundaryNote: "Road edges are a conservative NavGraph-centreline width estimate, not detected lane boundaries.",
  plannerControlNote: "RoadMind now tracks the selected trajectory with its own Lua controller. The first controller is experimental and still needs in-game validation across vehicles, traffic and intersections.",
  plannerStopError: "The local planner stopped the route pilot because it could not approve a safe path.",

  scanRate: "Scan rate",
  range: "Maximum range",
  lidarNote: "Horizontal density and range follow the selected performance profile. The separate point-cloud limit controls how many world-space returns are retained.",
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
  visualizationNote: "Inertia smooths eligible world-space LiDAR returns and route/planner overlays. Vehicle-mounted sensor origins, mounts, cameras, calibration bounds and traffic OBBs stay locked to their sampled/live vehicle poses; LiDAR hits on tracked vehicles are reconstructed in the vehicle's local body frame. Rendering changes do not alter measurements or the World Model. The accumulated static LiDAR map stays fixed in world coordinates. Hiding an overlay does not stop scans.",
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
  adas: "Системы помощи водителю (ADAS)",
  adasMaster: "Включить ADAS",
  adasNote: "ADAS требует запущенного сканирования и свежих измерений. AEB может тормозить при ручном управлении. ACC пока регулирует скорость только при активном маршрутном автопилоте RoadMind. Контроль ухода с полосы оценивается по оси NavGraph, а не по распознанной разметке.",
  adasFCW: "Предупреждение о столкновении (FCW)",
  adasAEB: "Автоматическое экстренное торможение (AEB)",
  adasACC: "Адаптивный круиз-контроль (ACC, автопилот)",
  adasBSM: "Контроль слепых зон (BSM)",
  adasParking: "Парковочное предупреждение о препятствиях",
  adasLDW: "Предупреждение о сходе с полосы (LDW)",
  adasState: "Состояние ADAS",
  adasStatus_disabled: "Выключено",
  adasStatus_standby: "Ожидание",
  adasStatus_waiting_for_sensors: "Ожидание свежих данных датчиков",
  adasStatus_vehicle_state_unavailable: "Нет данных автомобиля",
  adasStatus_active: "Мониторинг",
  adasStatus_fcw: "Предупреждение о препятствии",
  adasStatus_aeb: "Экстренное торможение",
  adasStatus_blind_spot: "Машина в слепой зоне",
  adasStatus_lane_departure: "Риск схода с полосы",
  adasAlertFCW: "Препятствие впереди",
  adasAlertAEB: "AEB тормозит",
  adasAlertLeft: "Слева слепая зона",
  adasAlertRight: "Справа слепая зона",
  adasAlertLane: "Риск ухода с полосы",
  adasParkingDistance: "Ближайшее препятствие при парковке",
  adasLeadDistance: "Дистанция до машины впереди",
  adasRequiresSensors: "Включи нужные датчики: LiDAR/радар для передних предупреждений, ультразвук для парковки, сканирование и отслеживание машин для слепых зон.",
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
  adaptivePerformance: "Экспериментальная подстройка нагрузки",
  adaptivePerformanceNote: "RoadMind меняет число лучей и частоту датчиков, пытаясь приблизиться к целевому FPS. Он не сможет обеспечить FPS, недостижимый для самой игры, сцены, трафика или оборудования.",
  adaptiveFpsEnabled: "Автоматически менять качество датчиков",
  targetFps: "Желаемый FPS",
  speedAdaptiveRate: "Менять частоту датчиков по скорости",
  currentFps: "Текущий FPS",
  sensorQuality: "Качество датчиков",
  trafficUpdateRate: "Обновление позиций трафика",
  targetFpsUnreachable: "Достигнута минимальная нагрузка датчиков. RoadMind не может заставить остальную игру держать заданный FPS.",
  activeProfile: "Активный профиль",
  custom: "Свои настройки",
  economy: "Экономный (только LiDAR)",
  balanced: "Сбалансированный",
  high: "Высокое качество",
  lidar: "Передний LiDAR",
  enabled: "Включён",
  resolution: "Вертикальные слои",
  persistence: "Накапливать облако точек",
  pointCloudLimit: "Лимит накопления",
  points: "точек",
  persistenceNote: "Сохраняет точки LiDAR в мировых координатах до конца игровой сессии, включая поверхности машин. Облако остаётся на месте при паузе; большое число точек может снизить FPS.",
  clearCloud: "Очистить облако точек",
  visualizationInertia: "Плавное движение с инерцией",
  detectedVehicles: "Обнаруженные автомобили",
  vehiclesDetected: "Подтверждённо видимых машин",
  vehicleCandidates: "Кандидатов в зонах датчиков",
  trackedObjects: "Треки RoadMind",
  worldModel: "Единая модель мира",
  worldTargets: "Целей",
  worldObstacles: "Кластеров препятствий",
  worldObservations: "Наблюдений",
  worldStaleSources: "Устаревших источников",
  worldQuality: "Ошибки / отброшено / конфликты",
  sourceFresh: "Реестр актуален",
  sourceStale: "Реестр устарел",
  sourceDisabled: "Реестр отключён",
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
  detectionNote: "Обнаружение машин использует данные симулятора. Зелёный контур — видимая машина, жёлтый — кандидат, ожидающий проверки видимости. Это не распознавание по камере или физическому LiDAR.",
  vehicleGeometry: "Геометрия автомобиля и калибровка датчиков",
  calibrated: "Геометрия определена",
  fallbackCalibration: "Используются резервные позиции датчиков",
  calibrationWaiting: "Калибровка ещё не выполнялась",
  recalibrate: "Перекалибровать датчики",
  calibrationNote: "RoadMind один раз при подключении или сбросе считывает ориентированный габарит автомобиля и рассчитывает крепления датчиков в локальных координатах. Включи отладочную визуализацию, чтобы увидеть габариты, оси и датчики. Размещение на разных классах машин ещё требует проверки в игре.",
  calibrationOverlay: "Габариты, оси и все крепления датчиков",
  routeWaypoints: "Точек дорожного маршрута",
  routePilot: "Автопилот RoadMind (эксперимент)",
  routeDescription: "Использует дорожный граф текущей карты. Введи точки маршрута в мировых координатах X,Y,Z (одна точка на строку). Собственный Lua-контроллер RoadMind следует выбранной безопасной траектории и управляет рулём, газом и тормозом со скоростью до 28,8 км/ч. Проверки датчиков и дороги экспериментальные; камеры показывают только поля обзора.",
  routePoints: "Точки маршрута по порядку (мировые координаты, метры)",
  routePointsPlaceholder: "X, Y, Z\nX, Y, Z\nX, Y, Z",
  applyRoutePoints: "Задать маршрут",
  clearRoutePoints: "Очистить маршрут",
  routeInputCount: "Точек задано",
  routeProgress: "Прогресс",
  roadLocalization: "Привязка к дороге",
  routePathOffset: "Отклонение от маршрута",
  destinationSnap: "Смещение до узла дороги",
  routePointsFormat: "Введи от 1 до 12 строк по три числа: X, Y, Z.",
  routePointsJson: "Не удалось прочитать данные маршрута.",
  routePointOffRoad: "Одна из точек слишком далеко от дороги на этой карте.",
  routeNoConnectedPath: "В дорожном графе нет связного пути между этими точками.",
  routePathLimit: "Маршрут превышает текущий лимит узлов.",
  routeLocalizationLost: "Автомобиль покинул маршрут. Автопилот остановлен и попробует перестроить путь.",
  routeCompleted: "Достигнут конечный узел дороги",
  roadQuality_high: "Высокая",
  roadQuality_medium: "Средняя",
  roadQuality_low: "Низкая",
  roadQuality_lost: "Потеряна",
  roadQuality_unavailable: "Недоступна",
  routeSpeed: "Ограничение скорости",
  routeRunning: "Автопилот включён",
  routePaused: "Остановка по датчикам",
  routeStopped: "Автопилот выключен",
  startRoute: "Ехать по дорожной сети",
  stopRoute: "Остановить автопилот",
  routeSafetyStop: "Автомобиль останавливается или ждёт новых данных без препятствий.",
  localPlanner: "Локальный планировщик траектории",
  plannerStatus_idle: "Неактивен",
  plannerStatus_waiting: "Ожидание данных",
  plannerStatus_ready: "Безопасная траектория выбрана",
  plannerStatus_blocked: "Нет безопасной траектории",
  plannerCandidates: "Вариантов",
  plannerRejected: "Отклонено",
  plannerOffset: "Боковое смещение",
  plannerClearance: "Запас до препятствий",
  plannerRoadMargin: "Запас до края коридора",
  plannerReason_inactive: "Автопилот по маршруту выключен.",
  plannerReason_sensor_stale: "Ожидание свежих данных включённого датчика препятствий.",
  plannerReason_road_unavailable: "Привязка к дороге недоступна или недостаточно точна.",
  plannerReason_vehicle_geometry: "Нет откалиброванного габарита автомобиля.",
  plannerReason_route_unavailable: "Нет корректного связного маршрута.",
  plannerReason_selected: "Лучший безопасный вариант показан зелёным.",
  plannerReason_no_safe_candidate: "Все варианты не прошли хотя бы одну проверку безопасности.",
  plannerReason_obstacle: "Все варианты пересекают препятствие или не имеют запаса.",
  plannerReason_vehicle: "Треки машин пересекаются с габаритами вариантов.",
  plannerReason_road_edge: "Габариты машины не помещаются в расчётный дорожный коридор.",
  plannerReason_turning_radius: "Маршрут превышает расчётную манёвренность машины.",
  plannerBoundaryNote: "Границы дороги оцениваются по ширине от NavGraph, а не определяются по полосам.",
  plannerControlNote: "RoadMind теперь следует выбранной траектории собственным Lua-контроллером. Первая версия экспериментальная и требует проверки на разных машинах, в трафике и на перекрёстках.",
  plannerStopError: "Планировщик остановил автопилот: безопасный путь не подтверждён.",
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
  lidarNote: "Плотность и дальность LiDAR задаются выбранным профилем. Отдельный лимит определяет число точек, сохраняемых в облаке.",
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
  visualizationNote: "Инерция сглаживает подходящие мировые точки LiDAR, маршрут и траектории планировщика. Крепления датчиков, камеры, габариты и боксы машин остаются привязаны к актуальной позе корпуса; точки LiDAR на отслеживаемых машинах пересчитываются в локальных координатах кузова. Измерения и World Model не меняются. Накопленная статическая карта LiDAR остаётся неподвижной. Скрытие слоя не останавливает сканирование.",
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

  if (value.startsWith("Local planner:")) return t("plannerStopError")
  if (value === "Waiting for fresh sensor data") return t("routeWaitingData")
  if (value === "Could not find connected road nodes near the vehicle and forward target") return t("routeNoRoadNodes")
  if (value === "No valid forward road path could be built from the current NavGraph") return t("routeNoPath")
  if (value === "NavGraph route API is unavailable on this map") return t("routeApiMissing")
  if (value === "Vehicle heading is invalid" || value === "Could not read vehicle position or heading") return t("routeHeadingInvalid")
  if (value === "The active vehicle changed") return t("routeVehicleChanged")
  if (value === "Route localization lost") return t("routeLocalizationLost")
  if (value === "Route points must be supplied as JSON") return t("routePointsJson")
  if (value === "Route requires 1-12 ordered XYZ points"
      || value === "Route points must contain finite world X, Y and Z coordinates") return t("routePointsFormat")
  if (value === "A route point is too far from the road network"
      || value === "Could not find a road segment for a route point") return t("routePointOffRoad")
  if (value === "No connected NavGraph path exists between route points"
      || value === "The route does not contain enough connected road nodes") return t("routeNoConnectedPath")
  if (value === "Route exceeds the NavGraph waypoint limit") return t("routePathLimit")

  const obstacle = value.match(/^Obstacle inside ([0-9.]+) m safety distance$/)
  if (obstacle) return t("routeObstacle").replace("{distance}", obstacle[1])
  return t("routeGenericError")
}

const settings = ref({
  preset: "economy",
  lidarEnabled: true,
  lidarQuality: "low",
  lidarHz: 1,
  radarEnabled: false,
  radarDensity: "low",
  radarHz: 1,
  ultrasonicEnabled: false,
  ultrasonicDensity: "single",
  ultrasonicHz: 1,
  poseHz: 2,
  lidarPersistenceEnabled: false,
  lidarPointCloudLimit: 5000,
  visualizationInertiaEnabled: true,
  adaptiveFpsEnabled: false,
  targetFps: 60,
  speedAdaptiveRateEnabled: true,
  adasEnabled: false,
  adasFCWEnabled: true,
  adasAEBEnabled: true,
  adasACCEnabled: false,
  adasBSMEnabled: true,
  adasParkingEnabled: true,
  adasLDWEnabled: true,
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
  adasStatus: {
    available: false, fcw: false, aeb: false, leftBlindSpot: false,
    rightBlindSpot: false, laneDeparture: false, status: "disabled"
  },
  plannerStatus: "idle",
  plannerReasonCode: "inactive",
  plannerCandidateCount: 0,
  plannerRejectedCandidateCount: 0,
  plannerSelectedScore: undefined,
  plannerMinimumClearance: undefined,
  plannerMinimumRoadMargin: undefined,
  plannerSelectedOffset: undefined,
  routeInputPointCount: 0,
  routeProgressPercent: 0,
  routeCompleted: false,
  routeDistanceToPath: undefined,
  routeDestinationSnapDistance: undefined,
  roadLocalizationQuality: "unavailable",
  roadDistanceToSegment: undefined,
  persistentLidarPointCount: 0,
  persistentLidarPointLimit: 5000,
  currentFps: undefined,
  targetFps: 60,
  adaptiveQualityTier: 4,
  performanceLimitReached: false,
  trafficUpdateHz: 0,
  unverifiedVehicleCount: 0,
  detectedVehicleCount: 0,
  visibleVehicleCount: 0,
  vehicleCandidateCount: 0,
  vehicleRegistryCount: 0,
  vehicleRegistryTruncated: false,
  occlusionRaycastsPerSecond: 0,
  trackedVehicleCount: 0,
  trackedObjects: [],
  worldModelSchemaVersion: 2,
  worldModelTargetCount: 0,
  worldModelObstacleCount: 0,
  worldModelObservationCount: 0,
  worldModelStaleSourceCount: 0,
  worldModelInvalidObservationCount: 0,
  worldModelDroppedObservationCount: 0,
  worldModelConflictingObservationCount: 0,
  worldModelRegistryEnabled: false,
  worldModelRegistryFresh: false,
  vehicleCalibration: { calibrated: false, error: "Calibration has not been run" },
})
const error = ref("")
const busy = ref(false)
const routePointsText = ref("")
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
    error.value = message === "No active player vehicle" ? t("noVehicle") : (message ? routeErrorText(message) : t("error"))
    return false
  }

  if (result.settings) settings.value = { ...settings.value, ...result.settings }
  if (result.status) status.value = { ...status.value, ...result.status }
  if (result.language) locale.value = result.language
  error.value = ""
  return true
}

let pointCloudLimitTimer = null
let pendingPointCloudLimit = null
let targetFpsTimer = null
let pendingTargetFps = null

function queuePointCloudLimit(value) {
  pendingPointCloudLimit = Number(value)
  settings.value.lidarPointCloudLimit = pendingPointCloudLimit
  if (pointCloudLimitTimer) clearTimeout(pointCloudLimitTimer)
  pointCloudLimitTimer = setTimeout(() => {
    const nextValue = pendingPointCloudLimit
    pendingPointCloudLimit = null
    pointCloudLimitTimer = null
    setSetting("lidarPointCloudLimit", nextValue)
  }, 180)
}

function queueTargetFps(value) {
  pendingTargetFps = Number(value)
  settings.value.targetFps = pendingTargetFps
  if (targetFpsTimer) clearTimeout(targetFpsTimer)
  targetFpsTimer = setTimeout(() => {
    const nextValue = pendingTargetFps
    pendingTargetFps = null
    targetFpsTimer = null
    setSetting("targetFps", nextValue)
  }, 180)
}

onBeforeUnmount(() => {
  if (statusRefreshTimer) {
    clearInterval(statusRefreshTimer)
    statusRefreshTimer = null
  }
  if (pointCloudLimitTimer) {
    clearTimeout(pointCloudLimitTimer)
    pointCloudLimitTimer = null
    if (pendingPointCloudLimit !== null) {
      setSetting("lidarPointCloudLimit", pendingPointCloudLimit)
    }
    pendingPointCloudLimit = null
  }
  if (targetFpsTimer) {
    clearTimeout(targetFpsTimer)
    targetFpsTimer = null
    if (pendingTargetFps !== null) setSetting("targetFps", pendingTargetFps)
    pendingTargetFps = null
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


async function applyRoutePoints() {
  const lines = routePointsText.value.split(/\r?\n/).map(line => line.trim()).filter(Boolean)
  if (lines.length < 1 || lines.length > 12) {
    error.value = t("routePointsFormat")
    return
  }

  const points = []
  for (const line of lines) {
    const values = line.split(/[,\s;]+/).filter(Boolean).map(Number)
    if (values.length !== 3 || values.some(value => !Number.isFinite(value))) {
      error.value = t("routePointsFormat")
      return
    }
    points.push({ x: values[0], y: values[1], z: values[2] })
  }

  busy.value = true
  try {
    acceptResult(await callRoadMind("setRoutePoints", [JSON.stringify(points)]))
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not set route points", e)
  } finally {
    busy.value = false
  }
}

async function clearRoutePoints() {
  busy.value = true
  try {
    acceptResult(await callRoadMind("clearRoutePoints"))
    routePointsText.value = ""
  } catch (e) {
    error.value = t("error")
    console.error("RoadMind: could not clear route points", e)
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
      if (result && result.language) locale.value = result.language
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
.roadmind-world-model {
  display: flex;
  flex-direction: column;
  gap: 0.3em;
  margin-top: 0.5em;
  padding: 0.45em 0.5em;
  border: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.12));
  border-radius: 0.25em;
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
.roadmind-route-points-label {
  font-size: 0.9em;
  font-weight: 600;
}
.roadmind-route-points {
  box-sizing: border-box;
  width: 100%;
  min-height: 5.5em;
  padding: 0.55em;
  border: 1px solid var(--bng-cool-gray-700, rgba(255,255,255,0.18));
  border-radius: 0.25em;
  color: var(--bng-cool-gray-100, #eee);
  background: var(--bng-cool-gray-900, rgba(0,0,0,0.25));
  font: inherit;
  font-family: monospace;
  resize: vertical;
}
.roadmind-route-actions {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 0.5em;
}
.roadmind-error {
  color: var(--bng-add-red-400, #f88);
  font-size: 0.9em;
}
</style>
