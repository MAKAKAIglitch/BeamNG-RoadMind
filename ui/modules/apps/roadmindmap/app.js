(function () {
  "use strict";

  angular.module("beamng.apps").directive("roadmindMap", ["$timeout", function ($timeout) {
    var template =
      '<div class="roadmind-map-app" aria-label="RoadMind Map">' +
        '<canvas class="roadmind-map-canvas"></canvas>' +
      '</div>';

    return {
      template: template,
      replace: true,
      restrict: "EA",
      link: function (scope, element) {
        var root = element[0];
        var canvas = root.querySelector("canvas");
        var ctx = canvas && canvas.getContext("2d");
        var snapshot = null;
        var pollTimer = null;
        var pollWatchdog = null;
        var rafId = 0;
        var lastRenderedAt = 0;
        var destroyed = false;
        var polling = false;
        var lastSnapshotAt = 0;
        var lastWidth = 0;
        var lastHeight = 0;
        var pixelRatio = 1;
        var extensionReady = false;
        var extensionLoading = false;
        var extensionWaiters = [];
        var renderErrorReported = false;
        var cachedSurfaceGeneration = null;
        var cachedSurfaceSnapshotTime = null;
        var cachedSurfacePoints = [];
        var cachedSurfacePointByKey = Object.create(null);
        var cachedSurfacePointBuckets = Object.create(null);
        var cachedSurfaceEdgeKeys = Object.create(null);
        var cachedSurfaceFaces = [];
        var cachedSurfaceEdges = [];
        var cachedSurfaceRenderBuckets = Object.create(null);
        var cachedSurfacePruneCenter = null;
        var cachedSurfacePruneRadius = null;
        var surfacePointCellSize = 2.0;
        var surfaceRenderCellSize = 20.0;

        function lua(command, callback) {
          if (!window.bngApi || typeof window.bngApi.engineLua !== "function") {
            if (callback) callback(null);
            return;
          }
          try {
            window.bngApi.engineLua(command, function (result) {
              if (callback) callback(result);
            });
          } catch (error) {
            if (callback) callback(null);
          }
        }

        // engineLua wraps callback values as a single Lua expression.
        // Load the GE extension once and return snapshots as explicit JSON strings.
        function ensureRoadMind(callback) {
          if (extensionReady) {
            callback();
            return;
          }
          extensionWaiters.push(callback);
          if (extensionLoading) return;
          extensionLoading = true;
          lua("extensions.load('roadmind')", function () {
            extensionLoading = false;
            extensionReady = true;
            var waiters = extensionWaiters.slice();
            extensionWaiters.length = 0;
            waiters.forEach(function (waiter) { waiter(); });
          });
        }

        function roadmindLua(command, callback) {
          ensureRoadMind(function () {
            if (!destroyed) lua(command, callback);
          });
        }

        function decodeLuaResult(value) {
          for (var i = 0; i < 5; i++) {
            if (typeof value === "string") {
              try { value = JSON.parse(value); } catch (error) { return null; }
              continue;
            }
            if (value && typeof value === "object" &&
                value.schemaVersion === undefined && value.result !== undefined) {
              value = value.result;
              continue;
            }
            break;
          }
          return value;
        }

        function asArray(value) {
          if (Array.isArray(value)) return value;
          if (!value || typeof value !== "object") return [];
          var keys = Object.keys(value).filter(function (key) {
            return /^[0-9]+$/.test(key);
          }).sort(function (a, b) { return Number(a) - Number(b); });
          return keys.map(function (key) { return value[key]; });
        }

        function unwrap(value) {
          value = decodeLuaResult(value);
          return value && Number(value.schemaVersion) === 2 ? value : null;
        }

        function schedulePoll(delay) {
          if (destroyed) return;
          if (pollTimer) $timeout.cancel(pollTimer);
          pollTimer = $timeout(poll, delay || 350);
        }

        function poll() {
          if (destroyed || polling) return;
          polling = true;
          pollWatchdog = $timeout(function () {
            polling = false;
            schedulePoll(700);
          }, 1400);
          roadmindLua(
            "jsonEncode(extensions.roadmind.getMapSnapshot())",
            function (result) {
              if (destroyed) return;
              if (pollWatchdog) $timeout.cancel(pollWatchdog);
              polling = false;
              var next = unwrap(result);
              if (next) {
                snapshot = next;
                lastSnapshotAt = Date.now();
              }
              scope.$evalAsync();
              schedulePoll(300);
            }
          );
        }

        function setMapEnabled(enabled) {
          roadmindLua(
            "jsonEncode(extensions.roadmind.setMapEnabled(" +
              (enabled ? "true" : "false") + "))"
          );
        }

        function enableMap() {
          if (destroyed) return;
          roadmindLua("jsonEncode(extensions.roadmind.setMapEnabled(true))", function (result) {
            if (destroyed) return;
            var status = decodeLuaResult(result);
            if (status && status.ok === false) {
              $timeout(enableMap, 1200);
            }
          });
        }

        function clamp(value, min, max) {
          return Math.max(min, Math.min(max, value));
        }

        function dot(a, b) {
          return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        function sub(a, b) {
          return {x: a.x - b.x, y: a.y - b.y, z: a.z - b.z};
        }

        function cross(a, b) {
          return {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
          };
        }

        function normalize(v) {
          var length = Math.sqrt(dot(v, v));
          if (!isFinite(length) || length < 0.00001) return null;
          return {x: v.x / length, y: v.y / length, z: v.z / length};
        }

        function localPoint(point, ego) {
          if (!point || !ego || !ego.position || !ego.forward || !ego.right || !ego.up) return null;
          var delta = sub(point, ego.position);
          return {
            x: dot(delta, ego.right),
            y: dot(delta, ego.forward),
            z: dot(delta, ego.up)
          };
        }

        function makeProjection(ego, width, height, mapSettings) {
          mapSettings = mapSettings || {};
          var speed = clamp(Number(ego.speedMps) || 0, 0, 45);
          var distance = clamp((Number(mapSettings.cameraDistanceM) || 10.5) + speed * 0.15, 8, 40);
          var cameraHeight = clamp((Number(mapSettings.cameraHeightM) || 7.5) + speed * 0.05, 4, 18);
          var lookAhead = clamp((Number(mapSettings.cameraLookAheadM) || 13) + speed * 0.25, 6, 45);
          var camera = {x: 0, y: -distance, z: cameraHeight};
          var target = {x: 0, y: lookAhead, z: -0.3};
          var forward = normalize(sub(target, camera));
          var right = normalize(cross(forward, {x: 0, y: 0, z: 1}));
          var up = right && normalize(cross(right, forward));
          if (!forward || !right || !up) return null;
          var focal = Math.min(width, height) * 1.22 * clamp(11.5 / distance, 0.62, 1.24);
          return function (point) {
            var relative = sub(point, camera);
            var depth = dot(relative, forward);
            if (depth <= 0.35) return null;
            var sx = width * 0.5 + dot(relative, right) * focal / depth;
            var sy = height * 0.64 - dot(relative, up) * focal / depth;
            if (!isFinite(sx) || !isFinite(sy)) return null;
            return {x: sx, y: sy, depth: depth};
          };
        }

        function shade(hex, factor) {
          var value = String(hex || "#4b6574").replace("#", "");
          if (value.length !== 6) return hex;
          var r = parseInt(value.substring(0, 2), 16);
          var g = parseInt(value.substring(2, 4), 16);
          var b = parseInt(value.substring(4, 6), 16);
          r = clamp(Math.round(r * factor), 0, 255);
          g = clamp(Math.round(g * factor), 0, 255);
          b = clamp(Math.round(b * factor), 0, 255);
          return "#" + [r, g, b].map(function (n) {
            return n.toString(16).padStart(2, "0");
          }).join("");
        }

        function draw(snapshotData) {
          if (!ctx || !canvas) return;
          var cssWidth = Math.max(1, canvas.clientWidth || root.clientWidth || 360);
          var cssHeight = Math.max(1, canvas.clientHeight || root.clientHeight || 260);
          pixelRatio = 1;
          var bufferWidth = Math.round(cssWidth * pixelRatio);
          var bufferHeight = Math.round(cssHeight * pixelRatio);
          if (bufferWidth !== lastWidth || bufferHeight !== lastHeight) {
            canvas.width = bufferWidth;
            canvas.height = bufferHeight;
            lastWidth = bufferWidth;
            lastHeight = bufferHeight;
          }
          ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
          ctx.clearRect(0, 0, cssWidth, cssHeight);
          var background = ctx.createLinearGradient(0, 0, 0, cssHeight);
          background.addColorStop(0, "#111b24");
          background.addColorStop(0.58, "#0b141c");
          background.addColorStop(1, "#070c11");
          ctx.fillStyle = background;
          ctx.fillRect(0, 0, cssWidth, cssHeight);

          var ego = snapshotData && snapshotData.vehicle;
          var mapSettings = snapshotData && snapshotData.mapSettings || {};
          var polygonLimit = clamp(Number(mapSettings.maxPolygons) || 3000, 500, 8000);
          var lineLimit = polygonLimit * 2;
          var markerLimit = Math.min(5000, polygonLimit);
          if (!ego || !ego.position || !ego.forward || !ego.right || !ego.up) {
            // Keep a dark perspective grid while pose data is unavailable.
            // No road or vehicle is fabricated from missing observations.
            ctx.save();
            ctx.strokeStyle = "rgba(75, 96, 112, 0.36)";
            ctx.lineWidth = 0.7;
            var vanishingX = cssWidth * 0.5;
            var vanishingY = cssHeight * 0.20;
            for (var gx = -8; gx <= 8; gx++) {
              ctx.beginPath();
              ctx.moveTo(vanishingX + gx * 2, vanishingY);
              ctx.lineTo(vanishingX + gx * cssWidth * 0.095, cssHeight);
              ctx.stroke();
            }
            for (var gy = 0; gy < 9; gy++) {
              var gridY = vanishingY + Math.pow((gy + 1) / 9, 1.7) * cssHeight * 0.82;
              ctx.beginPath();
              ctx.moveTo(0, gridY);
              ctx.lineTo(cssWidth, gridY);
              ctx.stroke();
            }
            ctx.restore();
            return;
          }

          var project = makeProjection(ego, cssWidth, cssHeight, mapSettings);
          if (!project) return;
          var polygons = [];
          var lines = [];
          var markers = [];

          function addPolygon(points, fill, alpha, stroke) {
            if (polygons.length >= polygonLimit) return;
            var projected = [];
            var depth = 0;
            for (var i = 0; i < points.length; i++) {
              var p = project(points[i]);
              if (!p) return;
              projected.push(p);
              depth += p.depth;
            }
            polygons.push({
              points: projected,
              fill: fill,
              alpha: alpha === undefined ? 1 : alpha,
              stroke: stroke || null,
              depth: depth / projected.length
            });
          }

          function addLine(points, color, lineWidth, alpha, dash) {
            if (lines.length >= lineLimit) return;
            var projected = [];
            var depth = 0;
            for (var i = 0; i < points.length; i++) {
              var p = project(points[i]);
              if (!p) return;
              projected.push(p);
              depth += p.depth;
            }
            if (projected.length < 2) return;
            lines.push({
              points: projected,
              color: color,
              width: lineWidth || 1,
              alpha: alpha === undefined ? 1 : alpha,
              dash: dash || [],
              depth: depth / projected.length
            });
          }

          function addMarker(point, color, radius, alpha) {
            if (markers.length >= markerLimit) return;
            var projected = project(point);
            if (!projected) return;
            markers.push({
              x: projected.x, y: projected.y, depth: projected.depth,
              color: color, radius: clamp(radius * 42 / projected.depth, 1.1, 3.8),
              alpha: alpha === undefined ? 0.9 : alpha
            });
          }

          function addBox(center, length, width, height, yaw, color, alpha) {
            var c = Math.cos(yaw || 0);
            var s = Math.sin(yaw || 0);
            function point(x, y, z) {
              return {
                x: center.x + x * c + y * s,
                y: center.y - x * s + y * c,
                z: center.z + z
              };
            }
            var w = width * 0.5;
            var l = length * 0.5;
            var h = height * 0.5;
            var v = [
              point(-w, -l, -h), point(w, -l, -h),
              point(w, l, -h), point(-w, l, -h),
              point(-w, -l, h), point(w, -l, h),
              point(w, l, h), point(-w, l, h)
            ];
            var faces = [
              [v[0], v[1], v[2], v[3]],
              [v[4], v[7], v[6], v[5]],
              [v[0], v[4], v[5], v[1]],
              [v[1], v[5], v[6], v[2]],
              [v[2], v[6], v[7], v[3]],
              [v[3], v[7], v[4], v[0]]
            ];
            var factors = [0.48, 1.2, 0.88, 0.72, 0.62, 0.8];
            for (var f = 0; f < faces.length; f++) {
              addPolygon(faces[f], shade(color, factors[f]), alpha);
            }
          }

          function addVehicle(position, dimensions, yaw, isPlayer, visualState) {
            if (!position) return;
            var length = clamp(Number(dimensions && dimensions.length) || (isPlayer ? 4.6 : 4.4), 2.2, 12);
            var width = clamp(Number(dimensions && dimensions.width) || (isPlayer ? 1.9 : 1.85), 1.0, 3.0);
            var height = clamp(Number(dimensions && dimensions.height) || 1.5, 0.8, 3.4);
            var base = {x: position.x, y: position.y, z: position.z};
            var bodyColor = isPlayer ? "#278b9c" : (visualState === "predicted" ? "#9b8250" : "#8197a7");
            var glassColor = isPlayer ? "#20414a" : "#344b59";
            var trimColor = "#111820";
            var lightColor = isPlayer ? "#b7f4ff" : "#d8e7ee";
            function vehiclePoint(x, y, z) {
              var cosine = Math.cos(yaw || 0);
              var sine = Math.sin(yaw || 0);
              return {
                x: base.x + x * cosine + y * sine,
                y: base.y - x * sine + y * cosine,
                z: base.z + z
              };
            }

            // Chassis, lower body and four visible wheel blocks give the car
            // a recognisable footprint instead of the old pair of stacked boxes.
            addBox(vehiclePoint(0, 0, -height * 0.18),
              length * 0.98, width * 0.94, height * 0.27, yaw, trimColor, 1);
            addBox(vehiclePoint(0, 0, -height * 0.04),
              length * 0.94, width * 0.96, height * 0.34, yaw, bodyColor,
              isPlayer ? 1 : 0.96);

            var wheelY = length * 0.30;
            var wheelX = width * 0.43;
            var wheelZ = -height * 0.24;
            for (var wx = -1; wx <= 1; wx += 2) {
              for (var wy = -1; wy <= 1; wy += 2) {
                addBox(vehiclePoint(wx * wheelX, wy * wheelY, wheelZ),
                  length * 0.17, width * 0.20, height * 0.43, yaw, "#090e13", 1);
              }
            }

            // Sloped cabin: windshield, rear glass, side windows and roof.
            var lowerHalf = width * 0.355;
            var roofHalf = width * 0.285;
            var frontBaseY = length * 0.19;
            var rearBaseY = -length * 0.22;
            var frontRoofY = length * 0.105;
            var rearRoofY = -length * 0.12;
            var baseZ = height * 0.105;
            var roofZ = height * 0.43;
            var fl = vehiclePoint(-lowerHalf, frontBaseY, baseZ);
            var fr = vehiclePoint(lowerHalf, frontBaseY, baseZ);
            var rl = vehiclePoint(-lowerHalf, rearBaseY, baseZ);
            var rr = vehiclePoint(lowerHalf, rearBaseY, baseZ);
            var ftl = vehiclePoint(-roofHalf, frontRoofY, roofZ);
            var ftr = vehiclePoint(roofHalf, frontRoofY, roofZ);
            var rtl = vehiclePoint(-roofHalf, rearRoofY, roofZ);
            var rtr = vehiclePoint(roofHalf, rearRoofY, roofZ);
            addPolygon([fl, fr, ftr, ftl], isPlayer ? "#315f69" : "#526b7b", 0.98);
            addPolygon([rl, rtl, rtr, rr], shade(glassColor, 1.12), 0.9);
            addPolygon([fl, ftl, rtl, rl], glassColor, 0.92);
            addPolygon([fr, rr, rtr, ftr], shade(glassColor, 0.82), 0.92);
            addPolygon([ftl, ftr, rtr, rtl], shade(bodyColor, 1.16), 1);
            addPolygon([
              vehiclePoint(-width * 0.43, length * 0.48, -height * 0.02),
              vehiclePoint(width * 0.43, length * 0.48, -height * 0.02),
              vehiclePoint(width * 0.38, length * 0.42, height * 0.06),
              vehiclePoint(-width * 0.38, length * 0.42, height * 0.06)
            ], lightColor, 0.92);
            addPolygon([
              vehiclePoint(-width * 0.40, -length * 0.48, -height * 0.01),
              vehiclePoint(width * 0.40, -length * 0.48, -height * 0.01),
              vehiclePoint(width * 0.36, -length * 0.42, height * 0.06),
              vehiclePoint(-width * 0.36, -length * 0.42, height * 0.06)
            ], "#e06e65", 0.88);
          }

          function drawPolyline(points, color, width, alpha, dash) {
            if (!points || points.length < 2) return;
            addLine(points, color, width, alpha, dash);
          }

          // A restrained ground plane and reference grid establish depth without
          // inventing road markings or revealing hidden level geometry.
          addPolygon([
            {x: -90, y: -35, z: -1.1}, {x: 90, y: -35, z: -1.1},
            {x: 90, y: 125, z: -1.1}, {x: -90, y: 125, z: -1.1}
          ], "#070c12", 1);
          for (var gy = -20; gy <= 100; gy += 10) {
            addLine([{x: -55, y: gy, z: -1.08}, {x: 55, y: gy, z: -1.08}],
              "#263441", 0.8, 0.58, [2, 8]);
          }
          for (var gx = -50; gx <= 50; gx += 10) {
            addLine([{x: gx, y: -20, z: -1.08}, {x: gx, y: 100, z: -1.08}],
              "#263441", 0.8, 0.48, [2, 8]);
          }

          var road = snapshotData.road || {};
          var laneGraph = snapshotData.laneGraph || {};
          var graphSegments = asArray(laneGraph.segments);
          var corridor = asArray(road.corridor).map(function (point) {
            return localPoint(point, ego);
          }).filter(Boolean);

          if (graphSegments.length > 0) {
            // Render the bounded connected road network, including branches.
            // Lane marks appear only where NavGraph lane metadata exists.
            for (var gs = 0; gs < Math.min(graphSegments.length, 96); gs++) {
              var graphSegment = graphSegments[gs];
              var graphStart = localPoint(graphSegment.start, ego);
              var graphFinish = localPoint(graphSegment.finish, ego);
              if (!graphStart || !graphFinish) continue;
              var gx = graphFinish.x - graphStart.x;
              var gy = graphFinish.y - graphStart.y;
              var gl = Math.sqrt(gx * gx + gy * gy);
              if (gl < 0.2) continue;
              var gnx = -gy / gl;
              var gny = gx / gl;
              var ghw = clamp(Number(graphSegment.roadHalfWidth) || 3.5, 2.0, 18);
              var gLeftA = {x: graphStart.x + gnx * ghw, y: graphStart.y + gny * ghw, z: graphStart.z - 0.10};
              var gRightA = {x: graphStart.x - gnx * ghw, y: graphStart.y - gny * ghw, z: graphStart.z - 0.10};
              var gLeftB = {x: graphFinish.x + gnx * ghw, y: graphFinish.y + gny * ghw, z: graphFinish.z - 0.10};
              var gRightB = {x: graphFinish.x - gnx * ghw, y: graphFinish.y - gny * ghw, z: graphFinish.z - 0.10};
              addPolygon([gLeftA, gRightA, gRightB, gLeftB],
                graphSegment.current ? "#263644" : "#202a34", graphSegment.current ? 0.99 : 0.94,
                graphSegment.current ? "#657e8d" : "#50616e");
              addLine([gLeftA, gLeftB], "#667b89", graphSegment.current ? 1.25 : 1.0, 0.78);
              addLine([gRightA, gRightB], "#667b89", graphSegment.current ? 1.25 : 1.0, 0.78);

              var graphLaneModel = graphSegment.laneModel || {};
              var graphBoundaries = asArray(graphLaneModel.laneBoundaryOffsets);
              var laneModelKnown = graphLaneModel.status === "metadata";
              var laneModelInferred = graphLaneModel.status === "traffic-inferred"
                || graphLaneModel.status === "traffic-provisional";
              if (laneModelKnown || laneModelInferred) {
                for (var gb = 0; gb < graphBoundaries.length; gb++) {
                  var boundaryOffset = Number(graphBoundaries[gb]);
                  if (!isFinite(boundaryOffset)) continue;
                  addLine([
                    {x: graphStart.x + gnx * boundaryOffset, y: graphStart.y + gny * boundaryOffset, z: graphStart.z - 0.055},
                    {x: graphFinish.x + gnx * boundaryOffset, y: graphFinish.y + gny * boundaryOffset, z: graphFinish.z - 0.055}
                  ], laneModelKnown ? "#8799a7" : "#b18b56",
                    graphSegment.current ? 1.05 : 0.85,
                    laneModelKnown ? (graphSegment.current ? 0.72 : 0.54) : 0.65,
                    laneModelKnown ? [5, 7] : [2, 5]);
                }
              }

              var graphLanes = asArray(graphSegment.lanes);
              for (var gli = 0; gli < graphLanes.length; gli++) {
                var graphLane = graphLanes[gli];
                var laneStart = localPoint(graphLane.start, ego);
                var laneFinish = localPoint(graphLane.finish, ego);
                if (laneStart && laneFinish) {
                  var currentLane = graphLane.id === laneGraph.currentLaneId;
                  var inferredLane = graphLane.source === "tracked-traffic-trajectories";
                  addLine([laneStart, laneFinish],
                    currentLane ? "#55b4f3" : (inferredLane ? "#b18b56" : "#718896"),
                    currentLane ? 1.45 : 0.95,
                    currentLane ? 0.88 : (inferredLane ? 0.56 : 0.46),
                    inferredLane ? [3, 5] : [7, 8]);
                }
              }
            }
          } else {
            var halfWidth = clamp(Number(road.roadHalfWidth) || 3.5, 2.2, 18);
            for (var r = 0; r < corridor.length - 1; r++) {
              var a = corridor[r], b = corridor[r + 1];
              var dx = b.x - a.x, dy = b.y - a.y;
              var magnitude = Math.sqrt(dx * dx + dy * dy);
              if (magnitude < 0.2) continue;
              var nx = -dy / magnitude, ny = dx / magnitude;
              var leftA = {x: a.x + nx * halfWidth, y: a.y + ny * halfWidth, z: a.z - 0.10};
              var rightA = {x: a.x - nx * halfWidth, y: a.y - ny * halfWidth, z: a.z - 0.10};
              var leftB = {x: b.x + nx * halfWidth, y: b.y + ny * halfWidth, z: b.z - 0.10};
              var rightB = {x: b.x - nx * halfWidth, y: b.y - ny * halfWidth, z: b.z - 0.10};
              addPolygon([leftA, rightA, rightB, leftB], "#202b35", 0.97);
              addLine([leftA, leftB], "#53616c", 1.05, 0.64);
              addLine([rightA, rightB], "#53616c", 1.05, 0.64);
            }
            var fallbackModel = road.laneModel || {};
            var fallbackBoundaries = asArray(fallbackModel.laneBoundaryOffsets);
            if (fallbackModel.status === "metadata") {
              for (var fli = 0; fli < fallbackBoundaries.length; fli++) {
                var fallbackOffset = Number(fallbackBoundaries[fli]);
                if (!isFinite(fallbackOffset)) continue;
                for (var fls = 0; fls < corridor.length - 1; fls++) {
                  var fa = corridor[fls], fb = corridor[fls + 1];
                  var fdx = fb.x - fa.x, fdy = fb.y - fa.y;
                  var fl = Math.sqrt(fdx * fdx + fdy * fdy);
                  if (fl < 0.2) continue;
                  var fnx = -fdy / fl, fny = fdx / fl;
                  addLine([
                    {x: fa.x + fnx * fallbackOffset, y: fa.y + fny * fallbackOffset, z: fa.z - 0.055},
                    {x: fb.x + fnx * fallbackOffset, y: fb.y + fny * fallbackOffset, z: fb.z - 0.055}
                  ], "#77838c", 0.85, 0.52, [5, 7]);
                }
              }
            }
          }

          var routeWorld = asArray(snapshotData.selectedTrajectory);
          if (routeWorld.length < 2) routeWorld = asArray(snapshotData.route);
          var routeLocal = routeWorld.map(function (point) {
            return localPoint(point, ego);
          }).filter(Boolean);

          if (asArray(snapshotData.selectedTrajectory).length >= 2) {
            var leftSide = [];
            var rightSide = [];
            var halfVehicleWidth = clamp(Number(ego.dimensions && ego.dimensions.width) || 1.9, 1, 3.2) * 0.5 + 0.24;
            for (var ti = 0; ti < routeLocal.length; ti++) {
              var previous = routeLocal[Math.max(0, ti - 1)];
              var next = routeLocal[Math.min(routeLocal.length - 1, ti + 1)];
              var tx = next.x - previous.x;
              var ty = next.y - previous.y;
              var tl = Math.sqrt(tx * tx + ty * ty);
              if (tl < 0.1) continue;
              var tnx = -ty / tl;
              var tny = tx / tl;
              leftSide.push({x: routeLocal[ti].x + tnx * halfVehicleWidth, y: routeLocal[ti].y + tny * halfVehicleWidth, z: routeLocal[ti].z - 0.035});
              rightSide.push({x: routeLocal[ti].x - tnx * halfVehicleWidth, y: routeLocal[ti].y - tny * halfVehicleWidth, z: routeLocal[ti].z - 0.035});
            }
            if (leftSide.length >= 2) {
              addPolygon(leftSide.concat(rightSide.slice().reverse()), "#164879", 0.38);
            }
          }
          drawPolyline(routeLocal, "#48a8ff", 2.5, 0.96);

          // Moving returns are a transient layer. Static geometry is cached in
          // BEAMNG WORLD coordinates so driving does not erase already mapped areas.
          var geometry = asArray(snapshotData.geometry);
          for (var gi = 0; gi < geometry.length; gi++) {
            var geometryItem = geometry[gi];
            var geometryPoint = localPoint(geometryItem.position, ego);
            var geometryAge = Number(geometryItem.age);
            if (!geometryPoint || !isFinite(geometryAge) || geometryAge > 2.5) continue;
            addMarker(geometryPoint, "#91aeba",
              0.10 + (Number(geometryItem.observationCount) || 1) * 0.018,
              clamp(0.92 - geometryAge * 0.18, 0.38, 0.9));
          }

          var staticGeometry = asArray(snapshotData.staticGeometry);
          for (var si = 0; si < staticGeometry.length; si++) {
            var staticItem = staticGeometry[si];
            var staticPoint = localPoint(staticItem.position, ego);
            if (!staticPoint) continue;
            var staticAge = Number(staticItem.dataAge) || 0;
            addMarker(staticPoint, "#718d9b",
              0.12 + Math.min(8, Number(staticItem.hitCount) || 0) * 0.012,
              clamp(0.82 - staticAge / 2400, 0.38, 0.82));
          }

          var sessionMap = snapshotData.sessionMap || {};
          var mapGeneration = Number(sessionMap.generation) || 1;
          if (cachedSurfaceGeneration !== mapGeneration) {
            cachedSurfaceGeneration = mapGeneration;
            cachedSurfaceSnapshotTime = null;
            cachedSurfacePoints = [];
            cachedSurfacePointByKey = Object.create(null);
            cachedSurfacePointBuckets = Object.create(null);
            cachedSurfaceEdgeKeys = Object.create(null);
            cachedSurfaceFaces = [];
            cachedSurfaceEdges = [];
            cachedSurfaceRenderBuckets = Object.create(null);
          }

          function surfacePointBucket(x, y) {
            return Math.floor(x / surfacePointCellSize) + ":" +
              Math.floor(y / surfacePointCellSize);
          }

          function surfaceRenderBucket(x, y) {
            return Math.floor(x / surfaceRenderCellSize) + ":" +
              Math.floor(y / surfaceRenderCellSize);
          }

          function addSurfaceRenderItem(item, x, y) {
            var key = surfaceRenderBucket(x, y);
            if (!cachedSurfaceRenderBuckets[key]) {
              cachedSurfaceRenderBuckets[key] = {faces: [], edges: []};
            }
            if (item.kind === "face") cachedSurfaceRenderBuckets[key].faces.push(item);
            else cachedSurfaceRenderBuckets[key].edges.push(item);
          }

          function nearbySurfaceIndices(point, radius) {
            var result = [];
            var minX = Math.floor((point.x - radius) / surfacePointCellSize);
            var maxX = Math.floor((point.x + radius) / surfacePointCellSize);
            var minY = Math.floor((point.y - radius) / surfacePointCellSize);
            var maxY = Math.floor((point.y + radius) / surfacePointCellSize);
            for (var bx = minX; bx <= maxX; bx++) {
              for (var by = minY; by <= maxY; by++) {
                var bucket = cachedSurfacePointBuckets[bx + ":" + by] || [];
                for (var bi = 0; bi < bucket.length; bi++) result.push(bucket[bi]);
              }
            }
            return result;
          }

          // Maintain a moving circular 500 m cache around the player.
          // Reindex retained data only after 25 m of movement or a radius change.
          var mapRadiusM = clamp(Number(sessionMap.radiusM || mapSettings.radiusM) || 500, 100, 500);
          var cacheCenterX = Number(ego.position.x);
          var cacheCenterY = Number(ego.position.y);
          var centerDeltaX = cachedSurfacePruneCenter
            ? cacheCenterX - cachedSurfacePruneCenter.x : Infinity;
          var centerDeltaY = cachedSurfacePruneCenter
            ? cacheCenterY - cachedSurfacePruneCenter.y : Infinity;
          if (!cachedSurfacePruneCenter || cachedSurfacePruneRadius !== mapRadiusM
              || centerDeltaX * centerDeltaX + centerDeltaY * centerDeltaY >= 625) {
            var radiusSquared = mapRadiusM * mapRadiusM;
            cachedSurfacePoints = cachedSurfacePoints.filter(function (point) {
              var dx = point.position.x - cacheCenterX;
              var dy = point.position.y - cacheCenterY;
              return dx * dx + dy * dy <= radiusSquared;
            });
            cachedSurfacePointByKey = Object.create(null);
            cachedSurfacePointBuckets = Object.create(null);
            for (var cp = 0; cp < cachedSurfacePoints.length; cp++) {
              var keptPoint = cachedSurfacePoints[cp];
              cachedSurfacePointByKey[keptPoint.key] = keptPoint;
              var keptBucketKey = surfacePointBucket(keptPoint.position.x, keptPoint.position.y);
              if (!cachedSurfacePointBuckets[keptBucketKey]) cachedSurfacePointBuckets[keptBucketKey] = [];
              cachedSurfacePointBuckets[keptBucketKey].push(cp);
            }
            cachedSurfaceEdges = cachedSurfaceEdges.filter(function (edgeItem) {
              if (!edgeItem || !edgeItem.a || !edgeItem.b) return false;
              var adx = edgeItem.a.position.x - cacheCenterX;
              var ady = edgeItem.a.position.y - cacheCenterY;
              var bdx = edgeItem.b.position.x - cacheCenterX;
              var bdy = edgeItem.b.position.y - cacheCenterY;
              return adx * adx + ady * ady <= radiusSquared
                && bdx * bdx + bdy * bdy <= radiusSquared;
            });
            cachedSurfaceFaces = cachedSurfaceFaces.filter(function (faceItem) {
              if (!faceItem || !faceItem.a || !faceItem.b || !faceItem.c) return false;
              var vertices = [faceItem.a.position, faceItem.b.position, faceItem.c.position];
              for (var vi = 0; vi < vertices.length; vi++) {
                var dx = vertices[vi].x - cacheCenterX;
                var dy = vertices[vi].y - cacheCenterY;
                if (dx * dx + dy * dy > radiusSquared) return false;
              }
              return true;
            });
            cachedSurfaceEdgeKeys = Object.create(null);
            cachedSurfaceRenderBuckets = Object.create(null);
            for (var ce = 0; ce < cachedSurfaceEdges.length; ce++) {
              var keptEdge = cachedSurfaceEdges[ce];
              cachedSurfaceEdgeKeys[keptEdge.key] = true;
              addSurfaceRenderItem(keptEdge,
                (keptEdge.a.position.x + keptEdge.b.position.x) * 0.5,
                (keptEdge.a.position.y + keptEdge.b.position.y) * 0.5);
            }
            for (var cf = 0; cf < cachedSurfaceFaces.length; cf++) {
              var keptFace = cachedSurfaceFaces[cf];
              addSurfaceRenderItem(keptFace,
                (keptFace.a.position.x + keptFace.b.position.x + keptFace.c.position.x) / 3,
                (keptFace.a.position.y + keptFace.b.position.y + keptFace.c.position.y) / 3);
            }
            cachedSurfacePruneCenter = {x: cacheCenterX, y: cacheCenterY};
            cachedSurfacePruneRadius = mapRadiusM;
          }

          // Snapshot timestamps change only when new sensor/world-model data arrives,
          // not every animation frame. Merge each snapshot once.
          var surfaceSnapshotTime = Number(snapshotData.timestamp) || 0;
          if (cachedSurfaceSnapshotTime !== surfaceSnapshotTime) {
            cachedSurfaceSnapshotTime = surfaceSnapshotTime;
            for (var sp = 0; sp < staticGeometry.length
                && cachedSurfacePoints.length < 12000; sp++) {
              var sourcePoint = staticGeometry[sp];
              var world = sourcePoint && sourcePoint.position;
              if (!world || !isFinite(Number(world.x)) ||
                  !isFinite(Number(world.y)) || !isFinite(Number(world.z))) continue;

              var pointKey = sourcePoint.geometryId ||
                (Math.round(Number(world.x) / 0.26) + ":" +
                 Math.round(Number(world.y) / 0.26) + ":" +
                 Math.round(Number(world.z) / 0.24));
              var cachedPoint = cachedSurfacePointByKey[pointKey];
              if (cachedPoint) {
                cachedPoint.confidence = Math.max(cachedPoint.confidence,
                  Number(sourcePoint.confidence) || 0.45);
                cachedPoint.hits = Math.max(cachedPoint.hits,
                  Number(sourcePoint.hitCount) || 1);
                continue;
              }

              var point = {
                key: pointKey,
                position: {x: Number(world.x), y: Number(world.y), z: Number(world.z)},
                confidence: clamp(Number(sourcePoint.confidence) || 0.45, 0.1, 1),
                hits: Math.max(1, Number(sourcePoint.hitCount) || 1),
                support: 0,
                airborne: false
              };
              var pointIndex = cachedSurfacePoints.length;
              cachedSurfacePoints.push(point);
              cachedSurfacePointByKey[pointKey] = point;
              var pointBucketKey = surfacePointBucket(point.position.x, point.position.y);
              if (!cachedSurfacePointBuckets[pointBucketKey]) {
                cachedSurfacePointBuckets[pointBucketKey] = [];
              }
              cachedSurfacePointBuckets[pointBucketKey].push(pointIndex);

              var nearby = nearbySurfaceIndices(point.position, 4.55);
              var neighbors = [];
              var sameLevelCount = 0;
              var lowerSupportCount = 0;
              var heights = [];
              for (var ni = 0; ni < nearby.length; ni++) {
                var neighborIndex = nearby[ni];
                var neighbor = cachedSurfacePoints[neighborIndex];
                if (!neighbor || neighborIndex === pointIndex) continue;
                var dx = neighbor.position.x - point.position.x;
                var dy = neighbor.position.y - point.position.y;
                var dz = neighbor.position.z - point.position.z;
                var horizontal = Math.sqrt(dx * dx + dy * dy);
                if (horizontal < 0.18 || horizontal > 4.55) continue;
                if (Math.abs(dz) <= 0.58) sameLevelCount++;
                if (dz < -0.78) lowerSupportCount++;
                if (horizontal <= 2.6) heights.push(neighbor.position.z);
                if (Math.abs(dz) <= 0.76) {
                  neighbors.push({
                    index: neighborIndex,
                    point: neighbor,
                    distance: horizontal,
                    angle: Math.atan2(dy, dx)
                  });
                }
              }
              heights.sort(function (a, b) { return a - b; });
              var medianHeight = heights.length
                ? heights[Math.floor(heights.length * 0.5)] : point.position.z;
              point.airborne = lowerSupportCount >= 2 && sameLevelCount < 2 &&
                point.position.z - medianHeight > 0.82;
              point.support = sameLevelCount;
              neighbors.sort(function (a, b) { return a.distance - b.distance; });

              // Build the local graph once per newly discovered world point.
              // Old points and their links stay in memory until the 1 km segment rolls over.
              if (!point.airborne) {
                var usedLongLinks = 0;
                var maxEdges = Math.min(neighbors.length, 18);
                for (var ei = 0; ei < maxEdges; ei++) {
                  var edgeNeighbor = neighbors[ei];
                  var edgeStable = point.confidence >= 0.42 &&
                    edgeNeighbor.point.confidence >= 0.42 &&
                    (point.support >= 1 || point.hits >= 2) &&
                    (edgeNeighbor.point.support >= 1 || edgeNeighbor.point.hits >= 2);
                  if (edgeNeighbor.distance > 3.15 &&
                      (edgeNeighbor.distance > 4.55 || !edgeStable || usedLongLinks >= 3)) continue;
                  if (edgeNeighbor.distance > 3.15) usedLongLinks++;
                  var edgeLow = Math.min(pointIndex, edgeNeighbor.index);
                  var edgeHigh = Math.max(pointIndex, edgeNeighbor.index);
                  var edgeKey = edgeLow + ":" + edgeHigh;
                  if (!cachedSurfaceEdgeKeys[edgeKey]) {
                    cachedSurfaceEdgeKeys[edgeKey] = true;
                    var edgeItem = {
                      key: edgeKey, kind: "edge",
                      a: point, b: edgeNeighbor.point
                    };
                    cachedSurfaceEdges.push(edgeItem);
                    addSurfaceRenderItem(edgeItem,
                      (point.position.x + edgeNeighbor.point.position.x) * 0.5,
                      (point.position.y + edgeNeighbor.point.position.y) * 0.5);
                  }
                }

                var angularNeighbors = neighbors.slice(0, 9).slice().sort(function (a, b) {
                  return a.angle - b.angle;
                });
                for (var ti = 0; ti < angularNeighbors.length; ti++) {
                  var first = angularNeighbors[ti];
                  var second = angularNeighbors[(ti + 1) % angularNeighbors.length];
                  if (!first || !second || first.index === second.index) continue;
                  var ex = first.point.position.x - second.point.position.x;
                  var ey = first.point.position.y - second.point.position.y;
                  var neighborEdge = Math.sqrt(ex * ex + ey * ey);
                  var heightDifference = Math.abs(first.point.position.z - second.point.position.z);
                  var angleGap = (second.angle - first.angle + Math.PI * 2) % (Math.PI * 2);
                  var area2 = Math.abs(
                    (first.point.position.x - point.position.x) *
                      (second.point.position.y - point.position.y) -
                    (first.point.position.y - point.position.y) *
                      (second.point.position.x - point.position.x));
                  var longestEdge = Math.max(first.distance, second.distance, neighborEdge);
                  var stableBridge = longestEdge <= 4.55 &&
                    point.confidence >= 0.48 && first.point.confidence >= 0.48 &&
                    second.point.confidence >= 0.48 &&
                    (point.support >= 1 || point.hits >= 2) &&
                    (first.point.support >= 1 || first.point.hits >= 2) &&
                    (second.point.support >= 1 || second.point.hits >= 2);
                  if (angleGap > Math.PI * 0.92 || neighborEdge < 0.18 ||
                      heightDifference > 0.68 || area2 < 0.055 || area2 > 9.5 ||
                      (longestEdge > 3.35 && !stableBridge) ||
                      first.point.airborne || second.point.airborne) continue;

                  var faceIndices = [pointIndex, first.index, second.index].sort(function (a, b) {
                    return a - b;
                  });
                  var faceKey = faceIndices.join(":");
                  var faceExists = cachedSurfacePoints[pointIndex].faceKeys;
                  if (!faceExists) faceExists = cachedSurfacePoints[pointIndex].faceKeys = Object.create(null);
                  if (faceExists[faceKey]) continue;
                  faceExists[faceKey] = true;
                  var faceItem = {
                    key: faceKey, kind: "face",
                    a: point, b: first.point, c: second.point,
                    confidence: Math.min(point.confidence, first.point.confidence,
                      second.point.confidence),
                    gapBridge: longestEdge > 3.35
                  };
                  cachedSurfaceFaces.push(faceItem);
                  addSurfaceRenderItem(faceItem,
                    (point.position.x + first.point.position.x + second.point.position.x) / 3,
                    (point.position.y + first.point.position.y + second.point.position.y) / 3);
                }
              }
            }
          }

          // Query only world-space buckets intersecting the current view. Geometry
          // remains global; only its projection is recalculated around the ego car.
          var visibleSurfaceFaces = [];
          var visibleSurfaceEdges = [];
          var viewMinX = ego.position.x - 145;
          var viewMaxX = ego.position.x + 145;
          var viewMinY = ego.position.y - 145;
          var viewMaxY = ego.position.y + 145;
          var minRenderX = Math.floor(viewMinX / surfaceRenderCellSize);
          var maxRenderX = Math.floor(viewMaxX / surfaceRenderCellSize);
          var minRenderY = Math.floor(viewMinY / surfaceRenderCellSize);
          var maxRenderY = Math.floor(viewMaxY / surfaceRenderCellSize);
          for (var rx = minRenderX; rx <= maxRenderX; rx++) {
            for (var ry = minRenderY; ry <= maxRenderY; ry++) {
              var renderBucket = cachedSurfaceRenderBuckets[rx + ":" + ry];
              if (!renderBucket) continue;
              visibleSurfaceFaces = visibleSurfaceFaces.concat(renderBucket.faces);
              visibleSurfaceEdges = visibleSurfaceEdges.concat(renderBucket.edges);
            }
          }

          for (var vf = 0; vf < visibleSurfaceFaces.length; vf++) {
            var face = visibleSurfaceFaces[vf];
            var faceA = localPoint(face.a.position, ego);
            var faceB = localPoint(face.b.position, ego);
            var faceC = localPoint(face.c.position, ego);
            if (!faceA || !faceB || !faceC) continue;
            if (Math.abs(faceA.x) > 58 || Math.abs(faceB.x) > 58 || Math.abs(faceC.x) > 58 ||
                faceA.y < -45 || faceA.y > 130 || faceB.y < -45 || faceB.y > 130 ||
                faceC.y < -45 || faceC.y > 130) continue;
            addPolygon([faceA, faceB, faceC],
              face.gapBridge ? "#315d70" : "#397786",
              clamp(0.20 + face.confidence * 0.30, 0.20, 0.48),
              face.gapBridge ? "#6c9eaa" : "#75b9c2");
          }
          for (var ve = 0; ve < visibleSurfaceEdges.length; ve++) {
            var edge = visibleSurfaceEdges[ve];
            var edgeA = localPoint(edge.a.position, ego);
            var edgeB = localPoint(edge.b.position, ego);
            if (!edgeA || !edgeB) continue;
            if (Math.abs(edgeA.x) > 58 || Math.abs(edgeB.x) > 58 ||
                edgeA.y < -45 || edgeA.y > 130 || edgeB.y < -45 || edgeB.y > 130) continue;
            addLine([edgeA, edgeB], "#5ca3ad", 0.65, 0.30);
          }

          var poiItems = asArray(snapshotData.poi && snapshotData.poi.items);
          for (var poiIndex = 0; poiIndex < Math.min(poiItems.length, 16); poiIndex++) {
            var poi = poiItems[poiIndex];
            var poiPoint = localPoint(poi.position, ego);
            if (!poiPoint) continue;
            var poiColor = poi.kind === "fuel" ? "#d4a14f"
              : poi.kind === "charging" ? "#55bba5"
              : poi.kind === "parking" ? "#7eaae1" : "#9c91c9";
            addMarker({x: poiPoint.x, y: poiPoint.y, z: poiPoint.z + 0.45}, poiColor, 0.25, 0.9);
          }

          var signalItems = asArray(snapshotData.signals && snapshotData.signals.items);
          for (var signalIndex = 0; signalIndex < Math.min(signalItems.length, 10); signalIndex++) {
            var signal = signalItems[signalIndex];
            var signalPoint = localPoint(signal.position, ego);
            if (!signalPoint) continue;
            var signalColor = signal.color === "red" ? "#e05b62"
              : signal.color === "amber" ? "#e2b65b"
              : signal.color === "green" ? "#5bca98" : "#8b969e";
            addMarker({x: signalPoint.x, y: signalPoint.y, z: signalPoint.z + 3.0},
              signalColor, 0.32, signal.relevantToCurrentSegment ? 1 : 0.55);
          }

          // Vehicle meshes are deliberately simple low-poly silhouettes, not OBB rectangles.
          addVehicle({x: 0, y: 0, z: 0}, ego.dimensions || {}, 0, true, "confirmed");
          var traffic = asArray(snapshotData.vehicles);
          for (var vehicleIndex = 0; vehicleIndex < traffic.length; vehicleIndex++) {
            var track = traffic[vehicleIndex];
            var age = Number(track && track.dataAge);
            if (!track || !track.position || !isFinite(age) || age > 1.25) continue;
            var point = localPoint(track.predictedPosition || track.position, ego);
            if (!point) continue;
            var heading = track.orientation && track.orientation.forward;
            var yaw = 0;
            if (heading) {
              var localHeading = {
                x: dot(heading, ego.right),
                y: dot(heading, ego.forward),
                z: dot(heading, ego.up)
              };
              yaw = Math.atan2(localHeading.x, localHeading.y);
            }
            if (point.y > -30 && point.y < 120 && Math.abs(point.x) < 50) {
              addVehicle(point, track.dimensions || {}, yaw, false,
                track.status === "confirmed" && age < 0.6 ? "confirmed" : "predicted");
            }
          }

          polygons.sort(function (a, b) { return b.depth - a.depth; });
          for (var pi = 0; pi < polygons.length; pi++) {
            var poly = polygons[pi];
            ctx.globalAlpha = poly.alpha;
            ctx.beginPath();
            ctx.moveTo(poly.points[0].x, poly.points[0].y);
            for (var pj = 1; pj < poly.points.length; pj++) {
              ctx.lineTo(poly.points[pj].x, poly.points[pj].y);
            }
            ctx.closePath();
            ctx.fillStyle = poly.fill;
            ctx.fill();
            if (poly.stroke) {
              ctx.strokeStyle = poly.stroke;
              ctx.lineWidth = 0.8;
              ctx.stroke();
            }
          }

          lines.sort(function (a, b) { return b.depth - a.depth; });
          for (var li2 = 0; li2 < lines.length; li2++) {
            var line = lines[li2];
            ctx.globalAlpha = line.alpha;
            ctx.strokeStyle = line.color;
            ctx.lineWidth = line.width;
            ctx.lineJoin = "round";
            ctx.lineCap = "round";
            ctx.setLineDash(line.dash);
            ctx.beginPath();
            ctx.moveTo(line.points[0].x, line.points[0].y);
            for (var lp = 1; lp < line.points.length; lp++) {
              ctx.lineTo(line.points[lp].x, line.points[lp].y);
            }
            ctx.stroke();
          }
          ctx.setLineDash([]);
          markers.sort(function (a, b) { return b.depth - a.depth; });
          for (var mi = 0; mi < markers.length; mi++) {
            var marker = markers[mi];
            ctx.globalAlpha = marker.alpha;
            ctx.fillStyle = marker.color;
            ctx.beginPath();
            ctx.arc(marker.x, marker.y, marker.radius, 0, Math.PI * 2);
            ctx.fill();
          }
          ctx.globalAlpha = 1;

          // Pin the player's own vehicle above the 3D layers so dense surface
          // geometry cannot hide it.
          var egoX = cssWidth * 0.5;
          var egoY = cssHeight * 0.83;
          ctx.save();
          ctx.translate(egoX, egoY);
          ctx.shadowColor = "rgba(70, 190, 220, 0.42)";
          ctx.shadowBlur = 10;
          ctx.fillStyle = "#d9e6ec";
          ctx.strokeStyle = "#8bc7d5";
          ctx.lineWidth = 1.1;
          ctx.beginPath();
          ctx.moveTo(0, -15);
          ctx.quadraticCurveTo(7, -14, 8, -5);
          ctx.lineTo(7, 10);
          ctx.quadraticCurveTo(6, 15, 0, 15);
          ctx.quadraticCurveTo(-6, 15, -7, 10);
          ctx.lineTo(-8, -5);
          ctx.quadraticCurveTo(-7, -14, 0, -15);
          ctx.closePath();
          ctx.fill();
          ctx.stroke();
          ctx.shadowBlur = 0;
          ctx.fillStyle = "#294653";
          ctx.beginPath();
          ctx.moveTo(-5, -5);
          ctx.lineTo(-4, -10);
          ctx.quadraticCurveTo(0, -12, 4, -10);
          ctx.lineTo(5, -5);
          ctx.closePath();
          ctx.fill();
          ctx.fillStyle = "#1b2932";
          ctx.fillRect(-9, -7, 2, 7);
          ctx.fillRect(7, -7, 2, 7);
          ctx.fillRect(-9, 6, 2, 6);
          ctx.fillRect(7, 6, 2, 6);
          ctx.restore();

          // A small forward glow keeps the map readable without adding telemetry panels.
          var glow = ctx.createRadialGradient(cssWidth * 0.5, cssHeight * 0.46, 2,
            cssWidth * 0.5, cssHeight * 0.46, cssWidth * 0.58);
          glow.addColorStop(0, "rgba(49, 111, 145, 0.08)");
          glow.addColorStop(1, "rgba(5, 10, 15, 0)");
          ctx.fillStyle = glow;
          ctx.fillRect(0, 0, cssWidth, cssHeight);

        }

        function frame(timestamp) {
          if (destroyed) return;
          var renderFps = clamp(Number(snapshot && snapshot.mapSettings
            && snapshot.mapSettings.renderFps) || 30, 15, 60);
          var frameInterval = 1000 / renderFps;
          if (!lastRenderedAt || timestamp - lastRenderedAt >= frameInterval) {
            lastRenderedAt = timestamp;
            try {
              draw(snapshot);
              renderErrorReported = false;
            } catch (error) {
              if (!renderErrorReported && window.console && window.console.error) {
                window.console.error("RoadMind Map render error:", error);
                renderErrorReported = true;
              }
            }
          }
          rafId = window.requestAnimationFrame(frame);
        }

        enableMap();
        schedulePoll(250);
        rafId = window.requestAnimationFrame(frame);

        scope.$on("$destroy", function () {
          destroyed = true;
          if (pollTimer) $timeout.cancel(pollTimer);
          if (pollWatchdog) $timeout.cancel(pollWatchdog);
          if (rafId) window.cancelAnimationFrame(rafId);
          if (extensionReady) lua("jsonEncode(extensions.roadmind.setMapEnabled(false))");
        });
      }
    };
  }]);
})();