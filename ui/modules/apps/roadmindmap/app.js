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

        function makeProjection(ego, width, height) {
          var speed = clamp(Number(ego.speedMps) || 0, 0, 45);
          var distance = clamp(10.5 + speed * 0.25, 10.5, 22);
          var cameraHeight = clamp(6.7 + speed * 0.11, 6.7, 10.5);
          var lookAhead = clamp(13 + speed * 0.58, 13, 34);
          var camera = {x: 0, y: -distance, z: cameraHeight};
          var target = {x: 0, y: lookAhead, z: -0.3};
          var forward = normalize(sub(target, camera));
          var right = normalize(cross(forward, {x: 0, y: 0, z: 1}));
          var up = right && normalize(cross(right, forward));
          if (!forward || !right || !up) return null;
          var focal = Math.min(width, height) * 1.22;
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
          if (!ego || !ego.position || !ego.forward || !ego.right || !ego.up) {
            return;
          }

          var project = makeProjection(ego, cssWidth, cssHeight);
          if (!project) return;
          var polygons = [];
          var lines = [];
          var markers = [];

          function addPolygon(points, fill, alpha, stroke) {
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
            addBox({x: base.x, y: base.y, z: base.z - height * 0.13},
              length * 0.94, width * 0.96, height * 0.34, yaw, bodyColor, isPlayer ? 1 : 0.96);
            addBox({x: base.x, y: base.y + length * 0.10, z: base.z + height * 0.17},
              length * 0.43, width * 0.68, height * 0.34, yaw, glassColor, isPlayer ? 0.98 : 0.88);
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
          ], "#0b131b", 1);
          for (var gy = -20; gy <= 100; gy += 10) {
            addLine([{x: -55, y: gy, z: -1.08}, {x: 55, y: gy, z: -1.08}],
              "#1c2a35", 0.7, 0.38, [2, 8]);
          }
          for (var gx = -50; gx <= 50; gx += 10) {
            addLine([{x: gx, y: -20, z: -1.08}, {x: gx, y: 100, z: -1.08}],
              "#1c2a35", 0.7, 0.28, [2, 8]);
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
                graphSegment.current ? "#263b49" : "#202b35", graphSegment.current ? 0.98 : 0.91);
              addLine([gLeftA, gLeftB], "#53616c", graphSegment.current ? 1.15 : 0.9, 0.66);
              addLine([gRightA, gRightB], "#53616c", graphSegment.current ? 1.15 : 0.9, 0.66);

              var graphLaneModel = graphSegment.laneModel || {};
              var graphBoundaries = asArray(graphLaneModel.laneBoundaryOffsets);
              if (graphLaneModel.status === "metadata") {
                for (var gb = 0; gb < graphBoundaries.length; gb++) {
                  var boundaryOffset = Number(graphBoundaries[gb]);
                  if (!isFinite(boundaryOffset)) continue;
                  addLine([
                    {x: graphStart.x + gnx * boundaryOffset, y: graphStart.y + gny * boundaryOffset, z: graphStart.z - 0.055},
                    {x: graphFinish.x + gnx * boundaryOffset, y: graphFinish.y + gny * boundaryOffset, z: graphFinish.z - 0.055}
                  ], "#778b99", graphSegment.current ? 1.0 : 0.8,
                    graphSegment.current ? 0.66 : 0.46, [5, 7]);
                }
              }

              var graphLanes = asArray(graphSegment.lanes);
              for (var gli = 0; gli < graphLanes.length; gli++) {
                var graphLane = graphLanes[gli];
                if (graphLane.id === laneGraph.currentLaneId) {
                  var laneStart = localPoint(graphLane.start, ego);
                  var laneFinish = localPoint(graphLane.finish, ego);
                  if (laneStart && laneFinish) {
                    addLine([laneStart, laneFinish], "#3c8fc8", 1.25, 0.72);
                  }
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

          // Spatial returns are a bounded point cloud, not per-return 3D boxes.
          var geometry = asArray(snapshotData.geometry);
          for (var gi = 0; gi < Math.min(geometry.length, 120); gi++) {
            var geometryItem = geometry[gi];
            var geometryPoint = localPoint(geometryItem.position, ego);
            var geometryAge = Number(geometryItem.age);
            if (!geometryPoint || !isFinite(geometryAge) || geometryAge > 2.5) continue;
            addMarker(geometryPoint, "#91aeba",
              0.10 + (Number(geometryItem.observationCount) || 1) * 0.018,
              clamp(0.92 - geometryAge * 0.18, 0.38, 0.9));
          }

          var staticGeometry = asArray(snapshotData.staticGeometry);
          for (var si = 0; si < Math.min(staticGeometry.length, 140); si++) {
            var staticItem = staticGeometry[si];
            var staticPoint = localPoint(staticItem.position, ego);
            if (!staticPoint || Number(staticItem.hitCount || 0) < 2) continue;
            var staticAge = Number(staticItem.dataAge) || 0;
            addMarker(staticPoint, "#718d9b",
              0.12 + Math.min(8, Number(staticItem.hitCount) || 0) * 0.012,
              clamp(0.82 - staticAge / 2400, 0.38, 0.82));
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
          for (var vehicleIndex = 0; vehicleIndex < Math.min(traffic.length, 6); vehicleIndex++) {
            var track = traffic[vehicleIndex];
            var age = Number(track && track.dataAge);
            if (!track || !track.position || !isFinite(age) || age > 1.25) continue;
            var point = localPoint(track.predictedPosition || track.position, ego);
            if (!point) return;
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
          if (!lastRenderedAt || timestamp - lastRenderedAt >= 83) {
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