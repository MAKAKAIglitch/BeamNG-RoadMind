(function () {
  "use strict";

  angular.module("beamng.apps").directive("roadmindMap", ["$timeout", function ($timeout) {
    var template =
      '<div class="roadmind-map-app" aria-label="RoadMind Map">' +
        '<canvas class="roadmind-map-canvas"></canvas>' +
        '<div class="roadmind-map-mark"><span class="roadmind-map-dot"></span>ROAD MIND</div>' +
        '<div class="roadmind-map-wait">WAITING FOR ROAD DATA</div>' +
      '</div>';

    return {
      template: template,
      replace: true,
      restrict: "EA",
      link: function (scope, element) {
        var root = element[0];
        var canvas = root.querySelector("canvas");
        var waitLabel = root.querySelector(".roadmind-map-wait");
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

        function unwrap(value) {
          if (typeof value === "string") {
            try { value = JSON.parse(value); } catch (error) { return null; }
          }
          if (value && !value.schemaVersion && value.result !== undefined) value = value.result;
          if (typeof value === "string") {
            try { value = JSON.parse(value); } catch (error) { return null; }
          }
          return value && value.schemaVersion ? value : null;
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
          lua(
            "extensions.load('roadmind'); return extensions.roadmind.getMapSnapshot()",
            function (result) {
              if (destroyed) return;
              if (pollWatchdog) $timeout.cancel(pollWatchdog);
              polling = false;
              var next = unwrap(result);
              if (next) {
                snapshot = next;
                lastSnapshotAt = Date.now();
                if (waitLabel) waitLabel.style.display = "none";
              } else if (!snapshot && waitLabel) {
                waitLabel.style.display = "block";
              }
              scope.$evalAsync();
              schedulePoll(350);
            }
          );
        }

        function setMapEnabled(enabled) {
          lua(
            "extensions.load('roadmind'); return extensions.roadmind.setMapEnabled(" +
              (enabled ? "true" : "false") + ")"
          );
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
          pixelRatio = Math.min(2, window.devicePixelRatio || 1);
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
            if (waitLabel) waitLabel.style.display = "block";
            return;
          }

          var project = makeProjection(ego, cssWidth, cssHeight);
          if (!project) return;
          var polygons = [];
          var lines = [];

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
            var length = clamp(Number(dimensions && dimensions.length) || (isPlayer ? 4.6 : 4.4), 2.2, 15);
            var width = clamp(Number(dimensions && dimensions.width) || (isPlayer ? 1.9 : 1.85), 1.0, 3.2);
            var height = clamp(Number(dimensions && dimensions.height) || 1.5, 0.8, 4.0);
            var base = {x: position.x, y: position.y, z: position.z};
            var bodyColor = isPlayer ? "#27838a" : (visualState === "predicted" ? "#9b8250" : "#8197a7");
            var glassColor = isPlayer ? "#20414a" : "#344b59";
            addBox({x: base.x, y: base.y, z: base.z - height * 0.13},
              length * 0.94, width * 0.96, height * 0.34, yaw, bodyColor, isPlayer ? 1 : 0.96);
            addBox({x: base.x, y: base.y + length * 0.23, z: base.z - height * 0.035},
              length * 0.25, width * 0.91, height * 0.18, yaw, shade(bodyColor, 1.12), 1);
            addBox({x: base.x, y: base.y - length * 0.015, z: base.z + height * 0.20},
              length * 0.43, width * 0.68, height * 0.42, yaw, glassColor, isPlayer ? 0.98 : 0.88);
            var wheelLength = clamp(length * 0.18, 0.48, 0.82);
            var wheelWidth = clamp(width * 0.16, 0.24, 0.42);
            var wheelHeight = clamp(height * 0.24, 0.28, 0.52);
            [-1, 1].forEach(function (side) {
              [-1, 1].forEach(function (axle) {
                addBox({
                  x: base.x + side * width * 0.43,
                  y: base.y + axle * length * 0.29,
                  z: base.z - height * 0.29
                }, wheelLength, wheelWidth, wheelHeight, yaw, "#101820", 1);
              });
            });
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
          var corridor = (road.corridor || []).map(function (point) {
            return localPoint(point, ego);
          }).filter(Boolean);
          var halfWidth = clamp(Number(road.roadHalfWidth) || 3.5, 2.2, 18);
          for (var r = 0; r < corridor.length - 1; r++) {
            var a = corridor[r];
            var b = corridor[r + 1];
            var dx = b.x - a.x;
            var dy = b.y - a.y;
            var magnitude = Math.sqrt(dx * dx + dy * dy);
            if (magnitude < 0.2) continue;
            var nx = -dy / magnitude;
            var ny = dx / magnitude;
            var leftA = {x: a.x + nx * halfWidth, y: a.y + ny * halfWidth, z: a.z - 0.10};
            var rightA = {x: a.x - nx * halfWidth, y: a.y - ny * halfWidth, z: a.z - 0.10};
            var leftB = {x: b.x + nx * halfWidth, y: b.y + ny * halfWidth, z: b.z - 0.10};
            var rightB = {x: b.x - nx * halfWidth, y: b.y - ny * halfWidth, z: b.z - 0.10};
            addPolygon([leftA, rightA, rightB, leftB], "#202b35", 0.97);
            addLine([leftA, leftB], "#53616c", 1.05, 0.64);
            addLine([rightA, rightB], "#53616c", 1.05, 0.64);
          }

          // Lane hints are shown only when the active NavGraph edge exposes
          // lane-count and lane-width metadata; otherwise lane markings stay absent.
          var laneModel = road.laneModel || {};
          var laneBoundaries = laneModel.laneBoundaryOffsets || [];
          if (laneModel.status === "metadata" && laneBoundaries.length > 0) {
            for (var li = 0; li < laneBoundaries.length; li++) {
              var laneOffset = Number(laneBoundaries[li]);
              if (!isFinite(laneOffset)) continue;
              for (var ls = 0; ls < corridor.length - 1; ls++) {
                var la = corridor[ls];
                var lb = corridor[ls + 1];
                var ldx = lb.x - la.x;
                var ldy = lb.y - la.y;
                var llen = Math.sqrt(ldx * ldx + ldy * ldy);
                if (llen < 0.2) continue;
                var lnx = -ldy / llen;
                var lny = ldx / llen;
                addLine([
                  {x: la.x + lnx * laneOffset, y: la.y + lny * laneOffset, z: la.z - 0.055},
                  {x: lb.x + lnx * laneOffset, y: lb.y + lny * laneOffset, z: lb.z - 0.055}
                ], "#77838c", 0.85, 0.52, [5, 7]);
              }
            }
          }

          var routeWorld = snapshotData.selectedTrajectory;
          if (!routeWorld || routeWorld.length < 2) routeWorld = snapshotData.route || [];
          var routeLocal = (routeWorld || []).map(function (point) {
            return localPoint(point, ego);
          }).filter(Boolean);

          if (snapshotData.selectedTrajectory && snapshotData.selectedTrajectory.length >= 2) {
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

          // Fused, recent scene-raycast points are drawn as small spatial returns.
          (snapshotData.geometry || []).forEach(function (item) {
            var point = localPoint(item.position, ego);
            if (!point || item.age > 2.5) return;
            var marker = clamp(0.11 + (Number(item.observationCount) || 1) * 0.025, 0.11, 0.24);
            addBox({x: point.x, y: point.y, z: point.z}, marker, marker, marker, 0,
              "#829ba8", clamp(0.9 - Number(item.age || 0) * 0.18, 0.36, 0.88));
          });

          // Session geometry is retained only after repeated spatially stable LiDAR hits.
          (snapshotData.staticGeometry || []).forEach(function (item) {
            var point = localPoint(item.position, ego);
            if (!point || Number(item.hitCount || 0) < 2) return;
            var age = Number(item.dataAge) || 0;
            var size = clamp(0.12 + Math.min(8, Number(item.hitCount) || 0) * 0.012, 0.12, 0.22);
            addBox(point, size, size, size, 0, "#617d89", clamp(0.82 - age / 2400, 0.38, 0.82));
          });

          // Nearby level POIs and traffic signals are level/runtime metadata,
          // never inferred from GPS or guessed from geometry.
          (snapshotData.poi && snapshotData.poi.items || []).forEach(function (item) {
            var point = localPoint(item.position, ego);
            if (!point) return;
            var color = item.kind === "fuel" ? "#b88b4b"
              : item.kind === "charging" ? "#55a5a0"
              : item.kind === "parking" ? "#7297c1" : "#8b83b5";
            addBox({x: point.x, y: point.y, z: point.z + 0.45}, 0.72, 0.72, 0.34, 0, color, 0.9);
          });
          (snapshotData.signals && snapshotData.signals.items || []).forEach(function (signal) {
            var point = localPoint(signal.position, ego);
            if (!point) return;
            var lamp = signal.color === "red" ? "#d94f54"
              : signal.color === "amber" ? "#d5aa4d"
              : signal.color === "green" ? "#50b88d" : "#75818a";
            var signalAlpha = signal.relevantToCurrentSegment ? 0.98 : 0.55;
            addBox({x: point.x, y: point.y, z: point.z + 1.7}, 0.26, 0.26, 2.8, 0, "#53616a", signalAlpha);
            addBox({x: point.x, y: point.y, z: point.z + 3.15}, 0.72, 0.38, 0.9, 0, "#17212a", signalAlpha);
            addBox({x: point.x, y: point.y, z: point.z + 3.18}, 0.27, 0.24, 0.25, 0, lamp, signal.live ? signalAlpha : 0.42);
          });

          // Vehicle meshes are deliberately simple low-poly silhouettes, not OBB rectangles.
          addVehicle({x: 0, y: 0, z: 0}, ego.dimensions || {}, 0, true, "confirmed");
          (snapshotData.vehicles || []).forEach(function (track) {
            var age = Number(track.dataAge);
            if (!track || !track.position || !isFinite(age) || age > 1.25) return;
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
            addVehicle(point, track.dimensions || {}, yaw, false,
              track.status === "confirmed" && age < 0.6 ? "confirmed" : "predicted");
          });

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
          ctx.globalAlpha = 1;

          // A small forward glow keeps the map readable without adding telemetry panels.
          var glow = ctx.createRadialGradient(cssWidth * 0.5, cssHeight * 0.46, 2,
            cssWidth * 0.5, cssHeight * 0.46, cssWidth * 0.58);
          glow.addColorStop(0, "rgba(49, 111, 145, 0.08)");
          glow.addColorStop(1, "rgba(5, 10, 15, 0)");
          ctx.fillStyle = glow;
          ctx.fillRect(0, 0, cssWidth, cssHeight);

          if (waitLabel) {
            var ageMs = Date.now() - lastSnapshotAt;
            waitLabel.style.display = ageMs > 2500 ? "block" : "none";
            waitLabel.textContent = ageMs > 2500 ? "WAITING FOR ROAD DATA" : "";
          }
        }

        function frame(timestamp) {
          if (destroyed) return;
          if (!lastRenderedAt || timestamp - lastRenderedAt >= 33) {
            lastRenderedAt = timestamp;
            draw(snapshot);
          }
          rafId = window.requestAnimationFrame(frame);
        }

        setMapEnabled(true);
        schedulePoll(250);
        rafId = window.requestAnimationFrame(frame);

        scope.$on("$destroy", function () {
          destroyed = true;
          if (pollTimer) $timeout.cancel(pollTimer);
          if (pollWatchdog) $timeout.cancel(pollWatchdog);
          if (rafId) window.cancelAnimationFrame(rafId);
          setMapEnabled(false);
        });
      }
    };
  }]);
})();