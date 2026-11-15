angular.module('beamng.apps')
.directive('roadmindApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/roadmind/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', '$element', function ($scope, $element) {
      var canvas = $element.find('canvas')[0];
      var ctx = canvas && canvas.getContext ? canvas.getContext('2d') : null;
      var latestFrame = null;
      var pollTimer = null;

      $scope.sensorEnabled = true;
      $scope.showPoints = true;
      $scope.pointCount = 0;
      $scope.sequence = 0;
      $scope.statusText = 'WAITING FOR SENSOR';

      function engineLua(command, callback) {
        if (typeof bngApi === 'undefined' || !bngApi.engineLua) {
          return;
        }
        bngApi.engineLua(command, callback);
      }

      function resizeCanvas() {
        if (!canvas || !ctx) {
          return { width: 0, height: 0 };
        }

        var width = Math.max(1, canvas.clientWidth || 360);
        var height = Math.max(1, canvas.clientHeight || 250);
        var ratio = Math.max(1, Math.min(2, window.devicePixelRatio || 1));

        var pixelWidth = Math.round(width * ratio);
        var pixelHeight = Math.round(height * ratio);

        if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
          canvas.width = pixelWidth;
          canvas.height = pixelHeight;
        }

        ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
        return { width: width, height: height };
      }

      function drawGrid(width, height, range, fov) {
        var halfAngle = (fov * Math.PI / 180) * 0.5;
        var originX = width * 0.5;
        var originY = height - 24;
        var scale = Math.min(
          (width - 28) / (2 * range * Math.sin(halfAngle)),
          (height - 42) / range
        );

        ctx.strokeStyle = 'rgba(94,145,175,.22)';
        ctx.lineWidth = 1;

        for (var distance = 10; distance <= range; distance += 10) {
          ctx.beginPath();
          ctx.arc(
            originX,
            originY,
            distance * scale,
            -Math.PI / 2 - halfAngle,
            -Math.PI / 2 + halfAngle
          );
          ctx.stroke();

          ctx.fillStyle = 'rgba(151,185,204,.65)';
          ctx.font = '9px Arial';
          ctx.fillText(distance + 'm', originX + 5, originY - distance * scale + 10);
        }

        [-1, -0.5, 0, 0.5, 1].forEach(function (fraction) {
          var angle = fraction * halfAngle;
          var endX = originX + Math.sin(angle) * range * scale;
          var endY = originY - Math.cos(angle) * range * scale;
          ctx.beginPath();
          ctx.moveTo(originX, originY);
          ctx.lineTo(endX, endY);
          ctx.stroke();
        });

        return { originX: originX, originY: originY, scale: scale };
      }

      function drawVehicle(originX, originY) {
        ctx.save();
        ctx.translate(originX, originY + 13);
        ctx.fillStyle = '#dbe8f3';
        ctx.strokeStyle = '#6b92aa';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, -18);
        ctx.lineTo(9, -8);
        ctx.lineTo(8, 12);
        ctx.lineTo(-8, 12);
        ctx.lineTo(-9, -8);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }

      function draw(frame) {
        if (!ctx || !canvas) {
          return;
        }

        var size = resizeCanvas();
        var width = size.width;
        var height = size.height;
        if (!width || !height) {
          return;
        }

        ctx.clearRect(0, 0, width, height);
        ctx.fillStyle = '#08131f';
        ctx.fillRect(0, 0, width, height);

        var range = frame && typeof frame.range === 'number' ? frame.range : 60;
        var fov = frame && typeof frame.horizontalFov === 'number'
          ? frame.horizontalFov : 120;
        var grid = drawGrid(width, height, range, fov);

        if ($scope.showPoints && frame && Array.isArray(frame.points)) {
          frame.points.forEach(function (point) {
            if (!point || typeof point.x !== 'number' || typeof point.y !== 'number') {
              return;
            }

            var x = grid.originX + point.x * grid.scale;
            var y = grid.originY - point.y * grid.scale;

            if (x < 0 || x > width || y < 0 || y > height) {
              return;
            }

            var distance = typeof point.distance === 'number'
              ? point.distance : Math.sqrt(point.x * point.x + point.y * point.y);
            var hue = Math.max(0, 190 - (distance / range) * 150);

            ctx.beginPath();
            ctx.fillStyle = 'hsl(' + hue + ', 95%, 62%)';
            ctx.shadowColor = 'hsla(' + hue + ', 95%, 62%, .6)';
            ctx.shadowBlur = 4;
            ctx.arc(x, y, 2.3, 0, Math.PI * 2);
            ctx.fill();
            ctx.shadowBlur = 0;
          });
        }

        drawVehicle(grid.originX, grid.originY);

        ctx.fillStyle = 'rgba(180,210,228,.8)';
        ctx.font = '9px Arial';
        ctx.fillText('FRONT', grid.originX - 14, 13);
      }

      function requestFrame() {
        engineLua('extensions.roadmind.getLatestData()', function (data) {
          if (!data || typeof data !== 'object') {
            return;
          }

          $scope.$evalAsync(function () {
            latestFrame = data;
            $scope.pointCount = typeof data.pointCount === 'number'
              ? data.pointCount : (Array.isArray(data.points) ? data.points.length : 0);
            $scope.sequence = typeof data.sequence === 'number' ? data.sequence : 0;
            $scope.statusText = $scope.sensorEnabled ? 'SENSOR STREAM' : 'SENSOR PAUSED';
            draw(latestFrame);
          });
        });
      }

      $scope.toggleSensor = function () {
        $scope.sensorEnabled = !$scope.sensorEnabled;
        engineLua(
          'extensions.roadmind.setSensorEnabled('
            + ($scope.sensorEnabled ? 'true' : 'false') + ')'
        );
        $scope.statusText = $scope.sensorEnabled ? 'STARTING SENSOR' : 'SENSOR PAUSED';
      };

      $scope.togglePoints = function () {
        $scope.showPoints = !$scope.showPoints;
        draw(latestFrame);
      };

      // Load the game-engine coordinator. It attaches the sensor to the
      // currently driven vehicle and forwards frames to this app.
      engineLua("extensions.load('roadmind')");

      requestFrame();
      pollTimer = setInterval(requestFrame, 200);

      $scope.$on('$destroy', function () {
        if (pollTimer) {
          clearInterval(pollTimer);
          pollTimer = null;
        }
      });
    }]
  };
}]);
