import QtQuick 2.12

Rectangle {
    id: root
    color: "#10131a"
    focus: true

    property real lastX: 0
    property real lastY: 0
    property real clickStartX: 0
    property real clickStartY: 0
    property bool draggingView: false
    property var hoverWorld: ({ valid: false })
    property bool hoverInsideImage: false
    property int hoverStationIndex: -1
    property real hoverCardX: 24
    property real hoverCardY: 24
    property bool hoverCardContainsMouse: false
    property real hoverPersistenceRadiusPx: 48
    property var pinnedCards: []

    function updateHoverWorld(mx, my) {
        var r = contentRect()
        if (r.width <= 1 || r.height <= 1) {
            hoverInsideImage = false
            hoverWorld = ({ valid: false })
            return
        }
        var nx = (mx - r.x) / r.width
        var ny = (my - r.y) / r.height
        if (nx < 0 || nx > 1 || ny < 0 || ny > 1) {
            hoverInsideImage = false
            hoverWorld = ({ valid: false, nx: nx, ny: ny })
            return
        }
        hoverInsideImage = true
        var info = airSimView.worldPointFromNormalized(nx, ny)
        info.nx = nx
        info.ny = ny
        hoverWorld = info
    }

    function contentRect() {
        var w = liveImage.paintedWidth > 0 ? liveImage.paintedWidth : liveImage.width
        var h = liveImage.paintedHeight > 0 ? liveImage.paintedHeight : liveImage.height
        return {
            x: liveImage.x + (liveImage.width - w) * 0.5,
            y: liveImage.y + (liveImage.height - h) * 0.5,
            width: w,
            height: h
        }
    }

    function mapX(nx) {
        var r = contentRect()
        return r.x + nx * r.width
    }

    function mapY(ny) {
        var r = contentRect()
        return r.y + ny * r.height
    }

    function rgbaColor(r, g, b, a) {
        return Qt.rgba(r, g, b, a)
    }

    function overlayByIndex(index) {
        var overlays = airSimView.stationOverlays
        for (var i = 0; i < overlays.length; ++i) {
            if (overlays[i].index === index)
                return overlays[i]
        }
        return null
    }

    function stationScreenPosition(index) {
        var overlay = overlayByIndex(index)
        if (!overlay || !overlay.visible)
            return { valid: false, x: 0, y: 0 }
        return { valid: true, x: mapX(overlay.x), y: mapY(overlay.y) }
    }

    function distanceSq(x0, y0, x1, y1) {
        var dx = x0 - x1
        var dy = y0 - y1
        return dx * dx + dy * dy
    }

    function stationIndexNearPoint(mx, my) {
        var overlays = airSimView.stationOverlays
        var bestIndex = -1
        var bestDist = hoverPersistenceRadiusPx * hoverPersistenceRadiusPx
        for (var i = 0; i < overlays.length; ++i) {
            var overlay = overlays[i]
            if (!overlay.visible)
                continue
            var sx = mapX(overlay.x)
            var sy = mapY(overlay.y)
            var d2 = distanceSq(mx, my, sx, sy)
            if (d2 <= bestDist) {
                bestDist = d2
                bestIndex = overlay.index
            }
        }
        return bestIndex
    }

    function isStationPinned(index) {
        for (var i = 0; i < pinnedCards.length; ++i) {
            if (pinnedCards[i].index === index)
                return true
        }
        return false
    }

    function updateHoverStation(mx, my) {
        var idx = stationIndexNearPoint(mx, my)
        if (idx >= 0) {
            hoverStationIndex = idx
            var pos = stationScreenPosition(idx)
            hoverCardX = Math.max(12, Math.min(width - 280, pos.x + 16))
            hoverCardY = Math.max(12, Math.min(height - 150, pos.y - 14))
            return
        }
        if (hoverStationIndex < 0)
            return
        if (hoverCardContainsMouse)
            return
        var anchor = stationScreenPosition(hoverStationIndex)
        if (anchor.valid && distanceSq(mx, my, anchor.x, anchor.y) <= hoverPersistenceRadiusPx * hoverPersistenceRadiusPx)
            return
        hoverStationIndex = -1
    }

    function pinHoverCard() {
        if (hoverStationIndex < 0 || isStationPinned(hoverStationIndex))
            return
        var offset = 16 * (pinnedCards.length % 4)
        var next = pinnedCards.slice(0)
        next.push({ index: hoverStationIndex, x: hoverCardX + offset, y: hoverCardY + offset })
        pinnedCards = next
    }

    function removePinnedCard(index) {
        var next = []
        for (var i = 0; i < pinnedCards.length; ++i) {
            if (pinnedCards[i].index !== index)
                next.push(pinnedCards[i])
        }
        pinnedCards = next
    }

    Connections {
        target: airSimView
        function onOverlayDataChanged() {
            pathCanvas.requestPaint()
            if (hoverStationIndex >= 0 && !root.overlayByIndex(hoverStationIndex))
                hoverStationIndex = -1
        }
        function onFrameTokenChanged() { pathCanvas.requestPaint() }
    }

    Image {
        id: liveImage
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
        cache: false
        smooth: false
        mipmap: false
        asynchronous: false
        source: airSimView.frameToken > 0 ? ("image://airsimlive/frame/" + airSimView.frameToken) : ""
        sourceSize.width: Math.max(1, width)
        sourceSize.height: Math.max(1, height)
        onPaintedWidthChanged: pathCanvas.requestPaint()
        onPaintedHeightChanged: pathCanvas.requestPaint()
    }

    Canvas {
        id: pathCanvas
        anchors.fill: parent
        z: 2
        onPaint: {
            var ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            var paths = airSimView.pathOverlays
            var r = root.contentRect()
            if (r.width <= 1 || r.height <= 1)
                return
            for (var i = 0; i < paths.length; ++i) {
                var path = paths[i]
                var segments = path.segments || []
                ctx.lineCap = "round"
                ctx.lineJoin = "round"
                ctx.strokeStyle = root.rgbaColor(path.colorR, path.colorG, path.colorB, path.opacity)
                ctx.lineWidth = path.width
                for (var s = 0; s < segments.length; ++s) {
                    var segment = segments[s]
                    if (!segment || segment.length < 2)
                        continue
                    ctx.beginPath()
                    ctx.moveTo(root.mapX(segment[0].x), root.mapY(segment[0].y))
                    for (var p = 1; p < segment.length; ++p)
                        ctx.lineTo(root.mapX(segment[p].x), root.mapY(segment[p].y))
                    ctx.stroke()
                }
            }
        }
    }

    Repeater {
        model: airSimView.stationOverlays
        delegate: Item {
            id: stationItem
            property var overlay: modelData
            visible: overlay.visible
            z: overlay.selected ? 5 : 4
            x: root.mapX(overlay.x) - width * 0.5
            y: root.mapY(overlay.y) - height * 0.5
            width: label.implicitWidth + 30
            height: 28

            Rectangle {
                id: markerDot
                x: 0
                y: parent.height * 0.5 - height * 0.5
                width: overlay.selected ? 16 : 12
                height: width
                radius: width * 0.5
                color: root.rgbaColor(overlay.colorR, overlay.colorG, overlay.colorB, overlay.selected ? 1.0 : 0.92)
                border.width: overlay.selected ? 3 : 2
                border.color: overlay.selected ? "#fff5c2" : "#f8fafc"
            }

            Rectangle {
                id: labelBg
                x: markerDot.x + markerDot.width + 8
                y: parent.height * 0.5 - height * 0.5
                height: 22
                width: label.implicitWidth + 14
                radius: 11
                color: overlay.selected ? "#d97706" : "#B0142230"
                border.width: 1
                border.color: overlay.selected ? "#fff5c2" : "#66ffffff"
            }

            Text {
                id: label
                anchors.verticalCenter: labelBg.verticalCenter
                x: labelBg.x + 7
                color: "white"
                font.pixelSize: 12
                font.bold: overlay.selected
                text: overlay.name
            }

            MouseArea {
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton
                onEntered: {
                    root.hoverStationIndex = overlay.index
                    root.hoverCardX = Math.max(12, Math.min(root.width - 280, stationItem.x + stationItem.width + 12))
                    root.hoverCardY = Math.max(12, Math.min(root.height - 150, stationItem.y - 10))
                }
                onClicked: {
                    mouse.accepted = true
                    airSimView.requestStationSelection(overlay.index)
                }
            }
        }
    }

    Item {
        id: droneItem
        property var overlay: airSimView.droneOverlay || ({})
        property bool overlayVisible: overlay.visible === true
        property real overlayX: overlay.x !== undefined ? overlay.x : 0
        property real overlayY: overlay.y !== undefined ? overlay.y : 0
        visible: overlayVisible
        z: 4
        x: root.mapX(overlayX) - 18
        y: root.mapY(overlayY) - 18
        width: 36
        height: 36

        Rectangle {
            anchors.centerIn: parent
            width: 20
            height: 20
            radius: 10
            color: "#1d4ed8"
            border.width: 2
            border.color: "#dbeafe"
        }
        Rectangle { anchors.centerIn: parent; width: 2; height: 26; color: "#dbeafe" }
        Rectangle { anchors.centerIn: parent; width: 26; height: 2; color: "#dbeafe" }
        Text {
            anchors.top: parent.bottom
            anchors.topMargin: 4
            anchors.horizontalCenter: parent.horizontalCenter
            text: droneItem.overlay.name ? droneItem.overlay.name : "UAV"
            color: "#dbeafe"
            font.pixelSize: 12
            font.bold: true
        }
    }

    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.color: "#2d3748"
        border.width: 1
    }

    Column {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        spacing: 8
        z: 6

        Rectangle {
            radius: 10
            color: "#B0101726"
            border.color: "#33ffffff"
            border.width: 1
            width: 430
            implicitHeight: infoColumn.implicitHeight + 16
            Column {
                id: infoColumn
                anchors.fill: parent
                anchors.margins: 8
                spacing: 4
                Text {
                    text: airSimView.running ? (airSimView.connected ? "UE Live View · Connected" : "UE Live View · Starting") : "UE Live View · Stopped"
                    color: "white"
                    font.pixelSize: 16
                    font.bold: true
                }
                Text {
                    text: airSimView.statusText
                    color: "#dbe4ff"
                    width: parent.width
                    wrapMode: Text.WordWrap
                    font.pixelSize: 12
                }
                Text {
                    text: "Host " + airSimView.host + ":" + airSimView.port + "   Preview camera " + airSimView.cameraName + (airSimView.vehicleName.length > 0 ? ("   Vehicle " + airSimView.vehicleName) : "")
                    color: "#9fb3d9"
                    font.pixelSize: 12
                }
                Text {
                    text: "Yaw " + airSimView.yawDeg.toFixed(1) + "°   Pitch " + airSimView.pitchDeg.toFixed(1) + "°   Roll " + airSimView.rollDeg.toFixed(1) + "°   Dist " + airSimView.distance.toFixed(1) + " m"
                    color: "#9fb3d9"
                    font.pixelSize: 12
                }
                Text {
                    text: "Overlay: BS " + airSimView.stationOverlays.length + "   RF Paths " + airSimView.pathOverlays.length + (airSimView.droneOverlay.visible ? "   UAV visible" : "") + (airSimView.depthAvailable ? ("   Depth " + airSimView.depthWidth + "x" + airSimView.depthHeight) : "   Depth unavailable")
                    color: "#9fb3d9"
                    font.pixelSize: 12
                }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    color: hoverInsideImage ? "#c7d2fe" : "#7c8aa8"
                    font.pixelSize: 12
                    text: {
                        if (!hoverInsideImage)
                            return "Mouse: outside image"
                        if (!hoverWorld.valid)
                            return "Mouse nx=" + hoverWorld.nx.toFixed(3) + " ny=" + hoverWorld.ny.toFixed(3) + "   depth/coord unavailable"
                        return "Mouse AirSim ("
                                + hoverWorld.airsimX.toFixed(3) + ", "
                                + hoverWorld.airsimY.toFixed(3) + ", "
                                + hoverWorld.airsimZ.toFixed(3) + ")   GUI ("
                                + hoverWorld.x.toFixed(3) + ", "
                                + hoverWorld.y.toFixed(3) + ", "
                                + hoverWorld.z.toFixed(3) + ")   depth "
                                + hoverWorld.depth.toFixed(3)
                    }
                }
            }
        }
    }

    Item {
        id: viewGizmo
        visible: airSimView.viewGizmoVisible
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 16
        width: 128
        height: 140
        z: 8
        property real dragLastX: 0
        property real dragLastY: 0
        property real dragStartX: 0
        property real dragStartY: 0
        property bool dragMoved: false
        property real centerX: 56
        property real centerY: 56
        property real axisScale: 37
        property int refreshToken: 0
        onVisibleChanged: {
            if (visible)
                refresh()
            else
                dragMoved = false
        }

        function refresh() {
            if (!visible)
                return
            refreshToken = refreshToken + 1
            axisCanvas.requestPaint()
        }

        function degToRad(deg) {
            return deg * Math.PI / 180.0
        }

        function normalized(v) {
            var n = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            if (n < 0.000001)
                return {x: 0, y: 0, z: 0}
            return {x: v.x / n, y: v.y / n, z: v.z / n}
        }

        function cross(a, b) {
            return {
                x: a.y * b.z - a.z * b.y,
                y: a.z * b.x - a.x * b.z,
                z: a.x * b.y - a.y * b.x
            }
        }

        function dot(a, b) {
            return a.x * b.x + a.y * b.y + a.z * b.z
        }

        function cameraBasis() {
            var yaw = degToRad(airSimView.yawDeg)
            var pitch = degToRad(airSimView.pitchDeg)
            var roll = degToRad(airSimView.rollDeg)
            var cp = Math.cos(pitch)
            var forward = normalized({x: cp * Math.cos(yaw), y: cp * Math.sin(yaw), z: -Math.sin(pitch)})
            var worldUp = {x: 0, y: 0, z: 1}
            var right = normalized(cross(worldUp, forward))
            if (Math.abs(right.x) + Math.abs(right.y) + Math.abs(right.z) < 0.000001)
                right = normalized({x: -Math.sin(yaw), y: Math.cos(yaw), z: 0})
            var up = normalized(cross(forward, right))
            var cr = Math.cos(roll)
            var sr = Math.sin(roll)
            var rolledRight = normalized({
                x: right.x * cr + up.x * sr,
                y: right.y * cr + up.y * sr,
                z: right.z * cr + up.z * sr
            })
            var rolledUp = normalized({
                x: -right.x * sr + up.x * cr,
                y: -right.y * sr + up.y * cr,
                z: -right.z * sr + up.z * cr
            })
            return {forward: forward, right: rolledRight, up: rolledUp}
        }

        function axisPoint(ax, ay, az) {
            var token = refreshToken
            var basis = cameraBasis()
            var axis = normalized({x: ax, y: ay, z: az})
            return {
                x: centerX + dot(axis, basis.right) * axisScale,
                y: centerY - dot(axis, basis.up) * axisScale,
                depth: dot(axis, basis.forward)
            }
        }

        function nodeX(ax, ay, az, diameter) {
            return axisPoint(ax, ay, az).x - diameter * 0.5
        }

        function nodeY(ax, ay, az, diameter) {
            return axisPoint(ax, ay, az).y - diameter * 0.5
        }

        function beginDrag(mx, my) {
            dragLastX = mx
            dragLastY = my
            dragStartX = mx
            dragStartY = my
            dragMoved = false
            root.forceActiveFocus()
        }

        function updateDrag(mx, my, buttons, modifiers) {
            if (!(buttons & Qt.LeftButton)) {
                dragLastX = mx
                dragLastY = my
                return
            }
            if (Math.abs(mx - dragStartX) > 2 || Math.abs(my - dragStartY) > 2)
                dragMoved = true
            var dx = mx - dragLastX
            var dy = my - dragLastY
            if (dragMoved) {
                airSimView.orbitCamera(dx, dy, buttons, (modifiers & Qt.ShiftModifier) !== 0)
                refresh()
            }
            dragLastX = mx
            dragLastY = my
        }

        function finishDrag(preset) {
            var wasDrag = dragMoved
            dragMoved = false
            if (!wasDrag && preset && preset.length > 0) {
                airSimView.setViewPreset(preset)
                refresh()
            }
        }

        Rectangle {
            anchors.fill: parent
            radius: 12
            color: "#80101726"
            border.color: "#40556b85"
            border.width: 1
        }

        Canvas {
            id: axisCanvas
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                if (!viewGizmo.visible)
                    return
                function axisLine(ax, ay, az, color) {
                    var p = viewGizmo.axisPoint(ax, ay, az)
                    ctx.beginPath()
                    ctx.moveTo(viewGizmo.centerX, viewGizmo.centerY)
                    ctx.lineTo(p.x, p.y)
                    ctx.strokeStyle = color
                    ctx.lineWidth = 3
                    ctx.lineCap = "round"
                    ctx.stroke()
                }
                axisLine(0, 0, 1, "#3b82f6")
                axisLine(1, 0, 0, "#ef4444")
                axisLine(0, 1, 0, "#84cc16")
                axisLine(-1, 0, 0, "#7f1d1d")
                axisLine(0, -1, 0, "#3f6212")
                axisLine(0, 0, -1, "#1e3a8a")
            }
        }

        Connections {
            target: airSimView
            enabled: viewGizmo.visible
            onCameraStateChanged: {
                viewGizmo.refresh()
            }
        }

        MouseArea {
            anchors.fill: parent
            enabled: viewGizmo.visible
            acceptedButtons: Qt.LeftButton
            hoverEnabled: true
            cursorShape: Qt.OpenHandCursor
            onPressed: viewGizmo.beginDrag(mouse.x, mouse.y)
            onPositionChanged: viewGizmo.updateDrag(mouse.x, mouse.y, mouse.buttons, mouse.modifiers)
            onReleased: viewGizmo.finishDrag("")
            onCanceled: viewGizmo.dragMoved = false
        }

        Repeater {
            model: ListModel {
                ListElement { axisText: "Z"; preset: "z_pos"; axisColor: "#3b82f6"; axisX: 0; axisY: 0; axisZ: 1; diameter: 24 }
                ListElement { axisText: "Y"; preset: "y_pos"; axisColor: "#84cc16"; axisX: 0; axisY: 1; axisZ: 0; diameter: 24 }
                ListElement { axisText: "X"; preset: "x_pos"; axisColor: "#ef4444"; axisX: 1; axisY: 0; axisZ: 0; diameter: 24 }
                ListElement { axisText: "z"; preset: "z_neg"; axisColor: "#1d4ed8"; axisX: 0; axisY: 0; axisZ: -1; diameter: 18 }
                ListElement { axisText: "y"; preset: "y_neg"; axisColor: "#65a30d"; axisX: 0; axisY: -1; axisZ: 0; diameter: 18 }
                ListElement { axisText: "x"; preset: "x_neg"; axisColor: "#b91c1c"; axisX: -1; axisY: 0; axisZ: 0; diameter: 18 }
                ListElement { axisText: ""; preset: "iso"; axisColor: "#263342"; axisX: 0; axisY: 0; axisZ: 0; diameter: 16 }
            }
            delegate: Rectangle {
                property color nodeColor: axisColor
                property int refreshTokenBinding: viewGizmo.refreshToken
                z: 1
                x: refreshTokenBinding >= 0 ? (preset === "iso" ? viewGizmo.centerX - diameter * 0.5 : viewGizmo.nodeX(axisX, axisY, axisZ, diameter)) : 0
                y: refreshTokenBinding >= 0 ? (preset === "iso" ? viewGizmo.centerY - diameter * 0.5 : viewGizmo.nodeY(axisX, axisY, axisZ, diameter)) : 0
                width: diameter
                height: diameter
                radius: diameter * 0.5
                color: axisMouse.containsMouse ? Qt.lighter(nodeColor, 1.25) : nodeColor
                border.color: axisMouse.containsMouse ? "#f8fafc" : "#d7dee9"
                border.width: axisMouse.containsMouse ? 2 : 1

                Text {
                    anchors.centerIn: parent
                    text: axisText
                    color: "white"
                    font.pixelSize: diameter >= 22 ? 12 : 9
                    font.bold: true
                }

                MouseArea {
                    id: axisMouse
                    anchors.fill: parent
                    enabled: viewGizmo.visible
                    hoverEnabled: true
                    acceptedButtons: Qt.LeftButton
                    cursorShape: Qt.PointingHandCursor
                    onPressed: viewGizmo.beginDrag(mouse.x, mouse.y)
                    onPositionChanged: viewGizmo.updateDrag(mouse.x, mouse.y, mouse.buttons, mouse.modifiers)
                    onReleased: viewGizmo.finishDrag(preset)
                    onCanceled: viewGizmo.dragMoved = false
                }
            }
        }

        Rectangle {
            x: 18
            y: 112
            width: 40
            height: 20
            radius: 8
            z: 2
            color: rollCounterMouse.containsMouse ? "#334155" : "#1f2937"
            border.color: rollCounterMouse.containsMouse ? "#e5e7eb" : "#64748b"
            border.width: 1
            Text {
                anchors.centerIn: parent
                text: "CCW"
                color: "white"
                font.pixelSize: 10
                font.bold: true
            }
            MouseArea {
                id: rollCounterMouse
                anchors.fill: parent
                enabled: viewGizmo.visible
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    mouse.accepted = true
                    airSimView.rollViewQuarter(-1)
                    viewGizmo.refresh()
                }
            }
        }

        Rectangle {
            x: 70
            y: 112
            width: 40
            height: 20
            radius: 8
            z: 2
            color: rollClockMouse.containsMouse ? "#334155" : "#1f2937"
            border.color: rollClockMouse.containsMouse ? "#e5e7eb" : "#64748b"
            border.width: 1
            Text {
                anchors.centerIn: parent
                text: "CW"
                color: "white"
                font.pixelSize: 10
                font.bold: true
            }
            MouseArea {
                id: rollClockMouse
                anchors.fill: parent
                enabled: viewGizmo.visible
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    mouse.accepted = true
                    airSimView.rollViewQuarter(1)
                    viewGizmo.refresh()
                }
            }
        }
    }

    Rectangle {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 12
        radius: 10
        color: "#B0101726"
        border.color: "#33ffffff"
        border.width: 1
        width: 360
        implicitHeight: helpText.implicitHeight + 16
        z: 6
        Text {
            id: helpText
            anchors.fill: parent
            anchors.margins: 8
            color: "#dbe4ff"
            wrapMode: Text.WordWrap
            font.pixelSize: 12
            text: "Left drag: orbit view\nRight drag or Shift+left drag: pan focus\nMouse wheel: zoom in/out\nDouble click: reset view camera\nClick a base-station marker: sync selection between 3D and Live\nClick empty space: clear base-station selection\nHover near a base station: show the card; pin it to keep it visible"
        }
    }

    Rectangle {
        width: 18
        height: 18
        radius: 9
        anchors.centerIn: parent
        color: "transparent"
        border.color: "#66ffffff"
        border.width: 1
        z: 3
        Rectangle { anchors.centerIn: parent; width: 2; height: 12; color: "#66ffffff" }
        Rectangle { anchors.centerIn: parent; width: 12; height: 2; color: "#66ffffff" }
    }

    Rectangle {
        id: hoverCard
        visible: hoverStationIndex >= 0 && !root.isStationPinned(hoverStationIndex)
        x: hoverCardX
        y: hoverCardY
        z: 7
        width: 260
        implicitHeight: hoverCardColumn.implicitHeight + 16
        radius: 10
        color: "#1f2430"
        border.color: "#394150"
        border.width: 1

        Column {
            id: hoverCardColumn
            anchors.fill: parent
            anchors.margins: 8
            spacing: 4

            Row {
                spacing: 6
                Text {
                    color: "white"
                    font.pixelSize: 13
                    font.bold: true
                    text: {
                        var overlay = root.overlayByIndex(root.hoverStationIndex)
                        if (!overlay)
                            return ""
                        return overlay.name + (overlay.selected ? " [selected]" : "")
                    }
                }
                Rectangle {
                    radius: 6
                    color: pinHoverMouse.pressed ? "#374151" : (pinHoverMouse.containsMouse ? "#2f3b4b" : "#263342")
                    border.color: "#4b5563"
                    border.width: 1
                    width: 42
                    height: 22
                    Text { anchors.centerIn: parent; color: "white"; font.pixelSize: 11; font.bold: true; text: "Pin" }
                    MouseArea {
                        id: pinHoverMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: root.pinHoverCard()
                    }
                }
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                color: "#e8ecf1"
                font.pixelSize: 12
                text: airSimView.stationInfoText(root.hoverStationIndex)
            }
        }

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.NoButton
            hoverEnabled: true
            onEntered: root.hoverCardContainsMouse = true
            onExited: root.hoverCardContainsMouse = false
        }
    }

    Repeater {
        model: root.pinnedCards
        delegate: Rectangle {
            property var cardData: modelData
            property var overlay: root.overlayByIndex(cardData.index)
            visible: overlay !== null
            x: cardData.x
            y: cardData.y
            z: 7
            width: 260
            implicitHeight: pinnedColumn.implicitHeight + 16
            radius: 10
            color: "#1f2430"
            border.color: "#394150"
            border.width: 1

            Column {
                id: pinnedColumn
                anchors.fill: parent
                anchors.margins: 8
                spacing: 4
                Row {
                    spacing: 6
                    Text {
                        color: "white"
                        font.pixelSize: 13
                        font.bold: true
                        text: overlay ? (overlay.name + (overlay.selected ? " [selected]" : "") + " [pinned]") : ""
                    }
                    Rectangle {
                        radius: 6
                        color: closeMouse.pressed ? "#7f1d1d" : (closeMouse.containsMouse ? "#651919" : "#4b1212")
                        border.color: "#fecaca"
                        border.width: 1
                        width: 22
                        height: 22
                        Text { anchors.centerIn: parent; color: "white"; font.pixelSize: 11; font.bold: true; text: "×" }
                        MouseArea {
                            id: closeMouse
                            anchors.fill: parent
                            hoverEnabled: true
                            onClicked: root.removePinnedCard(cardData.index)
                        }
                    }
                }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    color: "#e8ecf1"
                    font.pixelSize: 12
                    text: airSimView.stationInfoText(cardData.index)
                }
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        z: 1
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        hoverEnabled: true
        onPressed: {
            root.lastX = mouse.x
            root.lastY = mouse.y
            root.clickStartX = mouse.x
            root.clickStartY = mouse.y
            root.draggingView = false
            root.updateHoverWorld(mouse.x, mouse.y)
            root.updateHoverStation(mouse.x, mouse.y)
            root.forceActiveFocus()
        }
        onPositionChanged: {
            root.updateHoverWorld(mouse.x, mouse.y)
            root.updateHoverStation(mouse.x, mouse.y)
            if (!(mouse.buttons & (Qt.LeftButton | Qt.RightButton))) {
                root.lastX = mouse.x
                root.lastY = mouse.y
                return
            }
            if (Math.abs(mouse.x - root.clickStartX) > 2 || Math.abs(mouse.y - root.clickStartY) > 2)
                root.draggingView = true
            var dx = mouse.x - root.lastX
            var dy = mouse.y - root.lastY
            airSimView.orbitCamera(dx, dy, mouse.buttons, (mouse.modifiers & Qt.ShiftModifier) !== 0)
            viewGizmo.refresh()
            root.lastX = mouse.x
            root.lastY = mouse.y
        }
        onExited: {
            root.hoverInsideImage = false
            root.hoverWorld = ({ valid: false })
            if (!root.hoverCardContainsMouse)
                root.hoverStationIndex = -1
        }
        onClicked: {
            if (!root.draggingView && root.stationIndexNearPoint(mouse.x, mouse.y) < 0)
                airSimView.requestStationSelection(-1)
        }
        onReleased: root.draggingView = false
        onWheel: {
            airSimView.zoomCamera(wheel.angleDelta.y)
            viewGizmo.refresh()
        }
        onDoubleClicked: {
            airSimView.resetCamera()
            viewGizmo.refresh()
        }
    }

    Row {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 14
        spacing: 10
        z: 6

        Rectangle {
            id: startButton
            radius: 8
            color: startMouse.pressed ? "#334155" : (startMouse.containsMouse ? "#263342" : "#1f2937")
            border.color: "#4b5563"
            border.width: 1
            width: 170
            height: 36
            Text {
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 13
                font.bold: true
                text: airSimView.running ? "Restart Live View" : "Start Live View"
            }
            MouseArea {
                id: startMouse
                anchors.fill: parent
                hoverEnabled: true
                onClicked: {
                    if (airSimView.running) airSimView.restart()
                    else airSimView.start()
                }
            }
        }

        Rectangle {
            id: resetButton
            radius: 8
            color: resetMouse.pressed ? "#334155" : (resetMouse.containsMouse ? "#263342" : "#1f2937")
            border.color: "#4b5563"
            border.width: 1
            width: 150
            height: 36
            Text {
                anchors.centerIn: parent
                color: "white"
                font.pixelSize: 13
                font.bold: true
                text: "Reset Camera"
            }
            MouseArea {
                id: resetMouse
                anchors.fill: parent
                hoverEnabled: true
                onClicked: airSimView.resetCamera()
            }
        }
    }
}
