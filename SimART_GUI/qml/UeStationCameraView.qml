import QtQuick 2.12

Rectangle {
    id: root
    color: "#10131a"
    focus: true

    property real lastX: 0
    property real lastY: 0

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

    Connections {
        target: airSimView
        function onOverlayDataChanged() { pathCanvas.requestPaint() }
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

    Item {
        id: droneItem
        property var overlay: airSimView.droneOverlay
        visible: overlay.visible
        z: 4
        x: root.mapX(overlay.x) - 18
        y: root.mapY(overlay.y) - 18
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
            text: overlay.name ? overlay.name : "UAV"
            color: "#dbeafe"
            font.pixelSize: 12
            font.bold: true
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 8
        height: infoColumn.implicitHeight + 16
        radius: 8
        color: "#B0101726"
        border.color: "#33ffffff"
        border.width: 1
        z: 5

        Column {
            id: infoColumn
            anchors.fill: parent
            anchors.margins: 8
            spacing: 3

            Text {
                text: airSimView.viewLabel + (airSimView.running ? (airSimView.connected ? " · Connected" : " · Starting") : " · Stopped")
                color: "white"
                font.pixelSize: 14
                font.bold: true
            }
            Text {
                text: airSimView.statusText
                color: "#dbe4ff"
                font.pixelSize: 11
                wrapMode: Text.WrapAnywhere
                width: parent.width
            }
            Text {
                text: "Overlay: RF Paths " + airSimView.pathOverlays.length + (airSimView.droneOverlay.visible ? "   UAV visible" : "   UAV hidden")
                color: "#9fb3d9"
                font.pixelSize: 11
            }
        }
    }

    Rectangle {
        width: 16
        height: 16
        radius: 8
        anchors.centerIn: parent
        color: "transparent"
        border.color: "#66ffffff"
        border.width: 1
        z: 3
        Rectangle { anchors.centerIn: parent; width: 2; height: 10; color: "#66ffffff" }
        Rectangle { anchors.centerIn: parent; width: 10; height: 2; color: "#66ffffff" }
    }

    MouseArea {
        anchors.fill: parent
        z: 3
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        hoverEnabled: true
        onPressed: {
            root.lastX = mouse.x
            root.lastY = mouse.y
            root.forceActiveFocus()
        }
        onPositionChanged: {
            if (!(mouse.buttons & (Qt.LeftButton | Qt.RightButton))) {
                root.lastX = mouse.x
                root.lastY = mouse.y
                return
            }
            var dx = mouse.x - root.lastX
            var dy = mouse.y - root.lastY
            airSimView.orbitCamera(dx, dy, mouse.buttons, (mouse.modifiers & Qt.ShiftModifier) !== 0)
            root.lastX = mouse.x
            root.lastY = mouse.y
        }
        onDoubleClicked: airSimView.resetCamera()
        onWheel: airSimView.zoomCamera(wheel.angleDelta.y)
    }

    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.color: "#2d3748"
        border.width: 1
        z: 6
    }
}
