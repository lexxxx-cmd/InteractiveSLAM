import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import InteractiveSLAM 1.0

ApplicationWindow {
    id: mainWindow
    width: 1280
    height: 720
    visible: true
    title: qsTr("Interactive SLAM")

    // 【修改点 1】: 将带小数点的写法换成标准的分组括号写法，彻底杜绝解析器报错
    palette {
        window: "#1E2227"          // 主窗口背景 (深灰蓝)
        windowText: "#E0E0E0"      // 默认文字颜色
        base: "#21252B"            // 控件背景 (深色)
        text: "#FFFFFF"            // 控件文字
        highlight: "#FF8C00"       // 强调色：橙色
        highlightedText: "#FFFFFF" 
        button: "#009688"          // 按钮默认色：青色
        buttonText: "#FFFFFF"      
    }

    property bool isOptimizing: false
    property int mockVertexCount: 4531
    property int mockEdgeCount: 12048
    
    // --- 【新增控制变量】：控制悬浮窗口的显示与隐藏 ---
    property bool showRenderConfig: false 

    // ==========================================
    // 1. 顶部菜单栏
    // ==========================================
    menuBar: MenuBar {
        palette {
            window: "#181A1F"
            windowText: "#FFFFFF"
            text: "#FFFFFF"
            buttonText: "#FFFFFF"
        }
        background: Rectangle { color: "#181A1F" }

        Menu {
            title: qsTr("File")
            Menu {
                title: qsTr("Open")
                Action { text: qsTr("New map"); onTriggered: folderDialog.open() }
                Action { text: qsTr("Merge map"); onTriggered: log("Merge Map") }
            }
            Menu {
                title: qsTr("Save")
                Action { text: qsTr("Save map data") }
                Action { text: qsTr("Export PointCloud") }
            }
            Action { text: qsTr("Close Map"); onTriggered: GraphManager.closeMap() }
            Action { text: qsTr("Quit"); onTriggered: Qt.quit() }
        }

        Menu {
            title: qsTr("View")
            Action { text: qsTr("Reset camera"); onTriggered: graphViewport.resetCamera() }
            Action { 
                text: qsTr("Graph Rendering Setting")
                checkable: true
                checked: mainWindow.showRenderConfig
                onTriggered: mainWindow.showRenderConfig = checked
            }
            Action { text: qsTr("Clear selections") }
        }

        Menu {
            title: qsTr("Graph")
            Action { text: qsTr("Graph editor") }
            Action { text: qsTr("Automatic loop detection") }
            Action { text: qsTr("Edge Refinement") }
            Action { 
                text: qsTr("Optimize")
                shortcut: "Ctrl+O"
                onTriggered: {
                    mainWindow.isOptimizing = true;
                    log("Graph optimization started...");
                    optimizeTimer.start();
                }
            }
        }
    }

    FolderDialog {
        id: folderDialog
        title: qsTr("Select Map Folder")
        onAccepted: {
            log("Loading: " + selectedFolder);
            GraphManager.openMapData(selectedFolder);
        }
        onRejected: {
            log("Folder selection cancelled");
        }
    }

    Timer {
        id: optimizeTimer
        interval: 3000
        onTriggered: {
            mainWindow.isOptimizing = false;
            log("Graph optimization finished!");
        }
    }

    Rectangle {
        id: mainCanvas
        anchors.top: parent.top
        anchors.bottom: statusBar.top
        anchors.left: parent.left
        anchors.right: parent.right
        color: "#0D0F12" 

        // 3D Viewport (QQuickFramebufferObject — OpenGL rendering)
        GraphViewport {
            id: graphViewport
            anchors.fill: parent
            graphManager: GraphManager
        }

        // Mouse capture overlay (on top of GraphViewport)
        MouseArea {
            id: viewportMouse
            anchors.fill: parent
            hoverEnabled: true
            acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton

            property real lastX: 0
            property real lastY: 0
            property real pressX: 0
            property real pressY: 0
            property bool isDragging: false

            onPressed: (mouse) => {
                lastX = mouse.x; lastY = mouse.y;
                pressX = mouse.x; pressY = mouse.y;
                isDragging = false;
            }

            onPositionChanged: (mouse) => {
                var dx = mouse.x - lastX;
                var dy = mouse.y - lastY;

                if (!isDragging) {
                    if (Math.abs(mouse.x - pressX) > 3 || Math.abs(mouse.y - pressY) > 3) {
                        isDragging = true;
                        lastX = mouse.x;  // eat accumulated delta to avoid jump
                        lastY = mouse.y;
                        return;
                    }
                }

                if (isDragging) {
                    if (mouse.buttons & Qt.LeftButton) {
                        graphViewport.onMouseRotate(dx, dy);
                    } else if (mouse.buttons & Qt.MiddleButton) {
                        graphViewport.onMousePan(dx, dy);
                    }
                }
                lastX = mouse.x;
                lastY = mouse.y;
            }

            onReleased: (mouse) => {
                if (!isDragging && mouse.button === Qt.LeftButton) {
                    graphViewport.requestPick(mouse.x, mouse.y);
                }
            }

            onWheel: (wheel) => { graphViewport.onMouseZoom(wheel.angleDelta.y); }
        }

        // Keyboard shortcuts
        Shortcut { sequence: "R"; onActivated: graphViewport.resetCamera() }
        Shortcut { sequence: "F"; onActivated: graphViewport.resetCamera() }

        // Statistics overlay (real data from GraphManager)
        ColumnLayout {
            id: statsPanel
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 20
            spacing: 8

            Label { text: "STATISTICS"; font.bold: true; color: "#009688"; font.pixelSize: 16 }
            Label { text: "Vertices:  " + GraphManager.vertexCount; color: "#A0AABF"; font.pixelSize: 14 }
            Label { text: "Edges:     " + GraphManager.edgeCount; color: "#A0AABF"; font.pixelSize: 14 }
            Label { text: "Keyframes: " + GraphManager.keyframeCount; color: "#A0AABF"; font.pixelSize: 14 }
            Label { text: "FPS:       -- fps"; color: "#A0AABF"; font.pixelSize: 14 }
        }

        // Loading progress indicator (visible during map loading)
        ColumnLayout {
            id: loadingPanel
            anchors.centerIn: parent
            spacing: 12
            visible: GraphManager.isLoading

            BusyIndicator {
                running: true
                implicitWidth: 48; implicitHeight: 48
                palette { dark: "#FF8C00" }
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Label {
                id: loadingTitle
                text: ""
                color: "#FF8C00"
                font.pixelSize: 16
                font.bold: true
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Label {
                id: loadingText
                text: ""
                color: "#A0AABF"
                font.pixelSize: 13
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }

    // ==========================================
    // Connections: GraphManager signals → UI
    // ==========================================
    Connections {
        target: GraphManager

        function onLoadingStarted() {
            log("Loading map data...");
        }

        function onLoadingSucceeded() {
            log("Map loaded: " + GraphManager.vertexCount + " vertices, "
                + GraphManager.edgeCount + " edges, "
                + GraphManager.keyframeCount + " keyframes");
        }

        function onLoadingFailed(error) {
            log("ERROR: " + error);
        }
    }

    // Loading progress text
    Connections {
        target: GraphManager.progress

        function onTitleChanged(title) {
            loadingTitle.text = title;
        }
        function onTextChanged(text) {
            loadingText.text = text;
        }
    }

    // Pick result (one frame after requestPick)
    Connections {
        target: graphViewport

        function onPickResultReady() {
            if (graphViewport.pickedVertexId >= 0) {
                graphViewport.selectedVertexId = graphViewport.pickedVertexId;
                log("Picked vertex " + graphViewport.pickedVertexId
                  + " at (" + graphViewport.pickedWorldX.toFixed(2) + ", "
                  + graphViewport.pickedWorldY.toFixed(2) + ", "
                  + graphViewport.pickedWorldZ.toFixed(2) + ")");
            }
        }
    }

        // Rendering config popup (inside mainCanvas)
        Rectangle {
            id: renderingPopup
            width: 460
            height: 200
            x: 80; y: 60
            visible: mainWindow.showRenderConfig

            color: "#F21A1C20"
            border.color: "#3D4450"
            border.width: 1
            radius: 4

            Rectangle {
                id: popupTitleBar
                width: parent.width
                height: 30
                color: "#21252B"
                radius: 4

                Rectangle {
                    anchors.bottom: parent.bottom
                    width: parent.width; height: 4; color: parent.color
                }

                Label {
                    text: "  Rendering Config"
                    anchors.verticalCenter: parent.verticalCenter
                    color: "#009688"
                    font.bold: true
                    font.pixelSize: 13
                }

                Text {
                    text: "× "
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    color: closeArea.containsMouse ? "#FF8C00" : "#808A9F"
                    font.pixelSize: 18

                    MouseArea {
                        id: closeArea
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: mainWindow.showRenderConfig = false
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    anchors.rightMargin: 35
                    property point clickPos: "0,0"

                    onPressed: (mouse) => { clickPos = Qt.point(mouse.x, mouse.y) }
                    onPositionChanged: (mouse) => {
                        var delta = Qt.point(mouse.x - clickPos.x, mouse.y - clickPos.y)
                        renderingPopup.x += delta.x
                        renderingPopup.y += delta.y
                    }
                }
            }

            ScrollView {
                anchors.top: popupTitleBar.bottom
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 15
                clip: true

                ColumnLayout {
                    width: parent.width - 10
                    spacing: 18

                    // General switches
                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "General"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch {
                            text: "Draw Vertices"
                            checked: graphViewport.drawVertices
                            onCheckedChanged: graphViewport.drawVertices = checked
                        }
                        Switch {
                            text: "Draw Edges"
                            checked: graphViewport.drawEdges
                            onCheckedChanged: graphViewport.drawEdges = checked
                        }
                    }

                    // Vertex switches
                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "Vertex"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch {
                            text: "Keyframes"
                            checked: graphViewport.drawKeyframeVertices
                            onCheckedChanged: graphViewport.drawKeyframeVertices = checked
                        }
                    }

                    // Edge switches
                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "Edge"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch {
                            text: "SE3"
                            checked: graphViewport.drawSE3Edges
                            onCheckedChanged: graphViewport.drawSE3Edges = checked
                        }
                    }
                }
            }
        }

        // Optimization indicator (kept for future use)
        RowLayout {
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            anchors.margins: 30
            visible: mainWindow.isOptimizing
            spacing: 15

            BusyIndicator {
                running: mainWindow.isOptimizing
                implicitWidth: 32; implicitHeight: 32
                palette { dark: "#FF8C00" }
            }
            Label {
                text: "Optimizing Graph..."
                color: "#FF8C00"
                font.pixelSize: 18
                font.bold: true
            }
        }
    }  // mainCanvas

    // ==========================================
    // 3. 底部状态栏
    // ==========================================
    Rectangle {
        id: statusBar
        height: 28
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        color: "#009688"

        Text {
            id: logText
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: parent.left
            anchors.leftMargin: 15
            color: "#FFFFFF"
            font.pixelSize: 13
            text: "System Ready."
        }
    }

    function log(msg) {
        logText.text = "> " + msg;
    }
}