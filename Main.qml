import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

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
            Action { text: qsTr("Close Map") }
            Action { text: qsTr("Quit"); onTriggered: Qt.quit() }
        }

        Menu {
            title: qsTr("View")
            Action { text: qsTr("Reset camera") }
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
            log("Selected folder: " + selectedFolder)
            // TODO: 新建地图的初始化逻辑放在这里
        }
        onRejected: {
            log("Folder selection cancelled")
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

        Grid {
            anchors.centerIn: parent
            rows: 20; columns: 20; spacing: 40
            Repeater {
                model: 400
                Rectangle { width: 2; height: 2; radius: 1; color: "#009688"; opacity: 0.2 }
            }
        }

        Text {
            anchors.centerIn: parent
            text: "3D OpenGL Canvas Area\n(Awaiting C++ RHI Integration)"
            color: "#4A5260"
            font.pixelSize: 20
            font.letterSpacing: 2
            horizontalAlignment: Text.AlignHCenter
        }

        ColumnLayout {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.margins: 20
            spacing: 8

            Label { text: "STATISTICS"; font.bold: true; color: "#009688"; font.pixelSize: 16 }
            Label { text: "Vertices:  " + mainWindow.mockVertexCount; color: "#A0AABF"; font.pixelSize: 14 }
            Label { text: "Edges:     " + mainWindow.mockEdgeCount; color: "#A0AABF"; font.pixelSize: 14 }
            Label { text: "FPS:       60.0 fps"; color: "#A0AABF"; font.pixelSize: 14 }
        }

        // 优化状态提示
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
        Rectangle {
            id: renderingPopup
            width: 460                         // 紧凑宽度，类似 ImGui 大小
            height: 200                        // 紧凑高度
            x: 80; y: 60                       // 弹窗初始弹出的绝对坐标位置
            visible: mainWindow.showRenderConfig // 受控制变量绑定
            
            color: "#F21A1C20"                 // 95% 不透明度的暗色背景（高级半透明质感）
            border.color: "#3D4450"            // 灰色科技感细边框
            border.width: 1
            radius: 4

            // 悬浮窗标题栏（支持鼠标拖拽）
            Rectangle {
                id: popupTitleBar
                width: parent.width
                height: 30
                color: "#21252B"
                radius: 4

                // 用于修复圆角穿透的下边缘直角矩形
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

                // 右上角迷你关闭按钮 "×"
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

                // 【拖拽核心逻辑】：按住标题栏可以自由移动弹窗
                MouseArea {
                    anchors.fill: parent
                    anchors.rightMargin: 35 // 避开关闭按钮
                    property point clickPos: "0,0"
                    
                    onPressed: (mouse) => { clickPos = Qt.point(mouse.x, mouse.y) }
                    onPositionChanged: (mouse) => {
                        var delta = Qt.point(mouse.x - clickPos.x, mouse.y - clickPos.y)
                        renderingPopup.x += delta.x
                        renderingPopup.y += delta.y
                    }
                }
            }

            // 悬浮窗内部的开关内容（完全复用您原先的组件和排版）
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

                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "General"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch { text: "Draw Vertices"; checked: true }
                        Switch { text: "Draw Edges"; checked: true }
                    }

                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "Vertex"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch { text: "Keyframes"; checked: true }
                        Switch { text: "Planes"; checked: true }
                    }

                    RowLayout {
                        width: parent.width; spacing: 3
                        Label { text: "Edge"; color: "#FF8C00"; font.pixelSize: 11; font.bold: true; Layout.bottomMargin: 3 }
                        Switch { text: "SE3"; checked: true }
                        Switch { text: "Plane"; checked: true }
                        Switch { text: "SE3Plane"; checked: true }
                        Switch { text: "SE3Floor"; checked: true }
                    }
                }
            }
        }
    }

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