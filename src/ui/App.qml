import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15 as QQC
import QtQuick.Dialogs

import "."
import "components/"
import "menu/" as Menu

Rectangle {
    id: window;
    visible: true
    color: styleBackground;
    anchors.fill: parent;

    property bool isLandscape: width > height;
    onIsLandscapeChanged: {
        videoAreaCol.parent = null;
        if (!isLandscape) {
            videoAreaCol.x = 0;
            videoAreaCol.width = Qt.binding(() => window.width);
            videoAreaCol.height = Qt.binding(() => window.height * 0.5);
            leftPanel.fixedWidth = Qt.binding(() => window.width * 0.4);
            rightPanel.fixedWidth = Qt.binding(() => window.width * 0.6);
            mainLayout.y = Qt.binding(() => videoAreaCol.height);

            videoAreaCol.parent = window;
        } else {
            videoAreaCol.parent = mainLayout;
            videoAreaCol.width = Qt.binding(() => mainLayout.width - leftPanel.width - rightPanel.width);
            videoAreaCol.height = Qt.binding(() => mainLayout.height);
            leftPanel.fixedWidth = 0;
            rightPanel.fixedWidth = 0;
            mainLayout.y = 0;
            rightPanel.parent = null;
            rightPanel.parent = mainLayout;
        }
    }
    property alias videoArea: videoArea;
    property alias motionData: motionData;
    property alias lensProfile: lensProfile;
    property alias exportSettings: exportSettings;
    property alias outputFile: outputFile.text;
    property alias sync: sync;

    FileDialog {
        id: fileDialog;
        property var extensions: ["mp4", "mov", "mxf", "mkv", "webm"];

        title: qsTr("Choose a video file")
        nameFilters: Qt.platform.os == "android"? undefined : [qsTr("Video files") + " (*." + extensions.join(" *.") + ")"];
        onAccepted: videoArea.loadFile(fileDialog.selectedFile);
    }

    Row {
        id: mainLayout;
        width: parent.width;
        height: parent.height - y;

        SidePanel {
            id: leftPanel;
            direction: SidePanel.HandleRight;
            topPadding: gflogo.height;
            Item {
                id: gflogo;
                parent: leftPanel;
                width: parent.width;
                height: children[0].height + 35 * dpiScale;
                Image {
                    source: "qrc:/resources/logo" + (style === "dark"? "_white" : "_black") + ".svg"
                    sourceSize.width: parent.width * 0.9;
                    anchors.centerIn: parent;
                }
                Hr { anchors.bottom: parent.bottom; }
            }

            Menu.VideoInformation {
                id: vidInfo;
                onSelectFileRequest: fileDialog.open();
            }
            Menu.LensProfile {
                id: lensProfile;
            }
            Menu.MotionData {
                id: motionData;
            }
        }

        Column {
            id: videoAreaCol;
            width: parent? parent.width - leftPanel.width - rightPanel.width : 0;
            height: parent? parent.height : 0;
            VideoArea {
                id: videoArea;
                height: parent.height - exportbar.height;
                vidInfo: vidInfo;
            }

            // Bottom bar
            Rectangle {
                id: exportbar;
                width: parent.width;
                height: 60 * dpiScale;
                color: styleBackground2;

                Hr { width: parent.width; }

                Row {
                    height: parent.height;
                    spacing: 10 * dpiScale;
                    BasicText {
                        text: qsTr("Output path:");
                        anchors.verticalCenter: parent.verticalCenter;
                    }
                    TextField {
                        id: outputFile;
                        text: "";
                        anchors.verticalCenter: parent.verticalCenter;
                        width: exportbar.width - parent.children[0].width - exportbar.children[2].width - 30 * dpiScale;
                    }
                }

                SplitButton {
                    id: renderBtn;
                    accent: true;
                    anchors.right: parent.right;
                    anchors.rightMargin: 15 * dpiScale;
                    anchors.verticalCenter: parent.verticalCenter;
                    text: qsTr("Export");
                    icon.name: "video";
                    enabled: window.videoArea.vid.loaded && exportSettings.canExport && !videoArea.videoLoader.active;
                    opacity: enabled? 1.0 : 0.6;
                    Ease on opacity { }
                    fadeWhenDisabled: false;
                    property bool successShown: false;

                    model: [qsTr("Export .gyroflow file")];

                    function doRender() {
                        successShown = false;
                        controller.render(
                            exportSettings.outCodec, 
                            exportSettings.outCodecOptions, 
                            outputFile.text, 
                            videoArea.trimStart, 
                            videoArea.trimEnd, 
                            exportSettings.outWidth, 
                            exportSettings.outHeight, 
                            exportSettings.outBitrate, 
                            exportSettings.outGpu, 
                            exportSettings.outAudio
                        );
                    }
                    function renameOutput() {
                        const orgOutput = outputFile.text;
                        let output = orgOutput;
                        let i = 1;
                        while (controller.file_exists(output)) {
                            output = orgOutput.replace(/_stabilized(_\d+)?\.mp4/, "_stabilized_" + i++ + ".mp4");
                            if (i > 1000) break;
                        }

                        outputFile.text = output;
                        clicked(null);
                    }
                    onClicked: {
                        if (controller.file_exists(outputFile.text)) {
                            messageBox(Modal.NoIcon, qsTr("Output file already exists, do you want to overwrite it?"), [
                                { text: qsTr("Yes"), clicked: doRender },
                                { text: qsTr("Rename"), clicked: renameOutput },
                                { text: qsTr("No"), accent: true },
                            ]);
                        } else {
                            doRender();
                        }
                    }
                    popup.onClicked: (index) => {
                        controller.export_gyroflow();
                    }
                    
                    Connections {
                        target: controller;
                        function onRender_progress(progress, frame, total_frames) {
                            videoArea.videoLoader.active = progress < 1;
                            videoArea.videoLoader.progress = videoArea.videoLoader.active? progress : -1;
                            videoArea.videoLoader.text = videoArea.videoLoader.active? qsTr("Rendering %1... %2").arg("<b>" + (progress * 100).toFixed(2) + "%</b>").arg("<font size=\"2\">(" + frame + "/" + total_frames + ")</font>") : "";
                            videoArea.videoLoader.cancelable = true;
                            if (frame > 0 && frame == total_frames && !renderBtn.successShown) {
                                renderBtn.successShown = true;
                                messageBox(Modal.Success, qsTr("Rendering completed. The file was written to: %1.").arg("<br><b>" + outputFile.text + "</b>"), [
                                    { text: qsTr("Open rendered file"), clicked: () => controller.open_file_externally(outputFile.text) },
                                    { text: qsTr("Ok") }
                                ]);
                            }
                        }
                    }
                }
            }
        }

        SidePanel {
            id: rightPanel;
            direction: SidePanel.HandleLeft;
            Menu.Synchronization {
                id: sync;
            }
            Menu.Stabilization {
                id: stab;
            }
            Menu.Export {
                id: exportSettings;
            }
            Menu.Advanced {

            }
        }
    }

    function messageBox(type, text, buttons) {
        const el = Qt.createComponent("components/Modal.qml").createObject(window, { text: text, iconType: type });
        el.onClicked.connect((index) => {
            if (buttons[index].clicked)
                buttons[index].clicked();
            el.opened = false;
            el.destroy(1000);
        });
        let buttonTexts = [];
        for (const i in buttons) {
            buttonTexts.push(buttons[i].text);
            if (buttons[i].accent) {
                el.accentButton = i;
            }
        }
        el.buttons = buttonTexts;
        
        el.opened = true;
        return el;
    }

    Connections {
        target: controller;
        function onError(text, arg, callback) {
            messageBox(Modal.Error, qsTr(text).arg(arg), [ { "text": qsTr("Ok"), clicked: window[callback] } ]);
        }
    }

    function checkUpdate() {
        const xhr = new XMLHttpRequest()
        xhr.onreadystatechange = function() {
            if (xhr.readyState === XMLHttpRequest.DONE) {
                const obj = JSON.parse(xhr.responseText.toString());
                if (obj && obj.length) {
                    const latestVersion = obj[0].name.replace("v", "");
                    console.log('Latest version:', latestVersion, '. Current: ', version);
                    if (version != latestVersion) {
                        const body = obj[0].body? "\n\n" + obj[0].body : "";
                        messageBox(Modal.Info, qsTr("There's a newer version available.") + body, [ { text: qsTr("Download"), clicked: () => Qt.openUrlExternally("https://github.com/AdrianEddy/gyroflow/releases") }, { text: qsTr("Close") }])
                    }
                }
            }
        }
        xhr.open("GET", "https://api.github.com/repos/AdrianEddy/gyroflow/releases")
        xhr.send()
    }
    Component.onCompleted: {
        checkUpdate();
    }
}
