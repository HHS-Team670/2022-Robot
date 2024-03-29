
var PopupClass = require('js-popup');

document.getElementById('big-warning').style.display = "none";

// initial camera settings
var driveReversed = false;
var allKeysPressed = new Array();

var selectedPath = "";

function updatePath(evt) {
    if (document.querySelector(".path-dropdown:hover") != null) {
        evt.preventDefault();
        selectedPath = evt.target.innerHTML;
        let dropupBtn = document.querySelector(".dropbtn");
        dropupBtn.innerHTML = selectedPath;
    }
}

function setPaths(upper) {
    var content = document.querySelector(".dropup-content");
    content.innerHTML = "";

    var p1 = document.createElement("p");
    p1.appendChild(document.createTextNode("ATarmacEdge2Ball"));
    
    var p2 = document.createElement("p");
    p2.appendChild(document.createTextNode("BTarmacEdgeCenter2Ball"));

    var p3 = document.createElement("p");
    p3.appendChild(document.createTextNode("BTarmacEdgeLower2Ball"));

    if (upper) {
        var p4 = document.createElement("p");
        p4.appendChild(document.createTextNode("BTarmacHighHubTerminal"));
        content.append(p1, p2, p3, p4);
    } else {
        content.append(p1, p2, p3);
    }
    resetAndAddDropdownListeners();
}


var lower = document.querySelector('#Lower');
lower.onclick = () => {setPaths(false)};
var upper = document.querySelector('#Upper');
upper.onclick = () => {setPaths(true)};
var toggleCamera = document.querySelector('#toggle-camera');
toggleCamera.onclick = () => {
    var cameraDiv = document.querySelector("#camera-streams")
    if (document.querySelector(".camera-stream") == null) {
        var cameraStreams = `<image class="camera-stream" src="http://photonvision.local:1182/stream.mjpg?1664842058013"></image>
        <image class="camera-stream" src="http://photonvision.local:1184/stream.mjpg?1646964457545"></image>`
        // var cameraStreams = `<image class="camera-stream" src="http://photonvision.local:1184/stream.mjpg?1646964457545"></image>`;
        cameraDiv.insertAdjacentHTML( 'beforeend', cameraStreams );
    } else {
        while (cameraDiv.firstChild) {
            cameraDiv.removeChild(cameraDiv.firstChild);
        }
    }
}


function resetAndAddDropdownListeners() {
    document.querySelector(".dropbtn").innerHTML = "Choose Path";
    var paths = document.querySelectorAll(".path-dropdown .dropup-content p");
    for (let i = 0; i < paths.length; i+=1) {
        let path = paths[i];
        path.onclick = updatePath;
    }
}




// listens for robot-state and updates status lights and auton chooser accordingly
NetworkTables.addKeyListener('/SmartDashboard/robot-state', (key, value) => {
    if (value === "autonomousInit()" || value === "disabledPeriodic()") {
        document.getElementById('auton-chooser').style.display = "none";
    } else if (value === "autonomousPeriodic()") {
        document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
        document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
    } else if (value === "teleopInit()" || value === "teleopPeriodic()") {
        document.getElementById('auton-status').style.fill = "none";
        document.getElementById('auton-status').style.stroke = "rgb(255,255,255)";
    }
});

// listens for warnings
NetworkTables.addKeyListener('/SmartDashboard/warnings', (key, value) => {
    document.getElementById('big-warning').style.display = "inline";
    document.getElementById('warnings').innerHTML += (value + "\n");

    setTimeout(() => { document.getElementById('big-warning').style.display = "none"; }, 1000);
    var timeSinceWarningFlashed = Date.getTime();
});

NetworkTables.addGlobalListener((key, value) => {
    // console.log(key + ": " + value);
})

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/Conveyor Ball Count', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    document.getElementById('balls-in-robot').textContent = 'Balls in Robot: ' + value;

    document.getElementById('indicator1').style.fill = "rgb(0,0,0)";
    document.getElementById('indicator1').style.stroke = "rgb(255,255,255)";
    document.getElementById('indicator2').style.fill = "rgb(0,0,0)";
    document.getElementById('indicator2').style.stroke = "rgb(255,255,255)";

    if (value > 0) {
        document.getElementById('indicator1').style.fill = "rgb(0,255,0)";
        document.getElementById('indicator1').style.stroke = "rgb(255,255,255)";
    }
    
    if (value > 1) {
        document.getElementById('indicator2').style.fill = "rgb(0,255,0)";
        document.getElementById('indicator2').style.stroke = "rgb(255,255,255)";
    }
});



// updates vision frame
NetworkTables.addKeyListener('/SmartDashboard/vision-frame-updated', (key, value) => {
    if (value == true) {
        document.getElementById('vision-frame').src = document.getElementById('vision-frame').src;
        NetworkTables.putValue('vision-frame-updated', false);
    }
});

// updates robot sped
NetworkTables.addKeyListener('/SmartDashboard/expectedSpeed', (key, value) => {
    document.getElementById('speed').textContent = 'Speed: ' + value;
});

// updates robot angle and direction
    NetworkTables.addKeyListener('/SmartDashboard/vision-values-', (key, value) => {
        document.getElementById('distance').textContent = 'Distance: ' + value[0];
        document.getElementById('angle').textContent = 'Angle: ' + value[2];
    });

// updates status lights for driveBase
NetworkTables.addKeyListener('/SmartDashboard/DriveBase', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('drivebase-status').style.fill = "rgb(0,255,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('drivebase-status').style.fill = "rgb(255,255,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('drivebase-status').style.fill = "rgb(255,0,0)";
        document.getElementById('drivebase-status').style.stroke = "rgb(255,0,0)";
    }
});


// updates status lights for conveyor
NetworkTables.addKeyListener('/SmartDashboard/ConveyorSystem', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('conveyor-status').style.fill = "rgb(0,255,0)";
        document.getElementById('conveyor-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('conveyor-status').style.fill = "rgb(255,255,0)";
        document.getElementById('conveyor-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('conveyor-status').style.fill = "rgb(255,0,0)";
        document.getElementById('conveyor-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for shooter
NetworkTables.addKeyListener('/SmartDashboard/Shooter', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('shooter-status').style.fill = "rgb(0,255,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('shooter-status').style.fill = "rgb(255,255,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('shooter-status').style.fill = "rgb(255,0,0)";
        document.getElementById('shooter-status').style.stroke = "rgb(255,0,0)";
    }
});


// updates status lights for Climber 1
NetworkTables.addKeyListener('/SmartDashboard/Climber 1', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('climber1-status').style.fill = "rgb(0,255,0)";
        document.getElementById('climber1-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('climber1-status').style.fill = "rgb(255,255,0)";
        document.getElementById('climber1-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('climber1-status').style.fill = "rgb(255,0,0)";
        document.getElementById('climber1-status').style.stroke = "rgb(255,0,0)";
    }
    else if (value === 'UNKNOWN') {
        document.getElementById('climber1-status').style.fill = "rgb(255,255,255)";
        document.getElementById('climber1-status').style.stroke = "rgb(255,255,255)";
    }
});

// updates status lights for Climber 2
NetworkTables.addKeyListener('/SmartDashboard/Climber 2', (key, value) => {
    var statusLights = document.getElementById('status-lights-subsystems');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('climber2-status').style.fill = "rgb(0,255,0)";
        document.getElementById('climber2-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('climber2-status').style.fill = "rgb(255,255,0)";
        document.getElementById('climber2-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('climber2-status').style.fill = "rgb(255,0,0)";
        document.getElementById('climber2-status').style.stroke = "rgb(255,0,0)";
    }
    else if (value === 'UNKNOWN') {
        document.getElementById('climber2-status').style.fill = "rgb(255,255,255)";
        document.getElementById('climber2-status').style.stroke = "rgb(255,255,255)";
    }
});

// updates status lights for Intake
NetworkTables.addKeyListener('/SmartDashboard/Intake', (key, value) => {
    var statusLights = document.getElementById('status-lights-commands');
    var commands = statusLights.contentDocument;
    if (value === 'GREEN') {
        document.getElementById('intake-status').style.fill = "rgb(0,255,0)";
        document.getElementById('intake-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'YELLOW') {
        document.getElementById('intake-status').style.fill = "rgb(255,255,0)";
        document.getElementById('intake-status').style.stroke = "rgb(255,255,0)";
    } else if (value === 'RED') {
        document.getElementById('intake-status').style.fill = "rgb(255,0,0)";
        document.getElementById('intake-status').style.stroke = "rgb(255,0,0)";
    }
});

// updates status lights for vision
NetworkTables.addKeyListener('/Vision/vision-data', (key, value) => {
    var statusLights = document.getElementById('status-lights-commands');
    var commands = statusLights.contentDocument;
    if (value === 'engaged') {
        document.getElementById('vision-status').style.fill = "rgb(0,255,0)";
        document.getElementById('vision-status').style.stroke = "rgb(0,255,0)";
    } else if (value === 'invalid-target') {
        document.getElementById('vision-status').style.fill = "rgb(241,244,66)";
        document.getElementById('vision-status').style.stroke = "rgb(241,244,66)";
    } else if (value === 'error' || value === -99999) {
        document.getElementById('vision-status').style.fill = "rgb(255,0,0)";
        document.getElementById('vision-status').style.stroke = "rgb(255,0,0)";
    } else {
        document.getElementById('vision-status').style.fill = "none";
        document.getElementById('vision-status').style.stroke = "rgb(255,255,255)";
    }
});

// updates status lights for vision
NetworkTables.addKeyListener('/SmartDashboard/overrided-rpm', (key, value) => {
    var shooterSpeedIndicator = document.querySelector("#shooter-speed-indicator");
    switch(value) {
        case "NOT OVERRIDED":
            shooterSpeedIndicator.textContent = "USING DYNAMIC SPEED";
            shooterSpeedIndicator.style.stroke = "white";
            shooterSpeedIndicator.x.baseVal[0].valueAsString = "20%"
            document.querySelector("#shooter-speed-indicator-bg").style.fill = "none";
            break;
        case "LOW TOUCHING FENDER":
            shooterSpeedIndicator.textContent = "OVERRIDED: LOW TOUCHING FENDER";
            shooterSpeedIndicator.style.stroke = "black";
            shooterSpeedIndicator.x.baseVal[0].valueAsString = "0%"
            document.querySelector("#shooter-speed-indicator-bg").style.fill = "rgb(0, 255, 255)";
            break;
        case "LOW OUTSIDE TARMAC":
            shooterSpeedIndicator.textContent = "OVERRIDED: LOW OUTSIDE TARMAC";
            shooterSpeedIndicator.style.stroke = "black";
            shooterSpeedIndicator.x.baseVal[0].valueAsString = "0%"
            document.querySelector("#shooter-speed-indicator-bg").style.fill = "rgb(162, 0, 255)";
            break;
        case "HIGH JUST OUTSIDE TARMAC":
            shooterSpeedIndicator.textContent = "OVERRIDED: HIGH JUST OUTSIDE TARMAC";
            shooterSpeedIndicator.style.stroke = "black";
            shooterSpeedIndicator.x.baseVal[0].valueAsString = "0%"
            document.querySelector("#shooter-speed-indicator-bg").style.fill = "rgb(234, 0, 255)";
            break;
        default:
            shooterSpeedIndicator.textContent = "USING DYNAMIC SPEED";
            shooterSpeedIndicator.style.stroke = "white";
            shooterSpeedIndicator.x.baseVal[0].valueAsString = "20%"
            document.querySelector("#shooter-speed-indicator-bg").style.fill = "none";
            break;
    }
});

NetworkTables.addKeyListener('/SmartDashboard/aligned', (key, value) => {
    var db = document.querySelector('.camera-stream');
    value ? db.id = 'green-border' : db.id = '';
})

document.getElementById("confirm-button").onclick = function() {
  sendAuton();
};

// // listens for keystrokes from the external keypad and passes the corresponding values over networktables
var keys = [];
var allKeys = '';
document.addEventListener("keyup", function(event) {
    var pressed = event.key.replace("Enter", "");
    allKeys += pressed;
    var result = allKeys[allKeys.length - 1];
    var nextTask = getFromMap(result);

    console.log(nextTask);
    allKeysPressed.push(nextTask);

    // make sure the key pressed is a valid action
    if (nextTask != null) {
        if (nextTask.toUpperCase() === nextTask) NetworkTables.putValue('/SmartDashboard/xkeys-robotstates', nextTask);
        else if (nextTask.includes("cancel")) NetworkTables.putValue('/SmartDashboard/xkeys-cancel', nextTask);
        else if (nextTask.includes("shoot")) NetworkTables.putValue('/SmartDashboard/xkeys-shooter', nextTask);
        else if (nextTask.includes("updraw")) NetworkTables.putValue('/SmartDashboard/xkeys-updraw', nextTask);
        else if (nextTask.includes("climber")) NetworkTables.putValue('/SmartDashboard/xkeys-climber', nextTask);
        else if (nextTask.includes("vision")) NetworkTables.putValue('/Vision/vision-data', nextTask);
        else if (nextTask.includes("intake") || nextTask.includes("roller")) NetworkTables.putValue('/SmartDashboard/xkeys-intake', nextTask);

    }
});

function getFromMap(key) {

    // public static final double RUN_INTAKE_IN = 0;
    // public static final double RUN_INTAKE_OUT = 1;
    // public static final double TOGGLE_INTAKE = 2;

    // public static final double INIT_SHOOTER = 4;
    // public static final double SHOOT = 6;
    // public static final double SHOOT_ALL = 7;

    // public static final double INCREASE_SHOOTER_RPM = 8;
    // public static final double DECREASE_SHOOTER_RPM = 9;

    // public static final double INDEXER_INTAKE = 10;

    // public static final double EXTEND_CLIMBER = 12;
    // public static final double RETRACT_CLIMBER = 13;

    // public static final double SHOOT_NEAR = 14;
    // public static final double SHOOT_MID = 15;
    // public static final double SHOOT_LONG = 16;

    // public static final double CANCEL_ALL = 18;

    if (key === "3") return "4";
    if (key === "5") return "6";
    if (key === "2") return "7";

    if (key === "c") return "14";
    if (key === "j") return "15";
    if (key === "d") return "16";

    if (key === "k") return "2";
    if (key === "e") return "0";
    if (key === "f") return "1";

    if (key === "a") return "NEUTRAL";

    if (key === "m") return "12";
    if (key === "n") return "13";

    if (key === "y") return "vision";

    if (key === "v") return "18";

    return null;
}


function getAutonFromMap() {
    // console.log("SELECTED VALUE", document.querySelector('input[name="path"]:checked').value);
    console.log("SELECTED VALUE", selectedPath);
    // switch (document.querySelector('input[name="path"]:checked').value) {
    if (document.querySelector('input[name="hub-type"]:checked') != null) {
        switch (document.querySelector('input[name="hub-type"]:checked').value) {
            case "Upper":
                switch(selectedPath) {
                    case "ATarmacEdge2Ball":
                        return 0.0;
                    case "BTarmacEdgeCenter2Ball":
                        return 1.0;
                    case "BTarmacEdgeLower2Ball":
                        return 2.0;
                    case "BTarmacHighHubTerminal":
                        return 3.0;
                }
            case "Lower":
                switch(selectedPath) {
                    case "ATarmacEdge2Ball":
                        return 4.0;
                    case "BTarmacEdgeCenter2Ball":
                        return 5.0;
                    case "BTarmacEdgeLower2Ball":
                        return 6.0;
                }
        }
    }
    return -1;
}

function getDelayTime() {
    return parseFloat(document.querySelector('#delay input[name="delay-time"]').value)
}


function sendAuton() {
    var autonCommand = getAutonFromMap();
    var delayTime = getDelayTime();
    console.log("SELECTED AUTON COMMAND", autonCommand);
    NetworkTables.putValue('/SmartDashboard/auton-chooser', autonCommand);
    NetworkTables.putValue('/SmartDashboard/delayTime', delayTime);
    if (autonCommand !== -1 && NetworkTables.getValue('/SmartDashboard/auton-chooser') === autonCommand
    && NetworkTables.getKeys().length > 5) {
        document.getElementById('auton-status').style.fill = "rgb(0,255,0)";
        document.getElementById('auton-status').style.stroke = "rgb(0,255,0)";
    } else {
        document.getElementById('auton-status').style.fill = "rgb(255,0,0)";
        document.getElementById('auton-status').style.stroke = "rgb(255,0,0)";
    }
}