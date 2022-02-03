
var countDownTimer;
var endTime;
var matchPhase = document.getElementById("match-phase");
var timer = document.getElementById("timer");
var timerPrefixString = "Time of Match: ";
var phasePrefixString = "Match Phase: ";
var timeoutFunc = null;
var seconds;
var minutes;
const MatchPhases = Object.freeze ({
    NOT_STARTED: Object.freeze( {text: "NOT STARTED", color:"rgb(50,50,50)"}),
    AUTON: Object.freeze({text: "AUTON", color:"rgb(70,70,200)"}),
    TELEOP: Object.freeze({text:"TELEOP",color:"rgb(0,200,0)"}),
    ENDED: Object.freeze({text:"ENDED",color:"rgb(200,0,0)"})
});
NetworkTables.addKeyListener("/SmartDashboard/MatchTime", (key, value) => {
    updateTimer(value);
    var timeString = getTimeString(minutes, seconds);

});

NetworkTables.addKeyListener("/SmartDashboard/IsAuton", (key, value) => {
    
});


function stopTimer () {
    if (countDownTimer != null) clearInterval(countDownTimer);
    timer.textContent = timerPrefixString;
    runtimer = false;
}

function getTimeString (minutes, seconds) {
    return (seconds == 60 ? minutes + 1 : minutes) + ':' 
        + (Math.round(seconds) < 10 ? "0" : "") 
            + (seconds == 60 ? "00" : Math.round(seconds));
}

function updateTimer(time) {

    seconds =( time % 60);
    minutes = Math.floor( ( time % (60*60)) / 60);
}

function setMatchPhase(phase) {
    
    matchPhase.textContent = phasePrefixString + phase.text;
    matchPhase.style.backgroundColor = phase.color;
    if (phase == MatchPhases.AUTON || phase == MatchPhases.NOT_STARTED) {
        document.getElementById("auton-chooser").style.display = "block";
    } else {
        
        document.getElementById("auton-chooser").style.display = "none";
    }
}


document.getElementById("timer-stopper").onmouseup = function() {
    stopTimer();
    setMatchPhase(MatchPhases.NOT_STARTED);
}


document.getElementById("timer-starter").onmouseup = function() {
    if (runtimer) return;
    runtimer = true;
    endTime = new Date().getTime() + FULL_MATCH_LENGTH_MILLIS;
    
    updateTimer(endTime);

    timer.textContent = timerPrefixString + getTimeString(minutes, seconds);
    if (timeoutFunc != null) clearTimeout(timeoutFunc);
    setMatchPhase(MatchPhases.AUTON);
    
    countDownTimer = setInterval(function() {
        
        updateTimer(endTime);


        // TODO regex format the time
        timer.textContent = timerPrefixString + getTimeString(minutes, seconds);
            
        // If the count down is over, write some text 
        if (timeDifference < 0) {
            stopTimer();
            setMatchPhase(MatchPhases.ENDED);
            timeoutFunc = setTimeout(() => {
                setMatchPhase(MatchPhases.NOT_STARTED); 
                clearTimeout(timeoutFunc);
                timeoutFunc = null;
            }, 5000);
        } else if (timeDifference <= FULL_MATCH_LENGTH_MILLIS - AUTON_TIME_MILLIS) {
            setMatchPhase(MatchPhases.TELEOP);
        }
    }, 1000);
    runtimer = true;
};
