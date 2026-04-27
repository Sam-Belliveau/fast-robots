let ws;
let commandSchemas = {};
let robotConnected = false;

// --- WebSocket ---

function initWebSocket() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => { ws.send(JSON.stringify({type : "get_commands"})); };
    ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
    ws.onclose = () => { setTimeout(initWebSocket, 2000); };
}

function handleMessage(msg) {
    switch (msg.type) {
    case "commands":
        populateCommandDropdown(msg.commands);
        break;
    case "result":
        renderResponse(msg);
        break;
    case "error":
        renderError(msg);
        break;
    case "status":
        updateConnectionStatus(msg.status);
        break;
    }
}

// --- Connection Status ---

const STATUS_LABELS = {
    disconnected : "Disconnected",
    scanning : "Scanning...",
    connected : "Connected",
};

function updateConnectionStatus(status) {
    robotConnected = status === "connected";
    const el = document.getElementById("status");
    el.textContent = STATUS_LABELS[status] || status;
    el.className = status;
    updateSendButton();
}

// --- Helpers ---

function makeId() {
    return ("msg-" + Date.now() + "-" + Math.random().toString(36).slice(2, 6));
}

function wsSend(command, params, logToChat = true) {
    const id = makeId();
    if (logToChat)
        appendSentBubble(id, command, params);
    ws.send(JSON.stringify({type : "execute", id, command, params}));
    return id;
}

// --- Command Dropdown ---

function populateCommandDropdown(commands) {
    const select = document.getElementById("cmd-select");
    const groups = {};
    commands.forEach((cmd) => {
        let category = "Core";
        const name = cmd.name;
        if (name.startsWith("PID") || name === "SendPIDData" ||
            name.startsWith("AnglePID") || name === "SendAnglePIDData")
            category = "PID";
        else if (name.startsWith("Motor"))
            category = "Motors";
        else if (name.startsWith("ToF") || name === "SendToFData" || name === "ToFMode")
            category = "ToF";
        else if (name === "SendIMUData")
            category = "IMU";
        else if (name.startsWith("KF") || name === "SendKFData")
            category = "Kalman";
        else if (name === "StepResponse" || name === "SendStepData")
            category = "Step Response";
        else if (name.startsWith("Stunt") || name === "SendStuntData")
            category = "Stunts";
        else if (name.startsWith("Map") || name === "SendMapData")
            category = "Mapping";
        else if (
            ["StartRecording", "StopRecording", "StoreTimeMillis",
             "SendTimeMillis", "GetTimeMillis"].includes(name)
        )
            category = "Recording";

        if (!groups[category]) groups[category] = [];
        groups[category].push(cmd);
        commandSchemas[cmd.name] = cmd;
    });

    select.innerHTML = '<option value="">Select command...</option>';

    const order = [
        "Core",
        "Recording",
        "IMU",
        "ToF",
        "Motors",
        "PID",
        "Kalman",
        "Step Response",
        "Stunts",
        "Mapping"
    ];
    order.forEach((cat) => {
        if (!groups[cat]) return;
        const optgroup = document.createElement("optgroup");
        optgroup.label = cat;
        groups[cat].forEach((cmd) => {
            const opt = document.createElement("option");
            opt.value = cmd.name;
            opt.textContent = cmd.name;
            if (cmd.fields.length > 0) {
                const paramStr = cmd.fields
                    .map((f) => `${f.name}: ${f.type}`)
                    .join(", ");
                opt.textContent += ` (${paramStr})`;
            }
            optgroup.appendChild(opt);
        });
        select.appendChild(optgroup);
    });
}

// --- Parameter Fields ---

document.getElementById("cmd-select").addEventListener("change", (e) => {
    const name = e.target.value;
    const container = document.getElementById("param-fields");
    container.innerHTML = "";

    if (name && commandSchemas[name]) {
        const schema = commandSchemas[name];
        schema.fields.forEach((field) => {
            const label = document.createElement("label");
            label.textContent = field.name;
            const input = document.createElement("input");
            input.name = field.name;
            input.placeholder = field.name;
            if (field.type === "str") {
                input.type = "text";
            } else {
                input.type = "number";
                if (field.type === "float") input.step = "any";
            }
            if (field.default !== null && field.default !== undefined) {
                input.value = field.default;
            }
            label.appendChild(input);
            container.appendChild(label);
        });
    }
    updateSendButton();
});

function updateSendButton() {
    const btn = document.getElementById("btn-send");
    const selected = document.getElementById("cmd-select").value;
    btn.disabled = !selected || !robotConnected;
}

// --- Send Command ---

document.getElementById("btn-send").addEventListener("click", sendCommand);

document.getElementById("param-fields").addEventListener("keydown", (e) => {
    if (e.key === "Enter") sendCommand();
});

function sendCommand() {
    const select = document.getElementById("cmd-select");
    const name = select.value;
    if (!name)
        return;

    const schema = commandSchemas[name];
    const params = {};
    schema.fields.forEach((field) => {
        const input = document.querySelector(
            `#param-fields input[name="${field.name}"]`
        );
        if (input) params[field.name] = input.value;
    });

    wsSend(name, params);
}

// --- Chat Rendering ---

function appendSentBubble(id, name, params) {
    const log = document.getElementById("chat-log");

    const msg = document.createElement("div");
    msg.className = "message sent";
    msg.dataset.id = id;

    const cmdName = document.createElement("div");
    cmdName.className = "command-name";
    cmdName.textContent = name;
    msg.appendChild(cmdName);

    const paramKeys = Object.keys(params);
    if (paramKeys.length > 0) {
        const paramDiv = document.createElement("div");
        paramDiv.className = "params";
        paramDiv.textContent =
            paramKeys.map((k) => `${k}=${params[k]}`).join(", ");
        msg.appendChild(paramDiv);
    }

    const response = document.createElement("div");
    response.className = "response loading";
    response.textContent = "Waiting...";
    msg.appendChild(response);

    log.appendChild(msg);
    log.scrollTop = log.scrollHeight;
}

function renderResponse(msg) {
    const bubble =
        document.querySelector(`.message[data-id="${msg.id}"] .response`);
    if (!bubble)
        return;
    bubble.className = "response";

    if (msg.html) {
        bubble.innerHTML = msg.html;
    } else if (msg.data === null) {
        bubble.textContent = "OK";
    } else if (typeof msg.data === "string") {
        bubble.textContent = msg.data;
    } else {
        const pre = document.createElement("pre");
        pre.textContent = JSON.stringify(msg.data, null, 2);
        bubble.appendChild(pre);
    }

    // CSV copy button for array data
    if (Array.isArray(msg.data) && msg.data.length > 0 &&
        typeof msg.data[0] === "object") {
        const csvBtn = document.createElement("button");
        csvBtn.className = "btn-csv";
        csvBtn.textContent = "Copy CSV";
        csvBtn.addEventListener("click", () => {
            const keys = Object.keys(msg.data[0]);
            const header = keys.join(",");
            const rows = msg.data.map(
                (row) => keys.map((k) => row[k]).join(",")
            );
            const csv = [header, ...rows].join("\n");
            navigator.clipboard.writeText(csv).then(() => {
                csvBtn.textContent = "Copied!";
                setTimeout(() => { csvBtn.textContent = "Copy CSV"; }, 1500);
            });
        });
        bubble.appendChild(csvBtn);
    }

    const elapsed = document.createElement("div");
    elapsed.className = "elapsed";
    elapsed.textContent = `${msg.elapsed_ms} ms`;
    bubble.parentElement.appendChild(elapsed);

    const log = document.getElementById("chat-log");
    log.scrollTop = log.scrollHeight;
}

function renderError(msg) {
    if (!msg.id) {
        const log = document.getElementById("chat-log");
        const div = document.createElement("div");
        div.className = "message received error";
        const resp = document.createElement("div");
        resp.className = "response";
        resp.textContent = msg.message;
        div.appendChild(resp);
        log.appendChild(div);
        log.scrollTop = log.scrollHeight;
        return;
    }

    const bubble =
        document.querySelector(`.message[data-id="${msg.id}"] .response`);
    if (!bubble)
        return;
    bubble.className = "response";
    bubble.parentElement.classList.add("error");
    bubble.textContent = msg.message;

    const log = document.getElementById("chat-log");
    log.scrollTop = log.scrollHeight;
}

// ============================================================
// PID Tuning Dashboard
// ============================================================

const pidFields = document.querySelectorAll("#pid-panel .pid-group label");

pidFields.forEach((label) => {
    const input = label.querySelector("input");
    input.addEventListener("input", () => {
        const committed = input.dataset.committed;
        const dirty = input.value !== committed;
        input.classList.toggle("dirty", dirty);
        const span = label.querySelector(".pid-label");
        let marker = span.querySelector(".dirty-marker");
        if (dirty && !marker) {
            marker = document.createElement("span");
            marker.className = "dirty-marker";
            marker.textContent = " *";
            span.appendChild(marker);
        } else if (!dirty && marker) {
            marker.remove();
        }
    });

    input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") commitPidFields();
    });
});

function commitPidFields() {
    const get = (name) => {
        const label = document.querySelector(
            `#pid-panel label[data-pid="${name}"]`
        );
        return label.querySelector("input");
    };

    const kpIn = get("kp"), kiIn = get("ki"), kdIn = get("kd");
    const spIn = get("setpoint");
    const capIn = get("cap"), rangeIn = get("range");
    const rcIn = get("rc"), dbIn = get("deadband");
    const yawKpIn = get("yaw_kp"), yawKdIn = get("yaw_kd");

    // Check which groups are dirty
    const gainsDirty =
        [ kpIn, kiIn, kdIn ].some((i) => i.value !== i.dataset.committed);
    const setpointDirty = spIn.value !== spIn.dataset.committed;
    const paramsDirty = [ capIn, rangeIn, rcIn, dbIn ].some(
        (i) => i.value !== i.dataset.committed
    );
    const yawDirty =
        [ yawKpIn, yawKdIn ].some((i) => i.value !== i.dataset.committed);

    if (gainsDirty) {
        wsSend("PIDGains", {
            kp : kpIn.value,
            ki : kiIn.value,
            kd : kdIn.value,
        });
        markCommitted(kpIn);
        markCommitted(kiIn);
        markCommitted(kdIn);
    }
    if (setpointDirty) {
        wsSend("PIDSetpoint", {setpoint : spIn.value});
        markCommitted(spIn);
    }
    if (paramsDirty) {
        wsSend("PIDParams", {
            cap : capIn.value,
            range : rangeIn.value,
            rc : rcIn.value,
            deadband : dbIn.value,
        });
        markCommitted(capIn);
        markCommitted(rangeIn);
        markCommitted(rcIn);
        markCommitted(dbIn);
    }
    if (yawDirty) {
        wsSend("AnglePIDGains", {
            kp : yawKpIn.value,
            kd : yawKdIn.value,
        });
        markCommitted(yawKpIn);
        markCommitted(yawKdIn);
    }

    if (!gainsDirty && !setpointDirty && !paramsDirty && !yawDirty) {
        // Nothing dirty — still commit all as a "re-send"
        wsSend("PIDGains", {
            kp : kpIn.value,
            ki : kiIn.value,
            kd : kdIn.value,
        });
        wsSend("PIDSetpoint", {setpoint : spIn.value});
        wsSend("PIDParams", {
            cap : capIn.value,
            range : rangeIn.value,
            rc : rcIn.value,
            deadband : dbIn.value,
        });
        wsSend("AnglePIDGains", {
            kp : yawKpIn.value,
            kd : yawKdIn.value,
        });
    }
}

function markCommitted(input) {
    input.dataset.committed = input.value;
    input.classList.remove("dirty");
    const marker = input.closest("label").querySelector(".dirty-marker");
    if (marker)
        marker.remove();
}

let pidAutoFetchTimer = null;

document.getElementById("pid-start").addEventListener("click", () => {
    // Commit any dirty fields first
    commitPidFields();
    const dur = document.querySelector(
        '#pid-panel label[data-pid="duration"] input'
    ).value;
    wsSend("PIDStart", { duration_ms: dur });

    // Auto-fetch data after duration elapses
    if (pidAutoFetchTimer) clearTimeout(pidAutoFetchTimer);
    pidAutoFetchTimer = setTimeout(() => {
        wsSend("SendPIDData", {});
        pidAutoFetchTimer = null;
    }, Number(dur) + 500);
});

document.getElementById(
            "pid-stop"
).addEventListener("click", () => { wsSend("PIDStop", {}); });

document.getElementById(
            "pid-data"
).addEventListener("click", () => { wsSend("SendPIDData", {}); });

// ============================================================
// Kalman Filter Dashboard
// ============================================================

const kfFields = document.querySelectorAll("#kf-panel .pid-group label");

kfFields.forEach((label) => {
    const input = label.querySelector("input");
    input.addEventListener("input", () => {
        const committed = input.dataset.committed;
        const dirty = input.value !== committed;
        input.classList.toggle("dirty", dirty);
        const span = label.querySelector(".pid-label");
        let marker = span.querySelector(".dirty-marker");
        if (dirty && !marker) {
            marker = document.createElement("span");
            marker.className = "dirty-marker";
            marker.textContent = " *";
            span.appendChild(marker);
        } else if (!dirty && marker) {
            marker.remove();
        }
    });

    input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") commitKfFields();
    });
});

function commitKfFields() {
    const get = (name) => {
        const label = document.querySelector(
            `#kf-panel label[data-kf="${name}"]`
        );
        return label.querySelector("input");
    };

    const dragIn = get("drag");
    const momIn = get("momentum");
    const s1In = get("sigma_1");
    const s2In = get("sigma_2");
    const s3In = get("sigma_3");

    const all = [ dragIn, momIn, s1In, s2In, s3In ];

    wsSend("KFParams", {
        drag : dragIn.value,
        momentum : momIn.value,
        sigma_1 : s1In.value,
        sigma_2 : s2In.value,
        sigma_3 : s3In.value,
    });

    all.forEach(markCommitted);
}

document.getElementById("kf-params").addEventListener("click", commitKfFields);

document.getElementById(
            "kf-reset"
).addEventListener("click", () => { wsSend("KFReset", {}); });

document.getElementById("kf-data").addEventListener(
    "click", () => { wsSend("SendKFData", {}); }
);

// ============================================================
// Step Response Dashboard
// ============================================================

let stepAutoFetchTimer = null;

document.getElementById("step-start").addEventListener("click", () => {
    const dur = document.querySelector(
        '#step-panel label[data-step="duration"] input'
    ).value;
    const pwm = document.querySelector(
        '#step-panel label[data-step="pwm"] input'
    ).value;
    wsSend("StepResponse", { duration_ms: dur, pwm: pwm });

    if (stepAutoFetchTimer) clearTimeout(stepAutoFetchTimer);
    stepAutoFetchTimer = setTimeout(() => {
        wsSend("SendStepData", {});
        stepAutoFetchTimer = null;
    }, Number(dur) + 500);
});

document.getElementById(
            "step-data"
).addEventListener("click", () => { wsSend("SendStepData", {}); });

// ============================================================
// Joystick Dashboard
// ============================================================

const joystickCanvas = document.getElementById("joystick");
const ctx = joystickCanvas.getContext("2d");
const JOY_SIZE = 200;
const JOY_RADIUS = JOY_SIZE / 2;
const KNOB_RADIUS = 24;
const MAX_PWM = 255;
const SEND_INTERVAL_MS = 80;

let joyX = 0; // normalized -1..1
let joyY = 0; // normalized -1..1
let joyActive = false;
let joySendTimer = null;
let lastSentL = 0;
let lastSentR = 0;

function drawJoystick() {
    ctx.clearRect(0, 0, JOY_SIZE, JOY_SIZE);

    // Outer ring
    ctx.beginPath();
    ctx.arc(JOY_RADIUS, JOY_RADIUS, JOY_RADIUS - 4, 0, Math.PI * 2);
    ctx.strokeStyle = "#3c3c3c";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Crosshair
    ctx.beginPath();
    ctx.moveTo(JOY_RADIUS, 8);
    ctx.lineTo(JOY_RADIUS, JOY_SIZE - 8);
    ctx.moveTo(8, JOY_RADIUS);
    ctx.lineTo(JOY_SIZE - 8, JOY_RADIUS);
    ctx.strokeStyle = "#3c3c3c";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Knob position
    const knobX = JOY_RADIUS + joyX * (JOY_RADIUS - KNOB_RADIUS - 4);
    const knobY = JOY_RADIUS + joyY * (JOY_RADIUS - KNOB_RADIUS - 4);

    // Knob
    ctx.beginPath();
    ctx.arc(knobX, knobY, KNOB_RADIUS, 0, Math.PI * 2);
    ctx.fillStyle = joyActive ? "#89d185" : "#555";
    ctx.fill();
    ctx.strokeStyle = joyActive ? "#6bb365" : "#3c3c3c";
    ctx.lineWidth = 2;
    ctx.stroke();
}

function updateJoyFromEvent(e) {
    const rect = joystickCanvas.getBoundingClientRect();
    let clientX, clientY;
    if (e.touches) {
        clientX = e.touches[0].clientX;
        clientY = e.touches[0].clientY;
    } else {
        clientX = e.clientX;
        clientY = e.clientY;
    }
    const rawX = (clientX - rect.left - JOY_RADIUS) / (JOY_RADIUS - 4);
    const rawY = (clientY - rect.top - JOY_RADIUS) / (JOY_RADIUS - 4);

    // Clamp to unit circle
    const mag = Math.sqrt(rawX * rawX + rawY * rawY);
    if (mag > 1) {
        joyX = rawX / mag;
        joyY = rawY / mag;
    } else {
        joyX = rawX;
        joyY = rawY;
    }
    drawJoystick();
    updateMotorReadout();
}

function getMotorValues() {
    // y is inverted: up (negative) = forward (positive PWM)
    const fwd = -joyY;
    const turn = joyX;
    let left = Math.round((fwd + turn) * MAX_PWM);
    let right = Math.round((fwd - turn) * MAX_PWM);
    left = Math.max(-MAX_PWM, Math.min(MAX_PWM, left));
    right = Math.max(-MAX_PWM, Math.min(MAX_PWM, right));
    return {left, right};
}

function updateMotorReadout() {
    const {left, right} = getMotorValues();
    document.getElementById("motor-l").textContent = left;
    document.getElementById("motor-r").textContent = right;
}

function sendMotorCmd() {
    const {left, right} = getMotorValues();
    if (left === lastSentL && right === lastSentR)
        return;
    lastSentL = left;
    lastSentR = right;
    wsSend("MotorCmd", {left : String(left), right : String(right)}, false);
}

function startJoy(e) {
    e.preventDefault();
    joyActive = true;
    updateJoyFromEvent(e);
    sendMotorCmd();
    joySendTimer = setInterval(sendMotorCmd, SEND_INTERVAL_MS);
}

function moveJoy(e) {
    if (!joyActive)
        return;
    e.preventDefault();
    updateJoyFromEvent(e);
}

function endJoy(e) {
    if (!joyActive)
        return;
    e.preventDefault();
    joyActive = false;
    joyX = 0;
    joyY = 0;
    drawJoystick();
    updateMotorReadout();
    clearInterval(joySendTimer);
    joySendTimer = null;
    // Send stop
    lastSentL = 0;
    lastSentR = 0;
    wsSend("MotorCmd", {left : "0", right : "0"}, false);
}

joystickCanvas.addEventListener("mousedown", startJoy);
window.addEventListener("mousemove", moveJoy);
window.addEventListener("mouseup", endJoy);

joystickCanvas.addEventListener("touchstart", startJoy, {passive : false});
window.addEventListener("touchmove", moveJoy, {passive : false});
window.addEventListener("touchend", endJoy);

drawJoystick();

// ============================================================
// WASD Keyboard Controls
// ============================================================

const WASD_PWM = 255;
const wasdKeys = {
    w : false,
    a : false,
    s : false,
    d : false
};
let wasdActive = false;
let wasdLastL = 0;
let wasdLastR = 0;

function wasdUpdate() {
    let fwd = 0;
    let turn = 0;
    if (wasdKeys.w)
        fwd += 1;
    if (wasdKeys.s)
        fwd -= 1;
    if (wasdKeys.a)
        turn -= 1;
    if (wasdKeys.d)
        turn += 1;

    let left = Math.round((fwd + turn) * WASD_PWM);
    let right = Math.round((fwd - turn) * WASD_PWM);
    left = Math.max(-MAX_PWM, Math.min(MAX_PWM, left));
    right = Math.max(-MAX_PWM, Math.min(MAX_PWM, right));

    if (left === wasdLastL && right === wasdLastR)
        return;
    wasdLastL = left;
    wasdLastR = right;

    document.getElementById("motor-l").textContent = left;
    document.getElementById("motor-r").textContent = right;
    wsSend("MotorCmd", {left : String(left), right : String(right)}, false);
}

window.addEventListener("keydown", (e) => {
    const key = e.key.toLowerCase();
    if (key in wasdKeys && !wasdKeys[key]) {
        // Don't capture WASD when typing in an input
        if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
        e.preventDefault();
        wasdKeys[key] = true;
        wasdActive = true;
        wasdUpdate();
    }
});

window.addEventListener("keyup", (e) => {
    const key = e.key.toLowerCase();
    if (key in wasdKeys) {
        wasdKeys[key] = false;
        wasdActive = Object.values(wasdKeys).some(Boolean);
        wasdUpdate();
    }
});

// ============================================================
// Gamepad Controls
// ============================================================
// Right trigger = forward, Left trigger = backward
// Left stick X = turning
// DPad: left=-90deg, right=+90deg, down=180deg (angle PID)

const GP_SEND_INTERVAL_MS = 80;
const GP_DEADZONE = 0.1;
const GP_TRIGGER_DEADZONE = 0.05;
let gpSendTimer = null;
let gpLastL = 0;
let gpLastR = 0;
let gpConnected = false;

// Track DPad previous state to detect presses (not holds)
let gpDpadPrev = {left : false, right : false, down : false};

window.addEventListener("gamepadconnected", (e) => {
    gpConnected = true;
    if (!gpSendTimer) {
        gpSendTimer = setInterval(gpPoll, GP_SEND_INTERVAL_MS);
    }
});

window.addEventListener("gamepaddisconnected", (e) => {
    gpConnected = false;
    if (gpSendTimer) {
        clearInterval(gpSendTimer);
        gpSendTimer = null;
    }
    // Stop motors on disconnect
    if (gpLastL !== 0 || gpLastR !== 0) {
        gpLastL = 0;
        gpLastR = 0;
        wsSend("MotorCmd", {left : "0", right : "0"}, false);
    }
});

function gpPoll() {
    const gamepads = navigator.getGamepads();
    let gp = null;
    for (const pad of gamepads) {
        if (pad) {
            gp = pad;
            break;
        }
    }
    if (!gp)
        return;

    // Triggers: buttons[6]=LT, buttons[7]=RT (values 0..1)
    const lt = gp.buttons[6] ? gp.buttons[6].value : 0;
    const rt = gp.buttons[7] ? gp.buttons[7].value : 0;

    // Left stick X: axes[0]
    let stickX = gp.axes[0] || 0;
    if (Math.abs(stickX) < GP_DEADZONE)
        stickX = 0;

    // Forward/backward from triggers
    let fwd = 0;
    if (rt > GP_TRIGGER_DEADZONE)
        fwd += rt;
    if (lt > GP_TRIGGER_DEADZONE)
        fwd -= lt;

    let turn = stickX;

    let left = Math.round((fwd + turn) * MAX_PWM);
    let right = Math.round((fwd - turn) * MAX_PWM);
    left = Math.max(-MAX_PWM, Math.min(MAX_PWM, left));
    right = Math.max(-MAX_PWM, Math.min(MAX_PWM, right));

    if (left !== gpLastL || right !== gpLastR) {
        gpLastL = left;
        gpLastR = right;
        document.getElementById("motor-l").textContent = left;
        document.getElementById("motor-r").textContent = right;
        wsSend("MotorCmd", {left : String(left), right : String(right)}, false);
    }

    // DPad: buttons[14]=left, buttons[15]=right, buttons[13]=down
    const dpadLeft = gp.buttons[14] ? gp.buttons[14].pressed : false;
    const dpadRight = gp.buttons[15] ? gp.buttons[15].pressed : false;
    const dpadDown = gp.buttons[13] ? gp.buttons[13].pressed : false;

    if (dpadLeft && !gpDpadPrev.left) {
        wsSend("AnglePIDSetpointRel", {delta : "-90"});
        wsSend("AnglePIDStart", {duration_ms : "3000"});
    }
    if (dpadRight && !gpDpadPrev.right) {
        wsSend("AnglePIDSetpointRel", {delta : "90"});
        wsSend("AnglePIDStart", {duration_ms : "3000"});
    }
    if (dpadDown && !gpDpadPrev.down) {
        wsSend("AnglePIDSetpointRel", {delta : "180"});
        wsSend("AnglePIDStart", {duration_ms : "3000"});
    }

    gpDpadPrev.left = dpadLeft;
    gpDpadPrev.right = dpadRight;
    gpDpadPrev.down = dpadDown;
}

// ============================================================
// Stunts Dashboard
// ============================================================

const stuntFields = document.querySelectorAll("#stunt-panel .pid-group label");

stuntFields.forEach((label) => {
    const input = label.querySelector("input");
    input.addEventListener("input", () => {
        const committed = input.dataset.committed;
        const dirty = input.value !== committed;
        input.classList.toggle("dirty", dirty);
        const span = label.querySelector(".pid-label");
        let marker = span.querySelector(".dirty-marker");
        if (dirty && !marker) {
            marker = document.createElement("span");
            marker.className = "dirty-marker";
            marker.textContent = " *";
            span.appendChild(marker);
        } else if (!dirty && marker) {
            marker.remove();
        }
    });

    input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") commitStuntFields();
    });
});

function commitStuntFields() {
    const get = (name) => {
        const label = document.querySelector(
            `#stunt-panel label[data-stunt="${name}"]`
        );
        return label.querySelector("input");
    };

    const speedIn = get("speed");
    const triggerIn = get("trigger");
    const retIn = get("return_speed");
    const yawKpIn = get("yaw_kp");
    const yawKdIn = get("yaw_kd");

    const paramsDirty = [speedIn, triggerIn, retIn].some(
        (i) => i.value !== i.dataset.committed
    );
    const yawDirty = [yawKpIn, yawKdIn].some(
        (i) => i.value !== i.dataset.committed
    );

    if (paramsDirty || (!paramsDirty && !yawDirty)) {
        wsSend("StuntParams", {
            speed : speedIn.value,
            trigger_distance : triggerIn.value,
            return_speed : retIn.value,
        });
        [speedIn, triggerIn, retIn].forEach(markCommitted);
    }
    if (yawDirty || (!paramsDirty && !yawDirty)) {
        wsSend("StuntYawGains", {
            kp : yawKpIn.value,
            kd : yawKdIn.value,
        });
        [yawKpIn, yawKdIn].forEach(markCommitted);
    }
}

let stuntAutoFetchTimer = null;

document.getElementById("stunt-drift").addEventListener("click", () => {
    commitStuntFields();
    const dur = document.querySelector(
        '#stunt-panel label[data-stunt="duration"] input'
    ).value;
    wsSend("StuntDrift", { duration_ms: dur });

    if (stuntAutoFetchTimer) clearTimeout(stuntAutoFetchTimer);
    stuntAutoFetchTimer = setTimeout(() => {
        wsSend("SendStuntData", {});
        stuntAutoFetchTimer = null;
    }, Number(dur) + 500);
});

document.getElementById("stunt-flip").addEventListener("click", () => {
    commitStuntFields();
    const dur = document.querySelector(
        '#stunt-panel label[data-stunt="duration"] input'
    ).value;
    wsSend("StuntFlip", { duration_ms: dur });

    if (stuntAutoFetchTimer) clearTimeout(stuntAutoFetchTimer);
    stuntAutoFetchTimer = setTimeout(() => {
        wsSend("SendStuntData", {});
        stuntAutoFetchTimer = null;
    }, Number(dur) + 500);
});

document.getElementById(
    "stunt-stop"
).addEventListener("click", () => { wsSend("StuntStop", {}); });

document.getElementById(
    "stunt-data"
).addEventListener("click", () => { wsSend("SendStuntData", {}); });

// ============================================================
// Mapping Dashboard
// ============================================================

const mapFields = document.querySelectorAll("#map-panel .pid-group label");

mapFields.forEach((label) => {
    const input = label.querySelector("input");
    input.addEventListener("input", () => {
        const committed = input.dataset.committed;
        const dirty = input.value !== committed;
        input.classList.toggle("dirty", dirty);
        const span = label.querySelector(".pid-label");
        let marker = span.querySelector(".dirty-marker");
        if (dirty && !marker) {
            marker = document.createElement("span");
            marker.className = "dirty-marker";
            marker.textContent = " *";
            span.appendChild(marker);
        } else if (!dirty && marker) {
            marker.remove();
        }
    });

    input.addEventListener("keydown", (e) => {
        if (e.key === "Enter") commitMapFields();
    });
});

function commitMapFields() {
    const get = (name) => {
        const label = document.querySelector(
            `#map-panel label[data-map="${name}"]`
        );
        return label.querySelector("input");
    };

    const threshIn = get("settle_threshold");
    const settleIn = get("settle_time");
    const timeoutIn = get("step_timeout");
    const samplesIn = get("samples_per_step");
    const stdIn = get("angle_std");

    const fields = [threshIn, settleIn, timeoutIn, samplesIn, stdIn];
    const dirty = fields.some((i) => i.value !== i.dataset.committed);

    if (dirty) {
        wsSend("MapParams", {
            settle_threshold : threshIn.value,
            settle_time_ms : settleIn.value,
            step_timeout_ms : timeoutIn.value,
            samples_per_step : samplesIn.value,
            angle_std_deg : stdIn.value,
        });
        fields.forEach(markCommitted);
    }
}

document.getElementById("map-start").addEventListener("click", () => {
    commitMapFields();
    const steps = document.querySelector(
        '#map-panel label[data-map="num_steps"] input'
    ).value;
    wsSend("MapStart", {num_steps : steps});
});

document.getElementById(
    "map-stop"
).addEventListener("click", () => { wsSend("MapStop", {}); });

document.getElementById(
    "map-data"
).addEventListener("click", () => { wsSend("SendMapData", {}); });

// --- Init ---
initWebSocket();
