let ws;
let commandSchemas = {};

// --- WebSocket ---

function initWebSocket() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => {
        ws.send(JSON.stringify({ type: "get_commands" }));
    };
    ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
    ws.onclose = () => {
        setTimeout(initWebSocket, 2000);
    };
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
    }
}

// --- Command Dropdown ---

function populateCommandDropdown(commands) {
    const select = document.getElementById("cmd-select");
    const groups = {};
    commands.forEach((cmd) => {
        let category = "Core";
        const name = cmd.name;
        if (name.startsWith("PID") || name === "SendPIDData")
            category = "PID";
        else if (name.startsWith("Motor"))
            category = "Motors";
        else if (name.startsWith("ToF") || name === "SendToFData")
            category = "ToF";
        else if (name === "SendIMUData")
            category = "IMU";
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

    const order = ["Core", "Recording", "IMU", "ToF", "Motors", "PID"];
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
    btn.disabled = !selected;
}

// --- Send Command ---

document.getElementById("btn-send").addEventListener("click", sendCommand);

document.getElementById("param-fields").addEventListener("keydown", (e) => {
    if (e.key === "Enter") sendCommand();
});

function sendCommand() {
    const select = document.getElementById("cmd-select");
    const name = select.value;
    if (!name) return;

    const schema = commandSchemas[name];
    const params = {};
    schema.fields.forEach((field) => {
        const input = document.querySelector(
            `#param-fields input[name="${field.name}"]`
        );
        if (input) params[field.name] = input.value;
    });

    const id =
        "msg-" + Date.now() + "-" +
        Math.random().toString(36).slice(2, 6);

    appendSentBubble(id, name, params);
    ws.send(JSON.stringify({ type: "execute", id, command: name, params }));
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
        paramDiv.textContent = paramKeys
            .map((k) => `${k}=${params[k]}`)
            .join(", ");
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
    const bubble = document.querySelector(
        `.message[data-id="${msg.id}"] .response`
    );
    if (!bubble) return;
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

    const bubble = document.querySelector(
        `.message[data-id="${msg.id}"] .response`
    );
    if (!bubble) return;
    bubble.className = "response";
    bubble.parentElement.classList.add("error");
    bubble.textContent = msg.message;

    const log = document.getElementById("chat-log");
    log.scrollTop = log.scrollHeight;
}

// --- Init ---
initWebSocket();
