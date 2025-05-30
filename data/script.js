// const ws = new WebSocket("ws://192.168.0.100/ws");
const ws = new WebSocket("ws://192.168.147.215/ws");
let isSending = false;

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    const log = document.getElementById('log');
    console.log('Message from server: ', data);

    // Log the message
    log.textContent += event.data + '\n';

    // Handle byte response
    if (data.type === 'byte_response') {
        document.getElementById('s1m').textContent = data.s1m;
        document.getElementById('s2m').textContent = data.s2m;
    }

    // Handle LED response
    if (data.type === 'led_response') {
        log.textContent += `Set ${data.slave} LED to ${data.color}\n`;
    }
};

ws.onclose = function(event) {
    console.log('WebSocket closed: ', event);
};

ws.onerror = function(error) {
    console.log('WebSocket error: ', error);
};

function toggleSendByte() {
    const byteInput = document.getElementById('byteInput').value;
    const byte = parseInt(byteInput);
    const toggleButton = document.getElementById('toggleButton');

    if (isNaN(byte) || byte < 0 || byte > 255) {
        alert('Please enter a valid byte (0-255)');
        return;
    }

    if (!isSending) {
        // Bật gửi dữ liệu (chỉ gửi một lần)
        isSending = true;
        toggleButton.textContent = 'Stop Sending';
        const message = JSON.stringify({ byte: byte, action: 'start' });
        ws.send(message);
    } else {
        // Tắt gửi dữ liệu
        isSending = false;
        toggleButton.textContent = 'Start Sending';
        const message = JSON.stringify({ action: 'stop' });
        ws.send(message);
    }
}

function setColor(slave, color) {
    const message = JSON.stringify({ slave: slave, color: color });
    ws.send(message);
}

