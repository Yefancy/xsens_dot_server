const WebSocket = require('ws');
const SyncSensorDataManagement = require('./syncSensorDataManagement');
const SensorServer = require('./sensorServer');
const log = require('node-gyp/lib/log');

// 创建 WebSocket 服务器
const wss = new WebSocket.Server({ port: 8080 });

let currentParams = '';
function onRealDataUpdated(params) {
    currentParams = params
    // console.log(params);
}

wss.on('connection', (ws) => {
    console.log('Client connected');

    // 定时发送数据给客户端
    setInterval(() => {
        const data = JSON.stringify({ message: currentParams, timestamp: Date.now() });
        ws.send(data);
    }, 20);

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

console.log("Python socket server running at http://localhost:8080/");
module.exports.onRealDataUpdated = onRealDataUpdated;
