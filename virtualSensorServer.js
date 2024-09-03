'use strict';

const socket = require("socket.io");
const port = 11100;

class VirtualSensorServer {
    constructor(syncSensorDataManagement = null) {
        this.syncSensorDataManagement = syncSensorDataManagement;
        this.syncSensorDataManagement.virtualSensorServer = this;
        let http = require('http');
        let socket = require('socket.io');
        this.server = http.createServer();
        this.io = socket(this.server, {
            pingInterval: 10000,
            pingTimeout: 5000
        });

        this.io.use((socket, next) => {
            if (socket.handshake.query.token === "UNITY") {
                next();
            } else {
                next(new Error("Authentication error"));
            }
        });

        this.io.on('connection', socket => {
            console.log('connection: ' + socket.handshake.address);

            setTimeout(() => {
                socket.emit('connection', {date: new Date().getTime(), data: "Hello Unity!"})
            }, 1000);

            socket.on('sensorData', data => {
                for (let datum of data) {
                    this.syncSensorDataManagement.uploadVirtualSensorData(datum);
                }
                if (this.syncSensorDataManagement.sensorServer.gui != null) {
                    this.syncSensorDataManagement.sensorServer.gui.sendGuiEvent( 'virtualSensor', data);
                }
            });

            socket.on('recording', data => {
                switch (data) {
                    case 'Start':
                        this.syncSensorDataManagement.startRecording();
                        break;
                    case 'Stop':
                        this.syncSensorDataManagement.stopRecording();
                        break;
                    default:
                        console.log('Unknown recording command: ' + data);
                }
            })

            socket.once('disconnect', () => {
                console.log('disconnect:' + socket.handshake.address);
            });
        });

        this.server.listen(port, () => {
            console.log('listening on *:' + port);
        });
    }
}

module.exports = VirtualSensorServer;