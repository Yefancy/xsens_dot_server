let SyncSensorDataManagement = require('./syncSensorDataManagement.js');
let SensorServer = require('./sensorServer.js');
let VirtualSensorServer = require('./virtualSensorServer.js');
let PythonSocket = require('./pythonSocket.js');

let syncSensorDataManager = new SyncSensorDataManagement();
let sensorServer = new SensorServer(syncSensorDataManager);
let virtualSensorServer = new VirtualSensorServer(syncSensorDataManager);
// var datum = {
//     "tag": 'tag',
//     "timestamp": 2123,
//     "ex": 1,
//     "ey": 2,
//     "ez": 3,
//     "ax": 4,
//     "ay": 5,
//     "az": 6,
//     "lx": -1,
//     "ly": -2,
//     "lz": -3
// }
// var row = Object.values(datum).join();
// console.log(row);