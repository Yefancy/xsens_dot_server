//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.      Redistributions of source code must retain the above copyright notice,
//           this list of conditions, and the following disclaimer.
//  
//  2.      Redistributions in binary form must reproduce the above copyright notice,
//           this list of conditions, and the following disclaimer in the documentation
//           and/or other materials provided with the distribution.
//  
//  3.      Neither the names of the copyright holders nor the names of their contributors
//           may be used to endorse or promote products derived from this software without
//           specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

// =======================================================================================
// Sensor Server
// Documentation: documentation/Xsens DOT Server - Sensor Server.pdf
// =======================================================================================

// =======================================================================================
// Packages
// =======================================================================================
var fs                  = require('fs');
var BleHandler          = require('./bleHandler');
var WebGuiHandler       = require('./webGuiHandler');
var FunctionalComponent = require('./functionalComponent');
var SyncManager         = require('./syncManager');
var events              = require('events');

// =======================================================================================
// Constants
// =======================================================================================
const RECORDINGS_PATH = "/data/",
      RECORDING_BUFFER_TIME = 1000000;

// =======================================================================================
// State transitions table
// =======================================================================================
let PythonSocket = require('./pythonSocket.js');

// ========= Transform with magnetometer and accelerometer ==========
// function normalize(v) {
//     const mag = Math.sqrt(v[0]**2 + v[1]**2 + v[2]**2);
//     return v.map(x => x / mag);
// }

// function crossProduct(v1, v2) {
//     return [
//         v1[1] * v2[2] - v1[2] * v2[1],
//         v1[2] * v2[0] - v1[0] * v2[2],
//         v1[0] * v2[1] - v1[1] * v2[0]
//     ];
// }

// function dotProduct(v1, v2) {
//     return v1.reduce((sum, x, i) => sum + x * v2[i], 0);
// }

// function matrixTranspose(m) {
//     return m[0].map((_, i) => m.map(row => row[i]));
// }

// function multiplyMatrixVector(m, v) {
//     return m.map(row => dotProduct(row, v));
// }

// function transformToLocal(mag_x, mag_y, mag_z, ax, ay, az) {
//     const magnet = [mag_x, mag_y, mag_z];
//     const globalVector = [ax, ay, az];
//     const accel = [0, 0, 9.8];
//     const g = normalize(accel);
//     const m = normalize(magnet);
//     const east = normalize(crossProduct(m, g));
//     const north = crossProduct(g, east);
//     const rotationMatrix = [east, north, g];
//     const localVector = multiplyMatrixVector(matrixTranspose(rotationMatrix), globalVector);
//     return localVector
// }

// =====================Transform with Quaternion 2=========================
// function quaternionMultiply(q1, q2) {
//     return [
//       q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
//       q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
//       q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
//       q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
//     ];
//   }
  
//   // Helper function to calculate the conjugate of a quaternion
//   function quaternionConjugate(q) {
//     return [q[0], -q[1], -q[2], -q[3]];
//   }
  
//   // Function to rotate a vector using a quaternion
//   function rotateVectorByQuaternion(v, q) {
//     // Convert vector to quaternion form with w = 0
//     let vQuat = [0, v[0], v[1], v[2]];
    
//     // Calculate the conjugate of the quaternion
//     let qConjugate = quaternionConjugate(q);
  
//     // Perform the rotation: b = q * v * qConjugate
//     let resultQuat = quaternionMultiply(quaternionMultiply(q, vQuat), qConjugate);
  
//     // The resulting vector in local coordinates (ignoring the w component)
//     return [resultQuat[1], resultQuat[2], resultQuat[3]];
//   }
  
//   function transformToLocal(w, x, y, z, ax, ay, az) {
//     const a = [ax, ay, az];
//     const Q = [w, x, y, z];
//     const b = rotateVectorByQuaternion(a, Q);
//     return b;
//   }

// =====================Transform with Quaternion 1=========================

function quaternionToRotationMatrix(q) {
    const [q0, q1, q2, q3] = q;
    return [
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
    ];
}

function transposeMatrix(matrix) {
    return matrix[0].map((_, colIndex) => matrix.map(row => row[colIndex]));
}

function rotateVector(vector, rotationMatrix) {
    return vector.map((_, i) => 
        rotationMatrix[i].reduce((acc, val, j) => acc + val * vector[j], 0)
    );
}

function transformToLocal(w, x, y, z, ax, ay, az) {
    // 示例四元数 (w, x, y, z)
    const quaternion = [w, x, y, z];
    // 全球坐标系下的加速度
    const globalAccel = [ax, ay, az];

    // 计算旋转矩阵
    const rotationMatrix = transposeMatrix(quaternionToRotationMatrix(quaternion));

    // 将全球坐标转换为局部坐标
    const localAccel = rotateVector(globalAccel, rotationMatrix);
    return localAccel;
}

// =====================Transform with Euler ========================
// function eulerToRotationMatrix(roll, pitch, yaw) {
//     // 将角度转换为弧度
//     const rollRad = roll * Math.PI / 180;
//     const pitchRad = pitch * Math.PI / 180;
//     const yawRad = yaw * Math.PI / 180;

//     // 计算每个旋转矩阵
//     const Rx = [
//         [1, 0, 0],
//         [0, Math.cos(rollRad), -Math.sin(rollRad)],
//         [0, Math.sin(rollRad), Math.cos(rollRad)]
//     ];

//     const Ry = [
//         [Math.cos(pitchRad), 0, Math.sin(pitchRad)],
//         [0, 1, 0],
//         [-Math.sin(pitchRad), 0, Math.cos(pitchRad)]
//     ];

//     const Rz = [
//         [Math.cos(yawRad), -Math.sin(yawRad), 0],
//         [Math.sin(yawRad), Math.cos(yawRad), 0],
//         [0, 0, 1]
//     ];

//     // 计算 R = Rz * Ry * Rx
//     const Rzy = matrixMultiply(Rz, Ry);
//     const R = matrixMultiply(Rzy, Rx);

//     return R;
// }

// function matrixMultiply(A, B) {
//     // 矩阵乘法 A * B
//     const result = [];
//     for (let i = 0; i < A.length; i++) {
//         result[i] = [];
//         for (let j = 0; j < B[0].length; j++) {
//             let sum = 0;
//             for (let k = 0; k < A[0].length; k++) {
//                 sum += A[i][k] * B[k][j];
//             }
//             result[i][j] = sum;
//         }
//     }
//     return result;
// }

// function globalToLocalAcceleration(globalAccel, R) {
//     // 将全球坐标系下的加速度转换为局部坐标系
//     const localAccel = [];
//     for (let i = 0; i < R.length; i++) {
//         let sum = 0;
//         for (let j = 0; j < globalAccel.length; j++) {
//             sum += R[i][j] * globalAccel[j];
//         }
//         localAccel.push(sum);
//     }
//     return localAccel;
// }
// function transformToLocal(rx, ry, rz, ax, ay, az) {
// 	const R = eulerToRotationMatrix(rx, ry, rz);
//     const globalAccel = [ax, ay, az];
//     // 转换为局部坐标系下的加速度
//     const localAccel = globalToLocalAcceleration(globalAccel, R);
// 	// Transform vector v2 from B to A
// 	return localAccel;
//     console.log(localAccel);
// }

// function degreesToRadians(degrees) {
// 	return degrees * Math.PI / 180;
// }

// function createRotationMatrix(rx, ry, rz) {
// 	// Convert angles from degrees to radians
// 	rx = degreesToRadians(rx);
// 	ry = degreesToRadians(ry);
// 	rz = degreesToRadians(rz);

// 	// Rotation matrices for each axis
// 	let Rx = [
// 		[1, 0, 0],
// 		[0, Math.cos(rx), -Math.sin(rx)],
// 		[0, Math.sin(rx), Math.cos(rx)]
// 	];

// 	let Ry = [
// 		[Math.cos(ry), 0, Math.sin(ry)],
// 		[0, 1, 0],
// 		[-Math.sin(ry), 0, Math.cos(ry)]
// 	];

// 	let Rz = [
// 		[Math.cos(rz), -Math.sin(rz), 0],
// 		[Math.sin(rz), Math.cos(rz), 0],
// 		[0, 0, 1]
// 	];

// 	// Combined rotation matrix: R = Rz * Ry * Rx
// 	let Rzy = multiplyMatrices(Rz, Ry);
// 	let Rzyx = multiplyMatrices(Rzy, Rx);

// 	return Rzyx;
// }

// function multiplyMatrices(a, b) {
// 	let result = new Array(a.length).fill(0).map(row => new Array(b[0].length).fill(0));

// 	return result.map((row, i) => {
// 		return row.map((val, j) => {
// 			return a[i].reduce((sum, elm, k) => sum + (elm * b[k][j]), 0)
// 		})
// 	});
// }

// function transformVector(matrix, vector) {
// 	let result = matrix.map(function(row) {
// 		return row.reduce(function(sum, cell, i) {
// 			return sum + cell * vector[i];
// 		}, 0);
// 	});
// 	return { x: result[0], y: result[1], z: result[2] };
// }

// function transformToLocal(rx, ry, rz, ax, ay, az) {
// 	// Given vector in coordinate system B
// 	let v2 = [ax, ay, az];

// 	// Create rotation matrix from A to B
// 	let rotationMatrixAB = createRotationMatrix(rx, ry, rz);

// 	// Invert the rotation matrix to get from B to A
// 	let rotationMatrixBA = multiplyMatrices(
// 		multiplyMatrices(createRotationMatrix(-rx, 0, 0), createRotationMatrix(0, -ry, 0)),
// 		createRotationMatrix(0, 0, -rz)
// 	);
// 	// Transform vector v2 from B to A
// 	return transformVector(rotationMatrixBA, v2);
// }

var transitions =
[
    // -- Powering-on --

	{
		stateName: 'Powering-on',
		eventName: 'blePoweredOn',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            // NOP
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'startScanning',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            component.sensors = {};
            component.discoveredSensors = [];

            if (globalConnectedSensors != null && globalConnectedSensors != undefined)
            {
                globalConnectedSensors.forEach( function (sensor)
                {
                    if( component.sensors[sensor.address] == undefined )
                    {
                        component.sensors[sensor.address] = sensor;
                    }
                    component.discoveredSensors.push( sensor );
                });
            }

            component.ble.startScanning();
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'bleScanningStarted',
		nextState: 'Scanning',
		
		transFunc:function( component, parameters )
	    {
            component.gui.sendGuiEvent( 'scanningStarted' );	   
        }
    },
    {
		stateName: 'Idle',
		eventName: 'connectSensors',
		nextState: 'Connect next?',
		
		transFunc:function( component, parameters )
	    {
	    }
    },

    // -- Scanning --

    {
		stateName: 'Scanning',
		eventName: 'bleSensorDiscovered',
		nextState: 'New sensor?',
		
		transFunc:function( component, parameters )
	    {
            component.discoveredSensor = parameters.sensor;
	    }
    },
    {
		stateName: 'Scanning',
		eventName: 'stopScanning',
		nextState: 'Scanning',
		
		transFunc:function( component, parameters )
	    {
            component.ble.stopScanning();
	    }
    },
    {
		stateName: 'Scanning',
		eventName: 'bleScanningStopped',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            component.gui.sendGuiEvent( 'scanningStopped' );
	    }
    },

    // -- Discovering --

    {
		stateName: 'New sensor?',
		eventName: 'yes',
		nextState: 'Scanning',
		
		transFunc:function( component, parameters )
	    {
            if( component.sensors[component.discoveredSensor.address] == undefined )
            {
                component.sensors[component.discoveredSensor.address] = component.discoveredSensor;
            }
            component.discoveredSensors.push( component.discoveredSensor );
            component.gui.sendGuiEvent
            ( 
                'sensorDiscovered', 
                { 
                    name:    component.discoveredSensor.name,
                    address: component.discoveredSensor.address
                } 
            );
	    }
    },
    {
		stateName: 'New sensor?',
		eventName: 'no',
		nextState: 'Scanning',
		
		transFunc:function( component, parameters )
	    {
            // NOP
	    }
    },

    // -- Connecting --

    {
		stateName: 'Connect next?',
		eventName: 'yes',
		nextState: 'Connecting',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {                
                component.ble.connectSensor( sensor );
            }
	    }
    },
    {
		stateName: 'Connect next?',
		eventName: 'no',
		nextState: 'Sensor connected',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Connecting',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Connecting',
		eventName: 'stopConnectingSensors',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.disconnectSensor( sensor );
            }

            var connectedSensor = component.connectedSensors.indexOf(sensor);

            if (connectedSensor == -1)
            {
                component.gui.sendGuiEvent( 'sensorDisconnected', {address:address} );
            }
	    }
    },
    {
		stateName: 'Sensor connected',
		eventName: 'stopConnectingSensors',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.disconnectSensor( sensor );
            }
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'bleSensorConnected',
		nextState: 'Sensor disconnected?',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.disconnectSensor( sensor );
            }
	    }
    },
    {
		stateName: 'StopConnectingSensors',
		eventName: 'connectSensors',
		nextState: 'StopConnectingSensors',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'StopConnectingSensors',
		eventName: 'stopConnectingSensors',
		nextState: 'StopConnectingSensors',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Connecting',
		eventName: 'bleSensorConnected',
		nextState: 'Sensors connected?',
		
		transFunc:function( component, parameters )
	    {
            component.connectedSensors.push( parameters.sensor );

            var sensor = [parameters.sensor.address];
            component.gui.sendGuiEvent( 'sensorConnected', {address:parameters.sensor.address, addresses:sensor} );
	    }
    },
    {
		stateName: 'Connecting',
		eventName: 'bleSensorError',
		nextState: 'Connect next?',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'disconnectSensors',
		nextState: 'Sensor disconnected?',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Sensor connected',
		eventName: 'disconnectSensors',
		nextState: 'Sensor disconnected?',

		transFunc:function( component, parameters )
		{
		}
    },
    {
        stateName: 'Idle',
		eventName: 'startSyncing',
		nextState: 'Syncing',

		transFunc:function( component, parameters )
		{
            component.syncManager.startSyncing();
		}
    },
    {
		stateName: 'Sensor connected',
		eventName: 'bleSensorConnected',
		nextState: 'Sensor disconnected?',

		transFunc:function( component, parameters )
		{
		}
    },
    {
        stateName: 'Syncing',
		eventName: 'bleSensorConnected',
		nextState: 'Syncing',

		transFunc:function( component, parameters )
		{
		}
    },
    {
        stateName: 'Syncing',
		eventName: 'bleSensorDisconnected',
		nextState: 'Syncing',

		transFunc:function( component, parameters )
		{
		}
    },
    {
        stateName: 'Syncing',
		eventName: 'syncingDone',
		nextState: 'Idle',

		transFunc:function( component, parameters )
		{
		}
    },
    {
		stateName: 'Idle',
		eventName: 'startMeasuring',
		nextState: 'StartMeasuring',
		
		transFunc:function( component, parameters )
	    {
            var len = parameters.addresses;

            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            component.measuringPayloadId = parameters.measuringPayloadId;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.enableSensor( sensor, parameters.measuringPayloadId );
            }
	    }
    },
    {
		stateName: 'Sensor connected',
		eventName: 'startMeasuring',
		nextState: 'StartMeasuring',
		
		transFunc:function( component, parameters )
	    {
            var len = parameters.addresses;

            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            component.measuringPayloadId = parameters.measuringPayloadId;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.enableSensor( sensor, parameters.measuringPayloadId );
            }
	    }
    },
    {
		stateName: 'Sensor connected',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Sensor connected',
		eventName: 'connectSensors',
		nextState: 'Connect next?',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Sensor disconnected?',
		eventName: 'yes',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Sensor disconnected?',
		eventName: 'no',
		nextState: 'Disconnecting',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];

            if( sensor != undefined )
            {
                component.ble.disconnectSensor( sensor );
            }
	    }
    },
    {
		stateName: 'Disconnecting',
		eventName: 'bleSensorDisconnected',
		nextState: 'Sensor disconnected?',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Sensors connected?',
		eventName: 'yes',
		nextState: 'Sensor connected',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Sensors connected?',
		eventName: 'no',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
	    }
    },


    // -- Measuring --

    {
		stateName: 'Start next?',
		eventName: 'yes',
		nextState: 'Enabling',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Start next?',
		eventName: 'no',
		nextState: 'Measuring',
		
		transFunc:function( component, parameters )
	    {
        }
    },
    {
		stateName: 'StartMeasuring',
		eventName: 'bleSensorEnabled',
		nextState: 'Start next?',
		
		transFunc:function( component, parameters )
	    {
            component.measuringSensors.push( parameters.sensor );
            component.gui.sendGuiEvent( 'sensorEnabled', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'StartMeasuring',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );	    
        }
    },
    {
		stateName: 'Enabling',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {sensor:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Enabling',
		eventName: 'bleSensorData',
		nextState: 'Enabling',
		
		transFunc:function( component, parameters )
	    {
            // NOP
	    }
    },
    {
		stateName: 'Enabling',
		eventName: 'bleSensorError',
		nextState: 'Start next?',
		
		transFunc:function( component, parameters )
	    {
            component.ble.disconnectSensor( parameters.sensor );
	    }
    },
    {
		stateName: 'Measuring',
		eventName: 'stopMeasuring',
		nextState: 'StopMeasuring',
		
		transFunc:function( component, parameters )
	    {
            if( parameters == undefined ) return;
            
            var address = parameters.addresses[0];
            
            if( address == undefined ) return;
            
            var sensor = component.sensors[address];
            
            if( sensor != undefined )
            {
                component.ble.disableSensor( sensor, parameters.measuringPayloadId );
                component.measuringSensors.shift();
            }
	    }
    },
    {
		stateName: 'Measuring',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Measuring',
		eventName: 'bleSensorData',
		nextState: 'Measuring',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Idle',
		eventName: 'enableSync',
		nextState: 'Idle',

		transFunc:function( component, parameters )
	    {
            component.ble.enableSync( parameters.isSyncingEnabled );
	    }
    },
    {
		stateName: 'Recording',
		eventName: 'resetHeading',
		nextState: 'Recording',

		transFunc:function( component, parameters )
	    {
            parameters.measuringSensors.forEach( function (address)
            {
                var sensor = component.sensors[address];

                if( sensor != undefined )
                {
                    component.ble.resetHeading( sensor );
                }
            });
	    }
    },
    {
		stateName: 'Recording',
		eventName: 'revertHeading',
		nextState: 'Recording',

		transFunc:function( component, parameters )
	    {
            parameters.measuringSensors.forEach( function (address)
            {
                var sensor = component.sensors[address];

                if( sensor != undefined )
                {
                    component.ble.revertHeading( sensor );
                }
            });
	    }
    },
    {
		stateName: 'Measuring',
		eventName: 'startRecording',
		nextState: 'Measuring',
		
		transFunc:function( component, parameters )
	    {
            startRecordingToFile( component, parameters.filename );
	    }
    },
    {
		stateName: 'Measuring',
		eventName: 'fsOpen',
		nextState: 'Recording',
		
		transFunc:function( component, parameters )
	    {
            var now = new Date();

            // component.fileStream.write( "sep=,\n" );

            switch (component.measuringPayloadId)
            {
                case MEASURING_PAYLOAD_TYPE_COMPLETE_EULER:
                    component.fileStream.write( "Measurement Mode:,Complete (Euler)\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_EXTENDED_QUATERNION:
                    // component.fileStream.write( "Measurement Mode:,Extended (Quaternion)\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_RATE_QUANTITIES_WITH_MAG:
                    component.fileStream.write( "Measurement Mode:,Rate quantities (with mag)\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_1:
                    component.fileStream.write( "Measurement Mode:,Custom Mode 1\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_2:
                    component.fileStream.write( "Measurement Mode:,Custom Mode 2\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_3:
                    component.fileStream.write( "Measurement Mode:,Custom Mode 3\n" );
                    break;
            }

            // component.fileStream.write( "StartTime:," + now.toUTCString() + "\n" );
            // component.fileStream.write( "© Xsens Technologies B. V. 2005-" + now.getFullYear() + "\n\n" );

            switch (component.measuringPayloadId)
            {
                case MEASURING_PAYLOAD_TYPE_COMPLETE_EULER:
                    component.fileStream.write( "Timestamp,Address,Tag,Euler_x,Euler_y,Euler_z,FreeAcc_x,FreeAcc_y,FreeAcc_z\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_EXTENDED_QUATERNION:
                    component.fileStream.write( "tag,timestamp,ex,ey,ez,qw,qx,qy,qz,ax,ay,az,lx,ly,lz\n" );
                    break;
                    // var newData = {
                    //     "tag": parameters.tag,
                    //     "timestamp": parameters.timestamp,
                    //     "ex": parameters.euler_x,
                    //     "ey": parameters.euler_y,
                    //     "ez": parameters.euler_z,
                    //     "qw": parameters.quaternion_w,
                    //     "qx": parameters.quaternion_x,
                    //     "qy": parameters.quaternion_y,
                    //     "qz": parameters.quaternion_z,
                    //     "ax": parameters.freeAcc_x,
                    //     "ay": parameters.freeAcc_y,
                    //     "az": parameters.freeAcc_z,
                    //     "lx": parameters.localAcc_x,
                    //     "ly": parameters.localAcc_y,
                    //     "lz": parameters.localAcc_z
                    // }
                case MEASURING_PAYLOAD_TYPE_RATE_QUANTITIES_WITH_MAG:
                    component.fileStream.write( "Timestamp,Address,Tag,Acc_x,Acc_y,Acc_z,Gyr_x,Gyr_y,Gyr_z,Mag_x,Mag_y,Mag_z\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_1:
                    component.fileStream.write( "Timestamp,Address,Tag,Euler_X,Euler_Y,Euler_Z,FreeAcc_x,FreeAcc_y,FreeAcc_z,Gyr_X,Gyr_Y,Gyr_Z\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_2:
                    component.fileStream.write( "Timestamp,Address,Tag,Euler_X,Euler_Y,Euler_Z,FreeAcc_x,FreeAcc_y,FreeAcc_z,Mag_x,Mag_y,Mag_z\n" );
                    break;

                case MEASURING_PAYLOAD_TYPE_CUSTOM_MODE_3:
                    component.fileStream.write( "Timestamp,Address,Tag,Quaternion_w,Quaternion_x,Quaternion_y,Quaternion_z,Gyr_X,Gyr_Y,Gyr_Z\n" );
                    break;
            }

	    }
    },
    {
		stateName: 'Stop next?',
		eventName: 'yes',
		nextState: 'Disabling',
		
		transFunc:function( component, parameters )
	    {
	    }
    },
    {
		stateName: 'Stop next?',
		eventName: 'no',
		nextState: 'Sensor connected',
		
		transFunc:function( component, parameters )
	    {
            component.gui.sendGuiEvent( 'allSensorsDisabled' );
	    }
    },
    {
		stateName: 'StopMeasuring',
		eventName: 'bleSensorDisabled',
		nextState: 'Stop next?',
		
		transFunc:function( component, parameters )
	    {
            component.gui.sendGuiEvent( 'sensorDisabled', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'StopMeasuring',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );	    }
    },
    {
		stateName: 'Disabling',
		eventName: 'bleSensorError',
		nextState: 'Stop next?',
		
		transFunc:function( component, parameters )
	    {
            console.log( "bleSensorError:" + parameters.error );
            component.ble.disconnectSensor( parameters.sensor );
	    }
    },
    {
		stateName: 'Disabling',
		eventName: 'bleSensorData',
		nextState: 'Disabling',
		
		transFunc:function( component, parameters )
	    {
            // NOP
	    }
    },
    {
		stateName: 'Disabling',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },

    // -- Recording --

    {
		stateName: 'Recording',
		eventName: 'bleSensorDisconnected',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            removeSensor( parameters.sensor, component.connectedSensors );
            removeSensor( parameters.sensor, component.measuringSensors );
            component.gui.sendGuiEvent( 'sensorDisconnected', {address:parameters.sensor.address} );
	    }
    },
    {
		stateName: 'Recording',
		eventName: 'bleSensorData',
		nextState: 'Store data?',
		
		transFunc:function( component, parameters )
	    {
            parameters.freeAcc_z -= 9.8;
            component.lastTimestamp = parameters.timestamp;
			// let localAcc = transformToLocal(
			// 	parameters.euler_x, parameters.euler_y, parameters.euler_z,
			// 	parameters.freeAcc_x, parameters.freeAcc_y, parameters.freeAcc_z);
            let localAcc = transformToLocal(
                parameters.quaternion_w, parameters.quaternion_x, parameters.quaternion_y, parameters.quaternion_z,
                parameters.freeAcc_x, parameters.freeAcc_y, parameters.freeAcc_z);
            // let localAcc = transformToLocal(
            //     parameters.mag_x, parameters.mag_y, parameters.mag_z,
            //     parameters.freeAcc_x, parameters.freeAcc_y, parameters.freeAcc_z);
            console.log(parameters);
            console.log(localAcc[0].toFixed(2), localAcc[1].toFixed(2), localAcc[2].toFixed(2));
			parameters.localAcc_x = localAcc[0];
			parameters.localAcc_y = localAcc[1];
			parameters.localAcc_z = localAcc[2];
            var newData = {
                "tag": parameters.tag,
                "timestamp": parameters.timestamp,
                "ex": parameters.euler_x,
                "ey": parameters.euler_y,
                "ez": parameters.euler_z,
                "qw": parameters.quaternion_w,
                "qx": parameters.quaternion_x,
                "qy": parameters.quaternion_y,
                "qz": parameters.quaternion_z,
                "ax": parameters.freeAcc_x,
                "ay": parameters.freeAcc_y,
                "az": parameters.freeAcc_z,
                "lx": parameters.localAcc_x,
                "ly": parameters.localAcc_y,
                "lz": parameters.localAcc_z
            }
            PythonSocket.onRealDataUpdated(newData);
            component.csvBuffer += Object.values(newData).join() + '\n';
			if (component.syncSensorDataManagement != null) {
				component.syncSensorDataManagement.uploadRealXsensData(
					{
						"tag": parameters.tag,
						"timestamp": parameters.timestamp,
						// "ex": parameters.euler_x,
						// "ey": parameters.euler_y,
						// "ez": parameters.euler_z,
                        "qw": parameters.quaternion_w,
                        "qx": parameters.quaternion_x,
                        "qy": parameters.quaternion_y,
                        "qz": parameters.quaternion_z,
						"ax": parameters.freeAcc_x,
						"ay": parameters.freeAcc_y,
						"az": parameters.freeAcc_z,
						"lx": parameters.localAcc_x,
                        "ly": parameters.localAcc_y,
                        "lz": parameters.localAcc_z
					});
			}
            component.gui.sendGuiEvent( 'sensorOrientation', parameters);
	    }
    },
    {
		stateName: 'Recording',
		eventName: 'stopRecording',
		nextState: 'Recording',
		
		transFunc:function( component, parameters )
	    {
            component.fileStream.write( component.csvBuffer );
            component.fileStream.end();
	    }
    },
    {
		stateName: 'Recording',
		eventName: 'fsClose',
		nextState: 'Idle',
		
		transFunc:function( component, parameters )
	    {
            component.gui.sendGuiEvent( 'recordingStopped' );
	    }
    },
    {
		stateName: 'Store data?',
		eventName: 'yes',
		nextState: 'Recording',
		
		transFunc:function( component, parameters )
	    {
            component.fileStream.write( component.csvBuffer );
            component.csvBuffer = "";
            component.lastWriteTime = component.lastTimestamp;
	    }
    },
    {
		stateName: 'Store data?',
		eventName: 'no',
		nextState: 'Recording',
		
		transFunc:function( component, parameters )
	    {
            // NOP
	    }
    }
];

// =======================================================================================
// Choice-points
// =======================================================================================

var choicePoints =
[
    {
        name:'Connect next?', 
        evalFunc: function( component, parameters )
        {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];
            var connectedSensor = component.connectedSensors.indexOf(sensor);

            if( sensor != undefined && connectedSensor == -1)
            {
                return true;
            }

            return false;
        }
    },
    {
        name:'Start next?', 
        evalFunc: function( component, parameters )
        {
            return false;
        }
    },
    {
        name:'Store data?', 
        evalFunc: function( component )
        {
            return ( component.lastTimestamp - component.lastWriteTime > RECORDING_BUFFER_TIME );
        }
    },
    {
        name:'Stop next?', 
        evalFunc: function( component )
        {
            return false;
        }
    },
    {
        name:'Sensor disconnected?', 
        evalFunc: function( component, parameters )
        {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];
            var connectedSensor = component.connectedSensors.indexOf(sensor);

            if( sensor != undefined && connectedSensor == -1 )
            {
                return true;
            }

            return false;
        }
    },
    {
        name:'New sensor?', 
        evalFunc: function( component )
        {
            return ( component.discoveredSensors.indexOf(component.discoveredSensor) == -1 );
        }
    },
    {
        name:'Sensors connected?', 
        evalFunc: function( component, parameters )
        {
            if( parameters == undefined ) return;

            var address = parameters.addresses[0];

            if( address == undefined ) return;

            var sensor = component.sensors[address];
            var connectedSensor = component.connectedSensors.indexOf(sensor);

            if( sensor != undefined && connectedSensor != -1 )
            {
                return true;
            }

            return false;
        }
    },
   
];

// =======================================================================================
// Class definition
// =======================================================================================
class SensorServer extends FunctionalComponent
{
    constructor(syncSensorDataManagement = null)
    {        
        super( "SensorServer", transitions, choicePoints );
        var component = this;
		this.syncSensorDataManagement = syncSensorDataManagement;
		this.syncSensorDataManagement.sensorServer = this;
        this.bleEvents = new events.EventEmitter();
        this.bleEvents.on( 'bleEvent', function(eventName, parameters )
        {
            component.eventHandler( eventName, parameters );
        });

        this.syncingEvents      = new events.EventEmitter();

        // Properties
        this.sensors            = {};
        this.discoveredSensors  = [];
        this.connectedSensors   = [];
        this.measuringSensors   = [];
        this.discoveredSensor   = null;
        this.fileStream         = null;
        this.csvBuffer          = "";
        this.recordingStartime  = 0;
        this.measuringPayloadId = 0;
        this.lastTimestamp      = 0;
        this.lastWriteTime      = 0;
        this.gui                = new WebGuiHandler(this);
        this.ble                = new BleHandler(this.bleEvents, this.syncingEvents, this.gui);
        this.syncManager        = new SyncManager(this.ble, this.gui, this.syncingEvents);
    }
}

// =======================================================================================
// Local functions
// =======================================================================================

// ---------------------------------------------------------------------------------------
// -- Remove sensor --
// ---------------------------------------------------------------------------------------
function removeSensor( sensor, sensorList )
{
    var idx = sensorList.indexOf( sensor );
    if( idx != -1 )
    {
        sensorList.splice( idx, 1 );
    }
}

// ---------------------------------------------------------------------------------------
// -- Start recording to file --
// ---------------------------------------------------------------------------------------
function startRecordingToFile( component, name )
{
    var dataDir = process.cwd() + RECORDINGS_PATH;
    if (!fs.existsSync(dataDir))
    {
        fs.mkdirSync(dataDir);
    }

    var fullPath = dataDir + name + ".csv";

    if (fs.existsSync(fullPath))
    {
        console.log('The logging file exists!');
        return;
    }

    component.fileStream = fs.createWriteStream( fullPath );
    
    const hrTime = process.hrtime();
    component.recordingStartTime = hrTime[0] * 1000000 + hrTime[1] / 1000;
    component.lastWriteTime = component.recordingStartTime;

    component.csvBuffer = "";

    component.fileStream.on( 'open', function() 
    {
        component.eventHandler( 'fsOpen' );
    });

    component.fileStream.on( 'close', function() 
    {
        component.eventHandler( 'fsClose' );
    });
}


// =======================================================================================
// Export the Sensor Server class
// =======================================================================================
module.exports = SensorServer;