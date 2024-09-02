const fs = require("fs");
const ansi = require('ascii-art-ansi');
const color = require('ascii-art-ansi/color');

class SyncSensorDataManagement {
    constructor() {
        this.isRecording = false;
        this.realXsensData = [];
        this.virtualSensorData = [];
    }

    recordingExpData(params) {
        console.log(params);
        this.scheduleExpDataCollection(params);
    }

    scheduleExpDataCollection(params) {
        if (params.leftTimes > 0) {
            console.log(`Waiting ${params.interval}ms for next recording: left times: ${params.leftTimes}`);
            setTimeout(() => {
                this.realXsensData = [];
                this.virtualSensorData = [];
                this.isRecording = true;
                console.log(`Start Recording ${params.duration}ms: left times: ${params.leftTimes}`);
                setTimeout(() => {
                    this.isRecording = false;
                    this.saveRecording(`exp_data/${params.expName}`, `rl-${params.leftTimes - 1}`, this.realXsensData);
                    this.saveRecording(`exp_data/${params.expName}`, `vs-${params.leftTimes - 1}`, this.virtualSensorData);
                    params.leftTimes--;
                    this.scheduleExpDataCollection(params);
                }, params.duration);
            }, params.interval);
        } else {
            console.log("Finished!!");
        }
    }

    uploadRealXsensData(data) {
        if (this.isRecording) {
            this.realXsensData.push(data);
        }
    }

    uploadVirtualSensorData(data) {
        if (this.isRecording) {
            this.virtualSensorData.push(data);
        }
    }

    startRecording() {
        this.isRecording = true;
    }

    stopRecording() {
        this.isRecording = false;
        let filename = getUniqueFilename();
        this.saveRecording('sync_data', `${filename}-realXsensData`, this.realXsensData);
        this.saveRecording('sync_data', `${filename}-virtualSensorData`, this.virtualSensorData);
        this.realXsensData = [];
        this.virtualSensorData = [];
    }

    saveRecording(path, filename, data) {
        let dataDir = process.cwd() + "/" + path + "/";
        if (!fs.existsSync(dataDir))
        {
            fs.mkdirSync(dataDir);
        }
        let fullPath = dataDir + filename + ".csv";
        let fileStream= fs.createWriteStream( fullPath );
        fileStream.write("tag,timestamp,ex,ey,ez,ax,ay,az,x,y,z\n");
        for (let datum of data) {
            fileStream.write(Object.values(datum).join() + '\n');
        }
        fileStream.end();
    }
}

function getUniqueFilename()
{
    var date    = new Date();
    var year    = (date.getFullYear()).toString();
    var month   = (date.getMonth()+1).toString();
    var day     = (date.getDate()).toString();
    var hours   = (date.getHours()).toString();
    var minutes = (date.getMinutes()).toString();
    var seconds = (date.getSeconds()).toString();

    return 	year + "-" +
        month.padStart(2,"0") + "-" +
        day.padStart(2,"0") + "-" +
        hours.padStart(2,"0") + "-" +
        minutes.padStart(2,"0") + "-" +
        seconds.padStart(2,"0");
}

module.exports = SyncSensorDataManagement;