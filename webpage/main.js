const app = Vue.createApp({
    data() {
        return {
            ros: null,
            bridgeUrl: "ws://localhost:9090",
            videoUrl: "http://localhost:8080",
            connected: false,
            selectedDebug: "final",
            playerSymbol: "X",
            robotSymbol: "O",
        };
    },

    methods: {
        connect() {
            console.log("Connecting to ROSBridge:", this.bridgeUrl);

            this.ros = new ROSLIB.Ros({
                url: this.bridgeUrl
            });

            this.ros.on("connection", () => {
                console.log("Connected to ROS!");
                this.connected = true;

                this.startCameraStream();
                this.startDebugStreams();
            });

            this.ros.on("error", (err) => {
                console.error("ROSBridge error:", err);
            });

            this.ros.on("close", () => {
                console.warn("ROSBridge connection closed");
                this.connected = false;
            });
        },

        startCameraStream() {
            new MJPEGCANVAS.Viewer({
                divID: "divCamera",
                host: this.videoUrl.replace("http://", "").replace("https://", ""),
                topic: "/image_raw",
                ssl: false,
                width: 1280,
                height: 720,
            });
        },

        startDebugStreams() {
            console.log("Starting debug MJPEG streams...");

            new MJPEGCANVAS.Viewer({
                divID: "debugBoard",
                host: "localhost:8080",
                topic: "/board_processor/board",
                ssl: false,
                width: 424,
                height: 240,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugEdges",
                host: "localhost:8080",
                topic: "/board_processor/edges",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugCells",
                host: "localhost:8080",
                topic: "/board_processor/cells",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugFinal",
                host: "localhost:8080",
                topic: "/board_processor/final",
                ssl: false,
                width: 810,
                height: 1067,
            });
        },

        processBoard() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/process_board",
                serviceType: "board_perception/srv/ProcessBoard"
            });

            const request = new ROSLIB.ServiceRequest({});
            console.log("Calling /process_board");

            service.callService(request, (result) => {
                console.log("Service response:", result);
            });
        },

        robotMove() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/play_move",
                serviceType: "move_manager/srv/PlayMove"
            });

            const request = new ROSLIB.ServiceRequest({});
            request.mode = 1;
            request.symbol = this.robotSymbol;

            console.log("Calling /play_move with symbol:", this.robotSymbol);

            service.callService(request, (result) => {
                console.log("Service response:", result);
            });
        },

        drawGrid() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/execute_trajectory",
                serviceType: "execute_trajectory/srv/ExecuteTrajectory"
            });

            const request = new ROSLIB.ServiceRequest({});
            request.type = 2;
            request.return_home = true;

            console.log("Calling /execute_trajectory to draw grid");

            service.callService(request, (result) => {
                console.log("Service response:", result);
            });
        },

        clearBoard() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/execute_trajectory",
                serviceType: "execute_trajectory/srv/ExecuteTrajectory"
            });

            const request = new ROSLIB.ServiceRequest({});
            request.type = 3;
            request.return_home = true;

            console.log("Calling /execute_trajectory to clear board");

            service.callService(request, (result) => {
                console.log("Service response:", result);
            });
        },

        playerChanged() {
            this.robotSymbol = this.playerSymbol === "X" ? "O" : "X";
        },

        robotChanged() {
            this.playerSymbol = this.robotSymbol === "X" ? "O" : "X";
        }
    }
});

app.mount("#app");
