const app = Vue.createApp({
    data() {
        return {
            ros: null,
            bridgeUrl: "ws://localhost:9090",
            videoHost: "localhost:8080",
            connected: false,
            selectedDebug: "final",
            playerSymbol: "X",
            robotSymbol: "O",
            board: Array(9).fill(null),
            gridDetected: false,
        };
    },

    created() {
        this.discoverUrls();
        this.connect();
    },

    methods: {
        discoverUrls() {
            const host = window.location.hostname;
            const pathParts = window.location.pathname.split("/").filter(x => x.length > 0);
            const session = pathParts[0];

            this.bridgeUrl = `wss://${host}/${session}/rosbridge/`;
            this.videoHost = `${host}/${session}/cameras`;

            console.log("Auto-resolved ROSBridge:", this.bridgeUrl);
            console.log("Auto-resolved video host:", this.videoHost);
        },

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
                host: this.videoHost,
                topic: "/camera/D435/color/image_raw",
                ssl: false,
                width: 1280,
                height: 720,
            });
        },

        startDebugStreams() {
            console.log("Starting debug MJPEG streams...");

            new MJPEGCANVAS.Viewer({
                divID: "debugBoard",
                host: this.videoHost,
                topic: "/board_processor/board",
                ssl: false,
                width: 424,
                height: 240,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugEdges",
                host: this.videoHost,
                topic: "/board_processor/edges",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugCells",
                host: this.videoHost,
                topic: "/board_processor/cells",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "debugFinal",
                host: this.videoHost,
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
                if (result && result.success) {
                    this.gridDetected = true;
                    for (let i = 0; i < 9; i++) {
                        if (result.board[i] === 0) {
                            this.board[i] = "O";
                        } else if (result.board[i] === 1) {
                            this.board[i] = "X";
                        } else {
                            this.board[i] = null;
                        }
                    }
                }
            });
        },

        moveRequest(mode, symbol, cell_number) {
            if (this.board[cell_number - 1] !== null && mode === 0) return;

            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/play_move",
                serviceType: "move_manager/srv/PlayMove"
            });

            const request = new ROSLIB.ServiceRequest({});
            request.mode = mode;
            request.cell_number = cell_number;

            if (symbol === "O") {
                request.symbol = 0;
            } else {
                request.symbol = 1;
            }

            console.log("Calling /play_move with symbol:", symbol);

            service.callService(request, (result) => {
                console.log("Service response:", result);
                if (mode === 0 && result && result.success) {
                    this.board[cell_number - 1] = symbol;
                }
            });
        },

        playerMove(cell_number) {
            this.moveRequest(0, this.playerSymbol, cell_number);
        },

        robotMove() {
            this.moveRequest(1, this.robotSymbol, 0);
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

            this.board = Array(9).fill(null);
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
