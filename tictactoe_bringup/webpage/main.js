const app = Vue.createApp({
    data() {
        return {
            ros: null,
            bridgeUrl: "ws://localhost:9090",
            videoHost: "localhost:8080",
            connected: false,
            selectedImage: "final",
            playerSymbol: "X",
            robotSymbol: "O",
            board: Array(9).fill(null),
            gridDetected: false,
            winLine: null, // {from: idx1, to: idx2}
            gameStarted: false,
        };
    },

    created() {
        this.discoverUrls();
        this.connect();
    },

    methods: {
        discoverUrls() {
            const host = window.location.hostname;
            console.log("Discovered host:", host);

            if (host !== "0.0.0.0") {
                const pathParts = window.location.pathname.split("/").filter(x => x.length > 0);
                const session = pathParts[0];
                this.bridgeUrl = `wss://${host}/${session}/rosbridge/`;
                this.videoHost = `${host}/${session}/cameras`;
            }

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
                this.startImageStreams();
            });

            this.ros.on("error", (err) => {
                console.error("ROSBridge error:", err);
            });

            this.ros.on("close", () => {
                console.warn("ROSBridge connection closed");
                this.connected = false;
            });
        },

        disconnect() {
            if (this.ros) {
                this.ros.close();
                this.connected = false;
            }
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

        startImageStreams() {
            console.log("Starting debug MJPEG streams...");

            new MJPEGCANVAS.Viewer({
                divID: "imageFinal",
                host: this.videoHost,
                topic: "/board_processor/final",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "imageDetection",
                host: this.videoHost,
                topic: "/board_processor/detection",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "imageCells",
                host: this.videoHost,
                topic: "/board_processor/cells",
                ssl: false,
                width: 810,
                height: 1067,
            });

            new MJPEGCANVAS.Viewer({
                divID: "imageEdges",
                host: this.videoHost,
                topic: "/board_processor/edges",
                ssl: false,
                width: 810,
                height: 1067,
            });
        },

        startGame() {
            this.drawGrid();
            this.gameStarted = true;
        },

        endGame() {
            this.clearBoard();
            this.gameStarted = false;
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
                if (result && result.success) {
                    this.board[result.cell_number - 1] = symbol;

                    // Call processBoard to update detection images
                    this.processBoard();
                    this.processBoard();

                    // Check if game is over
                    if (result.game_over) {
                        let cell1 = result.winner[1];
                        let cell2 = result.winner[2];
                        this.winLine = { from: cell1, to: cell2 };
                    }
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
                this.board = Array(9).fill(null);
                this.gridDetected = false;
                this.winLine = null;
            });
        },

        playerChanged() {
            this.robotSymbol = this.playerSymbol === "X" ? "O" : "X";
        },

        robotChanged() {
            this.playerSymbol = this.robotSymbol === "X" ? "O" : "X";
        },

        getCellCenter(idx) {
            // idx: 0-8, left-to-right, top-to-bottom
            const col = idx % 3;
            const row = Math.floor(idx / 3);
            const cellW = 300 / 3;
            const cellH = 400 / 3;
            return {
                x: col * cellW + cellW / 2,
                y: row * cellH + cellH / 2
            };
        },
    }
});

app.mount("#app");
