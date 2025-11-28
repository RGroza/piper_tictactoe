const app = Vue.createApp({
    data() {
        return {
            ros: null,
            bridgeUrl: "ws://localhost:9090",
            videoHost: "localhost:8080",
            connected: false,
            selectedImage: "final",
            playerSymbol: 1,
            robotSymbol: 0,
            autoRobotMove: true,
            board: Array(9).fill(-1),
            boardDetected: false,
            winLine: null, // {from: idx1, to: idx2}
            isDraw: false,
            boardStateTopic: null,
            gameResultTopic: null,
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

                this.updateRobotSymbol();

                this.subscribeBoardState();
                this.subscribeGameResult();
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
            this.unsubscribeBoardState();
            this.unsubscribeGameResult();
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
                this.setGameStarted(true);
            });
        },

        endGame() {
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
                this.board = Array(9).fill(-1);
                this.boardDetected = false;
                this.winLine = null;
                this.isDraw = false;
                this.setGameStarted(false);
            });
        },

        moveRequest(mode, symbol, cell_number) {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: "/play_move",
                serviceType: "move_manager/srv/PlayMove"
            });

            const request = new ROSLIB.ServiceRequest({});
            request.mode = mode;
            request.cell_number = cell_number;
            request.symbol = symbol; // symbol is now integer 0 or 1

            console.log("Calling /play_move with symbol:", symbol);

            service.callService(request, (result) => {
                console.log("Service response:", result);
            });
        },

        playerMove(cell_number) {
            this.moveRequest(0, this.playerSymbol, cell_number);
        },

        robotMove() {
            this.moveRequest(1, this.robotSymbol, 0);
        },

        playerChanged() {
            this.robotSymbol = this.playerSymbol === 1 ? 0 : 1;
            this.updateRobotSymbol();
        },

        robotChanged() {
            this.playerSymbol = this.robotSymbol === 1 ? 0 : 1;
            this.updateRobotSymbol();
        },

        updateRobotSymbol() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: '/ttt_manager/set_parameters',
                serviceType: 'rcl_interfaces/srv/SetParameters'
            });
            const request = new ROSLIB.ServiceRequest({
                parameters: [
                    {
                        name: 'robot_symbol',
                        value: { type: 2, integer_value: this.robotSymbol }
                    }
                ]
            });

            service.callService(request, (result) => {
                console.log('Set parameter result:', result);
            });
        },

        updateAutoRobotMove() {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: '/ttt_manager/set_parameters',
                serviceType: 'rcl_interfaces/srv/SetParameters'
            });

            const request = new ROSLIB.ServiceRequest({
                parameters: [
                    {
                        name: 'auto_robot_move',
                        value: { type: 1, bool_value: this.autoRobotMove }
                    }
                ]
            });

            service.callService(request, (result) => {
                console.log('Set parameter result:', result);
            });
        },

        setGameStarted(gameStarted) {
            const service = new ROSLIB.Service({
                ros: this.ros,
                name: '/ttt_manager/set_parameters',
                serviceType: 'rcl_interfaces/srv/SetParameters'
            });

            const request = new ROSLIB.ServiceRequest({
                parameters: [
                    {
                        name: 'game_started',
                        value: { type: 1, bool_value: gameStarted }
                    }
                ]
            });

            service.callService(request, (result) => {
                console.log('Set parameter result:', result);
            });
        },

        subscribeBoardState() {
            if (this.boardStateTopic) return;
            this.boardStateTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: "/board_state",
                messageType: "board_perception/msg/BoardState"
            });

            this.boardStateTopic.subscribe((msg) => {
                if (msg.board_detected) {
                    this.boardDetected = true;
                    this.board = msg.board.slice();
                }
            });
        },

        unsubscribeBoardState() {
            if (this.boardStateTopic) {
                this.boardStateTopic.unsubscribe();
                this.boardStateTopic = null;
            }
        },

        subscribeGameResult() {
            if (this.gameResultTopic) return;
            this.gameResultTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: "/game_result",
                messageType: "move_manager/msg/GameResult"
            });

            this.gameResultTopic.subscribe((msg) => {
                if (msg.game_over) {
                    if (msg.winner !== -1) {
                        this.winLine = { from: msg.winner_start_cell, to: msg.winner_end_cell };
                    } else {
                        this.isDraw = true;
                    }
                }
            });
        },

        unsubscribeGameResult() {
            if (this.gameResultTopic) {
                this.gameResultTopic.unsubscribe();
                this.gameResultTopic = null;
            }
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
