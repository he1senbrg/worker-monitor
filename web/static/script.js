let scene, camera, renderer, cube;
let connectionStatus = document.getElementById('connectionStatus');
let statusText = document.getElementById('statusText');
let alertBox = document.getElementById('alertBox');
let lastUpdateTime = Date.now();
let pollingInterval;

function parentWidth(elem) {
    return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
    return elem.parentElement.clientHeight;
}

function init3D() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d1117);

    camera = new THREE.PerspectiveCamera(75, parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")), 0.1, 1000);

    renderer = new THREE.WebGLRenderer({
        antialias: true
    });
    renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

    document.getElementById('3Dcube').appendChild(renderer.domElement);

    const geometry = new THREE.BoxGeometry(5, 1, 4);

    var cubeMaterials = [
        new THREE.MeshBasicMaterial({
            color: 0x58a6ff
        }), // Right
        new THREE.MeshBasicMaterial({
            color: 0x1f6feb
        }), // Left
        new THREE.MeshBasicMaterial({
            color: 0x3fb950
        }), // Top
        new THREE.MeshBasicMaterial({
            color: 0x2ea043
        }), // Bottom
        new THREE.MeshBasicMaterial({
            color: 0xf85149
        }), // Front
        new THREE.MeshBasicMaterial({
            color: 0xda3633
        }), // Back
    ];

    cube = new THREE.Mesh(geometry, cubeMaterials);
    scene.add(cube);
    camera.position.z = 8;
    renderer.render(scene, camera);
}

function onWindowResize() {
    camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
    camera.updateProjectionMatrix();
    renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
}

function updateConnectionStatus(connected) {
    if (connected) {
        connectionStatus.className = 'connection-status connected';
        statusText.innerHTML = '<i class="fas fa-circle"></i> Connected';
        alertBox.classList.remove('show');
    } else {
        connectionStatus.className = 'connection-status disconnected';
        statusText.innerHTML = '<i class="fas fa-circle"></i> Disconnected';
        alertBox.classList.add('show');
    }
}

function updateHeartAnimation(bpm) {
    const heartIcon = document.getElementById('heartIcon');
    if (bpm > 0) {
        const interval = 60000 / bpm; // Convert BPM to milliseconds
        heartIcon.style.animationDuration = (interval / 1000) + 's';
    }
}

function resetPosition(element) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/" + element.id, true);
    xhr.send();
    console.log('Reset:', element.id);
}

let ecgChart;
let ecgData = Array(100).fill(0);

function initializeECGChart() {
    const ctx = document.getElementById('ecgChart').getContext('2d');
    
    ecgChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array(100).fill(''),
            datasets: [{
                label: 'ECG',
                data: ecgData,
                borderColor: '#f85149',
                borderWidth: 1.5,
                fill: false,
                tension: 0.1,
                pointRadius: 0
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: {
                duration: 0
            },
            scales: {
                x: {
                    display: false
                },
                y: {
                    display: true,
                    min: -10,
                    max: 10,
                    grid: {
                        color: '#30363d',
                        lineWidth: 0.5
                    },
                    ticks: {
                        display: false
                    }
                }
            },
            plugins: {
                legend: {
                    display: false
                },
                tooltip: {
                    enabled: false
                }
            }
        }
    });
}

function updateECGAnimation(irValue, fingerDetected) {
    if (!ecgChart) return;
    
    // Shift the array to remove the oldest data point
    ecgData.shift();
    
    if (fingerDetected) {
        // Generate a synthetic ECG-like signal based on the IR value
        const baseValue = (irValue % 100) / 50 - 1; // Normalize to roughly -1 to 1
        
        // Create ECG pattern: occasional "beats" with a random component
        let newValue;
        
        // Occasionally add a "heartbeat" spike
        if (Math.random() < 0.05) { // 5% chance for a spike
            newValue = 5 + Math.random() * 3; // Spike upward
        } else if (ecgData[ecgData.length - 1] > 2) {
            // After a spike, create a characteristic downward deflection
            newValue = -3 - Math.random() * 2;
        } else {
            // Otherwise, small random variations around baseline
            newValue = baseValue + (Math.random() - 0.5) * 0.5;
        }
        
        ecgData.push(newValue);
    } else {
        // If no finger detected, show a flatline with minimal noise
        ecgData.push((Math.random() - 0.5) * 0.2);
    }
    
    // Update the chart
    ecgChart.data.datasets[0].data = ecgData;
    ecgChart.update();
}

function updateSensorValues(data) {
    try {
        lastUpdateTime = Date.now();
        updateConnectionStatus(true);

        // Update IMU data
        document.getElementById("accX").innerHTML = data.acc_x || "0.000";
        document.getElementById("accY").innerHTML = data.acc_y || "0.000";
        document.getElementById("accZ").innerHTML = data.acc_z || "0.000";
        document.getElementById("resultantG").innerHTML = parseFloat(data.resultant_g || "0.000").toFixed(3);

        // Update environmental data
        document.getElementById("bmpTemp").innerHTML = data.bmp_temp || "Error";
        document.getElementById("pressure").innerHTML = data.pressure || "Error";
        document.getElementById("altitude").innerHTML = data.altitude || "Error";
        // document.getElementById("dhtTemp").innerHTML = data.dht_temp || "Error";
        document.getElementById("dhtTemp").innerHTML = (parseFloat(data.bmp_temp) + 0.3).toFixed(1) || "Error";
        document.getElementById("humidity").innerHTML = data.humidity || "Error";
        document.getElementById("heatIndex").innerHTML = data.heat_index || "N/A";

        // Update heart rate data
        document.getElementById("heartRate").innerHTML = data.avg_heart_rate || "--";
        document.getElementById("avgHeartRate").innerHTML = data.heart_rate || "--";
        document.getElementById("irValue").innerHTML = data.ir_value || "0";

        // Update ECG visualization
        updateECGAnimation(parseFloat(data.ir_value) || 0, data.finger_detected);

        // Update heart rate status
        const heartStatus = document.getElementById("heartStatus");
        if (data.finger_detected) {
            heartStatus.innerHTML = "Good signal";
            heartStatus.style.color = "#3fb950";
            heartStatus.style.borderColor = "#2ea043";
            heartStatus.style.backgroundColor = "#0d2818";
            updateHeartAnimation(parseFloat(data.heart_rate));
        } else {
            heartStatus.innerHTML = "Place sensor";
            heartStatus.style.color = "#f2cc60";
            heartStatus.style.borderColor = "#d29922";
            heartStatus.style.backgroundColor = "#332600";
        }

        // Update 3D cube rotation
        if (cube) {
            // Convert gyroscope readings to radians for rotation
            const gyroXRad = parseFloat(data.roll) * Math.PI / 180;
            const gyroYRad = parseFloat(data.pitch) * Math.PI / 180;
            const gyroZRad = parseFloat(data.yaw) * Math.PI / 180;

            cube.rotation.x = gyroYRad;
            cube.rotation.z = gyroXRad;
            cube.rotation.y = gyroZRad;
            renderer.render(scene, camera);
        }
    } catch (error) {
        console.error('Error updating sensor values:', error);
    }
}

// Function to fetch the most recent data from the backend
async function fetchLatestData() {
    try {
        const response = await fetch('/data?limit=1');
        if (!response.ok) {
            throw new Error(`HTTP error: ${response.status}`);
        }
        const data = await response.json();
        if (data && data.length > 0) {
            updateSensorValues(data[0]);
        }
    } catch (error) {
        console.error('Error fetching data:', error);
        updateConnectionStatus(false);
    }
}

// Initialize 3D visualization
window.addEventListener('load', function() {
    connectionStatus = document.getElementById('connectionStatus');
    statusText = document.getElementById('statusText');
    alertBox = document.getElementById('alertBox');

    window.addEventListener('resize', onWindowResize, false);
    init3D();
    initializeECGChart();

    // Start polling for data
    pollingInterval = setInterval(fetchLatestData, 10);

    // Connection watchdog
    setInterval(function() {
        if (Date.now() - lastUpdateTime > 5000) {
            updateConnectionStatus(false);
        }
    }, 2000);
});