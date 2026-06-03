#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS3 lsm6ds3;
Adafruit_LIS3MDL lis3mdl;

WebServer server(80);

// --- MAPPING DES BROCHES MOTEURS ---
const int PIN_EN_D = 23;  const int PIN_IN1_D = 19; const int PIN_IN2_D = 18; 
const int PIN_EN_G = 4;   const int PIN_IN1_G = 17; const int PIN_IN2_G = 16; 

// --- MAPPING DES BROCHES ENCODEURS ---
const int ENC_A_G = 32; // Canal A Gauche
const int ENC_A_D = 27; // Canal A Droit
// On ne lit que le canal A pour la vitesse simple (le B sert pour le sens)

// --- VARIABLES D'ODOMÉTRIE (Performances max avec "volatile") ---
volatile long ticsGauches = 0;
volatile long ticsDroits = 0;

unsigned long tempsPrecedent = 0;
float vitesseGauche = 0.0;
float vitesseDroite = 0.0;

float ax = 0.0, ay = 0.0, az = 9.81; // Accélération
float gx = 0.0, gy = 0.0, gz = 0.0;  // Gyroscope
float mx = 0.0, my = 0.0, mz = 0.0;  // Magnétomètre

// --- VARIABLES ODOMÉTRIE CUMULÉE ---
float posX = 0.0, posY = 0.0; // Position du stylo en cm
float thetaEnc = 0.0;         // Orientation par encodeurs en radians
const float entraxe = 14.5;   // L en cm (à ajuster selon ta mesure réelle)
const float distParTic = 0.04; // cm par tic (28.27cm / 700 tics environ)

// On ajoute des compteurs globaux dans les interruptions
volatile long totalTicsG = 0;
volatile long totalTicsD = 0;

void IRAM_ATTR isrEncodeurGauche() { ticsGauches++; totalTicsG++; }
void IRAM_ATTR isrEncodeurDroit() { ticsDroits++; totalTicsD++; }

float orientationZ = 0.0; // Angle du robot en degrés
unsigned long dernierTempsIMU = 0;

const int VITESSE = 160; 
volatile int etatCourant = 0; 
int etatPrecedent = 0;


// --- ROUTINES MATÉRIELLES ---
void piloter(int pwmIn1G, int pwmIn2G, int pwmIn1D, int pwmIn2D) {
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, pwmIn1G); analogWrite(PIN_IN2_G, pwmIn2G);
  analogWrite(PIN_IN1_D, pwmIn1D); analogWrite(PIN_IN2_D, pwmIn2D);
}

void stopper() { piloter(0, 0, 0, 0); }

// ==============================================================================
// LE SITE WEB EMBARQUÉ (Avec Télémétrie Asynchrone)
// ==============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>HogRider Control</title>
    <style>
        body { font-family: sans-serif; text-align: center; background: #f0f2f5; margin: 0; padding: 20px; user-select: none; -webkit-user-select: none; }
        h1 { color: #1a73e8; margin-bottom: 5px; }
        
        /* Affichage de la Télémétrie */
        .telemetry { display: flex; justify-content: space-around; background: #fff; padding: 15px; border-radius: 10px; margin: 10px auto; max-width: 500px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .data-box { font-size: 14px; color: #555; }
        .data-val { font-size: 20px; font-weight: bold; color: #d84315; display: block; margin-top: 5px; }

        #terminal { width: 95%; max-width: 500px; height: 160px; background: #202124; color: #fff; text-align: left; padding: 15px; overflow-y: auto; margin: 15px auto; border-radius: 8px; font-family: monospace; font-size: 14px;}
        .tx { color: #8ab4f8; } .rx { color: #34a853; } .sys { color: #f29900; font-weight: bold; }
        .grid { display: grid; grid-template-columns: 85px 85px 85px; gap: 15px; justify-content: center; margin-top: 15px; }
        button { width: 85px; height: 85px; font-size: 32px; background: #5f6368; color: white; border: none; border-radius: 12px; touch-action: manipulation;}
        button:active { background: #3c4043; transform: scale(0.92); }
        .btn-stop { background: #d93025; font-size: 18px; font-weight: bold; }
        .btn-stop:active { background: #a50e0e; }
        /* Styles pour les capteurs IMU */
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px; }
        .sensor-card { background: #ffffff; border: 1px solid #ddd; border-radius: 10px; padding: 15px; box-shadow: 0 2px 5px rgba(0,0,0,0.05); }
        .sensor-card h4 { margin: 0 0 10px 0; color: #1a73e8; border-bottom: 2px solid #f0f2f5; font-size: 16px;}
        .axis { font-family: 'Consolas', monospace; font-size: 14px; margin: 5px 0; display: flex; justify-content: space-between; }
        .axis span { font-weight: bold; color: #d93025; }

        .needle-enc { 
        width: 2px; 
        height: 40px; 
        background: #1a73e8; /* Bleu pour les encodeurs */
        position: absolute; 
        left: 49%; 
        bottom: 50%; 
        transform-origin: bottom center; 
        opacity: 0.7; /* Légère transparence pour voir les deux */
        transition: transform 0.2s;
        }
    </style>
</head>
<body>
    <h1>🐗 HogRider</h1>

    <!-- 1. GRILLE DES CAPTEURS (LSM6DS3 & LIS3MDL) -->
    <div class="sensor-grid">
        <div class="sensor-card">
            <h4>Accéléromètre (m/s²)</h4>
            <div class="axis">X: <span id="ax">0.00</span></div>
            <div class="axis">Y: <span id="ay">0.00</span></div>
            <div class="axis">Z: <span id="az">0.00</span></div>
        </div>
        <div class="sensor-card">
            <h4>Gyroscope (°/s)</h4>
            <div class="axis">X: <span id="gx">0.00</span></div>
            <div class="axis">Y: <span id="gy">0.00</span></div>
            <div class="axis">Z: <span id="gz">0.00</span></div>
        </div>
        <div class="sensor-card">
            <h4>Magnétomètre (µT)</h4>
            <div class="axis">X: <span id="mx">0.00</span></div>
            <div class="axis">Y: <span id="my">0.00</span></div>
            <div class="axis">Z: <span id="mz">0.00</span></div>
        </div>
    </div>

    <!-- 2. VITESSES DES ROUES -->
    <div class="divider"><span>VITESSES DES ROUES</span></div>
    <div class="wheel-grid">
        <div class="wheel-card">
            <h4>Roue Gauche</h4>
            <div class="data-val" id="vitG_val">0.0 cm/s</div>
            <div class="speed-bar-container"><div id="barG" class="speed-bar"></div></div>
        </div>
        <div class="wheel-card">
            <h4>Roue Droite</h4>
            <div class="data-val" id="vitD_val">0.0 cm/s</div>
            <div class="speed-bar-container"><div id="barD" class="speed-bar"></div></div>
        </div>
    </div>

    <!-- 3. POSITION ET ORIENTATION (CÔTE À CÔTE) -->
    <div class="divider"><span>POSITION ET ORIENTATION (ODOMÉTRIE vs IMU)</span></div>
    <div class="pose-container" style="display: flex; justify-content: space-around; align-items: flex-start; gap: 15px; flex-wrap: wrap;">
        
        <!-- Bloc Position du Stylo -->
        <div class="wheel-card" style="flex: 1; min-width: 200px;">
            <h4>Position du Stylo (Enc)</h4>
            <div style="margin: 15px 0;">
                X : <span class="data-val" id="valX" style="font-size: 24px;">0.0 cm</span>
            </div>
            <div style="margin: 15px 0;">
                Y : <span class="data-val" id="valY" style="font-size: 24px;">0.0 cm</span>
            </div>
            <button onclick="fetch('/action?v=9')" style="width:100%; height:40px; font-size:14px; background:#f29900; color:white; border-radius:8px; border:none; cursor:pointer;">Réinitialiser Position</button>
        </div>

        <!-- Bloc Boussole Unique -->
        <div class="wheel-card" style="flex: 1; min-width: 200px;">
            <h4>Cap (Rouge: IMU | Bleu: Enc)</h4>
            <div class="compass">
                <div id="needle" class="needle"></div>
                <div id="needleEnc" class="needle-enc"></div>
            </div>
            <div style="font-size: 14px; color: #555; margin-top: 10px;">
                IMU : <span id="valYaw" style="color:#d93025; font-weight:bold;">0.0°</span> | 
                Enc : <span id="valYawEnc" style="color:#1a73e8; font-weight:bold;">0.0°</span>
            </div>
        </div>
    </div>

    <!-- 4. TERMINAL ET CONTRÔLES[cite: 2] -->
    <div id="terminal" style="margin-top: 25px;"></div>

    <div class="grid">
        <div></div>
        <button onmousedown="t(1, event)" onmouseup="t(0, event)" ontouchstart="t(1, event)" ontouchend="t(0, event)">▲</button>
        <div></div>
        <button onmousedown="t(3, event)" onmouseup="t(0, event)" ontouchstart="t(3, event)" ontouchend="t(0, event)">◄</button>
        <button class="btn-stop" onclick="t(0, event)">STOP</button>
        <button onmousedown="t(4, event)" onmouseup="t(0, event)" ontouchstart="t(4, event)" ontouchend="t(0, event)">►</button>
        <div></div>
        <button onmousedown="t(2, event)" onmouseup="t(0, event)" ontouchstart="t(2, event)" ontouchend="t(0, event)">▼</button>
        <div></div>
    </div>
</body>

<style>
    .pose-container { display: flex; justify-content: center; margin-top: 10px; }
    .compass { 
        width: 100px; height: 100px; border: 3px solid #1a73e8; 
        border-radius: 50%; margin: 10px auto; position: relative; 
        background: white;
    }
    .needle { 
        width: 4px; height: 50px; background: #d93025; 
        position: absolute; left: 48%; bottom: 50%; 
        transform-origin: bottom center; transition: transform 0.2s;
    }
</style>

<style>
    .wheel-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-top: 10px; }
    .wheel-card { background: white; padding: 15px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
    .speed-bar-container { background: #eee; height: 10px; border-radius: 5px; margin-top: 10px; overflow: hidden; }
    .speed-bar { background: #d84315; height: 100%; width: 0%; transition: width 0.3s; }
</style>

    <!-- TERMINAL -->
    <div id="terminal" style="margin-top: 25px;"></div>

    <!-- BOUTONS DE PILOTAGE -->
    <div class="grid">
        <div></div>
        <button onmousedown="t(1, event)" onmouseup="t(0, event)" ontouchstart="t(1, event)" ontouchend="t(0, event)">▲</button>
        <div></div>
        
        <button onmousedown="t(3, event)" onmouseup="t(0, event)" ontouchstart="t(3, event)" ontouchend="t(0, event)">◄</button>
        <button class="btn-stop" onclick="t(0, event)">STOP</button>
        <button onmousedown="t(4, event)" onmouseup="t(0, event)" ontouchstart="t(4, event)" ontouchend="t(0, event)">►</button>
        
        <div></div>
        <button onmousedown="t(2, event)" onmouseup="t(0, event)" ontouchstart="t(2, event)" ontouchend="t(0, event)">▼</button>
        <div></div>
    </div>

    <!-- SCRIPT JAVASCRIPT -->
    <script>
        let currentCmd = 0; let isTouch = false;
        const cmds = ["STOP", "AVANT", "ARRIERE", "GAUCHE", "DROITE"];

        function log(msg, type) {
            const term = document.getElementById('terminal');
            term.innerHTML += `<div class="${type}">[${new Date().toLocaleTimeString()}] ${msg}</div>`;
            term.scrollTop = term.scrollHeight; 
        }

        window.onload = () => {
            fetch('/ping').then(() => log("Réseau OK", "sys"));
            setInterval(fetchTelemetry, 500); 
        };

        function fetchTelemetry() {
            fetch('/telemetry')
            .then(res => res.json())
            .then(data => {
                let factor = 0.04;
                let speedG = (data.vg * factor).toFixed(1);
                let speedD = (data.vd * factor).toFixed(1);

                document.getElementById('vitG_val').innerText = speedG + " cm/s";
                document.getElementById('vitD_val').innerText = speedD + " cm/s";
                document.getElementById('barG').style.width = Math.min(100, (speedG / 50) * 100) + "%";
                document.getElementById('barD').style.width = Math.min(100, (speedD / 50) * 100) + "%";

                if(data.yaw !== undefined && data.yawEnc !== undefined) {
                    document.getElementById('valYaw').innerText = data.yaw.toFixed(1) + "°";
                    document.getElementById('valYawEnc').innerText = data.yawEnc.toFixed(1) + "°";
                    document.getElementById('valX').innerText = data.x.toFixed(1) + " cm";
                    document.getElementById('valY').innerText = data.y.toFixed(1) + " cm";
                    
                    document.getElementById('needle').style.transform = `rotate(${-data.yaw}deg)`;
                    document.getElementById('needleEnc').style.transform = `rotate(${-data.yawEnc}deg)`;
                }

                document.getElementById('ax').innerText = data.ax.toFixed(2);
                document.getElementById('ay').innerText = data.ay.toFixed(2);
                document.getElementById('az').innerText = data.az.toFixed(2);

                document.getElementById('gx').innerText = data.gx.toFixed(2);
                document.getElementById('gy').innerText = data.gy.toFixed(2);
                document.getElementById('gz').innerText = data.gz.toFixed(2);

                document.getElementById('mx').innerText = data.mx.toFixed(2);
                document.getElementById('my').innerText = data.my.toFixed(2);
                document.getElementById('mz').innerText = data.mz.toFixed(2);
            }).catch(err => console.log("Erreur de télémétrie :", err));
        }

        function t(cmdCode, e) {
            if (e && e.type.includes('touch')) isTouch = true;
            if (e && e.type.includes('mouse') && isTouch) return; 
            if (cmdCode === currentCmd && cmdCode !== 0) return; 
            currentCmd = cmdCode;
            log("PC -> " + cmds[cmdCode], "tx");
            fetch('/action?v=' + cmdCode).then(res => res.text()).then(text => log("HogRider -> " + text, "rx"));
        }
    </script>
</body>
</html>
)rawliteral";
// ==============================================================================

// --- ROUTES WEB ---
void handleRoot() { server.send(200, "text/html", index_html); }
void handlePing() { server.send(200, "text/plain", "PONG"); }
void handleAction() {
  if (server.hasArg("v")) {
    int cmd = server.arg("v").toInt();
    
    if (cmd == 9) { // Commande spéciale de Reset
      posX = 0.0; posY = 0.0; thetaEnc = 0.0; orientationZ = 0.0;
      totalTicsG = 0; totalTicsD = 0;
      server.send(200, "text/plain", "Odométrie réinitialisée");
    } else {
      etatCourant = cmd; // Commandes moteurs classiques
      server.send(200, "text/plain", "Ordre recu");
    }
  }
}

void handleTelemetry() {
  String json = "{";
  // On envoie chaque variable UNE SEULE FOIS, séparée par une virgule.
  json += "\"vg\":" + String(vitesseGauche, 1) + ",";
  json += "\"vd\":" + String(vitesseDroite, 1) + ",";
  json += "\"x\":" + String(posX, 1) + ",";
  json += "\"y\":" + String(posY, 1) + ",";
  json += "\"yawEnc\":" + String(thetaEnc * 180.0 / PI, 1) + ",";
  json += "\"ax\":" + String(ax, 2) + ",";
  json += "\"ay\":" + String(ay, 2) + ",";
  json += "\"az\":" + String(az, 2) + ",";
  json += "\"gx\":" + String(gx, 2) + ",";
  json += "\"gy\":" + String(gy, 2) + ",";
  json += "\"gz\":" + String(gz, 2) + ",";
  json += "\"mx\":" + String(mx, 2) + ",";
  json += "\"my\":" + String(my, 2) + ",";
  json += "\"mz\":" + String(mz, 2) + ",";
  json += "\"yaw\":" + String(orientationZ, 1); // Pas de virgule à la toute dernière ligne
  json += "}";
  
  server.send(200, "application/json", json);
}
// --- INITIALISATION ---
void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_EN_D, OUTPUT); pinMode(PIN_IN1_D, OUTPUT); pinMode(PIN_IN2_D, OUTPUT);
  pinMode(PIN_EN_G, OUTPUT); pinMode(PIN_IN1_G, OUTPUT); pinMode(PIN_IN2_G, OUTPUT);
  
  pinMode(ENC_A_G, INPUT_PULLUP);
  pinMode(ENC_A_D, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A_G), isrEncodeurGauche, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_D), isrEncodeurDroit, RISING);

  stopper();
  
  WiFi.softAP("HogRider", "micaca67"); 
  server.on("/", handleRoot);        
  server.on("/ping", handlePing);    
  server.on("/action", handleAction); 
  server.on("/telemetry", handleTelemetry); 
  server.begin();

  Serial.println("Initialisation capteurs...");
  
  if (!lsm6ds3.begin_I2C(0x6B) || !lis3mdl.begin_I2C(0x1E)) {
    Serial.println("Erreur : Capteur introuvable !");
    while (1) delay(10);
  }
  
  // Configuration de base
  lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
}

// --- BOUCLE PRINCIPALE ---
void loop() {
  server.handleClient();

  // 1. DÉCLARATION 
  sensors_event_t accel, gyro, temp, mag;

  // 2. PREMIÈRE LECTURE POUR LE CAP
  lsm6ds3.getEvent(&accel, &gyro, &temp);

  // Calcul du temps écoulé (dt) pour l'intégration
  unsigned long maintenant = millis();
  float dt = (maintenant - dernierTempsIMU) / 1000.0;
  dernierTempsIMU = maintenant;

  // Mise à jour de l'orientation
  if (abs(gyro.gyro.z) > 0.05) { 
    orientationZ += gyro.gyro.z * dt * (180.0 / PI); 
  }

  // 3. DEUXIÈME LECTURE POUR LE MAGNÉTOMÈTRE
  lis3mdl.getEvent(&mag);

  // 4. MISE À JOUR DES VARIABLES GLOBALES POUR LE JSON
  ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
  gx = gyro.gyro.x;          gy = gyro.gyro.y;          gz = gyro.gyro.z;
  mx = mag.magnetic.x;       my = mag.magnetic.y;       mz = mag.magnetic.z;
  
  // 1. CALCUL DE LA VITESSE (Toutes les 100 millisecondes)
  unsigned long tempsActuel = millis();
  if (tempsActuel - tempsPrecedent >= 100) {
    vitesseGauche = (float)ticsGauches / 0.1;
    vitesseDroite = (float)ticsDroits / 0.1;
    
    ticsGauches = 0;
    ticsDroits = 0;
    
    tempsPrecedent = tempsActuel;
  }

  // 2. EXÉCUTION MOTEURS (Corrigée selon le comportement physique réel)
  if (etatCourant != etatPrecedent) {
    switch (etatCourant) {
      case 1: piloter(VITESSE, 0, 0, VITESSE); break; // AVANT (Haut)
      case 2: piloter(0, VITESSE, VITESSE, 0); break; // ARRIERE (Bas)
      case 3: piloter(0, VITESSE, 0, VITESSE); break; // GAUCHE (Gauche)
      case 4: piloter(VITESSE, 0, VITESSE, 0); break; // DROITE (Droite)
      case 0: stopper(); break;                       // STOP
    }
    etatPrecedent = etatCourant; 
  }

  static long dernierTicsG = 0, dernierTicsD = 0;

long deltaG = totalTicsG - dernierTicsG;
long deltaD = totalTicsD - dernierTicsD;

if (deltaG != 0 || deltaD != 0) {
    float dG = deltaG * distParTic;
    float dD = deltaD * distParTic;
    float dCentre = (dG + dD) / 2.0;
    float dTheta = (dD - dG) / entraxe;

    posX += dCentre * cos(thetaEnc + dTheta / 2.0);
    posY += dCentre * sin(thetaEnc + dTheta / 2.0);
    thetaEnc += dTheta;

    dernierTicsG = totalTicsG;
    dernierTicsD = totalTicsD;
}
}