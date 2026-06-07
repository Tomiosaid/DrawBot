#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS3 lsm6ds3;
Adafruit_LIS3MDL lis3mdl;

WebServer server(80);

// --- CONSTANTES PHYSIQUES DU ROBOT ---
#define WHEEL_DIAMETER_MM 90.0
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * PI)
#define ENTRAXE_MM 82.0        // centre-à-centre : 77mm intérieur + 5mm demi-roue×2
#define TICKS_PAR_TOUR 857.0   // calibré : 1000 ticks = 330mm réels (était 700)
#define MM_PAR_TICK (WHEEL_CIRCUMFERENCE / TICKS_PAR_TOUR)
#define PEN_OFFSET_MM 135.0    // distance axe roues → stylo, mesurée

// Paramètres escalier (Séquence 1)
#define STEP_WIDTH_MM 200.0
#define STEP_HEIGHT_MM 100.0
#define NB_STEPS 2

// --- MAPPING DES BROCHES MOTEURS ---
const int PIN_EN_D = 23;  const int PIN_IN1_D = 19; const int PIN_IN2_D = 18;
const int PIN_EN_G = 4;   const int PIN_IN1_G = 17; const int PIN_IN2_G = 16;

// --- MAPPING DES BROCHES ENCODEURS ---
const int ENC_A_G = 32;
const int ENC_A_D = 27;

// --- VARIABLES D'ODOMÉTRIE ---
volatile long ticsGauches = 0;
volatile long ticsDroits = 0;

unsigned long tempsPrecedent = 0;
float vitesseGauche = 0.0;
float vitesseDroite = 0.0;

float ax = 0.0, ay = 0.0, az = 9.81;
float gx = 0.0, gy = 0.0, gz = 0.0;
float mx = 0.0, my = 0.0, mz = 0.0;

// --- ODOMÉTRIE CUMULÉE ---
float posX = 0.0, posY = 0.0;
float thetaEnc = 0.0;
const float entraxe = 13.5;
const float distParTic = MM_PAR_TICK / 10.0; // en cm

volatile long totalTicsG = 0;
volatile long totalTicsD = 0;

// --- MACHINE À ÉTATS AUTONOME ---
enum EtatRobot { REPOS, MOVING_FORWARD, ROTATING };
EtatRobot currentState = REPOS;

float targetDistance = 0;
float targetAngle = 0;
float angleZ = 0;
unsigned long lastGyroTime = 0;

// PID distance
float kp_dist = 1.1;
float ki_dist = 0.16;
float kd_dist = 1.0;
float lastErrorDist = 0;
float integralDist = 0;

// PID correction directionnelle (garder une trajectoire droite)
float kp_dir = 1.0;
float ki_dir = 0.04;
float kd_dir = 0.08;
float lastErrorDir = 0;
float integralDir = 0;

// Compteurs pour mouvement autonome (indépendants de l'odométrie globale)
volatile long autoTicsG = 0;
volatile long autoTicsD = 0;

const int VITESSE_AUTO = 150;
const int VITESSE_MIN = 100;

// --- VARIABLES INTERFACE ---
float orientationZ = 0.0;
unsigned long dernierTempsIMU = 0;

const int VITESSE = 160;
volatile int etatCourant = 0;
int etatPrecedent = 0;
bool sequenceEnCours = false;

void IRAM_ATTR isrEncodeurGauche() { ticsGauches++; totalTicsG++; autoTicsG++; }
void IRAM_ATTR isrEncodeurDroit() { ticsDroits++; totalTicsD++; autoTicsD++; }

// ==============================================================================
// FONCTIONS MOTEUR DIRECTIONNELLES (pour le mode autonome)
// ==============================================================================

void arreterMoteurs() {
  analogWrite(PIN_IN1_G, 0);
  analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, 0);
  analogWrite(PIN_IN2_D, 0);
}

void avancerMoteurs(int leftPWM, int rightPWM) {
  digitalWrite(PIN_EN_G, HIGH);
  digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, leftPWM);
  analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, 0);
  analogWrite(PIN_IN2_D, rightPWM);
}

void reculerMoteurs(int leftPWM, int rightPWM) {
  digitalWrite(PIN_EN_G, HIGH);
  digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, 0);
  analogWrite(PIN_IN2_G, leftPWM);
  analogWrite(PIN_IN1_D, rightPWM);
  analogWrite(PIN_IN2_D, 0);
}

void tournerDroite(int speed) {
  digitalWrite(PIN_EN_G, HIGH);
  digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, speed);
  analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, speed);
  analogWrite(PIN_IN2_D, 0);
}

void tournerGauche(int speed) {
  digitalWrite(PIN_EN_G, HIGH);
  digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, 0);
  analogWrite(PIN_IN2_G, speed);
  analogWrite(PIN_IN1_D, 0);
  analogWrite(PIN_IN2_D, speed);
}

// Fonction originale pour le pilotage manuel
void piloter(int pwmIn1G, int pwmIn2G, int pwmIn1D, int pwmIn2D) {
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, pwmIn1G); analogWrite(PIN_IN2_G, pwmIn2G);
  analogWrite(PIN_IN1_D, pwmIn1D); analogWrite(PIN_IN2_D, pwmIn2D);
}

void stopper() { piloter(0, 0, 0, 0); }

// ==============================================================================
// FONCTIONS DE CONTRÔLE AUTONOME
// ==============================================================================

void resetAutoEncoders() {
  autoTicsG = 0;
  autoTicsD = 0;
}

float getAutoDistanceG() {
  return autoTicsG * MM_PAR_TICK;
}

float getAutoDistanceD() {
  return autoTicsD * MM_PAR_TICK;
}

float getAutoAverageDistance() {
  return (getAutoDistanceG() + getAutoDistanceD()) / 2.0;
}

void moveDistance(float distanceMm) {
  resetAutoEncoders();

  // Compensation pour inertie et glissement (à ajuster sur le terrain)
  float distance_compensee = distanceMm - 10.0 - (0.05 * distanceMm);
  targetDistance = distance_compensee;
  currentState = MOVING_FORWARD;
  lastErrorDist = targetDistance;
  integralDist = 0;
  lastErrorDir = 0;
  integralDir = 0;

  if (distance_compensee > 0) {
    avancerMoteurs(VITESSE_AUTO, VITESSE_AUTO);
  } else {
    reculerMoteurs(VITESSE_AUTO, VITESSE_AUTO);
  }
}

void rotate(float angleDegrees) {
  resetAutoEncoders();
  angleZ = 0;
  lastGyroTime = millis();
  targetAngle = angleDegrees;
  currentState = ROTATING;
}

void updateEtatRobot() {
  if (currentState == REPOS) return;

  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastTime = now;

  switch (currentState) {
    case MOVING_FORWARD: {
      float currentDistance = getAutoAverageDistance();
      float errorDist = targetDistance - currentDistance;
      integralDist += errorDist * dt;
      float derivativeDist = (errorDist - lastErrorDist) / dt;
      float pid_dist = kp_dist * errorDist + ki_dist * integralDist + kd_dist * derivativeDist;
      lastErrorDist = errorDist;

      // Correction directionnelle : garder la trajectoire droite
      float errorDir = (float)(autoTicsG - autoTicsD);
      integralDir += errorDir * dt;
      float derivativeDir = (errorDir - lastErrorDir) / dt;
      float pid_dir = kp_dir * errorDir + ki_dir * integralDir + kd_dir * derivativeDir;
      lastErrorDir = errorDir;

      int baseSpeed = VITESSE_AUTO + constrain((int)pid_dist, -50, 50);
      int leftSpeed = baseSpeed - constrain((int)pid_dir, -30, 30);
      int rightSpeed = baseSpeed + constrain((int)pid_dir, -30, 30);

      leftSpeed = constrain(leftSpeed, VITESSE_MIN, 255);
      rightSpeed = constrain(rightSpeed, VITESSE_MIN, 255);

      if (abs(errorDist) < 8.0) {
        arreterMoteurs();
        currentState = REPOS;
      } else if (targetDistance > 0) {
        if (currentDistance < targetDistance) {
          avancerMoteurs(leftSpeed, rightSpeed);
        } else {
          arreterMoteurs();
          currentState = REPOS;
        }
      } else {
        if (currentDistance < abs(targetDistance)) {
          reculerMoteurs(leftSpeed, rightSpeed);
        } else {
          arreterMoteurs();
          currentState = REPOS;
        }
      }
      break;
    }
    case ROTATING: {
      // Lire le gyroscope et intégrer l'angle
      sensors_event_t accel, gyro, temp;
      lsm6ds3.getEvent(&accel, &gyro, &temp);

      unsigned long currentTime = millis();
      float dtGyro = (currentTime - lastGyroTime) / 1000.0;
      lastGyroTime = currentTime;

      // Adafruit retourne rad/s, on convertit en deg/s
      float gyroZ_dps = gyro.gyro.z * (180.0 / PI);
      angleZ += gyroZ_dps * dtGyro;

      float error = targetAngle - angleZ;

      if (abs(error) < 4.0) {
        arreterMoteurs();
        currentState = REPOS;
      } else {
        int rotateSpeed = VITESSE_AUTO;
        if (abs(error) < 20.0) rotateSpeed = VITESSE_MIN;

        if (error > 0) {
          tournerDroite(rotateSpeed);
        } else {
          tournerGauche(rotateSpeed);
        }
      }
      break;
    }
    case REPOS:
      break;
  }
}

void ATTENDREREPOS() {
  while (currentState != REPOS) {
    updateEtatRobot();
    server.handleClient();
    delay(10);
  }
}

// ==============================================================================
// SÉQUENCE 1 : ESCALIER
// 20 cm → virage gauche 90° → 10 cm → virage droit 90° → 40 cm
// Toutes les fonctions sont bloquantes (busy-wait).
// Virages : arc avant (roue intérieure positive = pas de pivot sur place).
// Angles   : gyroscope (convention : gauche → angleZ négatif).
// Distances: encodeurs, correction proportionnelle pour tenir la ligne droite.
// ==============================================================================

// Remet à zéro l'intégration gyro et l'horodatage
static void seq_resetGyro() {
  angleZ = 0.0f;
  lastGyroTime = millis();
}

// Lit le gyro et intègre l'angle (à appeler en boucle rapide)
static void seq_majGyro() {
  sensors_event_t a, g, t;
  lsm6ds3.getEvent(&a, &g, &t);
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;   // cap pour éviter saut si interruption longue
  lastGyroTime = now;
  angleZ += g.gyro.z * (180.0f / PI) * dt;
}

// Avance en ligne droite sur distanceMm (mm).
// Correction P sur le différentiel d'encodeurs pour compenser la dérive.
// STOP_THRESHOLD : distance d'anticipation pour absorber l'inertie (à calibrer).
static void seq_avancer(float distanceMm) {
  delay(100);            // stabilisation mécanique avant démarrage
  resetAutoEncoders();   // RAZ propre après stabilisation (élimine ticks de rebond)
  seq_resetGyro();       // cap actuel = référence 0°

  const float STOP_THRESHOLD = -10.0f;
  const int   BASE_SPEED     = 150;
  const float KP_GYRO        = 6.0f;  // correction gyroscopique (augmenter si dérive persiste)

  while (getAutoAverageDistance() < distanceMm - STOP_THRESHOLD) {
    seq_majGyro();
    int correction = (int)(-KP_GYRO * angleZ);
    int leftSpeed  = constrain(BASE_SPEED + correction, 80, 255);
    int rightSpeed = constrain(BASE_SPEED - correction, 80, 255);
    avancerMoteurs(leftSpeed, rightSpeed);
    server.handleClient();
    delay(10);
  }
  arreterMoteurs();
  delay(300);
}

// ==============================================================================
// VIRAGES PAR ENCODEURS (pivot sur place, 90° exact)
//
// Formule : arc_roue = (π/2) × (ENTRAXE_MM/2) = π×ENTRAXE_MM/4
// ticks_90 = arc_roue / MM_PAR_TICK
// Avec ENTRAXE_MM=82, MM_PAR_TICK=π×90/857=0.330 → ticks_90 ≈ 195
//
// Pivot sur place : une roue avance, l'autre recule à vitesse identique.
// → le stylo trace un point (pivot pur, pas d'arc).
// ==============================================================================

#define TICKS_90 198   // calibré : 176 ticks = 80° réels → 176×90/80 = 198
#define SPEED_PIVOT 130

// Pivot gauche 90° : G recule, D avance
static void seq_virerGauche() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  // G arrière, D avant
  analogWrite(PIN_IN1_G, 0);          analogWrite(PIN_IN2_G, SPEED_PIVOT);
  analogWrite(PIN_IN1_D, 0);          analogWrite(PIN_IN2_D, SPEED_PIVOT);
  while (autoTicsG < TICKS_90 && autoTicsD < TICKS_90) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(300);
}

// Pivot droit 90° : G avance, D recule
static void seq_virerDroit() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  // G avant, D arrière
  analogWrite(PIN_IN1_G, SPEED_PIVOT); analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, SPEED_PIVOT); analogWrite(PIN_IN2_D, 0);
  while (autoTicsG < TICKS_90 && autoTicsD < TICKS_90) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(300);
}

// ==============================================================================
// TESTS DE CALIBRATION
// ==============================================================================

// TEST DISTANCE : avance de 40 cm via seq_avancer (encodeurs + correction).
// Mesurer la vraie distance au sol pour vérifier la calibration.
String testDistance() {
  seq_avancer(400.0f);

  String msg = "TEST DIST | ticsG=" + String(autoTicsG)
             + " ticsD=" + String(autoTicsD)
             + " | Cible=400mm — mesurez la vraie distance.";
  Serial.println(msg);
  return msg;
}

// TEST ANGLE : pivot sur place de TICKS_90 ticks (encodeurs purs, pas de gyro).
// Mesurer le vrai angle au sol → si ≠ 90°, ajuster TICKS_90.
// Règle : angle_reel < 90° → augmenter TICKS_90 ; angle_reel > 90° → diminuer.
String testAngle() {
  seq_virerDroit();   // réutilise exactement la même logique que la séquence

  float angleEnc = (autoTicsG + autoTicsD) * MM_PAR_TICK / ENTRAXE_MM * (180.0f / PI);

  String msg = "TEST ANGLE encodeurs | ticsG=" + String(autoTicsG)
             + " ticsD=" + String(autoTicsD)
             + " | angle_enc=" + String(angleEnc, 1) + "deg"
             + " | TICKS_90=" + String(TICKS_90)
             + " | Mesurez l'angle reel.";
  Serial.println(msg);
  return msg;
}

// ==============================================================================
// MARQUAGE DÉPART / ARRIVÉE — arêtes orthogonales (ET1.1)
// Petit pivot gauche ~20° + retour + petit pivot droit ~20° + retour
// TICKS_20 = 198 × 20/90 = 44 ticks
// ==============================================================================
#define TICKS_20 44

static void seq_pivotPetitGauche() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, 0);           analogWrite(PIN_IN2_G, SPEED_PIVOT);
  analogWrite(PIN_IN1_D, 0);           analogWrite(PIN_IN2_D, SPEED_PIVOT);
  while (autoTicsG < TICKS_20 && autoTicsD < TICKS_20) { server.handleClient(); delay(5); }
  arreterMoteurs();
  delay(150);
}

static void seq_pivotPetitDroit() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, SPEED_PIVOT); analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, SPEED_PIVOT); analogWrite(PIN_IN2_D, 0);
  while (autoTicsG < TICKS_20 && autoTicsD < TICKS_20) { server.handleClient(); delay(5); }
  arreterMoteurs();
  delay(150);
}

static void marquerPoint() {
  // gauche 20° → retour droit 20° → droit 20° → retour gauche 20°
  // résultat : trait en croix orthogonal, robot revient à l'orientation de départ
  seq_pivotPetitGauche();
  seq_pivotPetitDroit();
  seq_pivotPetitDroit();
  seq_pivotPetitGauche();
  delay(200);
}

// ==============================================================================
// MARCHE : arc gauche doux → le stylo trace la "marche" (~10cm de tracé)
//          puis petit arc droit de correction pour réaligner vers segment 3.
//
// Les deux roues avancent (pas de pivot) : L=lente, D=rapide → arc gauche.
// TICKS_MARCHE : ticks roue droite (rapide) pour ~10cm de tracé stylo → à calibrer.
// TICKS_CORRECT : ticks roue gauche (rapide) pour réaligner → à calibrer.
// ==============================================================================
// ==============================================================================
// MARCHE : arc gauche 90° + avancer 10cm + arc droit 90°
// Les deux roues avancent (pas de pivot), arrêt par différentiel d'encodeurs.
// Formule : diff_ticks_90 = (π/2 × ENTRAXE_MM) / MM_PAR_TICK = 390 ticks
// Arc gauche : G lente, D rapide → diff = ticsD - ticsG
// Arc droit  : G rapide, D lente → diff = ticsG - ticsD
// SPEED_ARC_SLOW à ajuster pour rendre l'arc plus ou moins serré visuellement.
// ==============================================================================
// ==============================================================================
// ARCS SERRÉS : roue intérieure en MARCHE ARRIÈRE légère + roue extérieure en avant.
//
// Calcul : avec inner=-50 et outer=+150, l'effet de rotation est
//   proportionnel à (150+50)=200, et la vitesse linéaire à (150-50)/2=50.
//   Pour 90° : outer_ticks = (π/2 × 82 / 0.330) × (150/200) ≈ 293 ticks ≈ 10cm ✓
//
// TICKS_MARCHE : ticks roue extérieure pour ~10cm et ~90° (à calibrer)
// ==============================================================================
#define SPEED_ARC_OUT   150   // roue extérieure : avant
#define SPEED_ARC_IN    50    // roue intérieure : arrière (marche arrière légère)
#define TICKS_MARCHE    195   // réduit de 1/3 (293→195)
#define TICKS_CORRECT    30   // correction très légère pour réaligner segment 3

// Arc gauche serré : G=intérieure (arrière), D=extérieure (avant)
static void seq_arcGauche() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, 0);            analogWrite(PIN_IN2_G, SPEED_ARC_IN);   // G arrière
  analogWrite(PIN_IN1_D, 0);            analogWrite(PIN_IN2_D, SPEED_ARC_OUT);  // D avant
  while (autoTicsD < TICKS_MARCHE) { server.handleClient(); delay(5); }
  arreterMoteurs();
  delay(250);
}

// Arc droit serré (correction) : G=extérieure (avant), D=intérieure (arrière)
static void seq_arcDroit() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, SPEED_ARC_OUT); analogWrite(PIN_IN2_G, 0);            // G avant
  analogWrite(PIN_IN1_D, SPEED_ARC_IN);  analogWrite(PIN_IN2_D, 0);            // D arrière
  while (autoTicsG < TICKS_MARCHE) { server.handleClient(); delay(5); }
  arreterMoteurs();
  delay(250);
}

// Repositionnement actif : avance en tournant doucement à droite
// G rapide, D lente (les deux en avant) → arc doux vers la droite
// Arrêt sur ticks roue gauche (extérieure)
#define SPEED_REPO_FAST  150
#define SPEED_REPO_SLOW   60
#define TICKS_REPO       400   // augmenté (120→195) : robot pas assez réaligné

static void seq_marche() {
  seq_arcGauche();   // segment 2 : arc serré gauche ~90°

  // Repositionnement actif : avance en arc doux vers la droite
  resetAutoEncoders();
  avancerMoteurs(SPEED_REPO_FAST, SPEED_REPO_SLOW);
  while (autoTicsG < TICKS_REPO) { server.handleClient(); delay(5); }
  arreterMoteurs();
  delay(250);
}

// ==============================================================================
// SÉQUENCE 1 : ESCALIER
// 20cm → marche (arc ~10cm) → correction → 40cm
// ET1.1 : arêtes orthogonales au départ et à l'arrivée
// ET1.2 : distances ±1cm (encodeurs, TICKS_PAR_TOUR=857)
// ET1.3 : angles tolérés sur la marche (arc accepté)
// ==============================================================================
void drawStairs() {
  sequenceEnCours = true;

  marquerPoint();        // ET1.1 : marquage départ

  seq_avancer(200.0f);  // segment 1 : 20 cm
  seq_marche();         // marche : arc ~10cm + correction
  seq_avancer(400.0f);  // segment 3 : 40 cm

  marquerPoint();        // ET1.1 : marquage arrivée

  arreterMoteurs();
  sequenceEnCours = false;
}

// ==============================================================================
// LE SITE WEB EMBARQUÉ
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
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px; }
        .sensor-card { background: #ffffff; border: 1px solid #ddd; border-radius: 10px; padding: 15px; box-shadow: 0 2px 5px rgba(0,0,0,0.05); }
        .sensor-card h4 { margin: 0 0 10px 0; color: #1a73e8; border-bottom: 2px solid #f0f2f5; font-size: 16px;}
        .axis { font-family: 'Consolas', monospace; font-size: 14px; margin: 5px 0; display: flex; justify-content: space-between; }
        .axis span { font-weight: bold; color: #d93025; }

        .needle-enc {
            width: 2px; height: 40px; background: #1a73e8;
            position: absolute; left: 49%; bottom: 50%;
            transform-origin: bottom center; opacity: 0.7; transition: transform 0.2s;
        }
        .wheel-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-top: 10px; }
        .wheel-card { background: white; padding: 15px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .speed-bar-container { background: #eee; height: 10px; border-radius: 5px; margin-top: 10px; overflow: hidden; }
        .speed-bar { background: #d84315; height: 100%; width: 0%; transition: width 0.3s; }
        .divider { margin: 20px 0 10px; font-size: 14px; color: #888; font-weight: bold; }
        .compass {
            width: 100px; height: 100px; border: 3px solid #1a73e8;
            border-radius: 50%; margin: 10px auto; position: relative; background: white;
        }
        .needle {
            width: 4px; height: 50px; background: #d93025;
            position: absolute; left: 48%; bottom: 50%;
            transform-origin: bottom center; transition: transform 0.2s;
        }
        .btn-sequence {
            width: auto; height: auto; padding: 15px 30px; font-size: 16px;
            background: #1a73e8; color: white; border-radius: 8px;
            margin: 10px; cursor: pointer;
        }
        .btn-sequence:active { background: #1557b0; }
        .sequence-section { margin: 20px auto; max-width: 500px; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
    </style>
</head>
<body>
    <h1>HogRider</h1>

    <!-- SECTION SÉQUENCES -->
    <div class="sequence-section">
        <h3>Séquences de dessin</h3>
        <button class="btn-sequence" onclick="lancerSequence(1)">Séquence 1 - Escalier</button>
        <hr style="margin:15px 0; border-color:#eee;">
        <b style="color:#555; font-size:14px;">Calibration</b><br>
        <button class="btn-sequence" style="background:#e67e22;" onclick="lancerTest('distance')">Test Distance (1000 ticks)</button>
        <button class="btn-sequence" style="background:#8e44ad;" onclick="lancerTest('angle')">Test Angle (90° pivot)</button>
        <div id="seq-status" style="margin-top:10px; font-style:italic; color:#555;"></div>
    </div>

    <!-- CAPTEURS IMU -->
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

    <!-- VITESSES DES ROUES -->
    <div class="divider">VITESSES DES ROUES</div>
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

    <!-- POSITION ET ORIENTATION -->
    <div class="divider">POSITION ET ORIENTATION</div>
    <div style="display: flex; justify-content: space-around; align-items: flex-start; gap: 15px; flex-wrap: wrap;">
        <div class="wheel-card" style="flex: 1; min-width: 200px;">
            <h4>Position du Stylo</h4>
            <div style="margin: 15px 0;">X : <span class="data-val" id="valX" style="font-size: 24px;">0.0 cm</span></div>
            <div style="margin: 15px 0;">Y : <span class="data-val" id="valY" style="font-size: 24px;">0.0 cm</span></div>
            <button class="btn-sequence" onclick="fetch('/action?v=9')" style="width:100%; font-size:14px; background:#f29900;">Reset Position</button>
        </div>
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

    <!-- TERMINAL -->
    <div id="terminal"></div>

    <!-- BOUTONS DE PILOTAGE -->
    <div class="grid">
        <div></div>
        <button onmousedown="t(1, event)" onmouseup="t(0, event)" ontouchstart="t(1, event)" ontouchend="t(0, event)">&#9650;</button>
        <div></div>
        <button onmousedown="t(3, event)" onmouseup="t(0, event)" ontouchstart="t(3, event)" ontouchend="t(0, event)">&#9664;</button>
        <button class="btn-stop" onclick="t(0, event)">STOP</button>
        <button onmousedown="t(4, event)" onmouseup="t(0, event)" ontouchstart="t(4, event)" ontouchend="t(0, event)">&#9654;</button>
        <div></div>
        <button onmousedown="t(2, event)" onmouseup="t(0, event)" ontouchstart="t(2, event)" ontouchend="t(0, event)">&#9660;</button>
        <div></div>
    </div>

    <script>
        let currentCmd = 0; let isTouch = false;
        const cmds = ["STOP", "AVANT", "ARRIERE", "GAUCHE", "DROITE"];

        function log(msg, type) {
            const term = document.getElementById('terminal');
            term.innerHTML += '<div class="' + type + '">[' + new Date().toLocaleTimeString() + '] ' + msg + '</div>';
            term.scrollTop = term.scrollHeight;
        }

        window.onload = () => {
            fetch('/ping').then(() => log("Réseau OK", "sys"));
            setInterval(fetchTelemetry, 500);
        };

        function lancerSequence(num) {
            document.getElementById('seq-status').innerText = "Séquence " + num + " en cours...";
            log("Lancement séquence " + num, "tx");
            fetch('/sequence' + num).then(res => res.text()).then(text => {
                log("Réponse: " + text, "rx");
                document.getElementById('seq-status').innerText = text;
            });
        }

        function lancerTest(type) {
            document.getElementById('seq-status').innerText = "Test " + type + " en cours...";
            log("Lancement test " + type, "tx");
            fetch('/test?t=' + type).then(res => res.text()).then(text => {
                log("Résultat: " + text, "rx");
                document.getElementById('seq-status').innerText = text;
            });
        }

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
                    document.getElementById('needle').style.transform = 'rotate(' + (-data.yaw) + 'deg)';
                    document.getElementById('needleEnc').style.transform = 'rotate(' + (-data.yawEnc) + 'deg)';
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
            }).catch(err => console.log("Erreur telemetry:", err));
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
// ROUTES WEB
// ==============================================================================

void handleRoot() { server.send(200, "text/html", index_html); }
void handlePing() { server.send(200, "text/plain", "PONG"); }

void handleAction() {
  if (server.hasArg("v")) {
    int cmd = server.arg("v").toInt();

    if (cmd == 9) {
      posX = 0.0; posY = 0.0; thetaEnc = 0.0; orientationZ = 0.0;
      totalTicsG = 0; totalTicsD = 0;
      server.send(200, "text/plain", "Position réinitialisée");
    } else {
      etatCourant = cmd;
      server.send(200, "text/plain", "OK");
    }
  }
}

void handleTelemetry() {
  String json = "{";
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
  json += "\"yaw\":" + String(orientationZ, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSequence1() {
  if (sequenceEnCours) {
    server.send(409, "text/plain", "Séquence déjà en cours");
    return;
  }
  server.send(200, "text/plain", "Escalier lancé");
  drawStairs();
}

void handleTest() {
  if (sequenceEnCours) {
    server.send(409, "text/plain", "Séquence déjà en cours");
    return;
  }
  if (!server.hasArg("t")) {
    server.send(400, "text/plain", "Paramètre t manquant");
    return;
  }
  String type = server.arg("t");
  sequenceEnCours = true;
  String result;
  if (type == "distance") {
    server.send(200, "text/plain", "Test distance lancé...");
    result = testDistance();
  } else if (type == "angle") {
    server.send(200, "text/plain", "Test angle lancé...");
    result = testAngle();
  } else {
    server.send(400, "text/plain", "Type inconnu");
    sequenceEnCours = false;
    return;
  }
  sequenceEnCours = false;
  Serial.println(result);
}

void handleStop() {
  arreterMoteurs();
  currentState = REPOS;
  sequenceEnCours = false;
  etatCourant = 0;
  server.send(200, "text/plain", "Arrêt forcé");
}

// ==============================================================================
// INITIALISATION
// ==============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Désactive le brownout detector
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
  server.on("/sequence1", handleSequence1);
  server.on("/test", handleTest);
  server.on("/stop", handleStop);
  server.begin();

  Serial.println("Init capteurs...");

  if (!lsm6ds3.begin_I2C(0x6B) || !lis3mdl.begin_I2C(0x1E)) {
    Serial.println("Erreur capteur!");
    while (1) delay(10);
  }

  lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  Serial.printf("AP 'HogRider' -> IP: %s\n", WiFi.softAPIP().toString().c_str());
}

// ==============================================================================
// BOUCLE PRINCIPALE
// ==============================================================================
void loop() {
  server.handleClient();

  // Lecture IMU pour télémétrie
  sensors_event_t accel, gyro, temp, mag;
  lsm6ds3.getEvent(&accel, &gyro, &temp);

  unsigned long maintenant = millis();
  float dt = (maintenant - dernierTempsIMU) / 1000.0;
  dernierTempsIMU = maintenant;

  if (abs(gyro.gyro.z) > 0.05) {
    orientationZ += gyro.gyro.z * dt * (180.0 / PI);
  }

  lis3mdl.getEvent(&mag);

  ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
  gx = gyro.gyro.x;          gy = gyro.gyro.y;          gz = gyro.gyro.z;
  mx = mag.magnetic.x;       my = mag.magnetic.y;       mz = mag.magnetic.z;

  // Calcul vitesse toutes les 100ms
  unsigned long tempsActuel = millis();
  if (tempsActuel - tempsPrecedent >= 100) {
    vitesseGauche = (float)ticsGauches / 0.1;
    vitesseDroite = (float)ticsDroits / 0.1;
    ticsGauches = 0;
    ticsDroits = 0;
    tempsPrecedent = tempsActuel;
  }

  // Pilotage manuel (seulement si pas de séquence en cours)
  if (!sequenceEnCours && etatCourant != etatPrecedent) {
    switch (etatCourant) {
      case 1: piloter(VITESSE, 0, 0, VITESSE); break;
      case 2: piloter(0, VITESSE, VITESSE, 0); break;
      case 3: piloter(0, VITESSE, 0, VITESSE); break;
      case 4: piloter(VITESSE, 0, VITESSE, 0); break;
      case 0: stopper(); break;
    }
    etatPrecedent = etatCourant;
  }

  // Odométrie globale
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

  // Mise à jour PID autonome (tourne en continu même hors séquence)
  updateEtatRobot();

  delay(10);
}
