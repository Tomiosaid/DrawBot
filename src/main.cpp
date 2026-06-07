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

float headingMag   = 0.0f;   // cap magnétique corrigé (-180..+180°), mis à jour dans loop()
float magOffsetX   = 0.0f;
float magOffsetY   = 0.0f;
bool  magCalibrated = false;
#define MOUNTING_OFFSET 73.0f // calibré empiriquement : cap_raw=-73°-θ, MOUNTING_OFFSET=+73° → cap_corr=-θ

// Log debug séquence 3 — récupérable via /seq3log depuis le dashboard WiFi
String seq3Log = "Aucun essai encore.";

void seq3Print(String msg) {
  seq3Log += msg + "\n";
  Serial.println(msg);
}

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
  resetAutoEncoders();

  const float STOP_THRESHOLD = -10.0f;
  const int   BASE_SPEED     = 150;
  const float KP_GYRO        = 8.0f;  // correction vers cap absolu 0°

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
// PIVOT ENCODEUR N TICKS (générique, utilisé par orienterNord)
// ==============================================================================

// Pivot gauche N ticks : G recule, D avance
static void seq_pivotGaucheN(long ticks, int speed) {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, 0);     analogWrite(PIN_IN2_G, speed);
  analogWrite(PIN_IN1_D, 0);     analogWrite(PIN_IN2_D, speed);
  while (autoTicsG < ticks && autoTicsD < ticks) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(200);
}

// Pivot droit N ticks : G avance, D recule
static void seq_pivotDroitN(long ticks, int speed) {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, speed); analogWrite(PIN_IN2_G, 0);
  analogWrite(PIN_IN1_D, speed); analogWrite(PIN_IN2_D, 0);
  while (autoTicsG < ticks && autoTicsD < ticks) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(200);
}

// Recul encodeurs purs (miroir de seq_avancer mais en arrière, sans gyro)
static void seq_reculer(float distanceMm) {
  resetAutoEncoders();
  const float STOP_THRESHOLD = -10.0f;
  const int   BASE_SPEED     = 150;
  reculerMoteurs(BASE_SPEED, BASE_SPEED);
  while (getAutoAverageDistance() < distanceMm - STOP_THRESHOLD) {
    server.handleClient();
    delay(10);
  }
  arreterMoteurs();
  delay(300);
}

// ==============================================================================
// CALIBRATION HARD-IRON — spin 360° lent, capture min/max de mx et my
// Résultat stocké dans magOffsetX / magOffsetY (µT)
// Durée : ~8s (720° à ~90°/s à SPEED_PIVOT=130 → ~4s × 2 tours de sécurité)
// ==============================================================================
#define SPEED_CALIB 110  // vitesse réduite pour maximiser la précision

void calibrerMagnetometre() {
  seq3Print("CALIB MAG : debut spin 360...");

  float mxMin = 1e9f, mxMax = -1e9f;
  float myMin = 1e9f, myMax = -1e9f;

  // Spin gauche sur place 2 tours complets (~8s) pour couvrir tout le cercle
  const long TICKS_360 = (long)(TICKS_90 * 4);
  const long TICKS_2TOURS = TICKS_360 * 2;

  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  // Pivot gauche lent
  analogWrite(PIN_IN1_G, 0);          analogWrite(PIN_IN2_G, SPEED_CALIB);
  analogWrite(PIN_IN1_D, 0);          analogWrite(PIN_IN2_D, SPEED_CALIB);

  while (autoTicsG < TICKS_2TOURS && autoTicsD < TICKS_2TOURS) {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    float lx = mag.magnetic.x;
    float ly = mag.magnetic.y;
    if (lx < mxMin) mxMin = lx;
    if (lx > mxMax) mxMax = lx;
    if (ly < myMin) myMin = ly;
    if (ly > myMax) myMax = ly;
    server.handleClient();
    delay(20);
  }
  arreterMoteurs();
  delay(300);

  magOffsetX = (mxMin + mxMax) / 2.0f;
  magOffsetY = (myMin + myMax) / 2.0f;
  magCalibrated = true;

  seq3Print("CALIB OK offX=" + String(magOffsetX, 2) + " offY=" + String(magOffsetY, 2));
  seq3Print("  mxMin=" + String(mxMin,2) + " mxMax=" + String(mxMax,2)
          + " myMin=" + String(myMin,2) + " myMax=" + String(myMax,2));
}

// ==============================================================================
// MESURE DU CAP MAGNÉTIQUE (après calibration)
// Retourne l'angle vers le Nord magnétique en degrés [-180 ; +180].
//   0°   = Nord, +90° = Est, -90° = Ouest, ±180° = Sud
// MOUNTING_OFFSET corrige l'orientation physique du LIS3MDL sur le robot.
// ==============================================================================
float mesurerCap() {
  const int NB_SAMPLES = 50;
  float sumX = 0, sumY = 0;

  for (int i = 0; i < NB_SAMPLES; i++) {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    sumX += mag.magnetic.x - magOffsetX;
    sumY += mag.magnetic.y - magOffsetY;
    delay(10);
  }

  float capRad = atan2(sumY / NB_SAMPLES, sumX / NB_SAMPLES);
  float capDeg = capRad * (180.0f / PI) + MOUNTING_OFFSET;

  // Normaliser dans [-180 ; +180]
  while (capDeg >  180.0f) capDeg -= 360.0f;
  while (capDeg < -180.0f) capDeg += 360.0f;

  return capDeg;
}

// ==============================================================================
// ORIENTATION VERS LE NORD — pivot encodeur pur
// Cap mesuré → angle à pivoter → N ticks calculés depuis TICKS_90
// Sens choisi = plus court chemin (|angle| ≤ 180°)
// ==============================================================================
// Une passe de correction : mesure le cap, pivote de -cap, retourne le cap residuel.
static float seq_passeCorrection(int numPasse) {
  float cap = mesurerCap();
  seq3Print("P" + String(numPasse) + " cap=" + String(cap, 1) + " deg");

  float correction = cap;
  while (correction >  180.0f) correction -= 360.0f;
  while (correction < -180.0f) correction += 360.0f;

  long ticks = (long)(fabs(correction) * TICKS_90 / 90.0f);
  seq3Print("  corr=" + String(correction, 1) + " -> " + String(ticks) + " ticks");

  if (ticks < 5) {
    seq3Print("  -> tolerance ok");
    return cap;
  }

  if (correction > 0) {
    seq_pivotDroitN(ticks, SPEED_PIVOT);
  } else {
    seq_pivotGaucheN(ticks, SPEED_PIVOT);
  }
  delay(400);
  return mesurerCap();
}

static void seq_orienterNord() {
  // Passe 1 : grande correction
  float r = seq_passeCorrection(1);
  seq3Print("  -> residu P1=" + String(r, 1) + " deg");

  // Passe 2 : affinage
  r = seq_passeCorrection(2);
  seq3Print("  -> residu P2=" + String(r, 1) + " deg");

  // Passe 3 : si encore > 10 deg
  if (fabs(r) > 10.0f) {
    r = seq_passeCorrection(3);
    seq3Print("  -> residu P3=" + String(r, 1) + " deg");
  }

  seq3Print("CAP FINAL : " + String(r, 1) + " deg");
}

// ==============================================================================
// FLÈCHE NORD — dessinée vers l'avant du robot (= le Nord après orienterNord)
//
// Géométrie :
//   Hampe : 40 mm tout droit
//   Tête  : depuis la pointe de la hampe,
//           - arc gauche  12 mm (roue gauche lente, droite rapide)
//           - retour au centre (recul 12 mm)
//           - arc droit   12 mm (roue droite lente, gauche rapide)
//
// Tolérance barème : longueur totale > 30 mm → 40 mm hampe largement satisfait.
// Les arcs de la tête sont en avant depuis la pointe : ils tracent deux courbes
// symétriques vers l'extérieur, formant une pointe de flèche lisible.
// ==============================================================================
#define FLECHE_HAMPE_MM   40.0f
#define FLECHE_BRANCHE_MM 12.0f

// Arc avant gauche : G lente, D rapide (robot courbe vers la gauche)
#define SPEED_BRANCH_FAST  130
#define SPEED_BRANCH_SLOW   50

static void seq_arcBrancheGauche() {
  resetAutoEncoders();
  float ticks_cible = FLECHE_BRANCHE_MM / MM_PAR_TICK;
  avancerMoteurs(SPEED_BRANCH_SLOW, SPEED_BRANCH_FAST);  // G lente → courbe gauche
  while (getAutoAverageDistance() < FLECHE_BRANCHE_MM * 0.85f) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(150);
}

// Arc avant droit : G rapide, D lente (robot courbe vers la droite)
static void seq_arcBrancheDroite() {
  resetAutoEncoders();
  avancerMoteurs(SPEED_BRANCH_FAST, SPEED_BRANCH_SLOW);  // D lente → courbe droite
  while (getAutoAverageDistance() < FLECHE_BRANCHE_MM * 0.85f) {
    server.handleClient();
    delay(5);
  }
  arreterMoteurs();
  delay(150);
}

static void seq_flecheNord() {
  seq_resetGyro();

  // 1. Hampe : avancer 40 mm tout droit
  seq_avancer(FLECHE_HAMPE_MM);

  // 2. Branche gauche depuis la pointe
  seq_arcBrancheGauche();

  // 3. Revenir au centre (recul 12 mm)
  seq_reculer(FLECHE_BRANCHE_MM);

  // 4. Branche droite depuis la pointe
  seq_arcBrancheDroite();
}

// ==============================================================================
// SÉQUENCE 3 : FLÈCHE NORD COMPLÈTE
//   Étape 1 — calibration hard-iron (spin 360° lent)
//   Étape 2 — orienter le robot face au Nord (pivot encodeur)
//   Étape 3 — dessiner la flèche vers l'avant
// ==============================================================================
void drawNorthArrow() {
  sequenceEnCours = true;
  seq3Log = "";   // reset log à chaque essai

  seq3Print("=== SEQ3 DEBUT ===");
  calibrerMagnetometre();

  seq_orienterNord();

  seq3Print("=== FLECHE ===");
  seq_flecheNord();

  arreterMoteurs();
  sequenceEnCours = false;
  seq3Print("=== SEQ3 FIN ===");
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
  while (autoTicsD < TICKS_MARCHE) { seq_majGyro(); server.handleClient(); delay(5); }
  arreterMoteurs();
  for (int i = 0; i < 50; i++) { seq_majGyro(); delay(5); }  // 250ms actif
}

// Arc droit serré (correction) : G=extérieure (avant), D=intérieure (arrière)
static void seq_arcDroit() {
  resetAutoEncoders();
  digitalWrite(PIN_EN_G, HIGH); digitalWrite(PIN_EN_D, HIGH);
  analogWrite(PIN_IN1_G, SPEED_ARC_OUT); analogWrite(PIN_IN2_G, 0);            // G avant
  analogWrite(PIN_IN1_D, SPEED_ARC_IN);  analogWrite(PIN_IN2_D, 0);            // D arrière
  while (autoTicsG < TICKS_MARCHE) { seq_majGyro(); server.handleClient(); delay(5); }
  arreterMoteurs();
  for (int i = 0; i < 50; i++) { seq_majGyro(); delay(5); }  // 250ms actif
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
  while (autoTicsG < TICKS_REPO) { seq_majGyro(); server.handleClient(); delay(5); }
  // PAS d'arrêt moteurs : on enchaîne directement avec seq_avancer()
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
  seq_resetGyro();       // cap absolu = 0° fixé au départ (segment 1 = référence)

  marquerPoint();        // ET1.1 : marquage départ

  seq_avancer(160.0f);  // segment 1 : 16 cm (+ ~4 cm début arc = 20 cm visuel)
  seq_marche();         // marche : arc ~10cm + correction
  seq_avancer(260.0f);  // segment 3 : 26 cm (repo ~14 + seg3 26 = 40 cm total)

  marquerPoint();        // ET1.1 : marquage arrivée

  arreterMoteurs();
  sequenceEnCours = false;
}

// ==============================================================================
// SÉQUENCE 2 : LE CERCLE
//
// L'utilisateur demande un rayon de tracé du STYLO (cm). Le stylo est déporté
// de D_PEN EN AVANT de l'axe des roues : le centre de l'essieu décrit donc un
// cercle plus petit, de rayon  Rc = sqrt(rayon_stylo² − D_PEN²)  (Pythagore).
//
// Principe (repris du modèle, adapté À NOS paramètres) :
//   - roue extérieure (gauche) à vitesse fixe ;
//   - roue intérieure (droite) asservie en P pour tenir le ratio des rayons ;
//   - fin du cercle quand l'écart d'encodeurs atteint 2π·ENTRAXE, ce qui
//     correspond TOUJOURS à 360° de rotation (la boucle se referme quel que
//     soit le rayon réellement tracé).
//
// Toutes les constantes sont dérivées des #define physiques en haut du fichier
// (PEN_OFFSET_MM, ENTRAXE_MM, MM_PAR_TICK) : aucune valeur en dur d'un autre robot.
// ==============================================================================
// NOTE (mise a jour) : la FERMETURE du cercle se fait desormais par GYROSCOPE
// (on integre la rotation reelle jusqu'a 360 deg), et NON plus par l'ecart
// d'encodeurs decrit ci-dessus. Raison : en virage les roues patinent et
// l'entraxe reel est incertain, donc l'ecart d'encodeurs vise etait atteint
// AVANT 360 deg -> le robot ne tracait qu'environ 3/4 de demi-cercle.
// L'asservissement des roues (plus bas) ne sert plus qu'a tenir le RAYON.
void executerCercle(float rayon_stylo_cm) {
  sequenceEnCours = true;

  // --- Paramètres dérivés de NOS constantes physiques ---
  const float D_PEN       = PEN_OFFSET_MM / 10.0;   // 13.5 cm (déport stylo mesuré)
  const float ENTRAXE_CM  = ENTRAXE_MM   / 10.0;    // 8.2 cm  (voie centre-à-centre)
  const float TICS_PAR_CM = 10.0 / MM_PAR_TICK;     // ≈ 30.3 ticks/cm (calibré : 857 t/tour)

  // Rayon stylo minimal traçable : on garde Rc > ENTRAXE/2 (+marge) pour que le
  // ratio reste positif et la roue intérieure n'ait jamais à reculer.
  const float RAYON_MIN = sqrt(pow(ENTRAXE_CM / 2.0 + 1.5, 2) + pow(D_PEN, 2)); // ≈ 14.6 cm

  Serial.println("\n=== SEQUENCE 2 : CERCLE ===");
  Serial.print("Rayon stylo demande (cm) : "); Serial.println(rayon_stylo_cm);

  // --- CALIBRATION EMPIRIQUE DU RAYON (mesure terrain) ---
  // Le robot trace TROP GRAND (roue interieure bridee par son plancher PWM +
  // entraxe reel > theorique). Modele lineaire mesure sur le robot :
  //     rayon_trace = CAL_GAIN * rayon_envoye + CAL_OFFSET
  //   donnees : 20->22.5 ; 17->20 ; 15->17.75   =>  CAL_GAIN ~= 0.95 , CAL_OFFSET ~= 3.5
  // On INVERSE le modele pour envoyer le bon rayon a la geometrie.
  // RECALIBRER : tracer 2 cercles, relever (envoye -> trace), resoudre les
  // 2 equations a 2 inconnues, puis remplacer CAL_GAIN / CAL_OFFSET ci-dessous.
  const float CAL_GAIN   = 0.95f;
  const float CAL_OFFSET = 3.5f;
  float rayon_cible = rayon_stylo_cm;                         // ce que l'utilisateur veut voir trace
  rayon_stylo_cm    = (rayon_cible - CAL_OFFSET) / CAL_GAIN;  // ce qu'on envoie reellement
  Serial.print("Rayon corrige -> envoye geometrie (cm) : "); Serial.println(rayon_stylo_cm);

  // --- Bouclier anti-crash : évite racine carrée négative ET ratio négatif ---
  if (rayon_stylo_cm < RAYON_MIN) {
    Serial.print("Rayon hors plage de la methode lisse -> clamp au mini (cm) : ");
    Serial.println(RAYON_MIN);
    Serial.println("(pour un cercle plus petit : utiliser le cercle PIVOT ~13.5 cm)");
    rayon_stylo_cm = RAYON_MIN;
  }

  // Rayon parcouru par le centre de l'essieu
  float Rc = sqrt(pow(rayon_stylo_cm, 2) - pow(D_PEN, 2));

  // Ratio des distances : roue extérieure / roue intérieure
  float ratio = (Rc + ENTRAXE_CM / 2.0) / (Rc - ENTRAXE_CM / 2.0);

  // Écart d'encodeurs à atteindre pour 360° = 2π·ENTRAXE (indépendant du rayon)
  long cible_diff_tics = (long)(2.0 * PI * ENTRAXE_CM * TICS_PAR_CM);

  // Vitesse roue extérieure + plancher PWM roue intérieure.
  // Cercle serré (rayon proche du déport) : on pousse l'extérieure et on
  // abaisse le plancher pour que le ratio puisse être tenu.
  int pwmExtBase = 180;
  int ajuste     = 40;                       // plancher PWM intérieure (≈ seuil démarrage moteur, à calibrer)
  if (rayon_stylo_cm < D_PEN + 3.0) {        // cercle serré (< ~16.5 cm)
    pwmExtBase = 210;
    ajuste     = 30;
  }

  const float Kp = 1.9;                      // gain correcteur roue intérieure (à ajuster sur le terrain)

  resetAutoEncoders();
  Serial.print("Rc(cm)="); Serial.print(Rc, 2);
  Serial.print(" | ratio="); Serial.print(ratio, 2);
  Serial.print(" | cible_diff_tics="); Serial.println(cible_diff_tics);
  Serial.println("--- DEPART MOTEURS ---");

  // Fermeture par GYROSCOPE : on integre la rotation reelle (axe Z) jusqu'a 360 deg.
  // Insensible au glissement des roues et a l'incertitude sur l'entraxe
  // -> la boucle se referme correctement, contrairement a la methode par encodeurs.
  const float ANGLE_CIBLE        = 365.0;   // un tour complet
  const unsigned long TIMEOUT_MS = 30000;   // securite anti-boucle-infinie (~6x la duree normale)
  float angleParcouru = 0.0;
  unsigned long chronoReseau = millis();
  unsigned long tGyro        = millis();
  unsigned long tDebut       = millis();

  while (fabs(angleParcouru) < ANGLE_CIBLE) {
    // Lecture gyro + integration de l'angle reellement tourne
    sensors_event_t aEvt, gEvt, tEvt;
    lsm6ds3.getEvent(&aEvt, &gEvt, &tEvt);
    unsigned long nowGyro = millis();
    float dtGyro = (nowGyro - tGyro) / 1000.0f;
    if (dtGyro > 0.1f) dtGyro = 0.1f;        // anti-saut si la boucle a ete interrompue
    tGyro = nowGyro;
    angleParcouru += gEvt.gyro.z * (180.0f / PI) * dtGyro;

    // Securite : arret si le gyro ne voit jamais 360 deg (capteur muet, blocage moteur...)
    if (millis() - tDebut > TIMEOUT_MS) {
      Serial.println("!!! TIMEOUT SECURITE (gyro suspect ?) -> arret !!!");
      break;
    }
    // On n'interroge le Wi-Fi que toutes les 50 ms (timing serré de la boucle)
    if (millis() - chronoReseau > 50) {
      server.handleClient();
      chronoReseau = millis();
    }
    if (!sequenceEnCours) {                  // arrêt d'urgence (bouton STOP)
      Serial.println("!!! ARRET D'URGENCE !!!");
      break;
    }

    // Asservissement P : la roue intérieure (D) suit  G / ratio
    float tics_D_cible = autoTicsG / ratio;
    float erreur       = tics_D_cible - (float)autoTicsD;
    int   pwmInt       = (int)(pwmExtBase / ratio + Kp * erreur);
    pwmInt = constrain(pwmInt, ajuste, 255);

    avancerMoteurs(pwmExtBase, pwmInt);      // G = extérieure (rapide), D = intérieure (asservie)
    delay(10);
  }

  arreterMoteurs();
  Serial.print("=== FIN DU CERCLE | angle gyro mesure = ");
  Serial.print(angleParcouru, 1); Serial.println(" deg ===");
  if (sequenceEnCours) delay(500);
  resetAutoEncoders();
  sequenceEnCours = false;
}

// ==============================================================================
// CERCLE MINIMAL : PIVOT SUR PLACE  (rayon trace = deport stylo D_PEN ~13.5 cm)
//
// Quand le robot pivote sur lui-meme autour du centre de l'essieu, le stylo
// (place D_PEN EN AVANT) decrit un cercle PARFAIT de rayon exactement D_PEN.
// C'est le PLUS PETIT cercle propre que cette geometrie autorise : aucun reglage
// logiciel ne permet de descendre sous D_PEN tant que le stylo est si en avant.
// Fermeture par gyroscope a 360 deg (comme la methode lisse).
// ==============================================================================
void executerCerclePivot() {
  sequenceEnCours = true;
  Serial.println("\n=== CERCLE PIVOT (rayon = deport stylo ~13.5 cm) ===");

  const float ANGLE_CIBLE        = 360.0;
  const unsigned long TIMEOUT_MS = 30000;
  const int  SPEED_PIVOT_CERCLE  = 130;     // vitesse de pivot (a ajuster si besoin)
  float angleParcouru = 0.0;

  resetAutoEncoders();
  unsigned long chronoReseau = millis();
  unsigned long tGyro        = millis();
  unsigned long tDebut       = millis();

  while (fabs(angleParcouru) < ANGLE_CIBLE) {
    // Integration gyro (rotation reelle)
    sensors_event_t aEvt, gEvt, tEvt;
    lsm6ds3.getEvent(&aEvt, &gEvt, &tEvt);
    unsigned long nowGyro = millis();
    float dtGyro = (nowGyro - tGyro) / 1000.0f;
    if (dtGyro > 0.1f) dtGyro = 0.1f;
    tGyro = nowGyro;
    angleParcouru += gEvt.gyro.z * (180.0f / PI) * dtGyro;

    if (millis() - tDebut > TIMEOUT_MS) { Serial.println("!!! TIMEOUT PIVOT !!!"); break; }
    if (millis() - chronoReseau > 50)  { server.handleClient(); chronoReseau = millis(); }
    if (!sequenceEnCours)              { Serial.println("!!! ARRET D'URGENCE !!!"); break; }

    tournerDroite(SPEED_PIVOT_CERCLE);      // pivot sur place (G avant, D arriere)
    delay(10);
  }

  arreterMoteurs();
  Serial.print("=== FIN CERCLE PIVOT | angle gyro mesure = ");
  Serial.print(angleParcouru, 1); Serial.println(" deg ===");
  if (sequenceEnCours) delay(500);
  resetAutoEncoders();
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
        <div style="margin-top:12px;">
            <label style="font-size:14px; color:#555;">Rayon stylo (cm) :</label>
            <input type="number" id="rayonCercle" value="20" min="13" step="0.5"
                   style="width:70px; padding:8px; font-size:16px; text-align:center;">
            <button class="btn-sequence" onclick="lancerCercle()">Séquence 2 - Cercle</button>
        </div>
        <button class="btn-sequence" style="background:#27ae60;" onclick="lancerSequence(3)">Séquence 3 - Flèche Nord</button>
        <button class="btn-sequence" style="background:#555; font-size:13px; padding:8px 16px;" onclick="voirDebugSeq3()">📋 Voir debug</button>
        <div id="mag-status" style="margin-top:6px; font-size:13px; color:#555;">
          Cap magnétique : <span id="heading-val" style="font-weight:bold; color:#27ae60;">--°</span>
          &nbsp;|&nbsp; Calibration : <span id="mag-cal" style="font-weight:bold; color:#e67e22;">NON</span>
        </div>
        <pre id="seq3-debug" style="display:none; text-align:left; background:#202124; color:#8ab4f8; font-size:12px; padding:10px; border-radius:8px; margin-top:8px; max-height:200px; overflow-y:auto; white-space:pre-wrap;"></pre>
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

        function voirDebugSeq3() {
            fetch('/seq3log').then(r => r.text()).then(txt => {
                const el = document.getElementById('seq3-debug');
                el.innerText = txt;
                el.style.display = el.style.display === 'none' ? 'block' : 'none';
            });
        }

        function lancerSequence(num) {
            document.getElementById('seq-status').innerText = "Séquence " + num + " en cours...";
            log("Lancement séquence " + num, "tx");
            fetch('/sequence' + num).then(res => res.text()).then(text => {
                log("Réponse: " + text, "rx");
                document.getElementById('seq-status').innerText = text;
            });
        }

        function lancerCercle() {
            const r = document.getElementById('rayonCercle').value;
            document.getElementById('seq-status').innerText = "Cercle r=" + r + " cm en cours...";
            log("Lancement cercle r=" + r + " cm", "tx");
            fetch('/sequence2?r=' + r).then(res => res.text()).then(text => {
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

                if (data.heading !== undefined) {
                    document.getElementById('heading-val').innerText = data.heading.toFixed(1) + "°";
                    const calEl = document.getElementById('mag-cal');
                    calEl.innerText = data.magCal ? "OUI" : "NON";
                    calEl.style.color = data.magCal ? "#27ae60" : "#e67e22";
                }
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
      if (cmd == 0) {            // STOP : interrompt aussi une séquence en cours (cercle)
        sequenceEnCours = false; // -> la boucle de executerCercle sort par "arrêt d'urgence"
        currentState = REPOS;
        arreterMoteurs();
      }
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
  json += "\"yaw\":" + String(orientationZ, 1) + ",";
  json += "\"heading\":" + String(headingMag, 1) + ",";
  json += "\"magCal\":" + String(magCalibrated ? 1 : 0);
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

void handleSequence2() {
  if (sequenceEnCours) {
    server.send(409, "text/plain", "Séquence déjà en cours");
    return;
  }

  float rayon = 20.0;                          // rayon stylo par défaut (cm)
  if (server.hasArg("r")) rayon = server.arg("r").toFloat();

  // Routage selon le rayon demande (stylo a ~13.5 cm EN AVANT de l'axe des roues) :
  const float D_PEN = PEN_OFFSET_MM / 10.0;    // 13.5 cm
  if (rayon < D_PEN - 1.0) {                   // < ~12.5 cm
    // Plus petit que le deport stylo : geometriquement impossible en cercle propre.
    server.send(400, "text/plain",
      "Rayon < 12.5 cm impossible : le stylo est a 13.5 cm en avant de l'axe. "
      "Rapprocher le stylo de l'axe des roues pour des cercles plus petits.");
    return;
  }
  if (rayon <= 15.0) {
    // Zone du plus petit cercle propre -> PIVOT sur place (trace ~13.5 cm).
    server.send(200, "text/plain", "Cercle PIVOT (~13.5 cm) lance");
    executerCerclePivot();
    return;
  }

  // Rayon > 15 cm : methode lisse (differentiel) avec calibration du rayon.
  server.send(200, "text/plain", "Cercle r=" + String(rayon, 1) + " cm lance");
  executerCercle(rayon);                       // bloquant ; calibre + clampe si besoin
}

void handleSequence3() {
  if (sequenceEnCours) {
    server.send(409, "text/plain", "Séquence déjà en cours");
    return;
  }
  server.send(200, "text/plain", "Fleche Nord lancee (calib + pivot + dessin)");
  drawNorthArrow();
}

void handleSeq3Log() {
  server.send(200, "text/plain; charset=utf-8", seq3Log);
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
  server.on("/sequence2", handleSequence2);
  server.on("/sequence3", handleSequence3);
  server.on("/seq3log",   handleSeq3Log);
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

  // Cap magnétique en temps réel (utilise les offsets si calibration faite)
  if (magCalibrated) {
    float cx = mx - magOffsetX;
    float cy = my - magOffsetY;
    headingMag = atan2(cy, cx) * (180.0f / PI) + MOUNTING_OFFSET;
    while (headingMag >  180.0f) headingMag -= 360.0f;
    while (headingMag < -180.0f) headingMag += 360.0f;
  }

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
