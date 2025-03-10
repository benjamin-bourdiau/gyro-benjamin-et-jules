#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// Définition des broches pour les moteurs
#define BRAKE_RIGHT  17
#define DIR_RIGHT    16
#define PWM_RIGHT     4
#define BRAKE_LEFT   26
#define DIR_LEFT     13
#define PWM_LEFT     27

// Définition des broches pour les batteries
#define BATT1_PIN  35  
#define BATT2_PIN  32  

// Configuration du PWM
#define PWM_FREQ  1000  
#define PWM_BITS  8     
#define PWM_CHANNEL_RIGHT  0
#define PWM_CHANNEL_LEFT   1

// Déclaration du MPU6050 
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Variables pour le filtrage du MPU6050
char FlagCalcul = 0;
float Te = 10;   // période d'échantillonnage en ms
float Tau = 1000; // constante de temps du filtre en ms
int v = 1; // Valeur de la consigne
float tG, tGF, tW, tWF, t;
float A, B;  // Coefficients du filtre
float Kp = 150;
float Kd = 0; //entre 1000 et 1800

// Prototypes des tâches
void tacheMoteurs(void *param);
void tacheBatteries(void *param);
void tacheMPU6050(void *param);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 

  // Initialisation des moteurs
  pinMode(BRAKE_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(BRAKE_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);

  digitalWrite(BRAKE_RIGHT, LOW);
  digitalWrite(BRAKE_LEFT, LOW);

  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PWM_RIGHT, PWM_CHANNEL_RIGHT);
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PWM_LEFT, PWM_CHANNEL_LEFT);

  // Initialisation de l'ADC pour les batteries
  analogReadResolution(12);

  // Initialisation du MPU6050
  Serial.println("Initialisation MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Échec de détection du MPU6050 !");
  } else {
    Serial.println("MPU6050 détecté !");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Calcul des coefficients du filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;

  // Création des tâches
  xTaskCreate(tacheMoteurs, "Tâche Moteurs", 10000, NULL, 1, NULL);
  xTaskCreate(tacheBatteries, "Tâche Batteries", 10000, NULL, 1, NULL);
  xTaskCreate(tacheMPU6050, "Tâche MPU6050", 10000, NULL, 10, NULL);
}

// Fonction pour contrôler le moteur droit
void setMotorRight(int speed) {
  digitalWrite(DIR_RIGHT, (speed < 0) ? LOW : HIGH);  
  ledcWrite(PWM_CHANNEL_RIGHT, constrain(abs(speed), 0, 255));
}

// Fonction pour contrôler le moteur gauche (inversion automatique)
void setMotorLeft(int speed) {
  digitalWrite(DIR_LEFT, (speed < 0) ? HIGH : LOW);  
  ledcWrite(PWM_CHANNEL_LEFT, constrain(abs(speed), 0, 255));
}

// Fonction pour activer ou désactiver le freinage
void stopMotor(int brakePin) { digitalWrite(brakePin, HIGH); }
void releaseMotor(int brakePin) { digitalWrite(brakePin, LOW); }

// ✅ Tâche 1 : Gestion des moteurs
void tacheMoteurs(void *param) {
  while (1) {
    if(FlagCalcul == 1) {
      /*if (t < 0 ) {
        setMotorRight(-t*425*v);
        setMotorLeft(-t*425*v);
      } else if (t > 0) {
        setMotorRight(-t*425*v);
        setMotorLeft(-t*425*v);
      } else {
        setMotorRight(0);
        setMotorLeft(0);
      }*/
      float erreur = 0-t;
      float erreur_precedente = 0;

      float cmd = Kp * erreur + Kd * (erreur - erreur_precedente);
      erreur_precedente = erreur;
      if(cmd >0 && cmd < 30){
        cmd = 30;
      }
      else if(cmd < 0 && cmd > -30){
        cmd = -30;
      }
      setMotorRight(cmd);
      setMotorLeft(cmd);

      FlagCalcul = 0;
    }
    delay(10);
  }
}

// Lecture des tensions des batteries
void tacheBatteries(void *param) {
  while (1) {
    
    float ADC_Batt1 = analogRead(BATT1_PIN) * (3.3 / 4095.0);
    float ADC_Batt2 = analogRead(BATT2_PIN) * (3.3 / 4095.0);
    
    float Tension_Batt1 = ADC_Batt1 * 2.5;
    float Tension_Batt2 = (ADC_Batt2 * 4.9) - Tension_Batt1;

    delay(1000);
/*
    Serial.print("Tension Batterie 1 : ");
    Serial.print(Tension_Batt1, 2);
    Serial.println(" V");

    Serial.print("Tension Batterie 2 : ");
    Serial.print(Tension_Batt2, 2);
    Serial.println(" V");

    Serial.println("----------------------");
*/
  }
}

// Acquisition et filtrage du MPU6050
void tacheMPU6050(void *param) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    mpu.getEvent(&a, &g, &temp);
    tG = atan2(a.acceleration.y, a.acceleration.x);
    tGF = A * tG + B * tGF;
    tW = Tau / 1000 + g.gyro.z;
    tWF = - A * tW + B * tWF;
    t = tGF + tWF;
    t += 1; 
    FlagCalcul = 1;

    Serial.print(tG);
    Serial.print(" ");
    Serial.print(tGF);
    Serial.print(" ");
    Serial.print(tWF);
    Serial.print(" ");
    Serial.println(t);
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

// Fonction de réception de commandes UART
void reception(char ch) {
  static String chaine = "";
  String commande, valeur;
  int index, length;

  if ((ch == 13) || (ch == 10)) {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1) {
      commande = chaine;
      valeur = "";
    } else {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau") {
      Tau = valeur.toFloat();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te") {
      Te = valeur.toInt();
    }
    if (commande == "Kp") {
      Kp = valeur.toFloat();
    }
    chaine = "";
  } else {
    chaine += ch;
  }
}

// Lecture des données UART
void serialEvent() {
  while (Serial.available() > 0) {
    reception(Serial.read());
  }
}

void loop() {
//  vTaskDelete(NULL); // Supprime loop() car tout est géré dans les tâches
}
//test