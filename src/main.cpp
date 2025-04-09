#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// Définition des broches pour les moteurs
#define BRAKE_RIGHT 17
#define DIR_RIGHT 16
#define PWM_RIGHT 4
#define BRAKE_LEFT 26
#define DIR_LEFT 13
#define PWM_LEFT 27

// Définition des broches pour les batteries
#define BATT1_PIN 35
#define BATT2_PIN 32

// Configuration du PWM
#define PWM_FREQ 20000
#define PWM_BITS 10
#define PWM_CHANNEL_RIGHT 0
#define PWM_CHANNEL_LEFT 1

//COnfiguration des Encoders
#define CLK_G 19
#define DT_G 18
#define CLK_D 25
#define DT_D 33

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
ESP32Encoder encoderG, encoderD;
BluetoothSerial SerialBT;
TaskHandle_t handleMPU6050 = NULL;

// Variables pour le filtrage du MPU6050
char FlagCalcul = 0;
float Te = 5;   // période d'échantillonnage en ms
float Tau = 200, tau_vitess = 206.5, tau_deriv = 71, tau_bat; // constante de temps du filtre en ms
float tG, tGF, tW, tWF, t, tetaCons; //Variables position
float consVit, obsVit, P, D, DF, I, vitD, vitG, obsVitF; // Variables vitesse
float A, B, A_v, B_v, A_d, B_d, A_bat, B_bat; // Coefficients du filtre
float tetaEq = 0.0308768; 
float Kp_ang = 4190; 
float Kd_ang = 8.05; 
float Kp_v = 0.0000846; 
float Kd_v = 0.00000159;
float Ki_v = 0.0000031;
float cf = 35.76; 
float cmd;
float erreurAng, erreurVit, erreurVitPrec;
int countG, countD, countDPrec = 0, countGPrec = 0; // Compteurs des encodeurs
float ADC_Batt1, ADC_Batt2, Tension_Batt1, Tension_Batt2, T_Bat1F, T_Bat2F;

// Prototypes des tâches
void gestionMoteurs(void);
void tacheBatteries(void *param);
void tacheMPU6050(void *param);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

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

  encoderD.attachFullQuad(DT_D, CLK_D);
  encoderD.setCount(0);
  encoderG.attachFullQuad(DT_G, CLK_G);
  encoderG.setCount(0);

  SerialBT.begin("gyro_J&B");

  // Initialisation de l'ADC pour les batteries
  analogReadResolution(12);

  // Initialisation du MPU6050
  Serial.println("Initialisation MPU6050...");
  if (!mpu.begin())
  {
    Serial.println("Échec de détection du MPU6050 !");
  }
  else
  {
    Serial.println("MPU6050 détecté !");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  }

  // Calcul des coefficients du filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;

  A_v = 1 / (1 + tau_vitess / Te);
  B_v = tau_vitess / Te * A_v;

  A_d = 1 / (1 + tau_deriv / Te);
  B_d = tau_deriv / Te * A_v;

  A_bat = 1 / (1 + tau_bat / Te);
  B_bat = tau_bat / Te * A_v;

  // Création des tâches

  xTaskCreate(tacheBatteries, "Tâche Batteries", 10000, NULL, 1, NULL);
  xTaskCreate(tacheMPU6050, "Tâche MPU6050", 10000, NULL, 10, &handleMPU6050);
}

// Fonction pour contrôler le moteur droit
void setMotorRight(int speed)
{
  digitalWrite(DIR_RIGHT, (speed < 0) ? LOW : HIGH);
  ledcWrite(PWM_CHANNEL_RIGHT, constrain(abs(speed), 0, 950));
}

// Fonction pour contrôler le moteur gauche (inversion automatique)
void setMotorLeft(int speed)
{
  digitalWrite(DIR_LEFT, (speed < 0) ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL_LEFT, constrain(abs(speed), 0, 950));
}

// Fonction pour activer ou désactiver le freinage
void stopMotor(int brakePin) { digitalWrite(brakePin, HIGH); }
void releaseMotor(int brakePin) { digitalWrite(brakePin, LOW); }

float getVitesse(void)
{
  countG = encoderG.getCount();
  countD = encoderD.getCount();
  vitG = (countG - countGPrec) / (Te/1000);
  vitD = (countD - countDPrec) / (Te/1000);
  countGPrec = countG;
  countDPrec = countD;
  obsVit = (vitG - vitD) / 2; //moyenne des vitesses des deux roues
  obsVitF = (A_v * obsVit + B_v * obsVitF);
  return obsVitF;
}

// Gestion des moteurs
void gestionMoteurs(void)
{
/***********************************************/
// asservissement en vitesse
/***********************************************/

  erreurVit = consVit - getVitesse();
  P = Kp_v * erreurVit;
  D = Kd_v * ((erreurVit - erreurVitPrec) / (Te/1000));
  DF = A_d * D + B_d * DF;
  I += Ki_v * erreurVit * (Te/1000);
  if(I > 0.05) I = 0.05;
  if(I < -0.05) I = -0.05; 
  erreurVitPrec = erreurVit;
  tetaCons = P + DF + I;

  if (tetaCons > 0.08) tetaCons = 0.08;
  else if (tetaCons < -0.08) tetaCons = -0.08;

  tetaCons += tetaEq; // Ajout de l'angle d'équilibre
  
/***********************************************/
// asservissement en position angulaire
/***********************************************/

  erreurAng = tetaCons - t;
  cmd = Kp_ang * erreurAng + Kd_ang * g.gyro.z;

  // commande moteurs
  if (cmd > 0) cmd += cf;
  else if (cmd < 0) cmd -= cf;
  
  //saturation de la commande
  if (cmd > 950) cmd = 950;
  if (cmd < -950) cmd = -950;
  setMotorRight(cmd);
  setMotorLeft(cmd);
}

/***********************************************/
// Lecture des tensions des batteries
/***********************************************/

void tacheBatteries(void *param)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while (1)
  {
    ADC_Batt1 = analogRead(BATT1_PIN) * (3.3 / 4095.0);
    ADC_Batt2 = analogRead(BATT2_PIN) * (3.3 / 4095.0);

    Tension_Batt1 = ADC_Batt1 * 2.5;
    Tension_Batt2 = (ADC_Batt2 * 4.9) - Tension_Batt1;

    T_Bat1F = A_bat * Tension_Batt1 + B_bat * T_Bat1F;
    T_Bat2F = A_bat * Tension_Batt2 + B_bat * T_Bat2F;

    if (T_Bat1F < 6.5 || T_Bat2F < 6.5)
    {
      if (handleMPU6050 != NULL)  // Vérifier si la tâche existe
      {
        vTaskDelete(handleMPU6050);  // Supprimer la tâche
        handleMPU6050 = NULL;        // Nettoyer le handle
        Serial.println("Tâche MPU6050 arrêtée !");
        setMotorRight(0);
        setMotorLeft(0);
      }
    }
    else
    {
      if (handleMPU6050 == NULL)  // Vérifier si la tâche n'existe pas
      {
        xTaskCreate(tacheMPU6050, "Tâche MPU6050", 10000, NULL, 10, &handleMPU6050);
        Serial.println("Tâche MPU6050 redémarrée !");
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}


// Acquisition et filtrage du MPU6050
void tacheMPU6050(void *param)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    mpu.getEvent(&a, &g, &temp);
    tG = atan2(a.acceleration.y, a.acceleration.x);
    tGF = A * tG + B * tGF;
    tW = -Tau / 1000 * g.gyro.z;
    tWF = A * tW + B * tWF;
    t = tGF + tWF;

    gestionMoteurs();
    FlagCalcul = 1;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

// Fonction de réception de commandes UART
void reception(char ch)
{
  static String chaine = "";
  String commande, valeur;
  int index, length;

  if ((ch == 13) || (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te") Te = valeur.toInt();
    if (commande == "Kp_ang") Kp_ang = valeur.toFloat();
    if (commande == "Kd_ang") Kd_ang = valeur.toFloat();
    if (commande == "Kp_v") Kp_v = valeur.toFloat();
    if (commande == "Kd_v") Kd_v = valeur.toFloat();
    if (commande == "Ki_v") Ki_v = valeur.toFloat();
    if (commande == "cf") cf = valeur.toFloat();
    if (commande == "tetaEq") tetaEq = valeur.toFloat();
    chaine = "";
  }
  else chaine += ch;
}

//fonction de réception des données bluetooth
void receptionBT(char ch)
{
    static String chaine = "";
    String commande, valeur;
    int index, length;

    if ((ch == 13) || (ch == 10)) // Fin de commande
    {
        index = chaine.indexOf(' ');
        length = chaine.length();
        if (index == -1)
        {
            commande = chaine;
            valeur = "";
        }
        else
        {
            commande = chaine.substring(0, index);
            valeur = chaine.substring(index + 1, length);
        }

        if (commande == "Tau")
        {
            Tau = valeur.toFloat();
            A = 1 / (1 + Tau / Te);
            B = Tau / Te * A;
        }
        if (commande == "Te") Te = valeur.toInt();
        if (commande == "Kp_ang") Kp_ang = valeur.toFloat();
        if (commande == "Kd_ang") Kd_ang = valeur.toFloat();
        if (commande == "Kp_v") Kp_v = valeur.toFloat();
        if (commande == "Kd_v") Kd_v = valeur.toFloat();
        if (commande == "Ki_v") Ki_v = valeur.toFloat();
        if (commande == "cf") cf = valeur.toFloat();
        if (commande == "tetaEq") tetaEq = valeur.toFloat();
        if (commande == "tau_vit") tau_vitess = valeur.toFloat();
        if (commande == "tau_deriv") tau_deriv = valeur.toFloat();
        if (commande == "tau_bat") tau_bat = valeur.toFloat();
        
        Serial.println("Commande reçue : " + commande + " -> " + valeur);
        chaine = "";
    }
    else chaine += ch;
}

// Lecture des données UART
void serialEvent()
{
  while (Serial.available() > 0) reception(Serial.read());
}

//lecture des données bluetooth
void serialBTEvent()
{
    while (SerialBT.available() > 0) receptionBT(SerialBT.read());
}

void loop()
{
  serialBTEvent();
  if (FlagCalcul == 1)
  {
    SerialBT.printf("%f ", T_Bat1F);
    SerialBT.printf("%f ", T_Bat2F);
    SerialBT.printf("%f ", Tension_Batt1);
    SerialBT.printf("%f\n", Tension_Batt2);
    // Serial.print(" ");
    // Serial.println(obsVitF);
    // SerialBT.print(Kp_v);
    // Serial.print(" ");
    // SerialBT.print(Kd_v);
    // Serial.print(" ");
    // SerialBT.println(Ki_v);
    FlagCalcul = 0;
  }
}

