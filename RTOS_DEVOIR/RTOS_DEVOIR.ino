// Inclusions du noyau RTOS
#include <Arduino_FreeRTOS.h> // noyau
#include "task.h"     // taches//
#include "timers.h"  // timers
#include "queue.h"    // files
#include "semphr.h"   // sémaphores

#define but1 3
#define but2 4
#define potentio 0

//definition de la structure
typedef struct 
  {
    int analog; 
    int num; 
    double tempoMS;
  }Capteurs;
   
  // Priorités
  #define PRIORITE_TACHE_1  (tskIDLE_PRIORITY+1) 
  #define PRIORITE_TACHE_2  (tskIDLE_PRIORITY+1) 
  #define PRIORITE_TACHE_3  (tskIDLE_PRIORITY+1)
  #define PRIORITE_TACHE_4  (tskIDLE_PRIORITY+1) 
  #define PRIORITE_TACHE_5  (tskIDLE_PRIORITY+3) // Plus importante

QueueHandle_t 1to3; // file de communication entre la tâche 1 et 2 
QueueHandle_t 2to3; // file de communication entre la tâche 1 et 3
QueueHandle_t 3to4; // file de communication entre la tâche 3 et 4
QueueHandle_t 4to5; // file de communication entre la tâche 4 et 5

SemaphoreHandle_t sem; // sémaphore pour 4 et 5

//Taches
void tache1( void *pvParameters);
void tache2( void *pvParameters);
void tache3( void *pvParameters);
void tache4( void *pvParameters);
void tache5( void *pvParameters);


void setup() 
{
  Serial.begin(9600);

  // Setup des pins en input ( lecture / reception )
  pinMode(potentio, INPUT);
  pinMode(but1, INPUT);
  pinMode (but2, INPUT);

  // Création des 5 tâches
  xTaskCreate(tache1 ,"Task1",1000,NULL, PRIORITE_TACHE_1,NULL);
  xTaskCreate(tache2 ,"Task2",1000,NULL, PRIORITE_TACHE_2,NULL);
  xTaskCreate(tache3 ,"Task3",1000,NULL, PRIORITE_TACHE_3,NULL);
  xTaskCreate(tache4 ,"Task4",1000,NULL, PRIORITE_TACHE_4,NULL);
  xTaskCreate(tache5 ,"Task5",1000,NULL, PRIORITE_TACHE_5,NULL);

  1to3=xQueueCreate(2,sizeof(int)); // création de la file entre tâche 1 et 3
  2to3=xQueueCreate(2,sizeof(int)); // création de la file entre tâche 2 et 3
  3to4=xQueueCreate(2,sizeof(Capteurs));// création de la file entre tâche 3 et 4
  4to5=xQueueCreate(2,sizeof(Capteurs));// création de la file entre tâche 4 et 5
  sem=xSemaphoreCreateBinary();// création du sémaphore

  // Démarrage de l’ordonnanceur
  vTaskStartScheduler(); 
}

void loop() // On utilise pas la LOOP
{
}

void tache1( void *pvParameters)
{
  while(1)
  {
    int analog= analogRead(potentio);
    xQueueSend(1to3,&analog, portMAX_DELAY);//envoi de la valeur analogique dans la queue
  }
}

void tache2( void *pvParameters)
{
  while(1)
  {
    int num = digitalRead(but1) + digitalRead(but2);
    xQueueSend(2to3,&num, portMAX_DELAY); //envoi de la valeur numérique dans la queue
  }
}

void tache3( void *pvParameters)
{
  int reception_analog;
  int reception_num;
  Capteurs capteurs;
  while(1)
  {
    unsigned long current_time = millis();//on récupére la valeur de la fonction millis
    if(xQueueReceive(1to3,&reception_analog,portMAX_DELAY)&&xQueueReceive(2to3,&reception_num,portMAX_DELAY))
    {
      capteurs = {reception_analog, reception_num, current_time};//stockage des valeurs
    }
    xQueueSend(3to4,&capteurs, portMAX_DELAY);//Envoie de valeurs
  }
  
}

void tache4( void *pvParameters)
{
   Capteurs reception_structure;
   while(1)
   {
     if(xQueueReceive(3to4,&reception_structure,portMAX_DELAY))//reception de la structure
     {
        Serial.println("Tâche 5");
        Serial.print("Valeur analogique: ");
        Serial.println(reception_structure.analog);
        
        Serial.print("Valeur numérique: ");
        Serial.println(reception_structure.num);
        
        Serial.print("Temps ms: ");
        Serial.println(reception_structure.tempoMS);
        vTaskDelay(30);
     }
     xQueueSend(4to5,&reception_structure,portMAX_DELAY);// envoi de la strucutre a la tache 5
     xSemaphoreGive(sem);//ouverture du sémaphore pour la tâche 4
   }
}

void tache5( void *pvParameters)
{
   valeursCapteurs reception_structure2;
   while(1)
   {
     if(xSemaphoreTake(sem,portMAX_DELAY))//attente sémaphore 
     {
        if(xQueueReceive(4to5,&reception_structure2,portMAX_DELAY))//reception de la structure
        { 
            reception_structure2.tempoMS = (reception_structure2.tempoMS*pow(10,-3))/60; // conversion ms min
            Serial.println("Tâche 5");
            Serial.print("Valeur analogique: ");
            Serial.println(reception_structure2.analog);
            
            Serial.print("Valeur numérique: ");
            Serial.println(reception_structure2.num);
            
            Serial.print("Temps min: ");
            Serial.println(reception_structure2.tempoMS);
            
            vTaskDelay(30);
        }  
     }
   }
}
