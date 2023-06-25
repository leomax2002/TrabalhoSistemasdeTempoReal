#include <Arduino_FreeRTOS.h>
#include <semphr.h>  
#include <LiquidCrystal.h>

#define sensorPin 8
#define buttonPin 2
#define relayPin 13

LiquidCrystal lcd(12, 11, 5, 4, 3, 7);

SemaphoreHandle_t mutex;
SemaphoreHandle_t interruptSemaphore;

TaskHandle_t TaskOpenValve_Handler;
TaskHandle_t TaskCloseValve_Handler;


int buttonState = 0;
int relayState = 0;

void readSensor( void *pvParameters );
void showScreen( void *pvParameters );
void buttonInterrupt( void *pvParameters );
void openValve( void *pvParameters );
void closeValve( void *pvParameters );

void setup() {

  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT_PULLUP); //Trocar só por Input ao usar Sensor
  pinMode(relayPin, OUTPUT);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }  
  xTaskCreate(
    readSensor
    ,  "DigitalRead"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle
  
  xTaskCreate(
    buttonInterrupt
    ,  "InterruptButton" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  4 // Priority
    ,  NULL ); //Task Handle

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.

  xTaskCreate(
    showScreen
    ,  "Screen" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  0  // Priority
    ,  NULL ); //Task Handle

      xTaskCreate(
    openValve
    ,  "openValve" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  3  // Priority
    ,  &TaskOpenValve_Handler ); //Task Handle

    xTaskCreate(
    closeValve
    ,  "closeValve" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  2  // Priority
    ,  &TaskCloseValve_Handler ); //Task Handle

  mutex = xSemaphoreCreateMutex();
  if(mutex != NULL)
    Serial.println("Mutex Created");
  interruptSemaphore = xSemaphoreCreateBinary();
  if(interruptSemaphore != NULL){
    attachInterrupt(digitalPinToInterrupt(buttonPin),interruptHandler,LOW);
    
  }

  
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void interruptHandler(){
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
  }


void buttonInterrupt( void *pvParameters) 
{

(void) pvParameters;

  // make the pushbutton's pin an input:
  const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {


   if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) {
      if(buttonState == 1 && xSemaphoreTake(mutex,1) == pdTRUE){
        relayState = 1;
        digitalWrite(relayPin,LOW);
        Serial.println("Relay ON");
        xSemaphoreGive(mutex);
      }

    }
    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms); 
  }
}

void readSensor( void *pvParameters)  // This is a Task.
{
  (void) pvParameters;
  // make the pushbutton's pin an input:
  const TickType_t xDelay500ms = pdMS_TO_TICKS( 500 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:

    if(xSemaphoreTake(mutex,1) == pdTRUE){
      buttonState = digitalRead(sensorPin);
      xSemaphoreGive(mutex);
    }
    if(buttonState == 1 && relayState == 0)
      vTaskResume(TaskOpenValve_Handler);
    else if(buttonState == 0 && relayState == 1)
      vTaskResume(TaskCloseValve_Handler);
    vTaskDelayUntil(&xLastWakeTime, xDelay500ms);  // one tick delay (15ms) in between reads for stability
  }
}

void showScreen( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  const TickType_t xDelay1000ms = pdMS_TO_TICKS( 1000 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    if(buttonState == 0){
    lcd.print("Pressao Normal");
    Serial.println("Pressao Normal");
    digitalWrite(relayPin,HIGH);
    }
    else{
      lcd.print("Pressao de Risco");
      Serial.println("Pressao de Risco");
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay1000ms);  // one tick delay (15ms) in between reads for stability
  }
}


void openValve( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // read the input on analog pin 0:
    if(xSemaphoreTake(mutex,1) == pdTRUE){
    relayState = 1;
    Serial.println("Valve ON");
    digitalWrite(relayPin,LOW);
    xSemaphoreGive(mutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms);
    vTaskSuspend(TaskOpenValve_Handler);
      // one tick delay (15ms) in between reads for stability
  }
}

void closeValve( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // read the input on analog pin 0:
    if(xSemaphoreTake(mutex,1) == pdTRUE){
    relayState = 0;
    Serial.println("Valve OFF");
    digitalWrite(relayPin,HIGH);
    xSemaphoreGive(mutex);
    }
    lcd.clear();

    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms);
    vTaskSuspend(TaskCloseValve_Handler);
    
  }
}
