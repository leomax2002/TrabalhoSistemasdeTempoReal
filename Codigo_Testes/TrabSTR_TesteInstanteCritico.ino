//////////Válvula Reguladora de Pressão///////////

//Leonardo Maximo Silva - 200022172
//Marco Antonio Nobre Rangel - 21104350
//Sabrina Carvalho Neves - 170113973

//////////////////////////////////////////////////

//Esse Código testa o Instante Crítico para as tarefas no momento em que se inicia o Código

//Inclui as Bibliotecas
#include <Arduino_FreeRTOS.h> //Biblioteca Padrão FreeRTOS
#include <semphr.h> //Biblioteca para Semáforos e Mutexes
#include <LiquidCrystal.h> //Biblioteca para o Display LCD

//Declaração de Variáveis Globais
#define sensorPin 8
#define buttonPin 2
#define relayPin 13

LiquidCrystal lcd(12, 11, 5, 4, 3, 7);

SemaphoreHandle_t mutex;
SemaphoreHandle_t interruptSemaphore;

TaskHandle_t TaskOpenValve_Handler;
TaskHandle_t TaskCloseValve_Handler;


int sensorState = 0;
int relayState = 0;

void readSensor( void *pvParameters );
void showScreen( void *pvParameters );
void buttonInterrupt( void *pvParameters );
void openValve( void *pvParameters );
void closeValve( void *pvParameters );

void setup() {
  //Código Genérico para tentar levar os aspectos de hardware do Arduino, como Memória Cache, ao pior caso
  int val_cod = 0;
  for(int i = 0; i < 100; i++){
    if(val_cod == 50){
      val_cod = val_cond*3;
      }
    else{
      val_cod = val_cond*2;
      }
    }
  pinMode(0,OUTPUT);
  digitalWrite(0,HIGH);
  digitalWrite(0,LOW);
  pinMode(1,OUTPUT);
  digitalWrite(1,HIGH);
  digitalWrite(1,LOW);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  digitalWrite(2,LOW);
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
  digitalWrite(3,LOW);
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  digitalWrite(4,LOW);
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  digitalWrite(5,LOW);
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  digitalWrite(6,LOW);
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);
  digitalWrite(7,LOW);
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  digitalWrite(8,LOW);
  pinMode(9,OUTPUT);
  digitalWrite(9,HIGH);
  digitalWrite(9,LOW);
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  digitalWrite(10,LOW);
  pinMode(11,OUTPUT);
  digitalWrite(11,HIGH);
  digitalWrite(11,LOW);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH);
  digitalWrite(12,LOW);
  pinMode(13,INPUT);
  digitalRead(13);
  pinMode(A0,OUTPUT);
  analogWrite(A0,HIGH);
  analogWrite(A0,LOW);
  pinMode(A1,OUTPUT);
  analogWrite(A1,HIGH);
  analogWrite(A1,LOW);
  pinMode(A2,OUTPUT);
  analogWrite(A2,HIGH);
  analogWrite(A2,LOW);
  pinMode(A3,OUTPUT);
  analogWrite(A3,HIGH);
  analogWrite(A3,LOW);
  pinMode(A4,OUTPUT);
  analogWrite(A4,HIGH);
  analogWrite(A4,LOW);
  pinMode(A5,OUTPUT);
  analogWrite(A5,HIGH);
  analogWrite(A5,LOW);
  pinMode(A6,OUTPUT);
  analogWrite(A6,HIGH);
  analogWrite(A6,LOW);
  pinMode(A7,OUTPUT);
  analogWrite(A7,HIGH);
  analogWrite(A7,LOW);
  /////////////////
  //Inicializa tela e define pinos
  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT_PULLUP); //Trocar só por Input ao usar Sensor
  pinMode(relayPin, OUTPUT);
  
  while (!Serial) {
    ; // Espera Conexão da Porta Serial. Necessário para alguns Arduinos, como o Leonardo e o Yun
  }  
  //Cria Tarefas
  xTaskCreate(
    readSensor
    ,  "DigitalRead"  // Nome
    ,  128  // Tamanho da Stack
    ,  NULL //Parâmetros da Tarefa
    ,  1  // Prioridade da Tarefa, sendo que, quanto maior o valor, maior a prioridade
    ,  NULL ); //Handle da Tarefa. É utilizado para suspendê-la ou acordá-la
  
  xTaskCreate(
    buttonInterrupt
    ,  "InterruptButton"
    ,  128  
    ,  NULL 
    ,  4 
    ,  NULL ); 

  xTaskCreate(
    showScreen
    ,  "Screen" 
    ,  128  
    ,  NULL 
    ,  0  
    ,  NULL ); 

      xTaskCreate(
    openValve
    ,  "openValve" 
    ,  128 
    ,  NULL 
    ,  3  
    ,  &TaskOpenValve_Handler ); 

    xTaskCreate(
    closeValve
    ,  "closeValve"
    ,  128  
    ,  NULL 
    ,  2  
    ,  &TaskCloseValve_Handler ); 

//Cria o Mutex que protege os valores lidos em sensorPin, e os valores escritos em relayPin
  mutex = xSemaphoreCreateMutex();
  if(mutex != NULL)
    Serial.println("Mutex Created");
//Cria o Semáforo Binário que sinaliza uma interrupção
  interruptSemaphore = xSemaphoreCreateBinary();
  if(interruptSemaphore != NULL){
    attachInterrupt(digitalPinToInterrupt(buttonPin),interruptHandler,LOW);
    
  }

  //O Schedular, agora, é responsável por escalonar o programa e garantir sua execução
}

void loop()
{
  // Loop está vazio. Tudo é realizado pelas Tarefas e pelo Schedular
}

/*----------------------------------------------------*/
/*---------------------- Tarefas ---------------------*/
/*----------------------------------------------------*/

//Concede o Semáforo para uma interrupção de Hardware (botão)
void interruptHandler(){
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
  }

//Tarefa esporádica executada pela interrupção de um botão
void buttonInterrupt( void *pvParameters) 
{

(void) pvParameters;

  const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {


   if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) { //Espera até ser recebida permissão para continuar
      if(sensorState == 1 && xSemaphoreTake(mutex,1) == pdTRUE){ //Escreve se a pressão está elevada (1) e se é fornecido o semáforo necessário
        relayState = 1;
        digitalWrite(relayPin,LOW);
        xSemaphoreGive(mutex); //Libera o Mutex
      }
      lcd.setCursor(1, 0);
      lcd.print("Button Pressed");
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms); 
  }
}

//Tarefa que lê o Sensor
void readSensor( void *pvParameters) 
{
  (void) pvParameters; //Parâmetros utilizados pelo Mutex
  const TickType_t xDelay500ms = pdMS_TO_TICKS( 500 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) 
  {
    //Protege a leitura do Sensor por Mutex
    if(xSemaphoreTake(mutex,1) == pdTRUE){
      sensorState = digitalRead(sensorPin); //Lê o Sensor
      xSemaphoreGive(mutex);
    }
    //Caso a pressão esteja acima do limiar estabelecido e o relé desativado, acorde a Tarefa que aciona o relé
    //Caso a pressão esteja abaixo do limiar estabelecido e o relé ativado, acorde a Tarefa que desaciona o relé
    if(sensorState == 1 && relayState == 0)
      vTaskResume(TaskOpenValve_Handler); //Acorda a Tarefa openValve
    else if(sensorState == 0 && relayState == 1)
      vTaskResume(TaskCloseValve_Handler); //Acorda a Tarefa closeValve
    vTaskDelayUntil(&xLastWakeTime, xDelay500ms);  // Tempo entre as ativações da Tarefa Periódica
  }
}
//Tarefa que mostra a Tela
void showScreen( void *pvParameters __attribute__((unused)) ) 
{

  const TickType_t xDelay1000ms = pdMS_TO_TICKS( 1000 );
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    //Limpa as informações da Tela LCD antes da escrita
    lcd.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    lcd.setCursor(0, 0);
    //Envia o texto entre aspas para o LCD
    if(sensorState == 0){
    lcd.print("Pressao Normal");
    digitalWrite(relayPin,HIGH);
    }
    else{
      lcd.print("Pressao de Risco");
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay1000ms); 
  }
}

//Tarefa que abre a válvula
void openValve( void *pvParameters __attribute__((unused)) )
{
    const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    //Protege por Mutex o acionamento do relé
    if(xSemaphoreTake(mutex,1) == pdTRUE){
    relayState = 1;
    digitalWrite(relayPin,LOW);
    xSemaphoreGive(mutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms); //Garante que a Tarefa seja esporádica e só chegue a cada período
    vTaskSuspend(TaskOpenValve_Handler); //Suspende a Tarefa por meio do TaskHandler

  }
}
//Tarefa que fecha a válvula
void closeValve( void *pvParameters __attribute__((unused)) ) 
{
    const TickType_t xDelay2000ms = pdMS_TO_TICKS( 2000 );
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    //Protege o desacionamento do relé por Mutex
    if(xSemaphoreTake(mutex,1) == pdTRUE){
    relayState = 0;
    digitalWrite(relayPin,HIGH);
    xSemaphoreGive(mutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay2000ms); //Garante que a Tarefa seja esporádica
    vTaskSuspend(TaskCloseValve_Handler); //Suspende a Tarefa, pois não está sendo utilizada
    
  }
}
