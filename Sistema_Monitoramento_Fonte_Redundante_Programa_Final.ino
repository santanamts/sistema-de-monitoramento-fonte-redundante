//Ultima atualizacao : 12/12/2022 20:52 SÃO PAULO BRASIL 


/********************** PINOS *****************************************

SENSOR DE CORRENTE ENTRADA AC = ADC5 (G33) OK  
SENSOR DE TENSAO ENTRADA AC = ADC4 (G32)  OK
SENSOR DE TENSAO SAIDA DC_1 = ADC0 (SP)  OK USAR DIV TENSAO RESISTOR ok
SENSOR DE TENSAO SAIDA DC_2 = ADC6 (G34) OK USAR DIV TENSAO RESISTOR ok
SENSOR DE CORRENTE SAIDA DC_1 = ADC3 (sn) OK USAR DIV TENSAO RESISTOR ok
SENSOR DE CORRENTE SAIDA DC_2 = ADC7 (G35) OK USAR DIV TENSAO RESISTOR ok

SENSOR DE TEMPERATURA_1 = ADC10 (G4) OK
SENSOR DE TEMPERATURA_2 = ADC11 (G0) OK

RELÉ FONTE 1 REDUNDANCIA = gpio12 (g12) OK USAR TRANSISTOR 2222A ok
BUZZER = G17 OK ok

led fonte 1 g14 ok
led fonte 2 g27 ok

*************************************************************************/
/**************************** ENTRADAS E SAIDAS ****************************/

int SELECAO_FONTE = 1; //Inicia selecao como fonte 1

/**************************** BIBLIOTECAS ****************************/

#include <Adafruit_Sensor.h> //Biblioteca para sensor de Temperatura E Umidade
#include "ThingSpeak.h" //Inclui a biblioteca para o ThingSpeak 
#include "WiFi.h" //Inclui a biblioteca do WiFi
#include "DHT.h" //Inclui a biblioteca do sensor de temperatura DHT11
#include "EmonLib.h" //Include Emon Library
#include <Robojax_AllegroACS_Current_Sensor.h> //Biblioteca sensor de corrente DC SAIDA

/**************************** PARAMETRIZAÇÃO **********************************/

float Voutput_maximo = 12.6; //Valor máximo de tensão de saída da fonte em V 
float Voutput_minimo = 10.5; //Valor mínimo de tensão de saída da fonte em V 

float Ioutput_maxima = 1; //Corrente máxima de saída da fonte em A 

float Iinput_maxima = 0.3; //Corrente máxima de entrada da fonte em A 

float temperatura_maxima = 27; //Define 27 como temperatura máxima


/**************************** Variaveis para confg. do sensor TENSÃO DC FONTE 1 ****************************/
int VDC_OUT = A0; //Pino para leitura de tensao de saida FONTE 1
int pin_tensao_entrada_dc = VDC_OUT; //Configura pino ADC0 (SP) como entrada do sensor VDC, vide configuracao de pinos 
float amostragem = 0; //Variavel amostragem que irá receber o valor exato do pino do sensor
float valor_medio=0;
float contador=0;
float ac = 0;
float resultado=0; //Variavel que irá receber o resultado do valor medido / contador
float vdc=0; //Variavel para enviar o valor final para o THINGSPEAK (Serve apenas p isso)

/**************************** Variaveis para confg. do sensor CORRENTE DC FONTE 1 ****************************/
int IDC_OUT = A3; //A3 pino de idc out fonte 1
int VIN2 = A7; //A7 pino VIN fonte 2
const int VIN1 = IDC_OUT; //Pino ADC3 onde sera instalado o sensor de corrente fonte 1, pino SN 39
const float VCC = 3.3; //Tensao do sensor ou do resultado do divisor de tensao
const int MODEL = 0;  //Modelo do sensor (0=5A, 1=5A e 2=30A)
const int MODEL1 = 1; //Modelo 5A para fonte 2
#include <Robojax_AllegroACS_Current_Sensor.h> //Biblitoeca SENSOR CORRENTE DC SAIDA
float corrente = 0;
float erro = 0;
Robojax_AllegroACS_Current_Sensor robojax1(MODEL,VIN1); //Configuracao SENSOR CORRENTE DC SAIDA
Robojax_AllegroACS_Current_Sensor robojax2(MODEL1,VIN2); //Configuracao SENSOR CORRENTE DC SAIDA  

/**************************** Variaveis para confg. do sensor TEMPERATURA ****************************/
int pin1 = 4; //Configura o pino G4 (ADC10) como entrada do sensor de temperatura FONTE 1
int pin2 = 0; //Configura o pino G0 (ADC11) como entrada do sensor de temperatura FONTE 2
DHT dht1(pin1, DHT11);    //Define na biblioteca do dht o pino 4 como entrada do sensor FONTE 1
DHT dht2(pin2, DHT11);    //Define na biblioteca do dht o pino 0 como entrada do sensor FONTE 2

/**************************** Variaveis para confg. do sensor CORRENTE AC ENTRADA ****************************/
int IAC_INT = A5; //Pino para leitura sensor corrente AC 
int VAC_INT = A4; //Pino para leitura sensor tensao AC
#include "EmonLib.h"  //Inclui a biblioteca EmonLib para realizar os calculos da corrente 
#define nsensores 8 //define como 8 o numero maximo de sensores para a biblioteca EMONLIB
EnergyMonitor emon[nsensores]; //Seta o numero de sensores de acordo com nsensores
EnergyMonitor emon1; //Cria uma instancia emon1; é obrigatoria no uso da biblioteca
float ruido = 0.04; //Ruido que será subtraido no calculo, o valor foi descoberto atraves de testes praticos sem carga 
double Irms = 0; //Inicia o valor da correte como 0 A

/**************************** Variaveis para confg. do sensor TENSÃO AC ****************************/
#define VOLT_CAL 77.5//Valor calibração sensor de tensão AC          
unsigned int tempo = 2000; //Tempo para caso a energia caia e retorne com os valores corretos sem PICO 

/**************************** Configuracao do WiFi & Canal Thingspeak ****************************/
const char* ssid = "GalaxyS20";                        //SSID da rede local
const char* password = "dnog8899";                //Senha rede local
unsigned long channelID = 1886874;                //ID do canal criado no thinkspeak
const char* WriteAPIKey = "C3HTCMV4KR7GO9VC";     //API do canal criado no thingspeak
WiFiClient cliente; 

int trava_wifi = 0; //Para nao ficar imprimindo WIFI CONECTADO

int pin_redundancia = 9; //Define o pino 9 como sendo a saida para RELE REUNDANCIA

int redundancia_ativacao = 0; //Variavel para mudar os pinos dos sensores no programa

/************************************************************************************************
******************************************** VOID SETUP******************************************
*************************************************************************************************/

void setup() {
  pinMode(14,OUTPUT); //Saida para LED FONTE 1
  pinMode(27,OUTPUT); //Saida para LED FONTE 2
  digitalWrite(14,HIGH); //inicia o LED  FONTE 1 como LIGADO
  digitalWrite(27,LOW); //inicia o LED FONTE 2 como desligado

  pinMode(17,OUTPUT); //Configura a saida do buzzer como saida
  digitalWrite(17,LOW); //inicia o buzzer como 0v

  pinMode(12,OUTPUT); //Configura o pino do rele como saida
  digitalWrite(12,LOW); //Garante 0V na saida para o rele de redundancia, zerando a situacao para FONTE1

  Serial.begin(115200); //Configura os baunds do monitor serial
  Serial.print("Conectando na rede..."); //Printa na saida serial a mensagem tentativa de conexao wifi

  WiFi.mode(WIFI_STA); // Configura como station mode
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED){ //Se caso o wifi n for conectado, irá fica nesse loop printando "."
   delay(500);
   Serial.print(".");
  }


  ThingSpeak.begin(cliente); //Inicia thingspeak

  dht1.begin();//Inicia sensor DHT1 fonte 1
  dht2.begin();//Inicia sensor DHT2 fonte 2

  emon[1].voltage(VAC_INT, VOLT_CAL, 1.7);  // PINO ENTRADA = ADC4, CALIBRACAO = 1.7, SENSOR TENSAO ENTRADA FONTE 1

  emon[1].current(IAC_INT, 0.60); //Pino ADC5 do ESP32 e valor da calibracao do sensor SENSOR CORRENTE ENTRADA FONTE 1
}

void loop() {

  if(SELECAO_FONTE == 2) //Configura os pinos para leitura dos parametros fonte 2
  {
    digitalWrite(14,LOW); //DESLIGA LED FONTE 1
    digitalWrite(27,HIGH); //LIGA LED FONTE 2
    VDC_OUT = 34; //muda pino de leitura tensao de entrada para 34 G34 ADC6
    IDC_OUT = 35; //muda pino de leitura corrente saida para 35 G35 ADC7
    pin1 = 0; //muda o pino de leitura do sensor temp para pino 0 ADC11
    
    pin_tensao_entrada_dc = VDC_OUT; //Define o pino 34 como sendo leitura DC fonte 2

    DHT dht1(pin1, DHT11);    //Define na biblioteca do dht o pino 4 como entrada do sensor

    const int VIN = IDC_OUT; //Pino 35 para leitura corrente fonte 2 
    Robojax_AllegroACS_Current_Sensor robojax(MODEL,VIN); //Configuracao SENSOR CORRENTE DC SAIDA 

  }

  if(WiFi.status() != WL_CONNECTED) { //Se caso o wifi n for conectado, irá fica nesse loop printando "."
    Serial.println("WiFi desconectado!!!!!");
    WiFi.mode(WIFI_STA); // Configura como station mode
    WiFi.begin(ssid,password);
    trava_wifi = 0;
  }
  if(WiFi.status() == WL_CONNECTED && trava_wifi == 0) { //Se caso o wifi n for conectado, irá fica nesse loop printando "."
    Serial.println("WiFi conectado =D");
    trava_wifi = 1;

  }

  sensor_tensao_entrada();//Vai para a void sensor tensao entrada
  sensor_corrente_entrada();//Vai para a void sensor corrente entrada
  sensor_tensao_saida();//Vai para void sensor tensao saida fonte 1
  sensor_corrente_saida();//Vai para void sensor corrente saida fonte 1
  

  sensor_temperatura();//Vai para void leerdht1

 /////////////ENVIAR DADOS AO THINGSPEAK//////////////
  ThingSpeak.writeFields(channelID,WriteAPIKey);
  Serial.println("Dados enviados ao ThingSpeak!");
}

void sensor_temperatura()  //Sensor temperatura
{
  
  float t2 = dht2.readTemperature(); //Le a temperatura e armazena o valor em t2

  float t1 = dht1.readTemperature(); //Le a temperatura e armazena o valor em t1

if(redundancia_ativacao == 0) //Se nao houver redundancia, lê sensor fonte 1
{
  if(t1 >= 0 || t1 <= 100)
  {
  Serial.print("Temperatura FONTE_1: ");
  Serial.print(t1); //Imprime no terminal de video o valor da temperatura 
  Serial.println(" ºC.");
  Serial.println("**************************************");
  Serial.println("**************************************");
  }
  else{
    Serial.println("Temperatura FONTE_1: DESLIGADO");
    Serial.println("**************************************");
    Serial.println("**************************************");
  }
  ThingSpeak.setField (1,t1); //Envia ao campo 1 do thingspeak o valor da temperatura FONTE 1
}

if(redundancia_ativacao == 1) //Se houver redundancia, lê sensor fonte 2
{
if(t2 >= 0 || t2 <= 100)
{
  Serial.print("Temperatura FONTE_2: ");
  Serial.print(t2); //Imprime no terminal de video o valor da temperatura 
  Serial.println(" ºC.");
  Serial.println("**************************************");
  Serial.println("**************************************");
  delay(2000);
  }
  else{
    Serial.println("Temperatura FONTE_2: DESLIGADO");
    Serial.println("**************************************");
    Serial.println("**************************************");
  }
  ThingSpeak.setField (6,t2); //Envia ao campo 6 do thingspeak o valor da temperatura FONTE 2
}

if(t1 > temperatura_maxima) //Se a temperatura da fonte 1 estiver acima do normal, faz a redundancia
  {
    redundancia(); 
    selecao();
    redundancia_ativacao = 1; //Ativa a redundancia para mudar os sensores a serem medidos
  }
if(t2 > temperatura_maxima) //Se a temp da fonte 2 estiver acima do normal, aciona o buzzer apenas
{
  digitalWrite(17,HIGH); //aciona buzzer
  digitalWrite(14,HIGH); //inicia o LED  FONTE 1 como LIGADO
  digitalWrite(27,LOW); //inicia o LED FONTE 2 como desligado
  delay(1000);
  digitalWrite(14,LOW); //inicia o LED  FONTE 1 como LIGADO
  digitalWrite(27,HIGH); //inicia o LED FONTE 2 como desligado
} 
}

void sensor_tensao_saida() //Sensor de tensao DC 
{ 
  contador=0;
  resultado=0;
  valor_medio=0;

for(int i=0;i<1000;i++)
{
  amostragem = analogRead(pin_tensao_entrada_dc); //Le o valor analogico do pino do sensor, e armazena em AMOSTRAGEM
  valor_medio = amostragem + valor_medio;
  contador++;
}
resultado = valor_medio/contador;
vdc = ((((resultado+108.5)/244)+0.404672131)); //Armazena o resultado do calculo na variavel VDC 

if(resultado>100 && SELECAO_FONTE == 1) //Se o valor estiver acima de 100, considerase que a tensão no pino é maior que 0 (zero)
{
  vdc = ((((resultado+108.5)/244)+0.404672131)); //Calcula novamente para evitar erros  
  Serial.print("Tensao DC FONTE_1: "); 
  Serial.print(vdc); //Imprime no term. de video o valor da variavel VDC 
  Serial.println(" V");
  ThingSpeak.setField (3,vdc); //Envia ao campo 3 do thingspeak o valor da tensao DC lida pelo sensor
}
if(vdc <= 0.05){ //Se caso a tensao DC não for maior que zero
      int erro=0;
      Serial.print("Tensao DC FONTE_1: "); 
      Serial.print(erro); //Imprime o valor de erro, sendo ZERO VOLTS
      Serial.println(" V");
      vdc = 0;
      ThingSpeak.setField (3,vdc); //Envia ao thingspeak o valor de ZERO volts, indicando que nao há tensao na saída
    }

if(resultado>100 && SELECAO_FONTE == 2) //Se o valor estiver acima de 100, considerase que a tensão no pino é maior que 0 (zero)
{
  vdc = ((((resultado+108.5)/244)+0.404672131)); //Calcula novamente para evitar erros  
  Serial.print("Tensao DC FONTE_2: "); 
  Serial.print(vdc); //Imprime no term. de video o valor da variavel VDC 
  Serial.println(" V");
  ThingSpeak.setField (7,vdc); //Envia ao campo 7 do thingspeak o valor da tensao DC lida pelo sensor
  if(vdc > Voutput_maximo || vdc < Voutput_minimo)//Se a tensao DC estiver abaixo do valor minimo estipulado irá ocorrer a redundancia
  {
    digitalWrite(17,HIGH); //aciona buzzer
    digitalWrite(14,HIGH); //inicia o LED  FONTE 1 como LIGADO
    digitalWrite(27,LOW); //inicia o LED FONTE 2 como desligado
    delay(1000);
    digitalWrite(14,LOW); //inicia o LED  FONTE 1 como LIGADO
    digitalWrite(27,HIGH); //inicia o LED FONTE 2 como desligado
  }
}
if(vdc <= 0.05){ //Se caso a tensao DC não for maior que zero
      int erro=0;
      Serial.print("Tensao DC FONTE_2: "); 
      Serial.print(erro); //Imprime o valor de erro, sendo ZERO VOLTS
      Serial.println(" V");
      vdc = 0;
      ThingSpeak.setField (7,vdc); //Envia ao thingspeak o valor de ZERO volts, indicando que nao há tensao na saída
    }

vdc = ((((resultado+108.5)/244)+0.404672131)); //Calcula novamente para caso tenha que realizar redundancia das fontes


if(vdc > Voutput_maximo || vdc < Voutput_minimo)//Se a tensao DC estiver abaixo do valor minimo estipulado irá ocorrer a redundancia
{
  redundancia(); //Vai para a void REDUNDANCIA e executa a redundancia das fontes 
  selecao();
}

}

void sensor_tensao_entrada()
{
  emon[1].calcVI(30,100);         // Calculate all. No.of half wavelengths (crossings), time-out
  delay(tempo);
  emon[1].calcVI(30,250);         // Calculate all. No.of half wavelengths
  float supplyVoltage   = emon[1].Vrms;             //extract Vrms into Variable(crossings), time-out
  
if(supplyVoltage < 100)
 {
   redundancia();
   selecao();
   supplyVoltage = 0;
 }
if(supplyVoltage >= 100)
  {
   supplyVoltage = 0;
   delay(tempo);
   emon[1].calcVI(30,250);         // Calculate all. No.of half wavelengths 
   emon[1].calcVI(30,250);         // Calculate all. No.of half wavelengths
   float supplyVoltage   = emon[1].Vrms;             //extract Vrms into Variable

    if(supplyVoltage >= 100){
     Serial.print("Tensao AC : ");  
     Serial.print(supplyVoltage);
     Serial.println(" V");
    }
     else{
       redundancia();
       selecao();
       supplyVoltage = 0;
       Serial.println("Tensao AC : 0 V");
      }
    tempo = 1;
    ThingSpeak.setField (4,supplyVoltage);
  }
   else
   {
     redundancia();
     selecao();
     Serial.println("Tensao AC : 0 V");
     supplyVoltage = 0;
     tempo = 2000;
     ThingSpeak.setField (4,supplyVoltage);
   }
}

void sensor_corrente_entrada()
{
  Irms = 0;
  float Irms = emon[1].calcIrms(5000); 
  Irms = Irms - ruido;
  Irms = emon[1].calcIrms(5000);
  Irms = Irms - ruido; //Repete o calculo 2x para evitar ruidos e picos na leitura
  
  if(Irms < 0){
    Irms = 0; //Zera o valor da corrente pois está menor que zero 
  }
  Serial.print("Corrente entrada: ");
  Serial.print(Irms);
  Serial.println(" A");


  if(Irms > Iinput_maxima)
  {
    redundancia();
    selecao();
  }

  ThingSpeak.setField (5,Irms);
}

void sensor_corrente_saida()
{
  if(SELECAO_FONTE == 1){
    corrente = robojax1.getCurrentAverage(1000),3;//Faz 3 amostra e armazena o valor com 3 casas decimais
    corrente = corrente - 1.065; //Subtrai o valor do erro do sensor sem CARGA
    if(corrente > 0)
    {
    Serial.print("Corrente FONTE_1: ");
    Serial.print(corrente); //Imprime o valor da corrente no monitor
    Serial.println(" A");
    ThingSpeak.setField (2,corrente);
    if(corrente > Ioutput_maxima)
      {
      redundancia(); //Vai para a void REDUNDANCIA e executa a redundancia das fontes 
      selecao();
      }
    }
    else
    {
    corrente = 0;
    Serial.print("Corrente FONTE_1: ");
    Serial.print(corrente); //Imprime o valor da corrente no monitor
    Serial.println(" A");
    ThingSpeak.setField (2,corrente);
    }
  }

  if(SELECAO_FONTE == 2){
    corrente = robojax2.getCurrentAverage(1000),3;//Faz 3 amostra e armazena o valor com 3 casas decimais
    corrente = corrente - 1.095; //Subtrai o valor do erro do sensor sem CARGA
    if(corrente > 0 && vdc > 1)
    {
    Serial.print("Corrente FONTE_2: ");
    Serial.print(corrente); //Imprime o valor da corrente no monitor
    Serial.println(" A");
    ThingSpeak.setField (8,corrente);
    if(corrente > Ioutput_maxima)//Se a tensao DC estiver abaixo do valor minimo estipulado irá ocorrer a redundancia
    {
    digitalWrite(17,HIGH); //aciona buzzer PINO G17
     digitalWrite(14,HIGH); //inicia o LED  FONTE 1 como LIGADO
     digitalWrite(27,LOW); //inicia o LED FONTE 2 como desligado
     delay(1000);
     digitalWrite(14,LOW); //inicia o LED  FONTE 1 como LIGADO
     digitalWrite(27,HIGH); //inicia o LED FONTE 2 como desligado
    }
    }
    else
    {
    corrente = 0;
    Serial.print("Corrente FONTE_2: ");
    Serial.print(corrente); //Imprime o valor da corrente no monitor
    Serial.println(" A");
    ThingSpeak.setField (8,corrente);
    }
  }
}

void redundancia()
{
digitalWrite(14,LOW); //DESLIGA LED FONTE 1
digitalWrite(27,HIGH); //LIGA LED FONTE 2
digitalWrite(12,HIGH); //Aciona o relé de redundancia G12
redundancia_ativacao = 1;
}

void selecao()
{
SELECAO_FONTE = 2; //Muda a selecao da fonte como sendo FONTE 2
}
