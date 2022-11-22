/********************** PINOS *****************************************
SENSOR DE CORRENTE AC-DC = ADC7 (G35)
SENSOR DE TENSAO AC = ADC3 (SN)
SENSOR DE TENSAO DC = ADC0 (SP)
SENSOR DE TEMPERATURA E UMIDADE = ADC10 (G4)
**********************************************************************/

/**************************** BIBLIOTECAS ****************************/
#include "ThingSpeak.h" //Inclui a biblioteca para o ThingSpeak 
#include "WiFi.h" //Inclui a biblioteca do WiFi
#include "DHT.h" //Inclui a biblioteca do sensor de temperatura DHT11
#include "EmonLib.h" // Include Emon Library

/**************************** Configuracao dos valores MIN E MAX de entrada e saída ****************************/
float Vinput = 127; //Valor da alimentação da rede elétrica em V
float Desvio_Entrada = 0.05; //Valor em porcentagem (x/100)
float Voutput = 12; //Valor de tensão de saída da fonte em V
float Desvio_Saida = 0.1; //Valor em porcentagem (x/100)
float Ioutput = 3; //Corrente de saída da fonte em A
float Temp_Maxima = 40; //Temperatura maxima da fonte de alimentação em °C
float Umidade_Maxima = 60; //Umidade maxima na fonte de alimentacao em %
float Frequencia_rede = 60; //Frequencia da rede elétrica em Hz

/**************************** Calculos para definir os valores min e max (Automaticamente) ****************************/

float Vinput_max = Vinput+(Vinput*Desvio_Entrada); //Tensão CA de entrada máxima em V
float Vinput_min = Vinput-(Vinput*Desvio_Entrada); //Tensão CA de entrada minima em V
float Voutput_max = Voutput+(Voutput*Desvio_Saida); //Tensão CC de saída máxima em V
float Voutput_min = Voutput-(Voutput*Desvio_Saida); //Tensão CC de saída minima em V
float Iinput = (Ioutput*Voutput)/Vinput; //Corrente maxima de entrada em A

/**************************** Variaveis para confg. do sensor TENSÃO DC ****************************/
int pin_tensao = A0; //Configura pino ADC0 (SP) como entrada do sensor VDC
float amostragem = 0; //Variavel amostragem que irá receber o valor exato do pino do sensor
float valor_medio=0;
float contador=0;
float ac = 0;
float resultado=0; //Variavel que irá receber o resultado do valor medido / contador
float vdc=0; //Variavel para enviar o valor final para o THINGSPEAK (Serve apenas p isso)

/**************************** Variaveis para confg. do sensor TEMPERATURA ****************************/
#define pin1 4       //Configura o pino 4 (ADC10) como entrada do sensor de temperatura
DHT dht1(pin1, DHT11);    //Define na biblioteca do dht o pino 4 como entrada do sensor

/**************************** Variaveis para confg. do sensor CORRENTE DC-AC ****************************/
#include "EmonLib.h"  //Inclui a biblioteca EmonLib para realizar os calculos da corrente 
EnergyMonitor emon1; //Cria uma instancia emon1; é obrigatoria no uso da biblioteca
float ruido = 0.04; //Ruido que será subtraido no calculo, o valor foi descoberto atraves de testes praticos sem carga 
double Irms = 0; //Inicia o valor da correte como 0A

/**************************** Variaveis para confg. do sensor TENSÃO AC ****************************/
#define VOLT_CAL 77.5//Valor calibração sensor de tensão AC          
unsigned int tempo = 2000; //Tempo para caso a energia caia e retorne com os valores corretos sem PICO 

/**************************** Configuracao do WiFi & Canal Thingspeak ****************************/
const char* ssid = "SJCS";                        //SSID da rede local
const char* password = "Roberto@1529";                //Senha rede local
unsigned long channelID = 1886874;                //ID do canal criado no thinkspeak
const char* WriteAPIKey = "C3HTCMV4KR7GO9VC";     //API do canal criado no thingspeak
WiFiClient cliente; 

/************************************************************************************************
******************************************** VOID SETUP******************************************
*************************************************************************************************/

void setup() {
  
  pinMode(5, OUTPUT); //Configura o pino 5 como saida para acionar o RELÉ 

  Serial.begin(115200); //Configura os baunds do monitor serial
  Serial.println("Conectando na rede..."); //Printa na saida serial a mensagem tentativa de conexao wifi

  WiFi.mode(WIFI_STA); // Configura como station mode
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) { //Se caso o wifi n for conectado, irá fica nesse loop printando "."
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado! =D"); //Caso for conectado essa msg sera printada

  ThingSpeak.begin(cliente); //Inicia thingspeak

  dht1.begin();//Inicia sensor DHT1

  emon1.voltage(A3, VOLT_CAL, 1.7);  // Voltage: input pin ADC3, calibration, phase_shift

  emon1.current(A7, 0.60); //Pino ADC7 do ESP32 
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) { //Se caso o wifi n for conectado, irá fica nesse loop printando "."
    delay(500);
    Serial.println("WiFi Desconectado");
    WiFi.mode(WIFI_STA); // Configura como station mode
    WiFi.begin(ssid,password);
  }
  sensor_temperatura1();//Vai para void leerdht1
  sensor_tensaodc1();//Vai para void sensor VDC
  sensor_tensaoac1();//Vai para a void sensor VAC 
  sensor_correnteac1();//Vai para a void sensor corrente AC

 /////////////ENVIAR DADOS AO THINGSPEAK//////////////
  ThingSpeak.writeFields(channelID,WriteAPIKey);
  Serial.println("Dados enviados ao ThingSpeak!");
}

void sensor_temperatura1() { //Sensor temperatura
   
  float t1 = dht1.readTemperature(); //Le a temperatura e armazena o valor em t1

  Serial.print("Temperatura DHT11: ");
  Serial.print(t1); //Imprime no terminal de video o valor da temperatura 
  Serial.println(" ºC.");

  ThingSpeak.setField (1,t1); //Envia ao campo 1 do thingspeak o valor da temperatura
}

void sensor_tensaodc1() //Sensor de tensao DC 
{ 
  contador=0;
  resultado=0;
  valor_medio=0;

for(int i=0;i<1000;i++)
{
  amostragem = analogRead(pin_tensao); //Le o valor analogico do pino do sensor, e armazena em AMOSTRAGEM
  valor_medio = amostragem + valor_medio;
  contador++;
}
resultado = valor_medio/contador;
vdc = ((((resultado+108.5)/244)+0.404672131)); //Armazena o resultado do calculo na variavel VDC 

if(resultado>100) //Se o valor estiver acima de 100, considerase que a tensão no pino é maior que 0 (zero)
{
  vdc = ((((resultado+108.5)/244)+0.404672131)); //Calcula novamente para evitar erros  
  Serial.print("Tensao DC : "); 
  Serial.print(vdc); //Imprime no term. de video o valor da variavel VDC 
  Serial.println(" V");
  ThingSpeak.setField (3,vdc); //Envia ao campo 3 do thingspeak o valor da tensao DC lida pelo sensor
}
else{ //Se caso a tensao DC não for maior que zero
      int erro=0;
      Serial.print("Tensao DC : "); 
      Serial.print(erro); //Imprime o valor de erro, sendo ZERO VOLTS
      Serial.println(" V");
      vdc = 0;
      ThingSpeak.setField (3,vdc); //Envia ao thingspeak o valor de ZERO volts, indicando que nao há tensao na saída
    }

vdc = ((((resultado+108.5)/244)+0.404672131)); //Calcula novamente para caso tenha que realizar redundancia das fontes

if(vdc < 10)//Se a tensao DC estiver abaixo do valor minimo estipulado irá ocorrer a redundancia
{
  redundancia(); //Vai para a void REDUNDANCIA e executa a redundancia das fontes 
}
}

void sensor_tensaoac1(){

  emon1.calcVI(30,100);         // Calculate all. No.of half wavelengths (crossings), time-out
  float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable(crossings), time-out

if(supplyVoltage < 100)
 {

   redundancia();
   supplyVoltage = 0;
 }
if(supplyVoltage >= 100)
  {
   delay(tempo);
   supplyVoltage = 0;
   emon1.calcVI(30,250);         // Calculate all. No.of half wavelengths
   emon1.calcVI(30,250);         // Calculate all. No.of half wavelengths 
   float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable

    if(supplyVoltage >= 100){
     //digitalWrite(2, LOW);
     Serial.print("Tensao AC : ");  
     Serial.print(supplyVoltage);
     Serial.println(" V");
     
    }

     else{
       supplyVoltage = 0;
       Serial.println("Tensao AC : 0V");
      // digitalWrite(2, HIGH);
      }

    tempo = 1;

    ThingSpeak.setField (4,supplyVoltage);
  }

   else
   {
     redundancia();
     Serial.println("Tensao AC : 0V");
     supplyVoltage = 0;
     tempo = 2000;
     ThingSpeak.setField (4,supplyVoltage);
     digitalWrite(2, HIGH);
   }
}

void sensor_correnteac1()
{
  Irms = 0;
  float Irms = emon1.calcIrms(5000); 
  Irms = Irms - ruido;
  Irms = emon1.calcIrms(5000);
  Irms = Irms - ruido; //Repete o calculo 2x para evitar ruidos e picos na leitura
  
  if(Irms < 0){
    Irms = 0; //Zera o valor da corrente pois está menor que zero 
  }
  Serial.print("Corrente: ");
  Serial.print(Irms);
  Serial.println(" A");
  Serial.println("**************************************");
  Serial.println("**************************************");

  ThingSpeak.setField (5,Irms);
}

void redundancia()
{
digitalWrite(5, HIGH); //ACIONA O RELÉ ATRAVES DE UM TRANSISTOR BC548B NO PINO 5
}