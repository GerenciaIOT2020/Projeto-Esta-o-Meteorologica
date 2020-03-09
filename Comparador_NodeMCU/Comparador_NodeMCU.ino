#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <PubSubClient.h>

WiFiMulti wifiMulti;

char servidorMqtt[] = "172.26.1.8";
char portaServidorMqtt[6] = "1883";
char tokenMqttDisp[33] = "ESP8266_TKN_NODEMCU";
char ID [] = "Davis WS II";
const int tempoLoop = 1000;

//------------------------------Deep sleep------------------------------------------------

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */

//------------------------------Rain------------------------------------------------
int RainSensorPin = 21;              //Rain REED-ILS sensor GPIO 21 on ESP32
#define Bucket_Size_EU 0.2           // rain bucket size milimetres ( 0.2mm)

volatile unsigned long tipCount1h;   // bucket tip counter used in interrupt routine
volatile unsigned long tipCount24h;  // bucket tip counter used in interrupt routine
unsigned int counter = 16;
volatile unsigned long contactTime;

//--------------------------------Wind  speed----------------------------------------
int WindSensorPin = 4;               // Wind speed -ILS sensor (anemometer) GPIO 14 on ESP32
volatile unsigned long Rotations;     // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime;  // Timer to avoid contact bounce in interrupt routine

//-------------------------------Wind direction-----------------------------------------------
int WindVanePin = A4;   // The pin the wind vane sensor is connected to A4 (wind direction) on ESP32

float wind_avg; // average wind direction
int vane_value;// raw analog value from wind vane
int Direction;// translated 0 - 360 direction
int CalDirection;// converted value with offset applied
#define Offset 0;
int windDirections[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

//--------------------------------Setup Wifi----------------------------------------------
const char* ssid1     = "IFPE-PUBLICA";   //I use multiple SSID In my home, but you can only define one.
const char* password1 = "S3Mf1@BJ";
const char* ssid2     = "ifpe_iot";
const char* password2 = "exensaoiot";
const char* ssid3     = "extensao_iot";
const char* password3 = "aluno123";

//--------------------------------WEATHER VAR---------------------------------------------
float listaTemperaturas[3] = {0};
int quantidadeTemps = 0;

float tRaw;              // RAW A2
float rhRaw;             // RAW A3
float tempMap;           // Temp celsius Davis
float rhMap;          // Humidity Davis
float batRaw;           //Valor RAW pino A13 (bateria)
float voltagem;         //valor em V da baterial
float windSpeed = 0;     // Wind speed (mph)
float wind_speed_min = 100; // Minimum wind speed (mph)
float wind_speed_avg;    // 10 minutes average wind speed ( mph)
float windgustmph = 0;   // Wind gust speed( mph)
float windmax = 0;
float rain1h = 0;        // Rain inches over the past hour
float rain = 0;          // Rain milimetres over the past hour
float rain24h = 0;       // Rain inches over the past 24 hours
float rainrate = 0;      // Rain milimetres over the past 24 hours
int dBm;                 // WiFi signal strenght dBm
int quality;             // WiFi signal quality %


bool debug = 1;           //debug = 1 -> enable debug

//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////MAIN PROGRAM START/////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void setup()
{
  Rotations = 0;   // Set Rotations to 0 ready for calculations
  tipCount1h = 0;
  tipCount24h = 0;
  wind_speed_min = 100;

  analogReadResolution(10); //define 10bits de resolução no ADC do ESP32

  pinMode(A2, INPUT); //temperature on analog pin A2 - fio amarelo
  pinMode(A3, INPUT);  //Relative Humidity on analog pin A3 - fio verde
  pinMode(A13, INPUT); //Pino usado para aferir bateria

  Serial.begin(115200);
  delay(7000);
  WiFi.mode(WIFI_STA);

  Serial.print("Start NodeMCU Weather Station ");
  Serial.println(ID);
  Serial.println();
  delay(2000);

  startwifi();

  pinMode(RainSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RainSensorPin), isr_rg, FALLING);
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
  sei();
}

void loop()
{
  read_data();             // read different sensors data from analog and digital pins of ESP32
  getRain();
  getWindSpeed();
  isr_rotation();
  getWindDirection();
  RSSIdBm();               // WiFi signal quality (RSSI)
  aferirCargaBateria();
  print_data();            // print data  in serial monitor
  thingsboard();            // sends data to thingsboard
  // enterSleepMode();
}


//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////READ THE DATA FROM TEM/HUMD SENSOR/////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void read_data(void)
{
  //Variavel temporaria para aferixao da umidade do ar
  float rhMapTemp = 0;
  float ultimaTemp = 0;
  
  // read the temperature on analog pin A2 - fio amarelo
  tRaw = analogRead(A2);
  // read the Relative Humidity on analog pin A3 - fio verde
  rhRaw = analogRead(A3);

  //tempMap = mapfloat((1023 - (tRaw - 100)), 0.0, 1023, -45.0, 60.0);
  ultimaTemp = mapfloat((1023 - (tRaw - 100)), 0.0, 1023, -45.0, 60.0);
  rhMapTemp = mapfloat((1023 - (rhRaw - 270)), 0, 1023, 0, 100);

  validarTemperatura(ultimaTemp);
  
  //caso seja primeira leitura erronia
  if(rhMapTemp > 79 or rhMapTemp < 30){
    read_data();
  }
  
  //funcao para validar a umidade do ar
  if( validarUmidade(rhMapTemp, rhMap) ){
    //altera a umidade do ar
    rhMap = rhMapTemp;
  }

  Counter();
}

void validarTemperatura(float ultimaTemp){
  float delta = abs(ultimaTemp - tempMap);

  if(delta < 0.5 or tempMap == 0){
    tempMap = ultimaTemp;
  }
  else{
    tempMap = tempMap;
  }

}

//Funcao para validar a umidade do ar
boolean validarUmidade(float rhMapTemp, float rhMapAtual) {
  if (rhMapTemp < (rhMap + 0.35) or rhMap == 0) {
    //rhMap = rhMapTemp;
    return true;
  }

}

//função map para tipo de dados float
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////Get wind speed  /////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindSpeed(void)
{

  Rotations = 0; // Set Rotations count to 0 ready for calculations
  delay (tempoLoop); // Wait 30 seconds to average wind speed

  /* convert to mp/h using the formula V=P(2.25/T)
    V = P(2.25/30) = P * 0.075       V - speed in mph,  P - pulses per sample period, T - sample period in seconds */
  windSpeed = Rotations * 0.15; // 30 seconds
  Rotations = 0;   // Reset count for next sample

  if (windSpeed > windgustmph) {
    windgustmph = windSpeed;
  }
  if (wind_speed_min > windSpeed ) {
    wind_speed_min = windSpeed;
  }

  wind_speed_avg = (windgustmph + wind_speed_min) * 0.5;   // average wind speed mph per 10 minutes

}

// This is the function that the interrupt calls to increment the rotation count
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////ISR rotation//////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void isr_rotation(void)
{
  if ((millis() - ContactBounceTime) > 30 ) {  // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

// Convert MPH to m/s
float getms(float speed) {
  return speed * 0.44704;           //metric m/s 0.44704;;
}

// Get Wind Direction
//-------------------------------------------------------------------------------------------------------------
/////////////////////////////////// Wind direction ////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindDirection(void)
{
  vane_value = analogRead(A0);
  Direction = map(vane_value, 0, 1023, 0, 360);
  CalDirection = Direction + Offset;

  if (CalDirection > 360)
    CalDirection = CalDirection - 360;

  if (CalDirection < 0)
    CalDirection = CalDirection + 360;

  DirectionCheck();

  //getHeading(CalDirection);
}

void DirectionCheck()
{

  if (vane_value >= 0 && vane_value < 32)
    wind_avg = 0;
  else if (vane_value >= 32 && vane_value < 96)
    wind_avg = 22.5;
  else if (vane_value >= 96 && vane_value < 160)
    wind_avg = 45;
  else if (vane_value >= 160 && vane_value < 224)
    wind_avg = 67.5;
  else if (vane_value >= 224 && vane_value < 288)
    wind_avg = 90;
  else if (vane_value >= 288 && vane_value < 352)
    wind_avg = 112.5;
  else if (vane_value >= 352 && vane_value < 416)
    wind_avg = 135;
  else if (vane_value >= 416 && vane_value < 480)
    wind_avg = 157.5;
  else if (vane_value >= 480 && vane_value < 544)
    wind_avg = 180;
  else if (vane_value >= 544 && vane_value < 608)
    wind_avg = 202.5;
  else if (vane_value >= 608 && vane_value < 672)
    wind_avg = 225;
  else if (vane_value >= 672 && vane_value < 736)
    wind_avg = 247.5;
  else if (vane_value >= 736 && vane_value < 800)
    wind_avg = 270;
  else if (vane_value >= 800 && vane_value < 864)
    wind_avg = 292.5;
  else if (vane_value >= 864 && vane_value < 928)
    wind_avg = 315;
  else if (vane_value >= 928 && vane_value < 992)
    wind_avg = 337.5;
  else if (vane_value >= 992 && vane_value < 1025)
    wind_avg = 0;


  DirectionAvg();

}

void DirectionAvg()
{
  int index = wind_avg / 22.5;
  if ((index < 0) or (index > 15))
  {
    exit;
  }
  windDirections[index]++;
}


int getWindDirectionMax()
{
  int max = windDirections[0];
  int index = 0;
  for (int i = 1; i < 16; i++)
  {
    if (max < windDirections[i])
    {
      max = windDirections[i]; // find max value
      index = i;
    }
  }
  return index * 22.5; //  return average wind direction
}


// Converts compass direction to heading
void getHeading(int direction) {
  if (direction < 22.5)
    Serial.print("N ");
  else if (direction < 67.5)
    Serial.print("NE ");
  else if (direction < 112.5)
    Serial.print("E ");
  else if (direction < 157.5)
    Serial.print("SE ");
  else if (direction < 202.5)
    Serial.print("S ");
  else if (direction < 247.5)
    Serial.print("SW ");
  else if (direction < 292.5)
    Serial.print("W ");
  else if (direction < 337.5)
    Serial.print("NW ");
  else
    Serial.print("N ");
}

// converts wind speed to wind strength
String getWindStrength(float speed)
{
  String strength = "";
  if (speed < 1)
    strength = "Calm";
  else if (speed >= 1 && speed < 3)
    strength = "Light Air";
  else if (speed >= 3 && speed < 7)
    strength = "Light Breeze";
  else if (speed >= 7 && speed < 12)
    strength = "Gentle Breeze";
  else if (speed >= 12 && speed < 18)
    strength = "Moderate Breeze";
  else if (speed >= 18 && speed < 24)
    strength = "Fresh Breeze";
  else if (speed >= 24 && speed < 31)
    strength = "Strong Breeze";
  else if (speed >= 31 && speed < 38)
    strength = "High wind";
  else if (speed >= 38 && speed < 46)
    strength = "Fresh Gale";
  else if (speed >= 46 && speed < 54)
    strength = "Strong Gale";
  else if (speed >= 54 && speed < 63)
    strength = "Storm";
  else if (speed >= 63 && speed < 72)
    strength = "Violent storm";
  else if (speed >= 72 && speed)
    strength = "Hurricane";
  Serial.println(strength);
  return strength;
}
//------------------------------------------------------------------------------------------------------------
///////////////////////////////// Get Rain data //////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void getRain(void)
{
  cli();         //Disable interrupts

  rainrate = tipCount1h * Bucket_Size_EU;

  rain = tipCount24h * Bucket_Size_EU;


  sei();         //Enables interrupts

}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////// RAIN Interrupt ///////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

// Interrrupt handler routine that is triggered when the W174 detects rain

void isr_rg() {

  if ((millis() - contactTime) > 500 ) { // debounce of sensor signal
    tipCount1h++;
    tipCount24h++;

    contactTime = millis();
  }
}

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// RSSI dBm //////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void RSSIdBm() {


  while (wifiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  dBm = WiFi.RSSI();
  quality = 2 * (dBm + 100);
  if (dBm >= -50)
    quality = 100;

}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////PRINT DATA IN SERIAL MONITOR/////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void print_data(void)
{
  if (debug)
  {

    Serial.println(" ");
    Serial.print("Davis Temperature = "); Serial.print(tempMap); Serial.println(" C  ");
    Serial.print("Davis Temp A2 RAW = "); Serial.println(tRaw);
    Serial.print("Davis Humidity = "); Serial.print(rhMap); Serial.println(" %  ");
    Serial.print("Davis Humidity A3 RAW = "); Serial.println(rhRaw);
    Serial.print("Wind_Speed= "); Serial.print((getms(windSpeed) * 3.6)); Serial.println(" km/h  ");
    Serial.print("Wind_Speed= "); Serial.print(getms(windSpeed)); Serial.println(" m/s  ");
    Serial.print("Min = ");  //Minimum wind speed
    Serial.print(wind_speed_min / 2.236 ); Serial.print(" m/s ");
    Serial.print("Avg = "); //Average wind speed
    Serial.print(wind_speed_avg  / 2.236); Serial.print(" m/s ");
    Serial.print("Gust = "); //Maximum wind gust speed
    Serial.print(windgustmph  / 2.236); Serial.println(" m/s ");
    Serial.print("Wind_Direction = ");
    getHeading(CalDirection); Serial.println(CalDirection);
    Serial.print("Wind Avg = "); Serial.println(getWindDirectionMax());
    Serial.print("Vane Value = "); Serial.println(vane_value);
    Serial.print("Rain_Tip_Count = "); Serial.println(tipCount24h);
    Serial.print("Precip_Rate = ");
    Serial.print(rain1h); Serial.print(" in  ");   Serial.print(" Precip_Accum_Total: "); Serial.print(rain24h); Serial.println(" in  ");
    Serial.print("Rain = "); Serial.print(rain); Serial.print(" mm  ");  Serial.print("Rain rate = "); Serial.print(rainrate); Serial.println(" mm");
    Serial.print("Signal quality  = "); Serial.println(quality);
    Serial.print("RSSI  = "); Serial.print(dBm); Serial.println("dBm ");
    Serial.print("Bateria RAW "); Serial.println(batRaw);
    Serial.print("Bateria "); Serial.print(voltagem); Serial.println(" V");
    Serial.println(" ");
    Serial.print("Counter = "); Serial.println(counter);
    Serial.println(" ");
  }
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////  SEND DATA TO Thingsboard ////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void thingsboard(void)
{
  Serial.print("connecting to ");
  Serial.print(servidorMqtt);
  Serial.println("...");
  WiFiClient wifiClient;
  PubSubClient client(wifiClient);
  client.setServer( servidorMqtt , 1883 );
  if (client.connect("Davis", tokenMqttDisp, NULL)) { // use ip 184.106.153.149 or api.thingspeak.com

    Serial.println( "[DONE]" );

    client.subscribe("v1/devices/me/rpc/request/+");
    String pluviometroStr = String(rain);
    String humidadeStr = String(rhMap);
    String temperatureStr = String(tempMap);
    String windStrengthStr = String(getWindStrength(windSpeed));
    String velocidadeVentoStr = String((getms(windSpeed) * 3.6));
    String rajadaVentoStr = String(windgustmph * 0.447 * 3);
    String pointerStr = String(CalDirection);
    String voltagemStr = String(voltagem);

    enviarInfoParaServidorMQTT("temperature", temperatureStr, client);
    enviarInfoParaServidorMQTT("compass", pointerStr, client);
    enviarInfoParaServidorMQTT("windSpeed", velocidadeVentoStr, client);
    enviarInfoParaServidorMQTT("windgus", rajadaVentoStr, client);
    enviarInfoParaServidorMQTT("windStrength", windStrengthStr, client);
    enviarInfoParaServidorMQTT("humidity", humidadeStr, client);
    enviarInfoParaServidorMQTT("quantidadeChuva", pluviometroStr, client);
    enviarInfoParaServidorMQTT("voltagem", voltagemStr, client);

    delay(1000);

  } else {
    Serial.print( "[FAILED] [ rc = " );
    Serial.print( client.state() );
    Serial.println( " : retrying in 5 seconds]" );
  }
}


//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Bateria /////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void aferirCargaBateria()
{
  batRaw = analogRead(A13);
  float volt = map (batRaw, 420, 585, 280, 413);
  voltagem = volt / 100;

}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Counter /////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void Counter()
{
  if (counter == 18)
  {
    windgustmph = 0;
    wind_speed_avg = 0;
    wind_speed_min = 100;
    counter = 0;  //10 minutes loop     30*20=600s = 10min
  }
  counter++;
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// ESP32 deep sleep mode ///////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void enterSleepMode() {
  Serial.print(F("Sleeping..."));
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// WIFI SETUP ////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void startwifi()
{
  Serial.print("Connecting to Wifi ");
  delay(1000);

  wifiMulti.addAP(ssid1, password1);        //if you have less SSID, delete the others
  wifiMulti.addAP(ssid2, password2);
  wifiMulti.addAP(ssid3, password3);

  while (wifiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);

}

void enviarInfoParaServidorMQTT(String atributo, String valor, PubSubClient client) {
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"" + atributo + "\":";
  payload += valor;
  payload += "}";

  // Send payload
  char attributes[120];
  payload.toCharArray( attributes, 120 );
  client.publish( "v1/devices/me/telemetry", attributes );
}
