
// do this in platformIO #define Kueche
//#define Jule

#if defined Kueche
#define Dual
#define Zimmer "Kueche_"
#elif defined Jule
// already defined by platform #define ESP32
#define SingleRoller
#define HaveLEDs
#define Zimmer "Jule_"
#elif defined Schlafzimmer
#define Dual
#define HaveLEDs
#define IOMCP
#define Zimmer "Schlafzimmer_"
#elif defined Bad
#define Dual
#define HaveLEDs
#define IOMCP
#define Zimmer "Bad_"
#elif defined Jan
#define Dual
#define HaveLEDs
#define IOMCP
#define Zimmer "Jan_"
#else
#define Dual
#define HaveLight
#define HaveLEDs
#define IOMCP
#define Zimmer "Wohnzimmer_"
#define Terrasse
#endif

// select board: NodeMCU 1.0 (ESP-12E Module)
// upload speed 115200
// 80MHz
// 4MB none OTA

#include <ArduinoOTA.h>
//#include <WebServer.h>
#ifdef ESP32
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include <PubSubClient.h>

#define Name0 Zimmer "rechts"
#define Name1 Zimmer "links"

#include "timer.h"

auto timer0 = timer_create_default();
Timer<>::Task stop0Timer = 0;
#ifndef Single
Timer<>::Task stop1Timer = 0;
#endif

void DebugPrintf(const char *, ...); //Our printf function
char *convert(int, int);    //Convert integer number into octal, hex, etc.

const char *mqtt_server = "openhab";
#include "../../../wifiPasswd.h"

WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

#define AUF_STATE LOW
#define AB_STATE HIGH
#define SSRDelay 500
#define RELAISDELAY 500

const int P_A0 = 0;
const int P_A1 = 1;
const int P_A2 = 2;
const int P_A3 = 3;
const int P_A4 = 4;
const int P_A5 = 5;
const int P_A6 = 6;
const int P_A7 = 7;
const int P_B0 = 8;
const int P_B1 = 9;
const int P_B2 = 10;
const int P_B3 = 11;
const int P_B4 = 12;
const int P_B5 = 13;
const int P_B6 = 14;
const int P_B7 = 15;



#ifndef IOMCP
class myIO
{
public:
  void pinMode(uint8_t pin, uint8_t mode) { ::pinMode(pin, mode); };
  void pullUp(uint8_t pin, uint8_t mode)
  {
    if (mode == HIGH)
      ::pinMode(pin, INPUT_PULLUP);
    else
      ::pinMode(pin, INPUT);
  };

  void digitalWrite(uint8_t pin, uint8_t val) { ::digitalWrite(pin, val); };
  int digitalRead(uint8_t pin) { return ::digitalRead(pin); };
};

myIO mcp;
#endif


#if defined Kueche

const int R_P1 = D0;
const int R_P2 = D2;

const int SSR_P1 = D7;
const int SSR_P2 = D6;
const int SSR_P3 = P_A4;
const int SSR_P4 = P_A5;



int bLightPin = -1;
int LightPin = -1;

int bAbPin1 = D3;
int bAufPin1 = D5;
int bAbPin2 = -1; //D8;
int bAufPin2 = -1;//D9;
#elif defined Jule

const int R_P1 = 27;

const int SSR_P1 = 26;
const int SSR_P2 = 25;


int bLightPin = -1;
int LightPin = -1;

int bAbPin1 = 19;
int bAufPin1 = 18;
int bAbPin2 = -1; //D8;
int bAufPin2 = -1;//D9;

const int LEDAbPin1 = 16;
const int LEDAufPin1 = 17;
const int LEDAufPin[1] = {LEDAufPin1};
const int LEDAbPin[1] = {LEDAbPin1};
#elif defined Bad
//SDA/SCL default to pins 4& 5
Adafruit_MCP23017 mcp;
const int R_P1 = P_A0;
const int R_P2 = P_A1;

const int SSR_P1 = P_A2;
const int SSR_P2 = P_A3;
const int SSR_P3 = P_A4;
const int SSR_P4 = P_A5;
const int aussenlicht = SSR_P3;

int bAbPin1 = P_B2;
int bAufPin1 = P_B3;
int bAbPin2 = P_B0;
int bAufPin2 = P_B1;
int bLightPin = P_B4;

int LEDAbPin2 = P_A6;
int LEDAufPin2 = P_A7;

int LEDAbPin1 = P_B6;
int LEDAufPin1 = P_B7;

int LEDLightPin = P_B5;
int LightPin = SSR_P3;

const int LEDAufPin[2] = {LEDAufPin1, LEDAufPin2};
const int LEDAbPin[2] = {LEDAbPin1, LEDAbPin2};
#elif defined Jan
//SDA/SCL default to pins 4& 5
Adafruit_MCP23017 mcp;
const int R_P1 = P_A0;
const int R_P2 = P_A1;

const int SSR_P1 = P_A2;
const int SSR_P2 = P_A3;
const int SSR_P3 = P_A4;
const int SSR_P4 = P_A5;
const int aussenlicht = SSR_P3;

int bAbPin1 = P_B2;
int bAufPin1 = P_B3;
int bAbPin2 = P_B0;
int bAufPin2 = P_B1;
int bLightPin = P_B4;

int LEDAbPin2 = P_A6;
int LEDAufPin2 = P_A7;

int LEDAbPin1 = P_B6;
int LEDAufPin1 = P_B7;

int LEDLightPin = P_B5;
int LightPin = SSR_P3;

const int LEDAufPin[2] = {LEDAufPin1, LEDAufPin2};
const int LEDAbPin[2] = {LEDAbPin1, LEDAbPin2};
#else
//SDA/SCL default to pins 4& 5
Adafruit_MCP23017 mcp;
const int R_P1 = P_A0;
const int R_P2 = P_A1;

const int SSR_P1 = P_A2;
const int SSR_P2 = P_A3;
const int SSR_P3 = P_A4;
const int SSR_P4 = P_A5;
const int aussenlicht = SSR_P3;

int bAbPin1 = P_B2;
int bAufPin1 = P_B3;
int bAbPin2 = P_B0;
int bAufPin2 = P_B1;
int bLightPin = P_B4;

int LEDAbPin2 = P_A6;
int LEDAufPin2 = P_A7;

int LEDAbPin1 = P_B6;
int LEDAufPin1 = P_B7;

int LEDLightPin = P_B5;
int LightPin = SSR_P3;

const int LEDAufPin[2] = {LEDAufPin1, LEDAufPin2};
const int LEDAbPin[2] = {LEDAbPin1, LEDAbPin2};
#endif

#ifdef SingleRoller
const int runPin[1] = {SSR_P1};
const int aufAbPin[1] = {R_P1};
#else
 #ifdef Bad
  const int runPin[2] = {SSR_P4, SSR_P1};
  const int aufAbPin[2] = {R_P1, R_P2};
 #elif defined Jan
  const int runPin[2] = {SSR_P4, SSR_P1};
  const int aufAbPin[2] = {R_P1, R_P2};
 #else
  const int runPin[2] = {SSR_P1, SSR_P2};
  const int aufAbPin[2] = {R_P1, R_P2};
 #endif
#endif

int lightState = LOW;     // the current state of the output pin
int buttonState;          // the current reading from the input pin
int previousState = HIGH; // the previous reading from the input pin

unsigned long pressTime = 0;  // the last time the output pin was toggled
unsigned long debounce = 100; // the debounce time, increase if the output flickers

class debounceButton
{
public:
  debounceButton(int port);
  bool wasPressed();
  static void update();

private:
  int port;
  bool previousButtonState;
  unsigned long lastTime;
  static unsigned long currentTime;
};
unsigned long debounceButton::currentTime = 0;

debounceButton::debounceButton(int p)
{
  port = p;
  lastTime=millis();
  previousButtonState=false;
}

void debounceButton::update()
{
  currentTime = millis();
}

bool debounceButton::wasPressed()
{
  if(port < 0)
  {
    return false;
  }
  bool buttonState = !mcp.digitalRead(port);
  if (!buttonState)
  {
    lastTime = currentTime;
    previousButtonState = false;
  }
  else{
    
    if(port == bAbPin1)
    DebugPrintf("false\n");
  }

  if ((buttonState != previousButtonState) )
  {
    
  }
  if (buttonState && (buttonState != previousButtonState) && (currentTime - lastTime > debounce))
  {
    previousButtonState = true;
    return true;
  }
  return false;
}

debounceButton bAuf1(bAufPin1);
debounceButton bAuf2(bAufPin2);
debounceButton bAb1(bAbPin1);
debounceButton bAb2(bAbPin2);
debounceButton bLight(bLightPin);

char linebuf[200];
unsigned int linePos = 0;
unsigned int printLine = 0;
void outputChar(char c)
{

  if (linePos < 199)
  {
    linebuf[linePos] = c;
    linePos++;
    linebuf[linePos] = '\0';
  }
  if (c == '\n')
  {
    linePos = 0;
    if (client.connected())
    {
      char top[50];
      sprintf(top, "Debug/" Zimmer "/%d", printLine);
      client.publish(top, linebuf);
      printLine++;
      if (printLine > 20)
        printLine = 0;
    }
    else
    {
      Serial.print(linebuf);
    }
    linebuf[0] = '\0';
  }
}
void outputCharp(const char *s)
{
  const char *c;
  for (c = s; *c != '\0'; c++)
  {
    outputChar(*c);
  }
}
void DebugPrintf(const char *format, ...)
{
  const char *traverse;
  int i;
  const char *s;
  char iBuf[20];

  va_list arg;
  va_start(arg, format);

  for (traverse = format; *traverse != '\0'; traverse++)
  {
    while (*traverse != '%' && *traverse != '\0')
    {
      outputChar(*traverse);
      traverse++;
    }
    if (*traverse == '\0')
    {
      break;
    }

    traverse++;

    //Module 2: Fetching and executing arguments
    switch (*traverse)
    {
    case 'c':
      i = va_arg(arg, int); //Fetch char argument
      outputChar(i);
      break;

    case 'd':
      i = va_arg(arg, int); //Fetch Decimal/Integer argument
      if (i < 0)
      {
        i = -i;
        outputChar('-');
      }
      outputCharp(itoa(i,iBuf, 10));
      break;

    case 'o':
      i = va_arg(arg, unsigned int); //Fetch Octal representation
      outputCharp(itoa(i,iBuf, 8));
      break;

    case 's':
      s = va_arg(arg, char *); //Fetch string
      outputCharp(s);
      break;

    case 'x':
      i = va_arg(arg, unsigned int); //Fetch Hexadecimal representation
      outputCharp(itoa(i,iBuf, 16));
      break;
    }
  }

  //Module 3: Closing argument list to necessary clean-up
  va_end(arg);
}

char *convert(int num, int base)
{
  static char Representation[] = "0123456789ABCDEF";
  static char buffer[50];
  char *ptr;

  ptr = &buffer[49];
  *ptr = '\0';

  do
  {
    *--ptr = Representation[num % base];
    num /= base;
  } while (num != 0);

  return (ptr);
}

unsigned long runtime = 29000; // 29 Sekunden Laufzeit von auf bis zu oder umgekehrt

void localLoop();

void sendState();


bool stopIt0(void *);
bool stopIt1(void *);

class Rollladen
{
public:
  typedef enum
  {
    SAuf,
    SAb,
    SStop,
    SNone
  } RState;
  Rollladen();
  void setID(int id_p);
  void ScheduleAuf();
  void ScheduleAb();
  void ScheduleStop();
  void setUnsure() { unsure = true; };
  bool isPositionUnsure() { return unsure; };
  void Stop();
  void TimeoutStop();
  bool isRunning() { return (auf || ab); };
  void update();
  void setTimeout(int timeout);    // timeout in seconds
  void setPosition(float percent); // 0 is up, 100 = down

  float getPosition()
  {
    return position;
  };

private:
  int id;

  bool auf;
  bool ab;
  int timeout;
  bool unsure = true;
  void Auf();
  void Ab();
  float position = -1;
  int oldPosition = -1;
  float destination = 0;
  unsigned long currentTime;
  unsigned long lastTime;
#ifdef ESP32
  SemaphoreHandle_t Mutex;
#endif
  RState scheduleState;
};

void Rollladen::setPosition(float percent) // 0 is up, 100 = down
{
  if (!unsure || percent == 0 || percent == 100)
  {
    if (percent >= 0 && percent <= 100)
    {
      destination = percent;
      if (destination < position)
      {
        ScheduleAuf();
      }
      else
      {
        ScheduleAb();
      }
    }
  }
}
void Rollladen::TimeoutStop()
{
  if (auf)
  {
    //position = 0.0;
  }
  if (ab)
  {
    //position = 100.0;
  }
  destination = -1;
  Stop();
}
void Rollladen::setTimeout(int t)
{
  timeout = t;
}
void Rollladen::update()
{
  if (scheduleState != SNone)
  {
    if (scheduleState == SAuf)
    {
      Auf();
    }
    else if (scheduleState == SAb)
    {
      Ab();
    }
    else if (scheduleState == SStop)
    {
      Stop();
    }
    scheduleState = SNone;
  }
  currentTime = millis();

  float fract = ((currentTime - lastTime) / (float)(runtime)) * 100.0;
  if (fract < 0)
    fract = 0;
  if (fract > 100)
    fract = 100;

  if (auf)
  {
    position -= fract;

    if (position < 0)
      position = 0;
    if ((destination >= 0) && (position <= destination))
    {
      DebugPrintf("DestinationReached\n");
      destination = -1;
      Stop();
    }
  }
  if (ab)
  {
    position += fract;
    if (position > 100)
      position = 100;
    if ((destination >= 0) && (position >= destination))
    {

      DebugPrintf("DestinationReached\n");
      destination = -1;
      Stop();
    }
  }
  if ((int)position != oldPosition)
  {
    oldPosition = (int)position;
    char state[20];
    sprintf(state, "%d", oldPosition);
    if (id == 0)
    {
      client.publish("rollladen/" Name0 "/status", state);
    }
    else
    {
      client.publish("rollladen/" Name1 "/status", state);
    }
  }

  if (position > 100.0)
    position = 100.0;
  if (position < 0.0)
    position = 0.0;
  lastTime = currentTime;
}
Rollladen::Rollladen()
{
#ifdef ESP32
  Mutex = xSemaphoreCreateMutex(); /* Create Mutex */
#endif
}

void Rollladen::setID(int id_p)
{
  id = id_p;
  scheduleState = SNone;
  //pinMode(ledPin, OUTPUT);
  mcp.pinMode(runPin[id], OUTPUT);
  mcp.pinMode(aufAbPin[id], OUTPUT);

  mcp.digitalWrite(runPin[id], HIGH);
  mcp.digitalWrite(aufAbPin[id], HIGH);
  destination = -1;
  position = 0.0; // auf
  oldPosition = 0;
}
void Rollladen::ScheduleAb()
{
  scheduleState = SAb;
}
void Rollladen::ScheduleAuf()
{
  scheduleState = SAuf;
}
void Rollladen::ScheduleStop()
{
  scheduleState = SStop;
}
void Rollladen::Ab()
{
#ifdef ESP32
  if (xSemaphoreTake(Mutex, (TickType_t)1000))
#endif
  {
    DebugPrintf("Ab %d %d %d\n", ab, auf, runPin[id]);

    if (auf == true)
    {
      mcp.digitalWrite(runPin[id], HIGH);
      DebugPrintf("AufLow\n");
      delay(SSRDelay);
      auf = false;
    }
    if (!ab)
    {
      if (id == 0)
      {
        timer0.cancel(stop0Timer);
        stop0Timer = timer0.in(1000 * timeout, stopIt0);
      }
      else
      {
        timer0.cancel(stop1Timer);
        stop1Timer = timer0.in(1000 * timeout, stopIt1);
      }
      mcp.digitalWrite(aufAbPin[id], AB_STATE);
      delay(RELAISDELAY);
      mcp.digitalWrite(runPin[id], LOW);
#ifdef HaveLEDs
      mcp.digitalWrite(LEDAufPin[id], HIGH);
#endif

      DebugPrintf("AbHigh\n");
      ab = true;
    }
#ifdef ESP32
    xSemaphoreGive(Mutex);
#endif
  }
  /* else
  {
    Serial.println("COULD NOT SWITH TO AB!!!!!!!!!!!");
    Serial.printf("Ab %d %d\n", ab, auf);
  }*/
}

void Rollladen::Auf()
{
#ifdef ESP32
  if (xSemaphoreTake(Mutex, (TickType_t)1000))
#endif
  {
    DebugPrintf("Auf %d %d\n", ab, auf);
    if (ab == true)
    {
      mcp.digitalWrite(runPin[id], HIGH);
      //DebugPrintf("AbLow\n");
      delay(SSRDelay);
      ab = false;
    }
    if (!auf)
    {
      if (id == 0)
      {
        timer0.cancel(stop0Timer);
        stop0Timer = timer0.in(1000 * timeout, stopIt0);
      }
      else
      {
        timer0.cancel(stop1Timer);
        stop1Timer = timer0.in(1000 * timeout, stopIt1);
      }
      mcp.digitalWrite(aufAbPin[id], AUF_STATE);
      delay(RELAISDELAY);
      mcp.digitalWrite(runPin[id], LOW);
#ifdef HaveLEDs
      mcp.digitalWrite(LEDAbPin[id], HIGH);
#endif
      DebugPrintf("AufHigh\n");
      auf = true;
    }
#ifdef ESP32
    xSemaphoreGive(Mutex);
#endif
  }
}

void Rollladen::Stop()
{
#ifdef ESP32
  if (xSemaphoreTake(Mutex, (TickType_t)1000))
#endif
  {
    if (id == 0)
      timer0.cancel(stop0Timer);
    else
      timer0.cancel(stop1Timer);
    DebugPrintf("AbLow\n");
    DebugPrintf("AufLow\n");
    mcp.digitalWrite(runPin[id], HIGH);
    delay(SSRDelay);
    mcp.digitalWrite(aufAbPin[id], HIGH);
#ifdef HaveLEDs
    mcp.digitalWrite(LEDAbPin[id], LOW);
    mcp.digitalWrite(LEDAufPin[id], LOW);
#endif
    DebugPrintf("Stop\n");
    if ((ab == true) || (auf == true))
    {
      delay(RELAISDELAY);
    }
    ab = false;
    auf = false;
    destination = -1;
#ifdef ESP32
    xSemaphoreGive(Mutex);
#endif
  }
  /* else
  {
    DebugPrintf("COULD NOT STOP!!!!!!!!!!!");
    //DebugPrintf("AbLow");
    //DebugPrintf("AufLow");
    digitalWrite(runPin, HIGH);
    delay(SSRDelay);
    digitalWrite(aufAbPin, LOW);
      delay(RELAISDELAY);
  }*/
}

Rollladen rollladen[2];
bool stopIt0(void *)
{
  DebugPrintf("stopIt0\n");
  rollladen[0].TimeoutStop();
  return true;
}

bool stopIt1(void *)
{
  DebugPrintf("stopIt1\n");
  rollladen[1].TimeoutStop();
  return true;
}

const char *command0String = "rollladen/" Name0 "/command";
const char *command1String = "rollladen/" Name0 "/command";
void callback(char *topicP, byte *payloadP, unsigned int length)
{
  char topic[200];
  char payload[200];
  strncpy(topic, topicP, 200);
  strncpy(payload, (char *)payloadP, length);
  payload[length] = '\0';

  DebugPrintf("Message arrived [%s] %s\n", topic, payload);
  if (strcmp(topic, command0String) == 0)
  {
    if ((char)payload[0] == '0')
    {
      rollladen[0].ScheduleAuf();
    }
    else if ((char)payload[0] == 'S')
    {
      rollladen[0].ScheduleStop();
    }
    else if (((char)payload[0] == '1') && ((char)payload[1] == '0') && ((char)payload[2] == '0'))
    {
      rollladen[0].ScheduleAb();
    }
    else
    {
      int pos = 0;
      sscanf((const char *)payload, "%d", &pos);

      rollladen[0].setPosition(pos);
    }
  }
  if (strcmp(topic, "rollladen/" Name1 "/command") == 0)
  {

    if ((char)payload[0] == '0')
    {
      rollladen[1].ScheduleAuf();
    }
    else if ((char)payload[0] == 'S')
    {
      rollladen[1].ScheduleStop();
    }
    else if (((char)payload[0] == '1') && ((char)payload[1] == '0') && ((char)payload[2] == '0'))
    {
      rollladen[1].ScheduleAb();
    }
    else
    {
      int pos = 0;
      sscanf((const char *)payload, "%d", &pos);
      rollladen[1].setPosition(pos);
    }
  }
  #ifdef Terrasse
  else if (strcmp(topic, "terrasse/licht/command") == 0)
  {
    // Switch on the LED if an N was received as second character
    if ((char)payload[1] == 'N') // "ON"
    {
      lightState = true;
    }
    else
    {
      lightState = false;
    }
  }
  #endif

  sendState();
}

void sendState()
{
  #ifdef Terrasse
  if (lightState)
  {
    client.publish("terrasse/licht/status", "ON");
  }
  else
  {
    client.publish("terrasse/licht/status", "OFF");
  }
  #endif
}

void reconnect()
{
  #ifdef NO_MQTT
  return;
  #endif

  DebugPrintf("Attempting MQTT connection...\n");
#ifdef ESP32
  esp_task_wdt_init(25, true); //socket timeout is 15seconds
#endif
  // Attempt to connect
  if (client.connect("cl1" Zimmer))
  {
    DebugPrintf("connected\n");
    // Once connected, publish an announcement...
    sendState();
    // ... and resubscribe
    client.subscribe("rollladen/" Name0 "/command");
    client.subscribe("rollladen/" Name1 "/command");
    #ifdef Terrasse
    client.subscribe("terrasse/licht/command");
    #endif
  }
  else
  {
    DebugPrintf("failed, rc=");
    DebugPrintf("%d", client.state());
    DebugPrintf(" try again in 5 seconds\n");
  }
}
void setup()
{
  //pinMode(D5,OUTPUT);
  //digitalWrite(D5,HIGH);
  Serial.begin(115200);
  DebugPrintf("\n");

#ifdef IOMCP
  mcp.begin(); // use default address 0
#endif
  rollladen[0].setID(0);
  rollladen[1].setID(1);
  rollladen[0].setTimeout(30);
  rollladen[1].setTimeout(30);

  if (bAufPin1 >= 0)
  {
    mcp.pinMode(bAufPin1, INPUT);
    mcp.pullUp(bAufPin1, HIGH); // turn on a 100K pullup internally
    mcp.pinMode(bAbPin1, INPUT);
    mcp.pullUp(bAbPin1, HIGH); // turn on a 100K pullup internally
  }
  if (bAufPin2 >= 0)
  {
    mcp.pinMode(bAufPin2, INPUT);
    mcp.pullUp(bAufPin2, HIGH); // turn on a 100K pullup internally
    mcp.pinMode(bAbPin2, INPUT);
    mcp.pullUp(bAbPin2, HIGH); // turn on a 100K pullup internally}
  }

  if (bLightPin >= 0)
  {
    mcp.pinMode(bLightPin, INPUT);
    mcp.pinMode(LightPin, OUTPUT);
    mcp.pullUp(bLightPin, HIGH); // turn on a 100K pullup internally
  }
  bool ledState = true;

#ifdef HaveLEDs
  mcp.pinMode(LEDAufPin1, OUTPUT);
  mcp.digitalWrite(LEDAufPin1, false);
  mcp.pinMode(LEDAbPin1, OUTPUT);
  mcp.digitalWrite(LEDAbPin1, false);
#ifndef SingleRoller
  mcp.pinMode(LEDAufPin2, OUTPUT);
  mcp.pinMode(LEDAbPin2, OUTPUT);
#endif
#ifdef HaveLight
  mcp.pinMode(LEDLightPin, OUTPUT);
  mcp.digitalWrite(LEDLightPin, ledState);
#endif
#endif
#ifdef Bad
  mcp.pinMode(SSR_P3, OUTPUT);
  mcp.digitalWrite(SSR_P3, true);
  mcp.pinMode(SSR_P2, OUTPUT);
  mcp.digitalWrite(SSR_P2, true);
#elif defined Jan
  mcp.pinMode(SSR_P3, OUTPUT);
  mcp.digitalWrite(SSR_P3, true);
  mcp.pinMode(SSR_P2, OUTPUT);
  mcp.digitalWrite(SSR_P2, true);
#endif

  DebugPrintf("sRollladen 2 connecting to wifi\n");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
#ifdef HaveLight
    ledState = !ledState;
    mcp.digitalWrite(LEDLightPin, ledState);
#endif
  }

  ArduinoOTA.setPort(8266);
#ifdef Jule
  ArduinoOTA.setHostname("julesrollladen");
#elif defined Bad
  ArduinoOTA.setHostname("RollladenBad");
#elif defined Jan
  ArduinoOTA.setHostname("RollladenJan");
#else
  ArduinoOTA.setHostname("Rollladen8266");
#endif
  ArduinoOTA.onStart([]() {
    DebugPrintf("Start\n");
  });
  ArduinoOTA.onEnd([]() {
    DebugPrintf("\nEnd\n");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DebugPrintf("Progress: %u %% \r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DebugPrintf("Error[ %u]: ", error);
    if (error == OTA_AUTH_ERROR)
      DebugPrintf("Auth Failed\n");
    else if (error == OTA_BEGIN_ERROR)
      DebugPrintf("Begin Failed\n");
    else if (error == OTA_CONNECT_ERROR)
      DebugPrintf("Connect Failed\n");
    else if (error == OTA_RECEIVE_ERROR)
      DebugPrintf("Receive Failed\n");
    else if (error == OTA_END_ERROR)
      DebugPrintf("End Failed\n");
  });
  ArduinoOTA.begin();

  DebugPrintf("%s\n", WiFi.localIP().toString().c_str());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  server.begin();

  rollladen[0].ScheduleStop();
  #ifndef SingleRoller
  rollladen[1].ScheduleStop();
  #endif
}

void reconnectWifi()
{
  bool ledState = false;
  while (WiFi.status() != WL_CONNECTED)
  {
    long start = millis();
    while (millis() - start < 500)
    {
      localLoop();
    }
    ledState = !ledState;
#ifdef HaveLight
    mcp.digitalWrite(LEDLightPin, ledState);
#endif
  }
}
long lastReconnectAttempt = 0;
long lastReconnectWifiAttempt = 0;


void loop()
{
  debounceButton::update();
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - lastReconnectWifiAttempt > 60000) // every 60 seconds
    {
      lastReconnectWifiAttempt = now;
      // Attempt to reconnect
      reconnectWifi();
    }
  }
  if (!client.connected())
  {
    if (now - lastReconnectAttempt > 10000) // every 10 seconds
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      reconnect();
    }
  }
  else
  {
    // Client connected

    client.loop();
  }

  localLoop(); // local updates which have to be done during reconnect as well;
}

void localLoop()
{
  ArduinoOTA.handle();
  if(bLight.wasPressed())
  {
    lightState = !lightState;
  }
 
  static bool os = false;

  //digitalWrite(D5,lightState);
  if (os != lightState)
  {
    os = lightState;
#ifdef HaveLight
    mcp.digitalWrite(LEDLightPin, lightState);
    mcp.digitalWrite(LightPin, !lightState);
#endif
    sendState();
  }

  if (bAuf1.wasPressed())
  {
    if (rollladen[0].isRunning())
    {
      DebugPrintf("BStop1\n");
      rollladen[0].ScheduleStop();
    }
    else
    {
      DebugPrintf("BstartAuf1\n");
      rollladen[0].ScheduleAuf();
    }
  }
  if (bAb1.wasPressed())
  {
    if (rollladen[0].isRunning())
    {
      DebugPrintf("BStop1\n");
      rollladen[0].ScheduleStop();
    }
    else
    {
      DebugPrintf("BstartAb1\n");
      rollladen[0].ScheduleAb();
    }
  }
  rollladen[0].update();
  if (bAuf2.wasPressed())
  {
    
      DebugPrintf("%d\n",bAufPin2);
    if (rollladen[1].isRunning())
    {
      DebugPrintf("BStop2\n");
      rollladen[1].ScheduleStop();
    }
    else
    {
      DebugPrintf("BstartAuf2\n");
      rollladen[1].ScheduleAuf();
    }
  }
  if (bAb2.wasPressed())
  {
    if (rollladen[1].isRunning())
    {
      DebugPrintf("BStop2\n");
      rollladen[1].ScheduleStop();
    }
    else
    {
      DebugPrintf("BstartAb2\n");
      rollladen[1].ScheduleAb();
    }
  }
  rollladen[0].update();
  timer0.tick();
  rollladen[1].update();
}
