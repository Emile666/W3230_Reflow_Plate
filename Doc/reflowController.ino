/* *****************************************
   Controller per la saldatura a rifusione

             www.elettronicain.it

             Autore: Massimo Divito

   ***************************************** */

#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <MAX6675_Thermocouple.h>
#include <PID_v1.h>

#include <FastPwmPin.h>

#define SCK_PIN 8   // definizione dei pin utilizzati
#define CS_PIN 9    // per la lettura della sonda tipo K
#define S0_PIN 11   // tramite il MAX6675
#define encoderA 2  // definizione dei pin utilizzati per
#define encoderB 4  // la lettura dell'encoder rotativo

U8X8_SSD1306_128X64_NONAME_SW_I2C oled(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display
MAX6675_Thermocouple tc = MAX6675_Thermocouple(SCK_PIN, CS_PIN, S0_PIN);

uint8_t thermometerUP[8] = {0x00, 0x00, 0xFE, 0xC1, 0xC1, 0x7FE, 0x00, 0x00}; //icona termometro parte superiore
uint8_t thermometerDWN[8] = {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C}; //icona termometro parte inferiore

int profili [2][6][2] = {  
  {{0, 0}, {60, 150}, {180, 150}, {240, 225}, {260, 225}, {380, 0}},  // profilo Lead
  {{0, 0}, {60, 150}, {180, 180}, {240, 255}, {255, 255}, {320, 0}}   // profilo Lead-free
};

char *fasi[] = {"Preheat", "Soak   ", "Soak   ", "Reflow ", "Cooling"};
int faseAttuale;

int tempMinPreheater = 60 ; //imposto la temperatura minima settabile come preriscaldatore
int tempMaxPreheater = 250; //imposto la temperatura massima settabile come preriscaldatore
int tempStepPreheater = 5; //imposto il valore degli step di regolazione per il preriscaldatore
int tempAmbiente = 50; //imposto la temperatura ambiente massima, se il piano è più caldo non è possibile avviare il processo di rifusione
int tempAdjust = 0; //correzione della temperatura letta dalla sonda

int piezo = 13;
int P1 = 7;
int riscaldatore = 10;
int encoder, encoderPrecedente;
bool tasto, tastoPrecedente;
int profilo = 0;
long secondo = 1000;
boolean reflowRunning = false;
boolean preheaterRunning = false;
int tempTarget, tempRead;
int tempPreheater;

unsigned long timer1, deltaTimer1, reflowStartTime ; //Timer per la gestione del processo di reflow
unsigned long timer2, deltaTimer2;  //Timer per la gestione della pressione di P1
unsigned long timer3, deltaTimer3;  //Timer per la gestione del preriscaldatore
unsigned long timer4, deltaTimer4;  //Timer per la gestione dell'aggiornamento del display

long reflowNowTime, xa, xb; //Variabili long per la funzione di interpolazione lineatre
int ya, yb; //Variabili int per la funzione di interpolazione lineare

double Setpoint, Input, Output; //Variabili in uso dal PID

double reKp = 8, reKi = 0.025 , reKd = 8; //valori PID reflow
double preKp = 3, preKi = 0.025 , preKd = 20; //valori PID preheater


PID myPID(&Input, &Output, &Setpoint, reKp, reKi, reKd, DIRECT);

int WindowSize = 250;

void setup() {
  pinMode(riscaldatore, OUTPUT);  //  Solid State Relay
  pinMode(encoderA, INPUT_PULLUP); //imposta i pin dell'encoder come input abilitando la resistenza di pullup integrata
  pinMode(encoderB, INPUT_PULLUP); //imposta i pin dell'encoder come input abilitando la resistenza di pullup integrata
  pinMode(P1, INPUT); //START/STOP button
  pinMode(piezo, OUTPUT); //piezo
  FastPwmPin::enablePwmPin(riscaldatore, 1L, 0);  //abilita il PWM ad 1Hz sull'uscita del ricaldatore
  oled.begin(); //inizializza il risplay
  displayRefresh(); //aggiorno il display
  encoderPrecedente = digitalRead(encoderA); //legge lo stato dell'encoder
  timer1 = millis();  //Imposto il timer1
  timer3 = millis();  //Imposto il timer3
  timer4 = millis();  //Imposto il timer4  
  myPID.SetOutputLimits(0, 100);
  myPID.SetMode(MANUAL);
  leggiSondaK();
  delay(1000); 
}

void loop() {

  // Lettura dell'encoder rotativo
  // -----------------------------

  if (!reflowRunning) { //se non è in corso il reflow.........
    if (!preheaterRunning) { //e non è avviato il preriscaldamento leggo l'encoder e setto la variabile 'profilo' per il menu
      encoder = digitalRead(encoderA);
      if ((encoderPrecedente == HIGH) && (encoder == LOW)) {
        if (digitalRead(encoderB)) {
          if (profilo > 0) {
            profilo--;
          }
        } else {
          if (profilo < 2) {
            profilo++;
          }
        }
        displayRefresh();  //aggiorno il display
      }
    } else {  //altrimenti leggo l'encoder e setto la variabile 'tempPreheater' per impostare la temperatura del piano
      encoder = digitalRead(encoderA);
      if ((encoderPrecedente == HIGH) && (encoder == LOW)) {
        if (digitalRead(encoderB)) {
          if (tempPreheater >= (tempMinPreheater + tempStepPreheater)) {
            tempPreheater -= tempStepPreheater; //diminuisco la temperatura
          }
        } else {
          if (tempPreheater <= (tempMaxPreheater - tempStepPreheater)) {
            tempPreheater += tempStepPreheater; //aumento la temperatura
          }
        }
        displayRefresh();  //aggiorno il display
      }
    }
    encoderPrecedente = encoder;
  }

  // Fine lettura encoder rotativo
  // -----------------------------




  // Lettura del pulsante P1 nelle condizioni di pressione breve e pressione prolungata
  // ----------------------------------------------------------------------------------

  tasto = digitalRead(P1);
  if (!tastoPrecedente && tasto) { // se premo P1
    timer2 = millis();  //Imposto il timer2
  }
  if (tastoPrecedente && tasto) {   //mantengo premuto P1
    deltaTimer2 = millis() - timer2;
    if (deltaTimer2 > 2000) {   //se lo tengo premuto per più di 2 secondi...
      if (reflowRunning) {  //.... se il processo di reflow è in croso lo interrompo
        reflowStop();  //fermo il reflow
        delay(500);
      }
      if (preheaterRunning) {  //.... se il processo di preriscaldamento è in croso lo interrompo
        preheaterEnd();  //termino il preriscaldamento
        delay(500);
      }
    }
  }
  if (tastoPrecedente && !tasto) {   //rilascio P1
    if ((deltaTimer2 < 2000) && (!reflowRunning) && (!preheaterRunning)) {   //se lo rilascio prima di due secondi e non è in corso il processo di reflow nè quello di preriscaldatore
      if ((profilo < 2)) { //se  ho selezionato un profilo di reflow....
        leggiSondaK();
        if (tempRead < tempAmbiente) { //... seil piano è abbastanza freddo ed lo avvio
          reflowStart();
        } else {
          displayError();
        }
      } else { //altrimenti avvio la funzione preriscaldatore
        preheaterStart();
      }
    }
  }
  tastoPrecedente = tasto;

  // Fine lettura del pulsante P1
  // ----------------------------




  // Gestione funzionalità Reflow
  // ----------------------------

  deltaTimer1 = millis() - timer1;

  if ((deltaTimer1 >= 250) && (reflowRunning)) {  //ogni 250 ms
    reflowNowTime = (unsigned long)(millis() - reflowStartTime); // Calcola da quanti millisecondi è iniziato il ciclo di reflow;
    for (int i = 0; i < 6; i++) {
      if ((reflowNowTime >= (profili[profilo][i][0] * secondo)) && ( reflowNowTime < (profili[profilo][i + 1][0] * secondo))) {
        xa = profili[profilo][i][0] * secondo;
        xb = profili[profilo][i + 1][0] * secondo;
        ya = profili[profilo][i][1];
        yb = profili[profilo][i + 1][1];

        tempTarget = ((((reflowNowTime - xa) * (yb - ya)) / (xb - xa))) + ya;
        // y0=( ( (x0-xa)*(yb-ya) ) / (xb-xa) ) + ya   Formula interpolazione lineare

        faseAttuale = i;
        regolaRiscaldatore();
      }
    }
    if (reflowNowTime > (profili[profilo][5][0] * secondo)) {   //Se abbiamo superato il tempo dell'ultima fase del profilo fermiamo il processo
      reflowEnd();
    }
    timer1 = millis();
  }

  // Fine gestione funzionalità Reflow
  // ---------------------------------




  // Gestione funzionalità preriscaldatore
  // -------------------------------------

  deltaTimer3 = millis() - timer3;

  if ((deltaTimer3 >= 250) && (preheaterRunning)) {  //ogni 250 ms
    tempTarget = tempPreheater;
    regolaRiscaldatore();
    timer3 = millis();
  }

  // Fine gestione funzionalità preriscaldatore
  // ------------------------------------------




  // Aggiornamento display ogni mezzo secondo durante il reflow ed il presiscaldamento
  // ---------------------------------------------------------------------------------

  deltaTimer4 = millis() - timer4;

  if ((deltaTimer4 >= 500) && (preheaterRunning || reflowRunning ))  {
    displayRefresh();
    timer4 = millis();
  }

  // Fine aggiornamento display
  // --------------------------

}

void beep() {
  tone(piezo, 1000, 100);
  delay(100);
}

void longBeep() {
  tone(piezo, 1000, 1000);
  delay(100);
}

void endBeep() {
  tone(piezo, 1000, 100);
  delay(1000);
  tone(piezo, 1000, 100);
  delay(1000);
  tone(piezo, 1000, 1000);
  delay(100);
}

void preheaterStart() {
  preheaterRunning = true;
  tempPreheater = tempMinPreheater; //imposta la temperatura al minimo
  tempTarget = tempPreheater;   
  myPID.SetTunings(preKp, preKi, preKd); 
  myPID.SetMode(AUTOMATIC);  //attiva il PID  
  //regolaRiscaldatore();
  beep();
  displayClear();
}

void preheaterEnd() {
  preheaterRunning = false;
  myPID.SetMode(MANUAL);  //disattiva il PID
  analogWrite2(0); //spegni riscaldatore
  longBeep();
  displayClear();
}

void reflowStart() {
  reflowRunning = true;
  reflowStartTime = millis();
  myPID.SetTunings(reKp, reKi, reKd);
  myPID.SetMode(AUTOMATIC);  //attiva il PID
  beep();
  displayClear();
}

void reflowStop() {
  reflowRunning = false;
  myPID.SetMode(MANUAL);  //disattiva il PID
  analogWrite2(0); //spegni riscaldatore
  longBeep();
  displayClear();
}

void reflowEnd() {
  reflowRunning = false;
  myPID.SetMode(MANUAL);  //disattiva il PID
  analogWrite2(0); //spegni riscaldatore
  endBeep();
  displayClear();
}

void leggiSondaK() {
  tempRead = tc.readCelsius() + tempAdjust;
}

void regolaRiscaldatore() {
  leggiSondaK();
  Setpoint = tempTarget;
  Input = tempRead;  
  myPID.Compute();
  analogWrite2(Output);
}

void displayClear() {
  oled.clear();
  delay(100);
  displayRefresh();
}

void displayError() {
  oled.clear();
  delay(100);
  char _str[5]; //variabile per la manipolazione delle stringhe
  oled.setFont(u8x8_font_amstrad_cpc_extended_f);                        
  oled.drawString(0, 1, "  Temperatura");
  oled.drawString(0, 2, "  troppo alta");
  oled.drawTile(3, 5, 1, thermometerUP); //scrive la parte superiore dell'icona del termometro
  oled.drawTile(3, 6, 1, thermometerDWN); //scrive la parte inferiore dell'icona del termometro
  sprintf(_str, "%3i\xb0", tempRead); //formatta la stringa come intero a tre cifre ed aggiunge il simbolo "°"
  oled.draw2x2String(5, 5, _str);
  sprintf(_str, "%3i", (int)tempRead);
  longBeep();
  delay(4000);
  displayClear();
}


void displayRefresh() {

  if ((!reflowRunning) && (!preheaterRunning)) { //Se non sono avviati reflow o preheater sono nel menu
    oled.setFont(u8x8_font_amstrad_cpc_extended_r);
    oled.inverse();
    oled.draw1x2String(0, 0, "Hotplate Reflow ");
    oled.noInverse();
    oled.drawString(1, 3, "Lead");
    oled.drawString(1, 5, "Lead-free");
    oled.drawString(1, 7, "Preriscaldatore");
    oled.drawString(0, 3, (profilo == 0 ? ">" : " ")); //se è profilo=0 scrivo ">" altrimenti scrivo " "
    oled.drawString(0, 5, (profilo == 1 ? ">" : " ")); //se è profilo=1 scrivo ">" altrimenti scrivo " "
    oled.drawString(0, 7, (profilo == 2 ? ">" : " ")); //se è profilo=2 scrivo ">" altrimenti scrivo " "
  }

  if (reflowRunning) {  //Se è avviato il reflow
    char _str[5]; //variabile per la manipolazione delle stringhe
    oled.setFont(u8x8_font_amstrad_cpc_extended_f);
    oled.drawString(3, 0, (profilo == 0 ? "   Lead" : "Lead-free")); //scrive il profilo che si sta eseguendo
    oled.drawTile(1, 2, 1, thermometerUP); //scrive la parte superiore dell'icona del termometro
    oled.drawTile(1, 3, 1, thermometerDWN); //scrive la parte inferiore dell'icona del termometro
    sprintf(_str, "%3i\xb0", tempTarget); //formatta la stringa come intero a tre cifre ed aggiunge il simbolo "°"
    oled.draw2x2String(3, 2, _str);
    sprintf(_str, "%3i\xb0", tempRead); //formatta la stringa come intero a tre cifre ed aggiunge il simbolo "°"
    oled.drawString(12, 2, _str);
    oled.drawString(1, 5, "Fase :");
    oled.drawString(8, 5, fasi[faseAttuale]);
    sprintf(_str, "%3i", (int)tempRead);
    oled.drawString(1, 7, "Tempo:");
    oled.drawString(11, 7, "/");
    sprintf(_str, "%3i", (int)(reflowNowTime / 1000));
    oled.drawString(8, 7, _str);
    sprintf(_str, "%3i", profili[profilo][5][0]);
    oled.drawString(12, 7, _str);
  }

  if (preheaterRunning) {  //Se è avviato il preriscaldatore
    char _str[5]; //variabile per la manipolazione delle stringhe
    oled.setFont(u8x8_font_amstrad_cpc_extended_f);
    oled.drawString(0, 1, "Preriscaldatore");
    oled.drawTile(1, 4, 1, thermometerUP); //scrive la parte superiore dell'icona del termometro
    oled.drawTile(1, 5, 1, thermometerDWN); //scrive la parte inferiore dell'icona del termometro
    sprintf(_str, "%3i\xb0", (int)tempPreheater); //formatta la stringa come intero a tre cifre ed aggiunge il simbolo "°"
    oled.draw2x2String(3, 4, _str);
    sprintf(_str, "%3i\xb0", (int)tempRead); //formatta la stringa come intero a tre cifre ed aggiunge il simbolo "°"
    oled.drawString(12, 4, _str);    
  }
}

void analogWrite2(int value) {
  if ((value > 0) & (value <= 100)) {
    FastPwmPin::enablePwmPin(riscaldatore, 1L, value);
  } else {
    digitalWrite(riscaldatore, LOW);
  }
}
