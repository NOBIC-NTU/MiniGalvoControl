#include "nanos.h"
#include "XY2_100.h"

XY2_100 galvo;
static uint64_t _stamp;
// FYI:
//const static int pwm_pin_clock = 22;
//const static int pwm_pin_sync = 17;  
//const static int pwm_pin_dataX = 19;
//const static int pwm_pin_dataY = 14;

/* FYI: c12f0001c22f0001 to set xy2-100 control
 *  Command:  8x2F 0000  Read Command Input Source
 *  Response: 552F yyyy
 *  Command:  Cx2F yyyy  Write Command Input Source
 *  Response: AA2F yyyy
 *  x = Axis indicator
 *  yyyy = Input Source: 00 = Analog; 1 = XY2-100-compatible digital; 2 = FB4-compatible digital
 *  0 is the default value
 */
// Keep track of last commanded position
int16_t x = -1;
int16_t y = 1;

// Temp variable
char msgbuf[256]; // line buffer to print messages
  
void setup() {
  Serial.begin(115200);
  galvo.begin();
  _stamp = nanos();
}

void listen_command() {
  // Fetch command if avaiable
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    char *cmd = (char *)str.c_str();
    if (strlen(cmd) > 1) {
      parse_command(cmd);
    } else {
      sprintf(msgbuf, "cmd too short?\n");
      Serial.print(msgbuf);
    }
  }
}
void parse_command(char *cmd) {
  // Check if command is valid, do associated actions.
  char *tok;
  tok = strtok(cmd, " \n");
  
  if (strcmp(tok, "??") == 0) {
    sprintf(msgbuf, "x,y: %d,%d\n", x,y);
    Serial.print(msgbuf);
  } else if (strcmp(tok, "sweepx") == 0) {
    sprintf(msgbuf, "%s!\n", "sweepx");
    Serial.print(msgbuf);
    sweepx();
  } else if (strcmp(tok, "sweepy") == 0) {
    sprintf(msgbuf, "%s!\n", "sweepy");
    Serial.print(msgbuf);
    sweepy();
  } else if (strcmp(tok, "togglex") == 0) {
    sprintf(msgbuf, "%s!\n", "togglex");
    Serial.print(msgbuf);
    togglex();
  } else if (strcmp(tok, "toggley") == 0) {
    sprintf(msgbuf, "%s!\n", "toggley");
    Serial.print(msgbuf);
    toggley();
  } else if (strcmp(tok, "togglesquare") == 0) {
    sprintf(msgbuf, "%s!\n", "togglesquare");
    Serial.print(msgbuf);
    togglesquare();
  } else if (strcmp(tok, "x?") == 0) {
    sprintf(msgbuf, "%s: %d\n", "x", x);
    Serial.print(msgbuf);
  } else if (strcmp(tok, "y?") == 0) {
    sprintf(msgbuf, "%s: %d\n", "y", y);
    Serial.print(msgbuf);
  } else if (strcmp(tok, "y") == 0) {
    tok = strtok(NULL, " \n");
    if (tok==NULL) {
      sprintf(msgbuf, "%s: %d\n", "y", y); // no value given, print actual
    } else {
      long val = strtol(tok, NULL, 10); // Parse as integer (base 10)
      sprintf(msgbuf, "set %s: %d\n", "y", (int)val);
      y = val;
    }
    Serial.print(msgbuf);
  } else if (strcmp(tok, "x") == 0) {
    tok = strtok(NULL, " \n");
    if (tok==NULL) {
      sprintf(msgbuf, "%s: %d\n", "x", x); // no value given, print actual
    } else {
      long val = strtol(tok, NULL, 10); // Parse as integer (base 10)
      sprintf(msgbuf, "set %s: %d\n", "x", (int)val);
      x = val;
    }
    Serial.print(msgbuf);
  } else if (strlen(tok)>1) {
    sprintf(msgbuf, "Unknown command: '%s'\n", tok);
    Serial.write(msgbuf);
  }
}

void galvogo()  {
  // Effectuate new coordinate
  uint16_t originx = 32768; // represents the middle of the scan field
  uint16_t originy = 32768; // represents the middle of the scan field
  galvo.goTo( x-originx, y-originy );
  // Serial.printf("x,y = %05d,%05d\n", x,y);
}

void sweepx() {
  for (x=-2500;x<2501;x+=1) {
    galvogo();
    delay(1);
  }
  for (x=2500;x>-2501;x-=1) {
    galvogo();
    delay(1);
  }
}
void sweepy() {
  for (int i=0; i<5; i++){
  for (y=-2500;y<2501;y+=1) {
    galvogo();
    nanosleep(10000L);
  }
  for (y=2500;y>-2501;y-=1) {
    galvogo();
    nanosleep(10000L);
  }
  }
}
void togglex(){
  for (int i=0;i<100;i++) {
    x = -x;
    galvogo();
    delay(1);
  }
}
void toggley(){
  for (int i=0;i<100;i++) {
    y = -y;
    galvogo();
    delay(1);
  }
}
void togglesquare() {
  for (int i=0;i<100;i++) {
    y = -y;
    galvogo();
    delay(1);
    x = -x;
    galvogo();
    delay(1);
  }
}
void loop() {  
  listen_command();
  galvogo();
  delay(200);
}
