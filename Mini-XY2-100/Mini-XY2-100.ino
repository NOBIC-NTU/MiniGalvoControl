#include "nanos.h"
#include "XY2_100.h"

XY2_100 galvo;
// FYI:
//const static int pwm_pin_clock = 22;
//const static int pwm_pin_sync = 17;
//const static int pwm_pin_dataX = 19;
//const static int pwm_pin_dataY = 14;

/* FYI: c12f0001c22f0001 to set xy2-100 control
    Command:  8x2F 0000  Read Command Input Source
    Response: 552F yyyy
    Command:  Cx2F yyyy  Write Command Input Source
    Response: AA2F yyyy
    x = Axis indicator
    yyyy = Input Source: 00 = Analog; 1 = XY2-100-compatible digital; 2 = FB4-compatible digital
    0 is the default value
*/

// Keep track of last commanded positions
int16_t x = 0;
int16_t y = 0;
// Several special points used to determine walks
int16_t c = 0; // (c)enter (x)
int16_t m = 0; // (m)iddle (y)
int16_t w = -1000; // (w)est point (smallest x)
int16_t e = 1000; // (e)ast point (biggest x)
int16_t s = -1000; // (s)outh point (smallest y)
int16_t n = 1000; // (n)orth point (biggest y)
int16_t dx = 10; // delta x (jog this many units per dt)
int16_t dy = 10; // delta y (jog this many units per dt)
uint64_t tx = 100000000; // delta (t)ime (update pos every 100000000 ns = 100 ms)
uint64_t ty = 100000000; // delta (t)ime (update pos every 100000000 ns = 100 ms)
uint64_t tlisten = 250000000; // delta (t)ime (update listen commands every 250ms)
uint64_t last_tx = 0; // last time tx was passed
uint64_t last_ty = 0; // last time ty was passed
uint64_t last_tlisten = 0; // last time tlisten was passed

// Walk functions (crude waveforms) in x (and y)
#define F_MANUAL 0 // go manual x
#define F_CENTER 1 // go c
#define F_ZAGZAG 2 // go w...ew...e
#define F_ZIGZIG 3 // go e...we...w
#define F_ZIGZAG 4 // go e...ww...e
#define F_ZIPZAP 5 // go ewewew
// Mind the order of the above and in here:
const char* f_names[F_ZIPZAP + 1] = {"MANUAL", "CENTER", "ZAGZAG", "ZIGZIG", "ZIGZAG", "ZIPZAP"};
int16_t fx = F_CENTER;
int16_t fy = F_CENTER;

// Temp variable
char msgbuf[256]; // line buffer to print messages
const int led_pin =  LED_BUILTIN;// debug using the led
bool led_state = LOW;

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
  galvo.begin();
  last_tx = nanos(); // last time dt was passed is now
  last_tlisten = nanos(); // last time tlisten was passed is now
}

void msg_help() {
  Serial.println("");
  Serial.println("Mini-XY2-100 control (1.0) (on Teensy40) (for Mach-DSP Servo Driver)");
  Serial.println("");
  Serial.println("Usage: <c> [v]");
  Serial.println("");
  Serial.println("Where <c>ommand is one of the mostly 1- or 2-letter commands below, which");
  Serial.println("allow to query or to set one of the 13 parameters (if optional [v]alue is provided):");
  Serial.println("");
  Serial.println("x,fx,(w)est, (c)enter,(e)east,dx   # x positioning (centidegrees)");
  Serial.println("y,fy,(s)outh,(m)iddle,(n)orth,dy   # y positioning (centidegrees)");
  Serial.println("tx,ty                              # time between coordinate (x,y) updates (ns)");
  Serial.println("?                                  # print current (x,y) and (fx,fy) function");
  Serial.println("help                               # print this help page");
  Serial.println("???, status                        # list the value of all 13 parameters");
  Serial.println("");
  Serial.println("N.B. Unless (fx,fy) are nonzero, (x,y) are stepped every (tx,ty) time, where ");
  Serial.println("(fx,fy) can be: 0(Manual), 1(Center), 2(ZagZag), 3(ZigZig), 4(ZigZag) and 5(ZipZap),");
  Serial.println("which describe crude functions walked with step size (dx,dy) every step.");
  Serial.println("");
  Serial.println("Just in case, if you need to set the Mach-DSP Servo Driver to listen and");
  Serial.println("to use proper offset position (centering x,y), issue these four commands,");
  Serial.println("c12f0001c22f0001c13a0089c23a0070 (on your other serial port)");
  sprintf(msgbuf, "\n");
}
void msg_status() {
  sprintf(msgbuf, "x,y: %+05d,%+05d, fx,fy: %s,%s\n", x, y, f_names[fx], f_names[fy]); Serial.print(msgbuf);
  sprintf(msgbuf, "(w)est ,(c)enter,(e)ast : %+05d,%+05d,%+05d\n", w, c, e); Serial.print(msgbuf);
  sprintf(msgbuf, "(s)outh,(m)iddle,(n)orth: %+05d,%+05d,%+05d\n", s, m, n); Serial.print(msgbuf);
  sprintf(msgbuf, "(dx)   ,(dy)    ,(dt)   : %+05d,%+05d,%05llu\n", dx, dy, tx); Serial.print(msgbuf);
}

void listen_command() {
  // Fetch command if avaiable
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    char *cmd = (char *)str.c_str();
    if (strlen(cmd) > 0) {
      parse_command(cmd);
    } else {
      sprintf(msgbuf, "cmd too short?\n");
      Serial.print(msgbuf);
    }
  }
}
void parse_command(char *words) {
  // Check if command is valid, do associated actions.
  char *cmd = strtok(words, " \n");
  char *arg = strtok(NULL, " \n");
  long val = 0;
  if (arg != NULL) {
    val = strtol(arg, NULL, 10); // Parse as integer (base 10)
  }
  sprintf(msgbuf, "Unknown command: '%s'\n", cmd); // fallback

  if (strlen(cmd) == 1 || strlen(cmd) == 2) { // single character command : print value
    if (arg == NULL) {
      switch ((char)cmd[0]) {
        case '?': sprintf(msgbuf, "x,y: %+05d,%+05d, fx,fy: %s,%s\n", x, y, f_names[fx], f_names[fy]); break; // '?' or "??"
        case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, x); break;
        case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, y); break;
        case 'c': sprintf(msgbuf, "%s: %+05d\n", cmd, c); break;
        case 'm': sprintf(msgbuf, "%s: %+05d\n", cmd, m); break;
        case 'w': sprintf(msgbuf, "%s: %+05d\n", cmd, w); break;
        case 'e': sprintf(msgbuf, "%s: %+05d\n", cmd, e); break;
        case 's': sprintf(msgbuf, "%s: %+05d\n", cmd, s); break;
        case 'n': sprintf(msgbuf, "%s: %+05d\n", cmd, n); break;
        case 'd': // two-letter commands with d
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, dx); break; // 'dx'
            case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, dy); break; // 'dy'
          } break;
        case 't': // two-letter commands with t
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %05llu\n", cmd, tx); break; // 'tx'
            case 'y': sprintf(msgbuf, "%s: %05llu\n", cmd, ty); break; // 'ty'
            default: sprintf(msgbuf, "tx,ty: (%05llu,%05llu)\n", tx, ty); break; // 't'
          } break;
        case 'f': // two-letter commands with f
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %d %s\n", cmd, fx, f_names[fx]); break; // 'fx'
            case 'y': sprintf(msgbuf, "%s: %d %s\n", cmd, fy, f_names[fy]); break; // 'fy'
            default: sprintf(msgbuf, "fx,fy: %d %s, %d %s\n", fx, f_names[fx], fy, f_names[fy]); break; // 'f'
          } break;
      }
    } else {
      // argument given
      sprintf(msgbuf, "set %s: %ld\n", cmd, val); // message
      // do assignment:
      switch ((char)cmd[0]) {
        case 'x': x = val; break; // useless unless in F_MANUAL mode
        case 'y': y = val; break; // useless unless in F_MANUAL mode
        case 'c': c = val; break;
        case 'm': m = val; break;
        case 'w': w = val; break;
        case 'e': e = val; break;
        case 's': s = val; break;
        case 'n': n = val; break;
        case 'd': // two-letter commands with d
          switch ((char)cmd[1]) {
            case 'x': dx = val; break; // 'dx'
            case 'y': dy = val; break; // 'dy'
          } break;
        case 't': // two-letter commands with t
          switch ((char)cmd[1]) {
            case 'x': tx = val; break; // 'tx'
            case 'y': ty = val; break; // 'ty'
          } break;
        case 'f': // two-letter commands with f
          switch ((char)cmd[1]) {
            case 'x': fx = val; break; // 'fx'
            case 'y': fy = val; break; // 'fy'
          } break;
      }
    }
  } else if (strcmp(cmd, "help") == 0) {
    msg_help();
  } else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "???") == 0) {
    msg_status();
    sprintf(msgbuf, "nanos(): %llu\n", nanos());
  }
  Serial.print(msgbuf);
}

void galvogo() {
  // Effectuate x, y coordinates
  uint16_t originx = 32768; // represents the middle of the scan field
  uint16_t originy = 32768; // represents the middle of the scan field
  galvo.goTo( x - originx, y - originy );
  // Serial.printf("x,y = %05d,%05d\n", x,y);
}

// Keep track of zig vs zag
bool f_zigx = true;
bool f_zigy = true;
bool f_touch = true;
void funny_walkx() {
  // Update positions according to their crude walk  // X...
  f_touch = true;
  switch (fx) {
    case F_CENTER: x = c; break; // zzzzzz... at center
    case F_ZIGZIG: x = x + dx; if (x > e || x < w) x = w; break; // zig: w..ew..e
    case F_ZAGZAG: x = x - dx; if (x > e || x < w) x = e; break; // zag: e..we..w
    case F_ZIGZAG:
      if ( x + dx > e || x - dx < w ) f_zigx = !f_zigx; // toggle zig<->zag
      if (f_zigx) x = x + dx; else x = x - dx; // go zig or go zag
      break;
    case F_ZIPZAP: if (x + dx > e) x = w; else x = e; break; // zipzap: wewewe
    default: fx = F_MANUAL; // if undefined
  }
}
void funny_walky() {
  // Update positions according to their crude walk  // Y...
  f_touch = true;
  switch (fy) {
    case F_CENTER: y = m; break; // zzzzzz... at middle
    case F_ZIGZIG: y = y + dy; if (y > n) y = s; break; // zig: s..ns..n
    case F_ZAGZAG: y = y - dy; if (y < s) y = n; break; // zag: n..sn..s
    case F_ZIGZAG:
      if ( y + dy > n || y - dy < s ) f_zigy = !f_zigy; // toggle zig<->zag
      if (f_zigy) y = y + dy; else y = y - dy; // go zig or go zag
      break;
    case F_ZIPZAP: if (y + dy > n) y = s; else y = n; break; // zipzap: snsnsn
    default: fy = F_MANUAL; // if undefined
  }
}
void blinky() {
  // for debugging purpose.
  // set the LED with the ledState of the variable:
  digitalWriteFast(led_pin, led_state);
  led_state = !led_state;
  //  sprintf(msgbuf, "nanos(): %llu\n", nanos());
  //  Serial.print(msgbuf);
}
void loop() {
  // no delay: run as fast as possible.
  //  nanosleep(1);
  // check if (tlisten) ns have passed, then listen for a command.
  if (nanos() > tlisten + last_tlisten) {
    last_tlisten = nanos();
    listen_command();
  }
  // check if (tx) ns have passed, then reposition.
  if (nanos() > tx + last_tx) {
    last_tx = nanos();
    funny_walkx();
  }
  // check if (ty) ns have passed, then reposition.
  if (nanos() > ty + last_ty) {
    last_ty = nanos();
    funny_walky();
  }
  if (f_touch) {
    f_touch = false;
    galvogo(); // update galvo with fresh x,y
    blinky();
  }

}
