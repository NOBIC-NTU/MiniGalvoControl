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
int16_t y = 500;
// Several special points used to determine walks
int16_t cx = 0; // (c)enter (x)
int16_t cy = 0; // (m)iddle (y)
int16_t ax = 500; // (a)mplitude (= -w = e)
int16_t ay = 500; // (a)mplitude (= n = -s)
int16_t w = -500; // (w)est point (smallest x)
int16_t e = 500; // (e)ast point (biggest x)
int16_t s = -500; // (s)outh point (smallest y)
int16_t n = 500; // (n)orth point (biggest y)
int16_t dx = 1; // delta x (jog this many units per dt)
int16_t dy = 1; // delta y (jog this many units per dt)
uint64_t tx = 10*1000'000'000; // delta (t)ime (update pos every 10000000 ns = 10 ms)
uint64_t ty = 10*1000'000'000; // delta (t)ime (update pos every 10000000 ns = 10 ms)
uint64_t tlisten = 250000000; // delta (t)ime (update listen commands every 250ms)
uint64_t last_tx = 0; // last time tx was passed
uint64_t last_ty = 0; // last time ty was passed
uint64_t last_tlisten = 0; // last time tlisten was passed
uint16_t ox = 32768; // represents the middle of the scan field
uint16_t oy = 32768; // represents the middle of the scan field

// Walk functions (crude waveforms) in x (and y)
#define F_NONE 0 // leave x as it were
#define F_CENTER 1 // go c
#define F_FORWARD 2 // go w...ew...e
#define F_REVERSE 3 // go e...we...w
#define F_TWOWAY 4 // go e...ww...e
#define F_ZIPZAP 5 // go ewewew
// Mind the order of the above and in here:
const char* f_names[F_ZIPZAP + 1] = {"NONE", "CENTER", "FORWARD", "REVERSE", "TWOWAY", "ZIPZAP"};
int16_t fx = F_TWOWAY;
int16_t fy = F_TWOWAY;

// Temp variable
char msgbuf[256]; // line buffer to print messages
const int led_pin = LED_BUILTIN;// debug using the led
bool led_state = LOW;

void setup() {
  Serial.begin(9600);
  Serial2.begin(256000); // scanner max
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
  Serial.println("is to set or query various parameters:");
  Serial.println("");
  Serial.println("x,y                          # current (x,y) coordinate (centidegrees)");
  Serial.println("fx,fy                        # time-function of coordinate (x(t),y(t)) (choose 0-5)");
  Serial.println("ax,ay                        # amplitude of time-function");
  Serial.println("cx,cy                        # center point of time-function");
  Serial.println("tx,ty                        # time-delta of coordinate (x,y) updates (ns)");
  Serial.println("dx,dy                        # angle-delta of coordinate (x,y) updates (centidegrees)");
  Serial.println("ox,oy                        # origin/offset of coordinate system (16-bit integer)");
  Serial.println("?                            # print current (x,y) and (fx,fy) function");
  Serial.println("help                         # print this help page");
  Serial.println("???, status                  # list the value of many parameters");
  Serial.println("");
  Serial.println("N.B. Unless (fx,fy) are nonzero, (x,y) change by (dx,dy) every (tx,ty) time, where ");
  sprintf(msgbuf, "(fx,fy) can be: 0 %s, 1 %s, 2 %s, 3 %s, 4 %s and 5 %s,\n",
        f_names[0], f_names[1], f_names[2], f_names[3], f_names[4], f_names[5]); Serial.print(msgbuf);
  Serial.println("");
  Serial.println("To initialize ScannerMax Mach-DSP controller, type: sm_init");
  Serial.println("(this issues c12f0001c22f0001c13a0000c23a0000 to the controller)");
  sprintf(msgbuf, "\n");
}
void msg_status() {
  sprintf(msgbuf, "x,y: %+05d,%+05d, fx,fy: %d %s, %d %s\n", x, y, fx, f_names[fx], fy, f_names[fy]); Serial.print(msgbuf);
  sprintf(msgbuf, "(w)est ,(cx),(e)ast : %+05d,%+05d,%+05d\n", w, cx, e); Serial.print(msgbuf);
  sprintf(msgbuf, "(s)outh,(cy),(n)orth: %+05d,%+05d,%+05d\n", s, cy, n); Serial.print(msgbuf);
  sprintf(msgbuf, "(dx)   ,(dy)            : %+05d,%+05d\n", dx, dy); Serial.print(msgbuf);
  sprintf(msgbuf, "(tx)   ,(ty)            : %05llu,%05llu\n", tx, ty); Serial.print(msgbuf);
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
        case 'w': sprintf(msgbuf, "%s: %+05d\n", cmd, w); break;
        case 'e': sprintf(msgbuf, "%s: %+05d\n", cmd, e); break;
        case 's': sprintf(msgbuf, "%s: %+05d\n", cmd, s); break;
        case 'n': sprintf(msgbuf, "%s: %+05d\n", cmd, n); break;
        case 'c': // two-letter commands with c
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, cx); break; // 'cx'
            case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, cy); break; // 'cy'
            default: sprintf(msgbuf, "cx,cy: (%+05d,%+05d)\n", cx, cy); break; // 'c'
          } break;
        case 'a': // two-letter commands with a
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, ax); break; // 'ax'
            case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, ay); break; // 'ay'
            default: sprintf(msgbuf, "ax,ay: (%+05d,%+05d)\n", ax, ay); break; // 'a'
          } break;
        case 'd': // two-letter commands with d
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, dx); break; // 'dx'
            case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, dy); break; // 'dy'
            default: sprintf(msgbuf, "dx,dy: (%+05d,%+05d)\n", dx, dy); break; // 'd'
          } break;
        case 't': // two-letter commands with t
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %05llu\n", cmd, tx); break; // 'tx'
            case 'y': sprintf(msgbuf, "%s: %05llu\n", cmd, ty); break; // 'ty'
            default: sprintf(msgbuf, "tx,ty: (%05llu,%05llu)\n", tx, ty); break; // 't'
          } break;
        case 'o': // two-letter commands with o
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %+05d\n", cmd, ox); break; // 'ox'
            case 'y': sprintf(msgbuf, "%s: %+05d\n", cmd, oy); break; // 'oy'
            default: sprintf(msgbuf, "ox,oy: (%+05d,%+05d)\n", ox, oy); break; // 'o'
          } break;
        case 'f': // two-letter commands with f
          switch ((char)cmd[1]) {
            case 'x': sprintf(msgbuf, "%s: %d %s\n", cmd, fx, f_names[fx]); break; // 'fx'
            case 'y': sprintf(msgbuf, "%s: %d %s\n", cmd, fy, f_names[fy]); break; // 'fy'
            default: sprintf(msgbuf, "fx,fy: %d %s, %d %s\n", fx, f_names[fx], fy, f_names[fy]); break; // 'f'
          } break;
      }
    } else { // (arg != NULL)
      // argument given
      sprintf(msgbuf, "set %s: %ld\n", cmd, val); // message
      // do assignment:
      switch ((char)cmd[0]) {
        case 'x': x = val; y = cy; break; // useless unless in F_NONE mode
        case 'y': y = val; x = cx; break; // useless unless in F_NONE mode
        case 'w': w = val; break;
        case 'e': e = val; break;
        case 's': s = val; break;
        case 'n': n = val; break;
        case 'c': // two-letter commands with c
          switch ((char)cmd[1]) {
            case 'x': cx = val; break; // 'cx'
            case 'y': cy = val; break; // 'cy'
            default:
              cx = val; cy = val; // 'a'
              x = cx; y = cy;
              sprintf(msgbuf, "set %s,%s: %ld,%ld\n", "cx","cy", val, val); // alternative message
          } break;
        case 'a': // two-letter commands with a
          switch ((char)cmd[1]) {
            case 'x': ax = val; w = -ax; e = ax; break; // 'ax'
            case 'y': ay = val; n = ay; s = -ay; break; // 'ay'
            default:
              ax = val; w = -ax; e = ax; // 'a'
              ay = val; n = ay; s = -ay;
              sprintf(msgbuf, "set %s,%s: %ld,%ld\n", "ax","ay", val, val); // alternative message
          } break;
        case 'd': // two-letter commands with d
          switch ((char)cmd[1]) {
            case 'x': dx = val; break; // 'dx'
            case 'y': dy = val; break; // 'dy'
            default:
              dx = val; dy = val; // 'd val'
              sprintf(msgbuf, "set %s,%s: %ld,%ld\n", "dx","dy", val, val); // alternative message
          } break;
        case 't': // two-letter commands with t
          switch ((char)cmd[1]) {
            case 'x': tx = val; break; // 'tx'
            case 'y': ty = val; break; // 'ty'
            default:
              tx = val; ty = val; // 't val'
              sprintf(msgbuf, "set %s,%s: %ld,%ld\n", "tx","ty", val, val); // alternative message
          } break;
        case 'o': // two-letter commands with o
          switch ((char)cmd[1]) {
            case 'x': ox = val; break; // 'ox'
            case 'y': oy = val; break; // 'oy'
          } break;
        case 'f': // two-letter commands with f
          switch ((char)cmd[1]) {
            case 'x': fx = val; break; // 'fx'
            case 'y': fy = val; break; // 'fy'
            default:
              fx = val; fy = val; // 'f val'
              sprintf(msgbuf, "set %s,%s: %ld,%ld\n", "fx","fy", val, val); // alternative message
          } break;
      }
    }
  } else if (strcmp(cmd, "sm_init") == 0) { // issue hex signal to scannermax device
    byte cmd1[] = {0xC1, 0x2F, 0x00, 0x01};
    byte cmd2[] = {0xC2, 0x2F, 0x00, 0x01};
    byte cmd3[] = {0xC1, 0x3A, 0x00, 0x00};
    byte cmd4[] = {0xC2, 0x3A, 0x00, 0x00};
    Serial2.write(cmd1, sizeof(cmd1));
    Serial2.write(cmd2, sizeof(cmd2));
    Serial2.write(cmd3, sizeof(cmd3));
    Serial2.write(cmd4, sizeof(cmd4));
    sprintf(msgbuf, "sm_init: '%x %x %x %x'\n", 0xC12F0001, 0xC22F0001, 0xC13A0000, 0xC23A0000);
  } else if (strcmp(cmd, "help") == 0) {
    msg_help();
  } else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "???") == 0) {
    msg_status();
    sprintf(msgbuf, "nanos(): %llu\n", nanos());
  }
  Serial.print(msgbuf);
}

void blinky() {
  // for debugging purpose.
  // set the LED with the ledState of the variable:
  digitalWriteFast(led_pin, led_state);
  led_state = !led_state;
  delay(1000);
  //  sprintf(msgbuf, "nanos(): %llu\n", nanos());
  //  Serial.print(msgbuf);
}

void galvogo() {
  // Effectuate x, y coordinates
  galvo.goTo( x - ox, y - oy );
  // Serial.printf("x,y = %05d,%05d\n", x,y);
}

// Keep track of zig vs zag
bool f_zigx = true;
bool f_zigy = true;
bool f_touch = true;
void funny_walkx() {
  f_touch = true; // Request update
  // Update positions according to their crude walk  // X...
  switch (fx) {
    case F_NONE: break; // zzz no change (or manual change x)
    case F_CENTER: x = cx; break; // zzzzzz... at center
    case F_FORWARD: x = x + dx; if (x > e || x < w+cx) x = w+cx; break; // zig: w..ew..e
    case F_REVERSE: x = x - dx; if (x > e || x < w+cx) x = e+cx; break; // zag: e..we..w
    case F_TWOWAY:
      x = x + dx*f_zigx - dx*(1-f_zigx); // +dx if f_zigx, -dx if not
      if (f_zigx && x > e+cx) f_zigx = false; // beyond east: reverse to zag
      if (!f_zigx && x < w+cx) f_zigx = true; // beyond west: reverse to zig
      break;
    case F_ZIPZAP: if (x + dx > e+cx) x = w+cx; else x = e+cx; break; // zipzap: wewewe
    default: blinky(); fx = F_NONE; blinky(); // if undefined
  }
}
void funny_walky() {
  f_touch = true; // Request update
  // Update positions according to their crude walk  // Y...
  switch (fy) {
    case F_NONE: break; // zzz no change (or manual change y)
    case F_CENTER: y = cy; break; // zzzzzz... at middle
    case F_FORWARD: y = y + dy; if (y > n+cy) y = s+cy; break; // zig: s..ns..n
    case F_REVERSE: y = y - dy; if (y < s+cy) y = n+cy; break; // zag: n..sn..s
    case F_TWOWAY:
      y = y + dy*f_zigy - dy*(1-f_zigy); // +dy if f_zigy, -dy if not
      if (f_zigy && y > n+cy) f_zigy = false; // above north: reverse to zag
      if (!f_zigy && y < s+cy) f_zigy = true; // below south: reverse to zig
      break;
    case F_ZIPZAP: if (y + dy > n+cy) y = s+cy; else y = n+cy; break; // zipzap: snsnsn
    default: blinky(); fy = F_NONE; blinky(); // if undefined
  }
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
//    blinky();
  }

}
