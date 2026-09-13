// *******************************************************************
// Hoverboard control with ESP32-C3 - Step 1: UART link test
//
// Purpose: bench test of the serial link between an ESP32-C3 and the
// hoverboard mainboard. It lets you drive each
// wheel by typing commands in the USB Serial Monitor, and prints the
// firmware feedback (commands, measured rpm, odometry ticks, battery
// voltage, board temperature, good/bad packet counters).
// Keep the wheels lifted off the ground during the first tests.
//
// Firmware side (platformio.ino, config.h):
//   VARIANT_USART, CONTROL_SERIAL_USART3, FEEDBACK_SERIAL_USART3,
//   TANK_STEERING, ENABLE_ODOMETRY, CTRL_MOD_REQ SPD_MODE
//
// Wiring (right sensor cable, USART3). NEVER connect the red 15V wire!
//   Hoverboard GND -> ESP GND
//   Hoverboard TX  -> ESP GPIO4 (RX)
//   Hoverboard RX  -> ESP GPIO5 (TX)
//
// Serial Monitor (115200, line ending "Newline"):
//   "<left> <right>"  set wheel commands, e.g. "50 50"  (SPD_MODE: 1000 = N_MOT_MAX rpm)
//   "s" or empty line stop (0 0)
//
// Arduino IDE: enable "USB CDC On Boot" to see Serial output over the native USB.
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Must match USART3_BAUD in firmware config.h
#define SERIAL_BAUD         115200      // [-] USB Serial Monitor
#define START_FRAME         0xABCD      // [-] Start frame for serial protocol
#define TIME_SEND           50          // [ms] Command period (firmware timeout is ~800 ms)
#define TIME_PRINT          200         // [ms] Feedback print period (feedback arrives every 10 ms)
#define CMD_LIMIT           300         // [-] Test safety clamp on commands (full scale is 1000)
#define ODOM_WRAP           9000        // [ticks] Firmware odometry counters wrap at this value
// #define DEBUG_RX                        // [-] Print raw received bytes in HEX

#define HOVER_RX            4           // ESP32-C3 GPIO (avoid 2, 8, 9 strapping pins and 20/21 UART0)
#define HOVER_TX            5
HardwareSerial& HoverSerial = Serial1;

// ########################## PROTOCOL ##########################
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  steer;         // TANK_STEERING: LEFT wheel command
  int16_t  speed;         // TANK_STEERING: RIGHT wheel command
  uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;   // [rpm]
  int16_t  speedL_meas;   // [rpm]
  int16_t  wheelR_cnt;    // [ticks] 0..ODOM_WRAP-1 (ENABLE_ODOMETRY)
  int16_t  wheelL_cnt;    // [ticks] 0..ODOM_WRAP-1 (ENABLE_ODOMETRY)
  int16_t  batVoltage;    // [V*100]
  int16_t  boardTemp;     // [degC*10]
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

SerialCommand  Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## STATE ##########################
int16_t  cmdLeft  = 0;
int16_t  cmdRight = 0;

bool     feedbackValid = false;
uint32_t packetsOk     = 0;
uint32_t packetsBad    = 0;
uint32_t lastPacketMs  = 0;

bool     odomInit  = false;
int16_t  prevCntL  = 0;
int16_t  prevCntR  = 0;
int32_t  totalTicksL = 0;         // Unwrapped cumulative ticks
int32_t  totalTicksR = 0;

String   lineBuf;

// ########################## SEND ##########################
void Send(int16_t left, int16_t right)
{
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = left;
  Command.speed    = right;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## ODOMETRY ##########################
int16_t wrapDelta(int16_t curr, int16_t prev)
{
  int16_t d = curr - prev;
  if (d >  ODOM_WRAP / 2) d -= ODOM_WRAP;
  if (d < -ODOM_WRAP / 2) d += ODOM_WRAP;
  return d;
}

void updateOdometry()
{
  if (!odomInit) {
    prevCntL = Feedback.wheelL_cnt;
    prevCntR = Feedback.wheelR_cnt;
    odomInit = true;
    return;
  }
  totalTicksL += wrapDelta(Feedback.wheelL_cnt, prevCntL);
  totalTicksR += wrapDelta(Feedback.wheelR_cnt, prevCntR);
  prevCntL = Feedback.wheelL_cnt;
  prevCntR = Feedback.wheelR_cnt;
}

// ########################## RECEIVE ##########################
void Receive()
{
  static uint8_t  idx = 0;
  static uint8_t *p;
  static uint8_t  incomingBytePrev = 0;

  while (HoverSerial.available()) {
    uint8_t incomingByte = HoverSerial.read();
    uint16_t bufStartFrame = ((uint16_t)incomingByte << 8) | incomingBytePrev;

    #ifdef DEBUG_RX
      Serial.printf("%02X ", incomingByte);
    #endif

    if (bufStartFrame == START_FRAME) {                        // New packet start detected
      p    = (uint8_t *)&NewFeedback;
      *p++ = incomingBytePrev;
      *p++ = incomingByte;
      idx  = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {     // Store packet payload
      *p++ = incomingByte;
      idx++;
    }

    if (idx == sizeof(SerialFeedback)) {                       // Packet complete
      uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2
                                   ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                                   ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt
                                   ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        feedbackValid = true;
        lastPacketMs  = millis();
        packetsOk++;
        updateOdometry();
      } else {
        packetsBad++;
      }
      idx = 0;
    }

    incomingBytePrev = incomingByte;
  }
}

// ########################## USB SERIAL COMMANDS ##########################
void handleUserInput()
{
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (lineBuf.length() < 32) lineBuf += c;   // Bound buffer if no newline is ever received
      continue;
    }

    lineBuf.trim();
    int l = 0, r = 0;
    if (lineBuf.length() == 0 || lineBuf == "s") {
      l = r = 0;
    } else if (sscanf(lineBuf.c_str(), "%d %d", &l, &r) != 2) {
      Serial.println("Format: \"<left> <right>\" or \"s\"");
      lineBuf = "";
      continue;
    }
    cmdLeft  = constrain(l, -CMD_LIMIT, CMD_LIMIT);
    cmdRight = constrain(r, -CMD_LIMIT, CMD_LIMIT);
    Serial.printf(">> CMD L=%d R=%d\n", cmdLeft, cmdRight);
    lineBuf = "";
  }
}

// ########################## PRINT ##########################
void printStatus()
{
  if (!feedbackValid || millis() - lastPacketMs > 500) {
    Serial.printf("NO FEEDBACK  ok=%lu bad=%lu  (check wiring / firmware)\n",
                  (unsigned long)packetsOk, (unsigned long)packetsBad);
    return;
  }
  Serial.printf("cmd L=%4d R=%4d | rpm L=%4d R=%4d | cnt L=%4d R=%4d | ticks L=%6ld R=%6ld | bat=%.2fV temp=%.1fC | ok=%lu bad=%lu\n",
                Feedback.cmd1, Feedback.cmd2,
                Feedback.speedL_meas, Feedback.speedR_meas,
                Feedback.wheelL_cnt, Feedback.wheelR_cnt,
                (long)totalTicksL, (long)totalTicksR,
                Feedback.batVoltage / 100.0, Feedback.boardTemp / 10.0,
                (unsigned long)packetsOk, (unsigned long)packetsBad);
}

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_RX, HOVER_TX);
  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
  Serial.println("Hoverboard UART test v2.0");
  Serial.printf("Command %u bytes, Feedback %u bytes\n", (unsigned)sizeof(SerialCommand), (unsigned)sizeof(SerialFeedback));
  Serial.println("Commands: \"<left> <right>\" (e.g. \"50 50\"), \"s\" = stop");
}

// ########################## LOOP ##########################
void loop()
{
  static uint32_t tSend = 0, tPrint = 0;
  Receive();
  handleUserInput();

  uint32_t now = millis();    // Read after Receive() so that now >= lastPacketMs

  if (now - tSend >= TIME_SEND) {
    tSend = now;
    Send(cmdLeft, cmdRight);
  }

  if (now - tPrint >= TIME_PRINT) {
    tPrint = now;
    #ifndef DEBUG_RX
      printStatus();
    #endif
  }

  // LED: slow blink = no feedback, fast blink = link OK
  bool linkOk = feedbackValid && (now - lastPacketMs < 500);
  digitalWrite(LED_BUILTIN, (now % (linkOk ? 200 : 2000)) < (linkOk ? 100 : 1000));
}

// ########################## END ##########################
