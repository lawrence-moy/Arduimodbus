
enum {
  PIN_MODE_INPUT = 0,
  PIN_MODE_OUTPUT
};

struct Pin {
  unsigned char number;
  unsigned char mode;
  unsigned char timer;
  unsigned char timerActivated;
  unsigned char internalTimer;
};

