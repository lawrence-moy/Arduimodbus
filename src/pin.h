
enum {
  PIN_MODE_INPUT = 0,
  PIN_MODE_OUTPUT
};

struct Pin {
  unsigned char number;
  unsigned char mode;
  unsigned long timer;
  unsigned char timerActivated;
  unsigned long internalTimer;
};

