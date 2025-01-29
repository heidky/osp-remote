/** uncomment to print LOG messages over serial */
#define DEBUG_OVER_SERIAL
/** uncomment to plot graphs over serial */
#define PLOT_OVER_SERIAL

#define SERIAL_WELCOME "**> OSP Remote " VERSION " <**"

#ifdef DEBUG_OVER_SERIAL
#define LOG_(x) Serial.print((x))
#define LOG(x) Serial.println((x))
#else
#define LOG_(x) (void)0;
#define LOG(x) (void)0;
#endif
