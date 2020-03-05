/* Ultrasonic distance sensor example Simple version

   Paul Carpenter, PC Services
   24-Jan-2017

   Uses HC-SR04 four pin ultrasonic sensor to continuously output
   distances found when in range (every second).

   Results output onto serial port at 115,200 baud

   Works on principle sound travels at 343.2 m/s in dry air at 20 deg C

   So time per cm is 29.137529 micro seconds /cm

   For round trip (there and back)  58.275058 micro seconds /cm

   In order to reduce code size and execution time values are kept as integers
   and factor of 58 to calculate distances
*/
#include <WiFi.h>
#include <SPI.h>
#include <WiFiUdp.h>
//#include <HTTPClient.h>
#include "MedianFilter.h"
//#include "esp32-hal-log.h"

const char* ssid = "WLED-AP";
const char* password = "superduper";

IPAddress local_IP(4, 3, 2, 99);
IPAddress gateway(4, 3, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(4, 3, 2, 1);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

IPAddress Wled_IP(4, 3, 2, 1); // send UPD Sync commands to

WiFiUDP Udp;

#define WLED_UDP_PORT   21324

/* Pin definitions */
#define echopin     5
#define trigpin     2

#define START_CHANNEL 1
#define CHANNEL_COUNT  300

#define BUFFER_SIZE 1024
/* time between readings in ms
   On Arduino Mega readings belown 20ms interval could
   have residual echo issues */
#define INTERVAL    30

/* Timeout (us) for distance sensing rather than 1 second */
#define MAX_ECHO    30000

/* Scale factor round trip micro seconds per cm */
#define SCALE_CM    58

/* Scale factor round trip micro seconds per mm */
#define SCALE_MM    6

/* Maximum Standard Deviation tolerated to determine if distance value is trusted */
#define DISTANCE_STDEV_MAX   20

#define DISTANCE_STDEV_HIGH  80

/* Maximum Standard Deviation tolerated to determine if distance value is trusted */
#define VELOCITY_STDEV_MAX   700

/* Distance (cm) threshold which we trigger turning lights on/off */
#define DISTANCE_THRESHOLD_CM   100

/* Distance (cm) threshold which we trigger turning lights on/off */
#define VELOCITY_THRESHOLD   400

#define MAX_HUE              290
#define MAX_SATURATION       255
#define MAX_BRIGHTNESS       255

/* time values to determine WHEN to do a ping */
unsigned long next_time, new_time;

// getRGB function stores RGB values in this array
// use these values for the red, blue, green led.
int rgb_colors[3];

/* colors we alter with motion, speed, and distance */
unsigned int hue = 0;
unsigned int last_hue = 0;
unsigned int saturation = 255;
unsigned int last_saturation = 255;
unsigned int brightness = 0;
unsigned int last_brightness = 0;
// HTTPClient client;                         //Declare an object of class HTTPClient  /* check if to run this time */



/*
  dim_curve 'lookup table' to compensate for the nonlinearity of human vision.
  Used in the getRGB function on saturation and brightness to make 'dimming' look more natural.
  Exponential function used to create values below :
  x from 0 - 255 : y = round(pow( 2.0, x+64/40.0) - 1)
*/

const byte dim_curve[] = {
  0,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
  3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,
  4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,
  6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
  8,   8,   9,   9,   9,   9,   9,   9,   10,  10,  10,  10,  10,  11,  11,  11,
  11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  14,  15,
  15,  15,  16,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,
  20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  25,  26,  26,
  27,  27,  28,  28,  29,  29,  30,  30,  31,  32,  32,  33,  33,  34,  35,  35,
  36,  36,  37,  38,  38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,
  48,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,
  63,  64,  65,  66,  68,  69,  70,  71,  73,  74,  75,  76,  78,  79,  81,  82,
  83,  85,  86,  88,  90,  91,  93,  94,  96,  98,  99,  101, 103, 105, 107, 109,
  110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
  146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
  193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

void getRGB(int hue, int sat, int val, int colors[3]) {
  /* convert hue, saturation and brightness ( HSB/HSV ) to RGB
     The dim_curve is used only on brightness/value and on saturation (inverted).
     This looks the most natural.
  */

  val = dim_curve[val];
  sat = 255 - dim_curve[255 - sat];

  int r;
  int g;
  int b;
  int base;

  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    colors[0] = val;
    colors[1] = val;
    colors[2] = val;
  } else  {

    base = ((255 - sat) * val) >> 8;

    switch (hue / 60) {
      case 0:
        r = val;
        g = (((val - base) * hue) / 60) + base;
        b = base;
        break;

      case 1:
        r = (((val - base) * (60 - (hue % 60))) / 60) + base;
        g = val;
        b = base;
        break;

      case 2:
        r = base;
        g = val;
        b = (((val - base) * (hue % 60)) / 60) + base;
        break;

      case 3:
        r = base;
        g = (((val - base) * (60 - (hue % 60))) / 60) + base;
        b = val;
        break;

      case 4:
        r = (((val - base) * (hue % 60)) / 60) + base;
        g = base;
        b = val;
        break;

      case 5:
        r = val;
        g = base;
        b = (((val - base) * (60 - (hue % 60))) / 60) + base;
        break;
    }

    colors[0] = r;
    colors[1] = g;
    colors[2] = b;
  }
}

struct WLED_INFO {
  byte purpose;
  byte callMode;  // do not notify
  byte bri;
  byte red;
  byte green;
  byte blue;
  byte nightlightActive;
  byte nightlightDelayMins;
  byte effectCurrent;
  byte effectSpeed;
  byte white;
  byte ver; // 4 transition time supported
  byte red2;
  byte green2;
  byte blue2;
  byte white2;
  byte effectIntensity;
  byte transitionDelayUpper;
  byte transitionDelayLower;
};

WLED_INFO wled_send;

bool setRGB(int red, int green, int blue) {
  wled_send.red = (byte) red;
  wled_send.green = (byte) green;
  wled_send.blue = (byte) blue;
//  wled_send.red = (byte) 255;
//  wled_send.green = (byte) 255;
//  wled_send.blue = (byte) 255;

  Udp.beginPacket(Wled_IP, WLED_UDP_PORT);
  Udp.write( (uint8_t *) &wled_send, sizeof wled_send);
  return Udp.endPacket();
}

void setup( )
{
  wled_send = {
    0,
    0,  // do not notify
    255,
    0,
    0,
    0,
    0,
    0,
    127,
    127,
    255,
    4, // 4 transition time supported
    0,
    0,
    0,
    255,
    127,
    0,
    1
  };
  next_time = INTERVAL;       /* set time from reset */
  Serial.begin( 115200 );

  /* Configure pins and ensure trigger is OFF */
  pinMode( trigpin, OUTPUT );
  digitalWrite( trigpin, LOW );       // Set trig pin LOW here NOT later
  pinMode( echopin, INPUT );

  /* Send signon message */
  Serial.println( F( "Somnia human light mixer" ) );

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  Serial.print("Connecting..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  //  client.setReuse(true);
}

int last_median_velocity = 0;
int last_velocity_stdev = 0;
int last_velocity = 0;
int last_distance_microns = 0;
int last_millis = 0;
MedianFilter velocities(6, 0);
MedianFilter distances(6, 100);
MedianFilter stdevs(6, DISTANCE_STDEV_MAX);
void loop( )
{
  /* calculated distance in centimetres */
  unsigned long distance = 0;
  new_time = millis( );
  if ( new_time >= next_time )
  {
    digitalWrite( trigpin, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( trigpin, LOW );
    /* echo time in microseconds
          Maximum  MAX_ECHO
          Minimum  0      (NO ECHO)

      Timeout for measurements set by MAX_ECHO define (constant)
    */
    distance = pulseIn( echopin, HIGH, MAX_ECHO );

    next_time = new_time + INTERVAL;   // save next time to run this part of loop
  }

  /* Calculate distance only for VALID readings 0 is no echo or timeout */
  if ( distance > 0 ) {
    bool print_summary = true;
    int distance_microns = 1000 * distance / SCALE_MM;
    int time_delta_ms = new_time - last_millis;
    int velocity = last_velocity;
    if (time_delta_ms > 0) {
      velocity =  abs(last_distance_microns - distance_microns) / time_delta_ms; // mm/ms: any velocity lower than 1mm/ms is not needed
    }
    int distance_cm = distance / SCALE_CM;
    int median_distance_cm = distances.in( distance_cm );
    int distance_stdev = distances.getStDev();
    int hue = last_hue;
    int saturation = last_saturation;
    int brightness = last_brightness;
    bool HSB_changed = false;
    int median_velocity = velocities.out();
    int velocity_stdev = last_velocity_stdev;

    if (distance_stdev < DISTANCE_STDEV_MAX) {
      /* zero to small amount jitter in last few measurements */
      if (median_distance_cm < DISTANCE_THRESHOLD_CM) {
        /* Median distance under our DISTANCE_THRESHOLD_CM and has a low stdev.
            We use distance for brightness and use to estimate velocity
        */
        last_millis = new_time;
        last_distance_microns = distance_microns;
        brightness = map(constrain(median_distance_cm, 0, DISTANCE_THRESHOLD_CM), 0, DISTANCE_THRESHOLD_CM, MAX_BRIGHTNESS, 0);
        if (brightness != last_brightness) {
          HSB_changed = true;
          last_brightness = brightness;
        }
        median_velocity = velocities.in( velocity );
        last_velocity = velocity;
        velocity_stdev = velocities.getStDev();
        last_velocity_stdev = velocity_stdev;
        if (velocity_stdev < VELOCITY_STDEV_MAX) {
          /* we can trust the median velocity measure and we'll constrain extreme outliers, let's use it to change saturation */
          saturation = map(constrain(median_velocity, 0, VELOCITY_THRESHOLD), 0, VELOCITY_THRESHOLD, MAX_SATURATION, 0);
          if (saturation != last_saturation) {
            HSB_changed = true;
            last_saturation = saturation;
          }
        }
      }
    }
    else {
      if (median_distance_cm < DISTANCE_THRESHOLD_CM) {
        /* some to a lot of jitter, let's use it for changing hue! */
        int median_stdev = stdevs.in(distance_stdev);
        hue = map(constrain(median_stdev, DISTANCE_STDEV_MAX, DISTANCE_STDEV_HIGH), DISTANCE_STDEV_MAX, DISTANCE_STDEV_HIGH, MAX_HUE, 0);
        if (hue != last_hue) {
          HSB_changed = true;
          last_hue = hue;
        }
      }
    }

    getRGB(hue, saturation, brightness, rgb_colors);
    if (HSB_changed) {
      /* update our lights through xSchedule */
      setRGB(rgb_colors[0], rgb_colors[1], rgb_colors[2]);
    }

    if (print_summary) {
      char buffer[BUFFER_SIZE];
      sprintf(buffer, "Dist=%d Med=%d StDev=%d Time=%d TimeDelta=%d, Vel=%d MedVel=%d VelStDev=%d Hue=%d Sat=%d B=%d Changed=%d R=%d, G=%d, B=%d",
              distance_cm, median_distance_cm, distance_stdev, new_time, time_delta_ms, velocity, median_velocity, velocity_stdev, hue, saturation, brightness, HSB_changed, rgb_colors[0], rgb_colors[1], rgb_colors[2]);
      Serial.println(buffer);
    }
  }
}
