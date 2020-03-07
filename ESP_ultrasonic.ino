/* Somnia - a gesture based ultrasonic LED mixer for use with WLED

   Jason Krueger
   5-March-2019

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

/* How many measurements to sample across for accuracy, will effect other thresholds so careful tuning */
#define SAMPLE_SIZE 6

#define BUFFER_SIZE 1024
/* time between readings in ms
   On Arduino Mega readings belown 20ms interval could
   have residual echo issues
   Should be set in accordance with expected distance for an echo
   Speed of sound is 343 mm/ms or roughly 1/3 meter per millisecond
   Since this is roundtrip your echo is calculated on 2x this distance */
/* 51 * 343/2 = 8,746.4 mm = good for 8 and 1/2 ish meters */
#define INTERVAL    31

/* Timeout (us) for distance sensing rather than 1 second, needs to be less than INTERVAL */
#define MAX_ECHO    30000

/* Approximate integer scaling factor round trip micro seconds per cm */
#define SCALE_CM    58

/* Approximate integer scaling factor round trip micro seconds per mm */
#define SCALE_MM    6

/* Maximum Standard Deviation tolerated to determine if distance value is trusted
    This will affect how much target jiggling is acceptable to trust a distance measurement
*/
#define DISTANCE_STDEV_MAX   20

/* Maximum stdev (correlating to variance), we will expect
    Tuning this higher means a person needs to wiggle their fingers faster
    to get the full range of hue.
    max in our use case is mapped inverted to low hue/red
*/
#define DISTANCE_STDEV_HIGH  80

/* Maximum Standard Deviation tolerated to determine if distance value is trusted */
#define MAX_VELOCITY_THRESHOLD   400

/* Distance (cm) threshold which we trigger effects */
#define MAX_DISTANCE_THRESHOLD_CM   100

/* max distance in cm we look for gestures */
#define MAX_EDGE_DISTANCE      100
/* min distance in cm we look for gestures */
#define MIN_EDGE_DISTANCE      10

#define MIN_COLOR_DELTA   30

#define DEFAULT_EFFECT 0
#define DEFAULT_SPEED  127

/* max time between high confidence distance measurements for edges */
#define MAX_EDGE_TIME_DELTA_MS 300
/* minimum cm to move in opposite direction before we fix SECOND_EDGE */
#define MIN_EDGE_DELTA_THRESHOLD_CM 3

#define MAX_GESTURE_DISTANCE_CM 100
#define MIN_GESTURE_DISTANCE_CM 6

#define MAX_GESTURE_VELOCITY 1000
#define MIN_GESTURE_VELOCITY 5

#define MAX_HUE              290
#define MAX_SATURATION       255
#define MAX_BRIGHTNESS       255
#define MIN_BRIGHTNESS       50

/* min delta (ms) since last setting secondary colar, rgb2 */
#define MIN_RGB2_DELTA_MS    1000

/* time values to determine WHEN to do a ping */
unsigned long next_time, new_time;

/* colors we alter with motion, speed, and distance */
unsigned int hue = 0;
unsigned int last_hue = 0;
unsigned int brightness = 0;
unsigned int last_brightness = 0;


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

struct RGB {
  byte red;
  byte green;
  byte blue;
};

void getRGB(int hue, int sat, int val, RGB *rgb) {
  /* convert hue, saturation and brightness ( HSB/HSV ) to RGB
     The dim_curve is used only on brightness/value and on saturation (inverted).
     This looks the most natural.
  */

  val = 255 - dim_curve[255 - val];
  sat = 255 - dim_curve[255 - sat];

  int r;
  int g;
  int b;
  int base;

  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    rgb->red   = val;
    rgb->green = val;
    rgb->blue  = val;
  }
  else  {
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

    rgb->red   = r;
    rgb->green = g;
    rgb->blue  = b;
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

bool setRGB(struct RGB rgb1, struct RGB rgb2) {
  wled_send.red = rgb1.red;
  wled_send.green = rgb1.green;
  wled_send.blue = rgb1.blue;

  wled_send.red2 = rgb2.red;
  wled_send.green2 = rgb2.green;
  wled_send.blue2 = rgb2.blue;

  Udp.beginPacket(Wled_IP, WLED_UDP_PORT);
  Udp.write( (uint8_t *) &wled_send, sizeof wled_send);
  return Udp.endPacket();
}

unsigned long last_distance_microns = 0;
unsigned long last_millis = 0;
unsigned long rgb_last_updated = 0;

MedianFilter distances(SAMPLE_SIZE, 0);
MedianFilter stdevs(SAMPLE_SIZE, DISTANCE_STDEV_MAX);

RGB rgb1 = {0, 0, 0};
RGB rgb2 = {0, 0, 0};

struct EDGE {
  int distance_cm;
  unsigned long time_ms;
};

struct EDGE edge_initial[3];

struct GESTURE {
  int index;
  struct EDGE edge[3];
};

#define NULL_EDGE     -1
#define FIRST_EDGE     0
#define SECOND_EDGE    1
#define THIRD_EDGE     2

#define MAX_EDGE_INDEX 2

bool validGesture(struct GESTURE gesture) {
  /* need 3 edges */
  if (gesture.index == MAX_EDGE_INDEX) {
    return true;
  }
  return false;
}

void resetGesture (struct GESTURE *gesture) {
  gesture->index = NULL_EDGE;
  for (int i = FIRST_EDGE; i <= MAX_EDGE_INDEX; i++)
  {
    gesture->edge[i] = (struct EDGE) {
      0, 0
    };
  }
}

bool addGestureEdge(struct GESTURE *gesture, struct EDGE edge) {
  Serial.println("addGestureEdge()");
  bool valid_gesture = false;
  /* edge detection, find out delta since last edge detection */
  unsigned long last_edge_time_ms = (gesture->index == NULL_EDGE) ? edge.time_ms : gesture->edge[gesture->index].time_ms;
  unsigned long edge_delta_ms = edge.time_ms - last_edge_time_ms;
  last_edge_time_ms = new_time;
  /* verify time of last edge detection is under a threshold */
  if (edge_delta_ms <= MAX_EDGE_TIME_DELTA_MS) {
    /* next edge */
    Serial.println("edge_delta_ms <= MAX_EDGE_TIME_DELTA_MS");
    if (edge.distance_cm > MAX_EDGE_DISTANCE) {
      /* within range, new edge detected */
      Serial.println("edge.distance_cm > MAX_EDGE_DISTANCE");
      valid_gesture = validGesture(*gesture);
      if (valid_gesture) {
        Serial.println("valid_gesture; return true");
        return true;
      }
      else {
        /* out of range, reset gesture and return */
        Serial.println("NOT valid_gesture; reset and return false");
        resetGesture(gesture);
        return false;
      }
    }
  }
  else {
    /* exceeded MAX_EDGE_TIME_DELTA_MS */
    Serial.println("NOT edge_delta_ms <= MAX_EDGE_TIME_DELTA_MS");
    valid_gesture = validGesture(*gesture);
    if (valid_gesture) {
      Serial.println("valid_gesture; return true");
      return true;
    }
    else {
      /* time since last edge detection is exceeded, reset and return */
      Serial.println("NOT valid_gesture; reset and return false");
      resetGesture(gesture);
      return false;
    }
  }

  switch (gesture->index) {
    case NULL_EDGE:
      /* first edge in gesture, no checks needed */
      Serial.println("NULL_EDGE; init FIRST_EDGE");
      if (edge.distance_cm >= MIN_EDGE_DISTANCE) {
        Serial.println("edge.distance_cm >= MIN_EDGE_DISTANCE");
        gesture->index = 0;
        gesture->edge[FIRST_EDGE] = edge;
        break;
      }
      else {
        Serial.println("NOT edge.distance_cm >= MIN_EDGE_DISTANCE");
        /* no gesture started, inside MIN_EDGE_DISTANCE, let's reset to DEFAULT_EFFECT */
        setEffect(DEFAULT_EFFECT, DEFAULT_SPEED);
      }

    case FIRST_EDGE:
      Serial.println("FIRST_EDGE");
      if (edge.distance_cm <= gesture->edge[FIRST_EDGE].distance_cm) {
        Serial.println("edge.distance_cm <= gesture->edge[FIRST_EDGE].distance_cm; init SECOND_EDGE");
        /* add if we've moved towards sensor */
        gesture->index = SECOND_EDGE;
        gesture->edge[SECOND_EDGE] = edge;
      }
      break;

    case SECOND_EDGE: {
        Serial.println("SECOND_EDGE");
        int distance_cm = gesture->edge[SECOND_EDGE].distance_cm - gesture->edge[FIRST_EDGE].distance_cm;
        int delta_cm = edge.distance_cm - gesture->edge[SECOND_EDGE].distance_cm;
        if (delta_cm <= 0 and distance_cm <= 0) {
          Serial.println("delta_cm <= 0 and distance_cm <= 0; update SECOND_EDGE");
          /* we're continuing to move towards sensor, update SECOND_EDGE */
          gesture->edge[SECOND_EDGE] = edge;
        }
        else {
          Serial.println("NOT delta_cm <= 0 and distance_cm <= 0");
          if (abs(delta_cm) >= MIN_EDGE_DELTA_THRESHOLD_CM) {
            Serial.println("abs(delta_cm) >= MIN_EDGE_DELTA_THRESHOLD_CM; init THIRD_EDGE");
            /* we've turned around finally, create THIRD_EDGE */
            gesture->index = THIRD_EDGE;
            gesture->edge[THIRD_EDGE] = edge;
          }
        }
        break;
      }

    case THIRD_EDGE: {
        Serial.println("THIRD_EDGE");
        int distance_cm = gesture->edge[THIRD_EDGE].distance_cm - gesture->edge[SECOND_EDGE].distance_cm;
        int delta_cm = edge.distance_cm - gesture->edge[THIRD_EDGE].distance_cm;
        if (delta_cm >= 0 and distance_cm >= 0) {
          Serial.println("delta_cm >= 0 and distance_cm >= 0; update THIRD_EDGE");
          /* we're continuing to move away from sensor, update THIRD_EDGE */
          gesture->edge[THIRD_EDGE] = edge;
        }
        else {
          Serial.println("NOT delta_cm > 0 and distance_cm => 0");
          if (abs(delta_cm) >= MIN_EDGE_DELTA_THRESHOLD_CM) {
            /* we've turned around again, call gesture completed */
            Serial.println("abs(delta_cm) >= MIN_EDGE_DELTA_THRESHOLD_CM; valid_gesture = true");
            valid_gesture = true;
          }
        }
        break;
      }
  }

  Serial.println("return from addGestureEdge()");
  return valid_gesture;
}

void setEffect(uint8_t effectIndex, uint8_t effectSpeed) {
  wled_send.effectCurrent = effectIndex;
  wled_send.effectSpeed = effectSpeed;

  Udp.beginPacket(Wled_IP, WLED_UDP_PORT);
  Udp.write( (uint8_t *) &wled_send, sizeof wled_send);
  Udp.endPacket();
}

void doTheGesture(struct GESTURE gesture) {
  Serial.println("doTheGesture()");
  word effect[3] = {2, 41, 80};
  /* map distance to effect */
  int distance_cm = abs(gesture.edge[SECOND_EDGE].distance_cm - gesture.edge[FIRST_EDGE].distance_cm) + abs(gesture.edge[SECOND_EDGE].distance_cm - gesture.edge[THIRD_EDGE].distance_cm);
  distance_cm = constrain(distance_cm, MIN_GESTURE_DISTANCE_CM, MAX_GESTURE_DISTANCE_CM);
  // 2 is last index of effect array
  int effect_index = map(distance_cm, MIN_GESTURE_DISTANCE_CM, MAX_GESTURE_DISTANCE_CM, 0, 2);
  uint8_t effectIndex = effect[effect_index];
  unsigned long time_ms = gesture.edge[THIRD_EDGE].time_ms - gesture.edge[FIRST_EDGE].time_ms;
  // mm / second - so we keep it as an int. 10 x distance_cm = distance_mm. distance_mm / ( time_ms / 1000 ) == 1000 * distance_mm / time_ms
  int velocity = 10 * 1000 * distance_cm / time_ms;
  velocity = constrain(velocity, MIN_GESTURE_VELOCITY, MAX_GESTURE_VELOCITY);
  uint8_t effectSpeed = map(velocity, MIN_GESTURE_VELOCITY, MAX_GESTURE_VELOCITY, 0, 255);

  char buffer[BUFFER_SIZE];
  sprintf(buffer, "GESTURE UPDATED: distance_cm=%03d velocity=%05d effect_index=%d effectCurrent=%03d effectSpeed=%03d", distance_cm, velocity, effect_index, effectIndex, effectSpeed);
  Serial.println(buffer);

  setEffect(effectIndex, effectSpeed);
}

struct GESTURE gesture;
void setup( )
{
  for (int i = FIRST_EDGE; i <= MAX_EDGE_INDEX; i++)
  {
    edge_initial[i] = {0, 0};
  }
  resetGesture(&gesture);
  wled_send = {     // byte/uint8_t index: name - description
    0,              // 0: purpose - Packet Purpose Byte
    0,              // 1: callMode - do not use notify
    MAX_BRIGHTNESS, // 2: bri - brightness overall, combined with RGB
    0,              // 3: red1 - Primary Red Value
    0,              // 4: green1 - Primary Green Value
    0,              // 5: blue1 - Primary Blue Value
    0,              // 6: nightlightActive - Nightlight running?
    0,              // 7: nightlightDelayMins - Nightlight Time
    0,              // 8: effectCurrent - Effect Index (0 based)
    127,            // 9: effectSpeed - Effect Speed (0-255)
    255,            // 10: white1 - Primary White Value
    4,              // 11: version - Ver 4 of protocol: transition time supported
    0,              // 12: red2 - Secondary Red Value
    0,              // 13: green2 - Secondary Green Value
    0,              // 14: blue2 - Secondary Blue Value
    255,            // 15: white2 - Secondary White Value
    127,            // 16: effectIntensity - Effect Intensity
    0,              // 17: transitionDelayUpperByte - Transition Duration Upper (word transition100ms &= 0xFF00 >> 8)
    1               // 18: transitionDelayLowerByte - Transition Duration Lower (word transition100ms &= 0x00FF)
  };

  next_time = INTERVAL;       /* set time from reset */
  Serial.begin( 115200 );

  /* Configure pins and ensure trigger is OFF */
  pinMode( trigpin, OUTPUT );
  digitalWrite( trigpin, LOW );       // Set trig pin LOW here NOT later
  pinMode( echopin, INPUT );

  /* Send signon message */
  Serial.println( F( "Somnia human gesture light mixer" ) );

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
  }
  else {
      Serial.println("Already connected?");
  }

  Serial.print("Connecting..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
}

void loop( )
{
  /* calculated distance in centimetres */
  unsigned long distance = 0;
  new_time = millis( );
  if ( new_time >= next_time )
  {
    /* send 10 microsecond chirp */
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

  if (gesture.index != NULL_EDGE) {
    if (gesture.index == THIRD_EDGE) {
      unsigned long time_delta = new_time - gesture.edge[gesture.index].time_ms;
      if (time_delta > MAX_EDGE_TIME_DELTA_MS) {
        /* do the gesture */
        doTheGesture(gesture);
        resetGesture(&gesture);
      }
    }
  }
  
  /* Calculate distance only for VALID readings 0 is no echo or timeout */
  if ( distance > 0 ) {
    bool print_summary = true;
    unsigned long distance_microns = 1000 * distance / SCALE_MM;
    int time_delta_ms = new_time - last_millis;
    int distance_cm = distance / SCALE_CM;
    int median_distance_cm = distances.in( distance_cm );
    int distance_stdev = distances.getStDev();
    int hue = last_hue;
    int brightness = last_brightness;
    bool HSB_changed = false;
    bool valid_gesture = false;

    if (distance_stdev <= DISTANCE_STDEV_MAX) {
      /* small amount jitter in last few measurements, we can trust the median of the population */
      if (median_distance_cm <= MAX_DISTANCE_THRESHOLD_CM) {
        /* Median distance under our MAX_DISTANCE_THRESHOLD_CM and has a low stdev.
            We use distance for brightness and then look for a new "edge"
        */
        last_millis = new_time;
        last_distance_microns = distance_microns;
        brightness = map(constrain(median_distance_cm, 0, MAX_DISTANCE_THRESHOLD_CM), 0, MAX_DISTANCE_THRESHOLD_CM, MAX_BRIGHTNESS, MIN_BRIGHTNESS);
        if (brightness != last_brightness) {
          HSB_changed = true;
          last_brightness = brightness;
        }

        EDGE new_edge;
        new_edge.distance_cm = median_distance_cm;
        new_edge.time_ms = new_time;
        valid_gesture = addGestureEdge(&gesture, new_edge);
      }
    }
    else {
      /* large amount jitter in last few measurements */
      if (median_distance_cm < MAX_DISTANCE_THRESHOLD_CM and gesture.index != NULL_EDGE) {
        /* some to a lot of jitter, let's use it for changing hue!
            Also make sure we're not in the middle of a gesture
        */
        int median_stdev = stdevs.in(distance_stdev);
        int max_stdev = stdevs.getMax();
        hue = map(constrain(max_stdev, DISTANCE_STDEV_MAX, DISTANCE_STDEV_HIGH), DISTANCE_STDEV_MAX, DISTANCE_STDEV_HIGH, MAX_HUE, 0);
        if (hue != last_hue) {
          HSB_changed = true;
          last_hue = hue;
        }
      }
    }

    if (HSB_changed) {
      struct RGB rgb_new;
      getRGB(hue, MAX_SATURATION, brightness, &rgb_new);
      int color_delta = abs(rgb_new.red - rgb1.red) + abs(rgb_new.green - rgb1.green) + abs(rgb_new.blue - rgb1.blue);
      unsigned long rgb_updated_delta = new_time - rgb_last_updated;
      if (rgb_updated_delta >= MIN_RGB2_DELTA_MS and color_delta >= MIN_COLOR_DELTA) {
        /* update our primary color through WLED */
        rgb2 = rgb1;
        rgb1 = rgb_new;
        setRGB(rgb1, rgb2);
        /* secondary color, rgb2, for use in WLED effects */
        /* min time and min color diff detected */
        char buffer[BUFFER_SIZE];
        sprintf(buffer, "RGB1 & RGB2 UPDATED: rgb_updated_delta=%03d color_delta=%d", rgb_updated_delta, color_delta);
        Serial.println(buffer);
        rgb_last_updated = new_time;
      }
    }

    if (valid_gesture) {
      /* do the gesture */
      doTheGesture(gesture);
      resetGesture(&gesture);
    }

    if (print_summary) {
      char buffer[BUFFER_SIZE];
      sprintf(buffer, "Dist=%03d Med=%03d StDev=%03d Hue=%03d B=%03d Changed=%d [R=%03d G=%03d B=%03d] [R2=%03d G2=%03d B2=%03d] TimeDelta=%08d G={%d, [{%03d, %05d},{%03d, %05d},{%03d, %05d}]}",
              distance_cm, median_distance_cm, distance_stdev, hue, brightness, HSB_changed, rgb1.red, rgb1.green, rgb1.blue, rgb2.red, rgb2.green, rgb2.blue, time_delta_ms,
              gesture.index,
              gesture.edge[FIRST_EDGE].distance_cm, gesture.edge[FIRST_EDGE].time_ms,
              gesture.edge[SECOND_EDGE].distance_cm, gesture.edge[SECOND_EDGE].time_ms,
              gesture.edge[THIRD_EDGE].distance_cm, gesture.edge[THIRD_EDGE].time_ms);
      Serial.println(buffer);
    }
  }
}
