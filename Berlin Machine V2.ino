//Berlin Machine Version 2
/*
   Stuff Done:
   Menu -  Use left button to select menu mode - Use 6 PB86 switches from left to choose an item
   Menu Things
   Note, Wave Form and Gate Set - Okay
   Volume
   Set all
   ADSR Rate
   LFOs
   Morphing

*/

//Reference Wave Data in external file
extern int16_t currentWaveForm[256];  // defined in myWaveForm.ino
extern int16_t triangleWaveform[256];
extern int16_t sineWaveform[256];
extern int16_t inverseTriangleWaveform[256];
extern int16_t inverseSineWaveform[256];
extern int16_t inverseDoubleSineWaveform[256];
extern int16_t wobbleSineWaveform[256];
extern int16_t rampWaveform[256];
extern int16_t modulatorWaveform[256];
extern int16_t carrierWaveform[256];
extern int16_t folderWaveform[256];
extern int16_t squareWaveform[256];
extern int16_t oddSquareWaveform[256];

//==================================================================================== Initializations
// --- Audio Card ----------------------------------------------------------------------
//Using A REV C card with appropriate remapping of connections using wires instead of pins
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=406,239
AudioSynthWaveform       waveform2;      //xy=421,333
AudioSynthWaveform       waveform3;      //xy=443,407
AudioFilterStateVariable filter1;        //xy=666,239
AudioFilterStateVariable filter2;        //xy=666,300
AudioFilterStateVariable filter3;        //xy=690,398
AudioMixer4              mixer1;         //xy=872,321
AudioOutputI2S           i2s2;           //xy=1020,300
AudioConnection          patchCord1(waveform1, 0, filter1, 0);
AudioConnection          patchCord2(waveform2, 0, filter2, 0);
AudioConnection          patchCord3(waveform3, 0, filter3, 0);
AudioConnection          patchCord4(filter1, 0, mixer1, 0);
AudioConnection          patchCord5(filter2, 0, mixer1, 1);
AudioConnection          patchCord6(filter3, 0, mixer1, 2);
AudioConnection          patchCord7(mixer1, 0, i2s2, 0);
AudioConnection          patchCord8(mixer1, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=696,564
// GUItool: end automatically generated code




int current_waveform = 0;
int modulated = 0;
int modAmp = 30000;

//--- Menu ------------------------------------------------------------------------- Menu

int menuIndex = 0;

//--- MIDI Music ------------------------------------------------------------------- MIDI Music

//midi notes to play for testing
//MIDI Notes to play for each voice
int notesBackUp[16] = {60, 62, 60, 64, 60, 65, 64, 65, 60, 62, 60, 64, 60, 65, 67, 65};
int notes1[16] = {60, 62, 60, 63, 60, 65, 63, 62, 60, 62, 60, 63, 60, 65, 63, 62};
int notes2[16] = {60, 62, 60, 63, 60, 65, 63, 62, 60, 62, 60, 63, 60, 65, 63, 62};
//Drone Notes keyboard
int notes3[16] = {60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75};
//a number that represents the current wave shape per step
//int waveTypeBackUp[16] = {3, 1, 2, 3, 4, 3, 3, 3, 3, 3, 3, 5, 5, 5, 5, 5};
int waveType1[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 5, 5, 5, 5, 5, 5};
int waveType2[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 5, 5, 5, 5, 5, 5};
//drone keyboard wavetypes
int waveType3[16] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
//gate length for each note
//Note: There is no BPM just gate lengths
int gates1[16] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
int gates2[16] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
int release1[16] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
int release2[16] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};
int gates3[16] = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80};
//rests to skip a note
bool rests1[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool rests2[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool rests3[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Note Volumes
float amplitude1[16] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
float amplitude2[16] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
float amplitude3[16] = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
//note edit active
bool edit1[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool edit2[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool edit3[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Which note up to in sequence
int noteIndex1 = 0;
int noteIndex2 = 0;
int noteIndex3 = -1;
float note3Amp = 0.0;
long time1, time2, time3;
int sequenceLength1 = 16;
int sequenceLength2 = 16;
int sequenceLength3 = 16;
bool canToggle = false;
int voiceIndex = 0;
int droneOctave = -12;

//Storage for filter sweep motion sequencing
float droneCutOff [128] = {};
float droneResonance [128] = {};
int filterUpdateInterval = 100;
int motionIndex = 0;

/*
   frequency(freq);
  Set the filter's corner frequency.
  When a signal is connected to the control input, the filter will implement this frequency when the signal is zero.
  resonance(Q);
  Set the filter's resonance.
  Q ranges from 0.7 to 5.0.
  Resonance greater than 0.707 will amplify the signal near the corner frequency.
  You must attenuate the signal before input to this filter, to prevent clipping.


*/
float wave1Filter = 5000;
float wave2Filter = 5000;
float wave3Filter = 1000;
float wave1Q = 0.707;
float wave2Q = 0.707;
float wave3Q = 0.707;



int notes [8] = {60, 62, 60, 63, 60, 65, 63, 62};
//Music Stuff
byte BPM = 120;//Beats per Minute, I make the maximum 250 later
byte BPMRunningAverage [5] = {120, 120, 120, 120, 120};// To stabilize value
int BPMAverageIndex = 0; //Current BPM in above array
// int waitTime = 40; //Time between sending MIDI clock bytes
int spareTime = 40; //Time left after processes are done that needs to be padded/wasted while waiting for mext beat/note

//Synth Notes
int rootNote = 36, lowestNote = 24, highestNote = 80;
//Map 128 MIDI notes to actual frequencies for A = 440Hz
const float noteFreqs[128] = {8.176, 8.662, 9.177, 9.723, 10.301, 10.913, 11.562, 12.25, 12.978, 13.75, 14.568, 15.434, 16.352, 17.324, 18.354, 19.445, 20.602, 21.827, 23.125, 24.5, 25.957, 27.5, 29.135, 30.868, 32.703, 34.648, 36.708, 38.891, 41.203, 43.654, 46.249, 48.999, 51.913, 55, 58.27, 61.735, 65.406, 69.296, 73.416, 77.782, 82.407, 87.307, 92.499, 97.999, 103.826, 110, 116.541, 123.471, 130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220, 233.082, 246.942, 261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 391.995, 415.305, 440, 466.164, 493.883, 523.251, 554.365, 587.33, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880, 932.328, 987.767, 1046.502, 1108.731, 1174.659, 1244.508, 1318.51, 1396.913, 1479.978, 1567.982, 1661.219, 1760, 1864.655, 1975.533, 2093.005, 2217.461, 2349.318, 2489.016, 2637.02, 2793.826, 2959.955, 3135.963, 3322.438, 3520, 3729.31, 3951.066, 4186.009, 4434.922, 4698.636, 4978.032, 5274.041, 5587.652, 5919.911, 6271.927, 6644.875, 7040, 7458.62, 7902.133, 8372.018, 8869.844, 9397.273, 9956.063, 10548.08, 11175.3, 11839.82, 12543.85};
const char  noteLabels[128][3] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#"};
float detune = 1.000;
int cutOffFrequency = 2000;
bool canUpdateCutOffFrequency = false;

//Scale Intervals
const byte majorScale[15]         = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24}; //e.g. D, E, F#,G, A, B, C#, D.  Two Octaves for Arpegiator
const byte harmonicMinorScale[15] = {0, 2, 3, 5, 7, 8, 11, 12, 14, 15, 17, 19, 20, 23, 24}; //e.g. D, E, F, G, A, B♭, C, D.
bool playMajorScale = false;
const char keys [12][3] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};//display current key. Display doesn't like the flat symbol 3 Chars needed for each element for null?
char displayChar [4]; //To display the current key to the screen




//Using SCL2 (pin 24) and SDA2 (25)
//--- Oled Screens ------------------------------------------------------------------- Oled Screens
// Screens on I2C bus 2
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define OLED_RESET -1         // Reset pin #
#define SCREEN_ADDRESS1 0x3C  // Found using scanner. I left the address switch set to off
#define SCREEN_ADDRESS2 0x3D  // Found using scanner. I changed the address switch on the PIICODev OLED to on
#define I2Cspeed 1000000      //Gives 12ms update instead of 28ms

//Initialize OLED screens.
Adafruit_SSD1306 displayR(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET, I2Cspeed);
Adafruit_SSD1306 displayL(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET, I2Cspeed);

int t1 = 0, t2 = 0, xaxis = 31, encoderValue;
bool upDateScreen = false;

// --- Encoder ------------------------------------------------------------------------- Encoders

#include <Encoder.h>
Encoder knobL(31, 32);  //Left Encoder - Can use any pin on Teensy 4.1 except 13 (LED)
Encoder knobR(33, 34);  //Right Encoder

long previousEncoderValueL = -999;
long previousEncoderValueR = -999;

// --- Multiplexer and Switches -------------------------------------------------------- Multiplexer and Switches
//Normal Switches and encoders
int encoderSwitchL = 0;   //pin 30
int encoderSwitchR = 0;   //pin 35
int toggleSwitchL = 0;   //pin 27
int toggleSwitchR = 0;   // pin 2
int buttonSwitchL = 0;   //pin 26
int buttonSwitchR = 0;   // pin 3

int previousButtonSwitchL = 0;

#include <SparkFunSX1509.h>
//Multiplexer PB86 Switches & pins
//Switches on each Multiplexer use pins 0-7
//Leds on Switches use pins 8-15
int previousSwitchValues [16]; //used to avoid retriggering synth during looping - Not using bounce library due to varying length of waitTimes and due to it being a footpedal (secondary instrument)
int switchValues [16];
bool ioLEDStatus[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address
SX1509 ioL;                        // Create an SX1509 object to be used throughout
SX1509 ioR;                        // Create another SX1509 object to be used throughout

bool ledState = false;

//sync out pulse
#define Sync_Pin 37
long pulseOnTime = 0;



//============================================================================ SetUp

void setup() {

  Serial.begin(115200);
  Serial.println("2 SX1509 and Two OLEDs");

  //--- OLED Screens ------------------------------------------------
  //     SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!displayR.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  if (!displayL.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS2)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }


  // --- Multiplexer and Switches -------------------------------------------------------- Multiplexer and Switches
  //Normal Switches and encoders
  int encoderSwitchL = 0;   //pin 30
  int encoderSwitchR = 0;   //pin 35
  int toggleSwitchL = 0;   //pin 27
  int toggleSwitchR = 0;   // pin 2
  int buttonSwitchL = 0;   //pin 26
  int buttonSwitchR = 0;   // pin 3

  //--- Multiplexers -----------------------------------------------
  Wire1.begin(); //Initialize I2C bus
  Wire2.begin(); //Initialize I2C bus
  //  Call ioL.begin(<address>) to initialize the SX1509. If it
  //  successfully communicates, it'll return 1.
  if (ioR.begin(SX1509_ADDRESS, Wire1) == false)
  {
    Serial.println("Failed to communicate. Check wiring and address of SX1509.");
    digitalWrite(13, HIGH); // If we failed to communicate, turn the pin 13 LED on
    while (1)
      ; // If we fail to communicate, loop forever.
  }
  if (ioL.begin(SX1509_ADDRESS, Wire2) == false)
  {
    Serial.println("Failed to communicate. Check wiring and address of SX1509.");
    digitalWrite(13, HIGH); // If we failed to communicate, turn the pin 13 LED on
    while (1)
      ; // If we fail to communicate, loop forever.
  }

  // Call ioL.pinMode(<pin>, <mode>) to set any SX1509 pin as
  //  // either an INPUT, OUTPUT, INPUT_PULLUP, or ANALOG_OUTPUT
  for (int i = 0; i < 8; i++) {
    // Set output for LEDs:
    ioL.pinMode(i + 8, OUTPUT); //8 to 15 are LEDs
    ioR.pinMode(i + 8, OUTPUT);
    //Switches
    ioL.pinMode(i, INPUT_PULLUP); //0 to 7 are buttons
    ioR.pinMode(i, INPUT_PULLUP); //0 to 7 are buttons
  }

  //Switches
  pinMode(30, INPUT_PULLUP);    //encoder switch
  pinMode(35, INPUT_PULLUP);    //encoder switch
  pinMode(2, INPUT_PULLUP);     //toggle switch
  pinMode(27, INPUT_PULLUP);    //toggle switch
  pinMode(3, INPUT_PULLUP);     //button switch
  pinMode(26, INPUT_PULLUP);    //button switch

  //sync out pin
  pinMode(Sync_Pin, INPUT_PULLUP);    //button switch

  // ---- WaveForm Generator ---------------------------------------------------------
  //makeWave();

  //--- Audio card ----------------------------------------------------
  AudioMemory(10);// 10 is more than enough for the mono synth and one lot of drums
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.2);

  // Configure to use "currentWaveForm" for WAVEFORM_ARBITRARY
  waveform1.arbitraryWaveform(currentWaveForm, 172.0);
  waveform2.arbitraryWaveform(currentWaveForm, 172.0);
  // waveform3.arbitraryWaveform(currentWaveForm, 172.0);

  // configure both waveforms for 440 Hz and maximum amplitude
  waveform1.frequency(220);
  waveform2.frequency(220);
  waveform3.frequency(220);
  waveform1.amplitude(0.0);
  waveform2.amplitude(0.0);
  waveform3.amplitude(0.0);

  current_waveform = WAVEFORM_ARBITRARY;
  waveform1.begin(WAVEFORM_SAWTOOTH);
  waveform2.begin(WAVEFORM_SAWTOOTH);
  waveform3.begin(WAVEFORM_SAWTOOTH);
  initNotes();
}

//============================================================================ Loop

void loop() {
  checkSwitches();
  checkEncoders();
  if (upDateScreen) showScreens();
  showLEDS();
  if (toggleSwitchL == LOW) {
    setNewNoteIf();
  } else notesOff();
  droneOn();
}


//=== Menu ==================================================================== Menu


//=== Music ==================================================================== Music

void initNotes() {
  for (int i = 0; i < 16; i++) {
    notes1[i] = notesBackUp[i] - 12 ;
    notes2[i] = notesBackUp[i] + 12;
    gates1[i] = 200;
    gates2[i] = 200;
    release1[i] = 100;
    release2[i] = 100;
    waveType1[i] = 3; //Ramp
    waveType2[i] = 3; //Ramp
    rests1[i] = 0;
    rests2[i] = 0;
    amplitude1[i] = 0.2;
    amplitude2[i] = 0.2;
  }
  noteIndex1 = 0;
  noteIndex2 = 0;
  noteIndex3 = -1;
  sequenceLength1 = 16;
  sequenceLength2 = 16;
  sequenceLength3 = 16;
  for (int i = 0; i < 64; i++) {
    droneCutOff [i] = 1000 - i * 10;
    droneResonance [i] = 2 + 0.02 * i;
    droneCutOff [i + 64] = 360 + i * 10;
    droneResonance [i + 64] = 3.28 - 0.02 * i;
  }
}


//zzzz

void startSyncPulse() {
  digitalWrite(Sync_Pin, HIGH); // Start pulse then wait 20ms and off
  pulseOnTime = millis(); //remember time on
}

void setNewNoteIf() {
  //check times and update note playing if the time is up.
  long now1 = millis();
  //Time to change note Voice 1
  if (now1 >= time1 + gates1[noteIndex1]) {
    noteIndex1++;
    if (noteIndex1 > sequenceLength1 - 1) noteIndex1 = 0;
    startSyncPulse();
    //Serial.println(noteIndex1);
    //when to next change to next note
    time1 = time1 + gates1[noteIndex1];
    //is it not a rest then play the note
    if (rests1[noteIndex1]) {
      waveform1.amplitude(0.0);
    } else {
      //wwwww set wavetype amplitude and frequency
      setWaveType(waveType1[noteIndex1]);
      waveform1.amplitude(amplitude1[noteIndex1]);
      filter1.frequency(wave1Filter);
      filter1.resonance(wave1Q);
      waveform1.frequency(noteFreqs[notes1[noteIndex1]]);
    }
  }
  if (now1 >= time1 + release1[noteIndex1]) waveform1.amplitude(0.0);

  //Time to change note Voice 2
  if (now1 >= time2 + gates2[noteIndex2]) {
    noteIndex2++;
    if (noteIndex2 > sequenceLength2 - 1) noteIndex2 = 0;
    //when to next change to next note
    time2 = time2 + gates2[noteIndex2];
    //is it not a rest then play the note
    if (rests2[noteIndex2]) {
      waveform2.amplitude(0.0);
    } else {
      //wwwww set wavetype amplitude and frequency
      setWaveType(waveType2[noteIndex2]);
      waveform2.amplitude(amplitude2[noteIndex2]);
      filter2.frequency(wave2Filter);
      filter2.resonance(wave2Q);
      waveform2.frequency(noteFreqs[notes2[noteIndex2]]);
    }
  }
  if (now1 >= time2 + release2[noteIndex2]) waveform2.amplitude(0.0);
  //sync pulse off?
  if (millis() > pulseOnTime + 20) digitalWrite(Sync_Pin, LOW); // Stop pulse
}

void droneOn() {
  //No Change if Menu selected
  if (buttonSwitchL == 0) return;
  //play drone note
  if (noteIndex3 >= 0) playDrone();
  else stopDrone();
  waveform3.amplitude(note3Amp);
  modifyFilter();
}

void modifyFilter() {
  long now1 = millis();
  if (now1 >= time3 + filterUpdateInterval) {
    wave3Filter = droneCutOff[motionIndex];
    wave3Q = droneResonance[motionIndex];
    motionIndex++;
    if (motionIndex >= 127) motionIndex = 0;
  }
}

void playDrone() {
  note3Amp = note3Amp + 0.001;
  if (note3Amp > 0.2) note3Amp = 0.2;
  filter3.frequency(wave3Filter);
  filter3.resonance(wave3Q);
  waveform3.frequency(noteFreqs[notes3[noteIndex3] + droneOctave]);
}

void stopDrone() {
  note3Amp = note3Amp - 0.003;
  if (note3Amp < 0.0) note3Amp = 0.0;
}

void notesOff() {
  waveform1.amplitude(0.0);
  waveform2.amplitude(0.0);
  time1 = millis();
  time2 = millis();
}

void setWaveType(int wType) {
  if (wType == 0) loadAWave(sineWaveform, 256, voiceIndex);
  if (wType == 1)  loadAWave(wobbleSineWaveform, 256, voiceIndex);
  if (wType == 2)  loadAWave(triangleWaveform, 256, voiceIndex);
  if (wType == 3)  loadAWave(rampWaveform, 256, voiceIndex);
  if (wType == 4)  loadAWave(squareWaveform, 256, voiceIndex);
  if (wType == 5)  loadAWave(inverseSineWaveform, 256, voiceIndex);
  if (wType == 6)  loadAWave(inverseTriangleWaveform, 256, voiceIndex);
  if (wType == 7)  loadAWave(inverseSineWaveform, 256, voiceIndex);
  if (wType == 8)  loadAWave(inverseDoubleSineWaveform, 256, voiceIndex);
  if (wType == 9)  loadAWave(oddSquareWaveform, 256, voiceIndex);
}

//============================================================================= Hardware Inputs

bool switchJustPressed(int i) {
  if (previousSwitchValues[i] == HIGH && switchValues[i] == LOW) return true;  //just pressed
  return false;
}

bool switchJustReleased(int i) {
  if (previousSwitchValues[i] == LOW && switchValues[i] == HIGH) return true;           //Just Released
  return false;
}

//---- Check a switch -----------------------------------------------------
//PB86 Switches
bool isSwitchPressed(int i) { //Checks to see if a switch has just been pressed.
  bool pressedIt = false;
  //check the switch ------------------------------
  if (i < 8) {
    switchValues[i] = ioL.digitalRead(7 - i);
    //Serial.print(switchValues[i]);
  } else {
    switchValues[i] = ioR.digitalRead(15 - i);
    //Serial.print(switchValues[i]);
  }
  //Switch pressed?
  if (switchJustPressed(i)) {
    pressedIt = true;
    previousSwitchValues[i] = LOW;
    // Serial.println("Checking");
  }
  //Switch released?
  if (switchJustReleased(i)) {
    previousSwitchValues[i] = HIGH;
  }

  return pressedIt;
}

void  checkSwitches() {
  //Check PB86 Switches
  for (int i = 0; i < 16; i++) {
    //ioLEDStatus[i] = LOW;
    //  Serial.print(edit1[i]);
    if (isSwitchPressed(i)) {
      //if (switchValues[i] == LOW) ioLEDStatus[i] = HIGH;
      upDateScreen = true;
      if (menuIndex == 0 || menuIndex == 1 || menuIndex == 3 || menuIndex == 5) {
        //if note edit mode then indicate active switches
        //Serial.println("Pressed PB");
        if (buttonSwitchL == HIGH) edit1[i] = !edit1[i];
        knobL.write(0);
        knobR.write(0);
      }
      //drone keyboard
      if (menuIndex == 4) {
        //mmmm Toggle Note On Off or change it, but not if menu selected
        if (buttonSwitchL != 0) {
          if (noteIndex3 == i) noteIndex3 = -1;
          else  noteIndex3 = i;
        }
      }
    }
  }
  // Serial.println();


  //Toggle Switches
  //Left Toggle Switch
  toggleSwitchL = digitalRead(27);

  //Right Toggle Switch
  toggleSwitchR = digitalRead(2);
  if (toggleSwitchR == LOW) voiceIndex = 0;
  else voiceIndex = 1;

  //Momentary Buttons
  //Left Button - Selects MENU
  buttonSwitchL = digitalRead(26);
  //Serial.println(buttonSwitchL);
  if (buttonSwitchL != previousButtonSwitchL) {
    previousButtonSwitchL = buttonSwitchL;
    upDateScreen = true;
  }
  //Menu Selected
  if (buttonSwitchL == LOW) {
    for (int i = 0; i < 6; i++) {
      if (switchValues[i] == LOW) {
        menuIndex = i;
      }
    }
  }

  //Right Button - Select all or deselect all edit states
  buttonSwitchR = digitalRead(3);
  if (buttonSwitchR == LOW) {
    toggleEditState();
  } else canToggle = true;

  //Right Encoder
  encoderSwitchR = digitalRead(35);
  if (encoderSwitchR == LOW) {
    initNotes();
    upDateScreen = true;
  }
  //Left Encoder
  encoderSwitchL = digitalRead(30);

}

void toggleEditState() {
  if (!canToggle) return;
  for (int i = 1; i < 16; i++) {
    edit1[i] = edit1[0];
  }
  for (int i = 0; i < 16; i++) {
    edit1[i] = !edit1[i];
  }
  canToggle = false;
}

void checkEncoders() {
  long newValue;
  newValue = -knobL.read();
  if (newValue != previousEncoderValueL) {
    previousEncoderValueL = newValue;
    upDateScreen = true;
    // Serial.println(previousEncoderValueL);
    // modAmp = previousEncoderValueL * 1000;
    //if (modulated == 1) combineWavesAM(inverseDoubleSineWaveform, 256);
    //if (modulated == 2) combineWavesAM(folderWaveform, 256);
    //set notes
    if (menuIndex == 0) {
      adjustNotes();
    }
    //set Amplitudes
    if (menuIndex == 1) {
      adjustAmplitudes();
    }
    if (menuIndex == 3) {
      adjustWaveFolds();
    }
  }
  newValue = knobR.read();
  if (newValue != previousEncoderValueR) {
    previousEncoderValueR = newValue;
    upDateScreen = true;
    //Serial.println(previousEncoderValueR);
    //Wave morphing
    //set notes
    if (menuIndex == 0) {
      adjustGates();
    }
    //set Sequence Lengths
    if (menuIndex == 1) {
      adjustSequenceLengths();
    }
    if (menuIndex == 3) {
      adjustWaveShapePerStep();
    }
    if (menuIndex == 4) {
      adjustDroneOctave();
    }
    if (menuIndex == 5) {
      adjustReleases();
    }
  }

  //filter fun
  if (menuIndex == 2) {
    if (voiceIndex == 0) {
      if (previousEncoderValueL > 0) wave1Filter = wave1Filter + 200.0;
      if (previousEncoderValueL < 0) wave1Filter = wave1Filter - 200.0;
      if (previousEncoderValueR > 0) wave1Q = wave1Q + float(previousEncoderValueR) * 0.01;
      if (previousEncoderValueR < 0) wave1Q = wave1Q + float(previousEncoderValueR) * 0.01;
    }
    if (voiceIndex == 1) {
      if (previousEncoderValueL > 0) wave2Filter = wave2Filter + 200.0;
      if (previousEncoderValueL < 0) wave2Filter = wave2Filter - 200.0;
      if (previousEncoderValueR > 0) wave2Q = wave2Q + float(previousEncoderValueR) * 0.01;
      if (previousEncoderValueR < 0) wave2Q = wave2Q + float(previousEncoderValueR) * 0.01;
    }
    knobL.write(0);
    knobR.write(0);
    upDateScreen = true;
  }
  //Drone filter fun
  if (menuIndex == 4) {
    if (previousEncoderValueL > 0) wave3Filter = wave3Filter + 20.0;
    if (previousEncoderValueL < 0) wave3Filter = wave3Filter - 20.0;
    if (previousEncoderValueR > 0) wave3Q = wave3Q + float(previousEncoderValueR) * 0.1;
    if (previousEncoderValueR < 0) wave3Q = wave3Q + float(previousEncoderValueR) * 0.1;
    knobL.write(0);
    knobR.write(0);
    //upDateScreen = true;
  }
}

void adjustDroneOctave() {
  if (previousEncoderValueR > 0) droneOctave = droneOctave + 12;
  if (previousEncoderValueR < 0) droneOctave = droneOctave - 12;
  if (droneOctave > 12) droneOctave = 12;
  if (droneOctave < -12) droneOctave = -12;
  knobL.write(0);
  knobR.write(0);
  upDateScreen = true;
}

void  adjustSequenceLengths() {
  if (voiceIndex == 0) {
    if (previousEncoderValueR > 0) sequenceLength1++;
    if (previousEncoderValueR < 0) sequenceLength1--;
    if (sequenceLength1 > 16) sequenceLength1 = 16;
    if (sequenceLength1 < 0) sequenceLength1 = 0;
  }
  if (voiceIndex == 1) {
    if (previousEncoderValueR > 0) sequenceLength2++;
    if (previousEncoderValueR < 0) sequenceLength2--;
    if (sequenceLength2 > 16) sequenceLength2 = 16;
    if (sequenceLength2 < 0) sequenceLength2 = 0;
  }
  knobL.write(0);
  knobR.write(0);
}

void adjustAmplitudes() {
  for (int i = 0; i < 16; i++) {
    if (voiceIndex == 0) {
      if (edit1[i]) {
        amplitude1[i] = amplitude1[i] + float(previousEncoderValueL) / 200;
        if (amplitude1[i] > 1.0) amplitude1[i] = 1.0;
        if (amplitude1[i] < 0.0) amplitude1[i] = 0.0;
        knobR.write(0);
      }
    }
    if (voiceIndex == 1) {
      if (edit1[i]) {
        amplitude2[i] = amplitude2[i] + float(previousEncoderValueL) / 200;
        if (amplitude2[i] > 1.0) amplitude2[i] = 1.0;
        if (amplitude2[i] < 0.0) amplitude2[i] = 0.0;
        knobR.write(0);
      }
    }
  }
}

void adjustWaveShapePerStep() {
  //wwwww
  //choose wave combinations based on right encoder value
  for (int i = 0; i < 16; i++) {
    if (voiceIndex == 0) {
      if (edit1[i]) {
        //waveType1[i] = previousEncoderValueR;
        waveType1[i] = waveType1[i] + int(previousEncoderValueR / 2);
        if (waveType1[i] > 9) waveType1[i] = 9;
        if (waveType1[i] < 0) waveType1[i] = 0;
        knobR.write(0);
      }
    }
    if (voiceIndex == 1) {
      if (edit1[i]) {
        //waveType1[i] = previousEncoderValueR;
        waveType2[i] = waveType2[i] + int(previousEncoderValueR / 2);
        if (waveType2[i] > 9) waveType2[i] = 9;
        if (waveType2[i] < 0) waveType2[i] = 0;
        knobR.write(0);
      }
    }
  }
}

void adjustWaveFolds() {

}

void adjustNotes() {
  for (int i = 0; i < 16; i++) {
    if (voiceIndex == 0) {
      if (edit1[i]) {
        notes1[i] = notes1[i] + int(previousEncoderValueL / 2);
        if (notes1[i] > 127) notes1[i] = 127;
        if (notes1[i] < 0) notes1[i] = 0;
        knobL.write(0);
      }
    }
    if (voiceIndex == 1) {
      if (edit1[i]) {
        notes2[i] = notes2[i] + int(previousEncoderValueL / 2);
        if (notes2[i] > 127) notes2[i] = 127;
        if (notes2[i] < 0) notes2[i] = 0;
        knobL.write(0);
      }
    }
  }
}

void adjustGates() {
  for (int i = 0; i < 16; i++) {
    if (voiceIndex == 0) {
      if (edit1[i]) {
        gates1[i] = gates1[i] + previousEncoderValueR;
        if (gates1[i] > 10000) gates1[i] = 10000;
        if (gates1[i] < 10) gates1[i] = 10;
        knobR.write(0);
      }
    }
    if (voiceIndex == 1) {
      if (edit1[i]) {
        gates2[i] = gates2[i] + previousEncoderValueR;
        if (gates2[i] > 10000) gates2[i] = 10000;
        if (gates2[i] < 10) gates2[i] = 10;
        knobR.write(0);
      }
    }
  }
}

void adjustReleases() {
  for (int i = 0; i < 16; i++) {
    if (voiceIndex == 0) {
      if (edit1[i]) {
        release1[i] = release1[i] + previousEncoderValueR;
        if (release1[i] > gates1[i]) release1[i] = gates1[i];
        if (release1[i] < 10) release1[i] = 10;
        knobR.write(0);
      }
    }
    if (voiceIndex == 1) {
      if (edit1[i]) {
        release2[i] = release2[i] + previousEncoderValueR;
        if (release2[i] > gates2[i]) release2[i] = gates2[i];
        if (release2[i] < 10) release2[i] = 10;
        knobR.write(0);
      }
    }
  }
}

//=== Waves ==================================================================================
//
//void waveMorpher() {
//  //choose wave combinations based on right encoder value
//  if (waveType1[noteIndex1] <= 0) loadAWave(sineWaveform, 256, voiceIndex);
//  if (waveType1[noteIndex1] > 0 && waveType1[noteIndex1] < 25) morphWaves(sineWaveform, 256, wobbleSineWaveform, 256, 0);
//  if (waveType1[noteIndex1] == 25) loadAWave(wobbleSineWaveform, 256, voiceIndex);
//  if (waveType1[noteIndex1] > 25 && waveType1[noteIndex1] < 50) morphWaves(wobbleSineWaveform, 256, triangleWaveform, 256, 25);
//  if (waveType1[noteIndex1] == 50) loadAWave(triangleWaveform, 256, voiceIndex);
//  if (waveType1[noteIndex1] > 50 && waveType1[noteIndex1] < 75) morphWaves(triangleWaveform, 256, rampWaveform, 256, 50);
//  if (waveType1[noteIndex1] == 75) loadAWave(rampWaveform, 256, voiceIndex);
//  if (waveType1[noteIndex1] > 75 && waveType1[noteIndex1] < 100) morphWaves(rampWaveform, 256, squareWaveform, 256, 75);
//  if (waveType1[noteIndex1] >= 100) loadAWave(squareWaveform, 256, voiceIndex);
//}

////combine waves with a weighted average
//void morphWaves(int16_t newWave1[], int16_t s1, int16_t newWave2[], int16_t s2, int16_t offSet) {
//  int amount = waveType1[noteIndex1] - offSet;
//  for (int i = 0; i < 256; i++) {
//    currentWaveForm[i] = (newWave2[i] * amount + newWave1[i] * (25 - amount)) / 25;
//  }
//}

void loadAWave(int16_t newWave[], int16_t s, int voiceIndex) {
  modulated = 0; //no modulation
  for (int i = 0; i < 256; i++) {
    currentWaveForm[i] = newWave[i];
  }
  AudioNoInterrupts();
  if (voiceIndex == 0) {
    waveform1.arbitraryWaveform(currentWaveForm, 172.0);
    waveform1.begin(WAVEFORM_ARBITRARY);
  }
  if (voiceIndex == 1) {
    waveform2.arbitraryWaveform(currentWaveForm, 172.0);
    waveform2.begin(WAVEFORM_ARBITRARY);
  }
  AudioInterrupts();
}

//void loadAModulator(int16_t newWave[], int16_t s) {
//  for (int i = 0; i < 256; i++) {
//    modulatorWaveform[i] = newWave[i];
//  }
//}
//
////combine waves with amplitude modulation
//void combineWavesAM(int16_t newWave[], int16_t s) {
//  for (int i = 0; i < 256; i++) {
//    float ftmp = newWave[i] * modAmp / 30000;
//    //Serial.println(ftmp);
//    int tmp = int(currentWaveForm[i]) +  int(ftmp);
//    if (tmp > 30000)tmp = 30000;
//    if (tmp < -30000)tmp = -30000;
//    carrierWaveform[i] = tmp;
//  }
//  AudioNoInterrupts();
//  waveform1.arbitraryWaveform(carrierWaveform, 172.0);
//  waveform1.begin(WAVEFORM_ARBITRARY);
//  AudioInterrupts();
//}


//=================================================================== Displays
void showLEDS() {
  if (menuIndex == 0 || menuIndex == 1 || menuIndex == 3 || menuIndex == 5) {
    for (int i = 0; i < 16; i++) {
      if (edit1[i]) {
        if (i < 8) ioL.digitalWrite(i + 8, HIGH);
        else ioR.digitalWrite(i, HIGH);
      } else {
        if (i < 8) ioL.digitalWrite(i + 8, LOW);
        else ioR.digitalWrite(i, LOW);
      }
    }
  }
  if (menuIndex == 4) {
    for (int i = 0; i < 16; i++) {
      if (noteIndex3 == i) {
        if (i < 8) ioL.digitalWrite(i + 8, HIGH);
        else ioR.digitalWrite(i, HIGH);
      } else {
        if (i < 8) ioL.digitalWrite(i + 8, LOW);
        else ioR.digitalWrite(i, LOW);
      }
    }
  }
}

void showScreens() {
  displayR.clearDisplay();
  displayL.clearDisplay();
  //show menu
  if (buttonSwitchL == LOW) showMenu();
  else showOtherStuff();

  displayL.display();
  displayR.display();
  upDateScreen = false;
}

void showMenu() {
  //xxxxxx
  displayL.setTextSize(2);
  displayR.setTextSize(2);

  displayL.setCursor(1, 0);
  setMenuFontStyle(0);
  displayL.print("Note Gate");

  displayL.setCursor(1, 20);
  setMenuFontStyle(1);
  displayL.print("Vol Length");

  displayL.setCursor(1, 40);
  setMenuFontStyle(2);
  displayL.print("Filters");

  displayR.setCursor(1, 0);
  setMenuFontStyle(3);
  displayR.print("?   Wave");

  displayR.setCursor(1, 20);
  setMenuFontStyle(4);
  displayR.print("DOff DRes");

  displayR.setCursor(1, 40);
  setMenuFontStyle(5);
  displayR.print("Release");
  //Serial.println("");
}

void setMenuFontStyle(int index) {
  //  Serial.print("\tMenu: ");
  //  Serial.print(index);
  //  Serial.print("\t");
  //  Serial.print(menuIndex);
  if (index == menuIndex) {
    if (index < 3) {
      displayL.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
      displayL.fillRect(0, index * 20, 128, 18, SSD1306_WHITE);

    } else {
      index = index - 3;
      displayR.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
      displayR.fillRect(0, index * 20, 128, 18, SSD1306_WHITE);
    }
  } else {
    displayL.setTextColor(SSD1306_WHITE);
    displayR.setTextColor(SSD1306_WHITE);
  }
}

void showOtherStuff() {
  switch (menuIndex) {
    case 0:
      // statements
      showNotesAndGates();
      break;
    case 1:
      // statements
      showAmplitudes();
      break;
    case 2:
      // statements
      showFilters();
      break;
    case 3:
      // statements
      showWaveTypes();
      break;
    case 4:
      // statements
      showDroneScreen();
      break;
    case 5:
      // statements
      showReleases();
      break;
    default:
      // statements
      break;
  }
}

void showDroneScreen() {
  displayL.setCursor(0, 0);
  displayR.setCursor(0, 0);
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(2);
  displayR.setTextSize(2);
  if (noteIndex3 == -1) displayL.print("-");
  else {
    displayL.println(noteLabels[notes3[noteIndex3] + droneOctave]);
    displayL.println(noteFreqs[notes3[noteIndex3] + droneOctave]);
  }
  displayR.println(droneOctave);
  // displayR.println(wave3Q);

}

void showFilters() {
  displayL.setCursor(0, 0);
  displayR.setCursor(0, 0);
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(2);
  displayR.setTextSize(2);
  displayL.println(wave1Filter);
  displayL.println(wave1Q);
  displayR.println(wave2Filter);
  displayR.println(wave2Q);
}

void showAmplitudes() {
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(1);
  displayR.setTextSize(1);
  for (int i = 0; i < 8; i++) {
    if (voiceIndex == 0) {
      displayL.setCursor(1 + i * 15, 0);
      //displayL.print(amplitude1[i]);
      if (sequenceLength1 > i) displayL.fillCircle(i * 15 + 4, 4 , 3, SSD1306_WHITE);
      displayL.fillRect(i * 15, 10 , 10, amplitude1[i] * 35, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      //displayR.print(amplitude1[i + 8]);
      if (sequenceLength1 > i + 8) displayR.fillCircle(i * 15 + 4, 4 , 3, SSD1306_WHITE);
      displayR.fillRect(i * 15, 10 , 10, amplitude1[i + 8] * 35, SSD1306_WHITE);
    }
    if (voiceIndex == 1) {
      displayL.setCursor(1 + i * 15, 0);
      //displayL.print(amplitude2[i]);
      if (sequenceLength2 > i) displayL.fillCircle(i * 15 + 4, 4 , 3, SSD1306_WHITE);
      displayL.fillRect(i * 15, 10 , 10, amplitude2[i] * 35, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      //displayR.print(amplitude2[i + 8]);
      if (sequenceLength2 > i + 8) displayR.fillCircle(i * 15 + 4, 4 , 3, SSD1306_WHITE);
      displayR.fillRect(i * 15, 10 , 10, amplitude2[i + 8] * 35, SSD1306_WHITE);
    }
  }
}

void showWaveTypes() {
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(1);
  displayR.setTextSize(1);
  for (int i = 0; i < 8; i++) {
    if (voiceIndex == 0) {
      displayL.setCursor(1 + i * 15, 0);
      displayL.print(waveType1[i]);
      //displayL.fillRect(i * 15, 10 , 10, gates1[i] / 5, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      displayR.print(waveType1[i + 8]);
      //displayR.fillRect(i * 15, 10 , 10, gates1[i + 8] / 5, SSD1306_WHITE);
    }
    if (voiceIndex == 1) {
      displayL.setCursor(1 + i * 15, 0);
      displayL.print(waveType2[i]);
      //displayL.fillRect(i * 15, 10 , 10, gates1[i] / 5, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      displayR.print(waveType2[i + 8]);
      //displayR.fillRect(i * 15, 10 , 10, gates1[i + 8] / 5, SSD1306_WHITE);
    }
  }
}

//nagnag
void showNotesAndGates() {
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(1);
  displayR.setTextSize(1);
  for (int i = 0; i < 8; i++) {
    if (voiceIndex == 0) {
      displayL.setCursor(1 + i * 15, 0);
      displayL.print(noteLabels[notes1[i]]);
      displayL.fillRect(i * 15, 10 , 10, gates1[i] / 5, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      displayR.print(noteLabels[notes1[i + 8]]);
      displayR.fillRect(i * 15, 10 , 10, gates1[i + 8] / 5, SSD1306_WHITE);
    }
    if (voiceIndex == 1) {
      displayL.setCursor(1 + i * 15, 0);
      displayL.print(noteLabels[notes2[i]]);
      displayL.fillRect(i * 15, 10 , 10, gates2[i] / 5, SSD1306_WHITE);

      displayR.setCursor(1 + i * 15, 0);
      displayR.print(noteLabels[notes2[i + 8]]);
      displayR.fillRect(i * 15, 10 , 10, gates2[i + 8] / 5, SSD1306_WHITE);
    }
  }
}

void showReleases() {
  displayL.setTextColor(SSD1306_WHITE);
  displayR.setTextColor(SSD1306_WHITE);
  displayL.setTextSize(1);
  displayR.setTextSize(1);
  for (int i = 0; i < 8; i++) {
    if (voiceIndex == 0) {
      // displayL.setCursor(1 + i * 15, 0);
      // displayL.print(noteLabels[notes1[i]]);
      displayL.fillRect(i * 15, 10 , 10, release1[i] / 5, SSD1306_WHITE);

      //displayR.setCursor(1 + i * 15, 0);
      //displayR.print(noteLabels[notes1[i + 8]]);
      displayR.fillRect(i * 15, 10 , 10, release1[i + 8] / 5, SSD1306_WHITE);
    }
    if (voiceIndex == 1) {
      //displayL.setCursor(1 + i * 15, 0);
      // displayL.print(noteLabels[notes2[i]]);
      displayL.fillRect(i * 15, 10 , 10, release2[i] / 5, SSD1306_WHITE);

      //displayR.setCursor(1 + i * 15, 0);
      //displayR.print(noteLabels[notes2[i + 8]]);
      displayR.fillRect(i * 15, 10 , 10, release2[i + 8] / 5, SSD1306_WHITE);
    }
  }

}

void showWave() {
  int screenCentre = 30;
  int scaleFactor = 1000;

  //--- Waves -----
  if (modulated == 0) {
    for (int i = 0; i < 255; i++) {
      if (i < 128) displayL.drawLine(i, screenCentre - currentWaveForm[i] / scaleFactor, i + 1, screenCentre - currentWaveForm[i + 1] / scaleFactor, SSD1306_WHITE);
      else displayR.drawLine(i - 128, screenCentre - currentWaveForm[i] / scaleFactor , i + 1 - 128, screenCentre - currentWaveForm[i + 1] / scaleFactor, SSD1306_WHITE);
    }
  } else {
    for (int i = 0; i < 255; i++) {
      if (i < 128) displayL.drawLine(i, screenCentre - carrierWaveform[i] / scaleFactor, i + 1, screenCentre - carrierWaveform[i + 1] / scaleFactor, SSD1306_WHITE);
      else displayR.drawLine(i - 128, screenCentre - carrierWaveform[i] / scaleFactor , i + 1 - 128, screenCentre - carrierWaveform[i + 1] / scaleFactor, SSD1306_WHITE);
    }
  }
  //--- x axis -----
  for (int i = 0; i < 16; i++) {
    displayL.drawPixel(i * 8, 32, SSD1306_WHITE);
    displayR.drawPixel(i * 8, 32, SSD1306_WHITE);
  }
  //displayL.drawLine(0, 32, 127, 32, SSD1306_WHITE);
}
