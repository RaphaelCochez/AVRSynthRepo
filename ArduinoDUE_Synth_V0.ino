#include <Scheduler.h>
#include <MIDIUSB.h>
#include <Filters.h>

const int dacPin = DAC0; // The pin connected to the DAC output
const float voltsPerOctave = 1.0f / 12.0f; // The voltage difference between two semitones
const float voltsAtZeroOctave = 0.0f; // The voltage corresponding to the lowest note (C0)
const float frequencyAtZeroOctave = 16.35f; // The frequency of the lowest note (C0) in Hz
const float sampleRate = 44100.0f; // The sample rate of the audio signal in Hz

// Define the pins for the four potentiometers
const int cutoffPin = A1; // The pin connected to the cutoff frequency potentiometer
const int resonancePin = A2; // The pin connected to the resonance potentiometer
const int frequencyPin = A3; // The pin connected to the oscillator frequency potentiometer
const int volumePin = A4; // The pin connected to the volume potentiometer

// Define the pins for the two buttons
const int waveformPin = 2; // The pin connected to the waveform button
const int filterPin = 3; // The pin connected to the filter button

// Define the variables to store the potentiometer values
int cutoffValue = 0; // The value read from the cutoff frequency potentiometer
int resonanceValue = 0; // The value read from the resonance potentiometer
int frequencyValue = 0; // The value read from the oscillator frequency potentiometer
int volumeValue = 0; // The value read from the volume potentiometer

// Define the variables to map the potentiometer values to filter, oscillator and envelope parameters
float cutoffFrequency = 0.0f; // The cutoff frequency of the filter in Hz
float resonance = 0.0f; // The resonance of the filter
float frequency = 0.0f; // The frequency of the oscillator in Hz
float volume = 0.0f; // The volume of the output signal

// Define the variables to store the MIDI note and velocity
uint8_t midiNote = 0; // The MIDI note number
uint8_t midiVelocity = 0; // The MIDI note velocity

// Define the variables to store the envelope phase and value
enum EnvelopePhase {
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
};

EnvelopePhase envelopePhase = ATTACK; // The current phase of the envelope
int envelopeValue = 0; // The current value of the envelope

// Define the variables to store the envelope parameters
int attackTime = 0; // The attack time of the envelope in milliseconds
int decayTime = 0; // The decay time of the envelope in milliseconds
int sustainLevel = 0; // The sustain level of the envelope
int releaseTime = 0; // The release time of the envelope in milliseconds

// Define the variables to store the start time of each phase
unsigned long phaseStartTime = 0; // The start time of the current phase in milliseconds

// Define the variables to store the waveform and filter indices
volatile int waveformIndex = 0; // The index of the waveform (0 = sawtooth, 1 = pulse, 2 = triangle)
volatile int filterIndex = 0; // The index of the filter (0 = lowpass, 1 = bandpass, 2 = highpass)

// Define filter objects for each filter type
Filter lowPass = Filter(LOWPASS, 1000, 0.707);
Filter bandPass = Filter(BANDPASS, 1000, 0.707);
Filter highPass = Filter(HIGHPASS, 1000, 0.707);

void setup() {
  MIDI.begin(); // Initialize the MIDI library
  analogWriteResolution(12); // Set the analog write resolution to 12 bits
  analogWrite(dacPin, 2048); // Set the initial output voltage to 2.5V
  pinMode(A0, INPUT); // Set the A0 pin as an input for the gate signal
  pinMode(waveformPin, INPUT_PULLUP); // Set the waveform pin as an input with a pull-up resistor
  pinMode(filterPin, INPUT_PULLUP); // Set the filter pin as an input with a pull-up resistor
  attachInterrupt(digitalPinToInterrupt(waveformPin), changeWaveform, RISING); // Attach an interrupt to the waveform pin to change the waveform on a rising edge
  attachInterrupt(digitalPinToInterrupt(filterPin), changeFilter, RISING); // Attach an interrupt to the filter pin to change the filter on a rising edge
  Scheduler.startLoop(waveformGenerator); // Start the waveform generator loop
  Scheduler.startLoop(midiReader); // Start the midi reader loop
}

void waveformGenerator() {
  static float amplitude = 2047.5f; // The amplitude of the waveform
  static float phase = 0.0f; // The phase of the waveform
  static float phaseIncrement = 0.0f; // The phase increment of the waveform
  static float samplesPerPeriod = 0.0f; // The number of samples per period of the waveform
  static float radians = 0.0f; // The radians of the waveform
  static float value = 0.0f; // The value of the waveform
  static uint32_t lastTime = 0; // The last time in microseconds
  uint32_t currentTime = micros(); // The current time in microseconds
  float deltaTime = (currentTime - lastTime) / 1000000.0f; // The delta time in seconds
  lastTime = currentTime;
  switch (waveformIndex) {
    case 0:
      // Generate sawtooth wave
      phaseIncrement = 2.0f * M_PI * frequency / sampleRate; // Calculate the phase increment based on the frequency and the sample rate
      samplesPerPeriod = sampleRate / frequency; // Calculate the number of samples per period based on the frequency and the sample rate
      for (int i = 0; i < samplesPerPeriod; i++) {
        radians = phase + i * phaseIncrement; // Calculate the radians for each sample
        value = amplitude * (radians - 2.0f * M_PI * floorf(radians / (2.0f * M_PI))) + amplitude; // Calculate the value for each sample
        switch (filterIndex) {
          case 0:
            value = lowPass.next(value); // Use the lowPass filter object
            break;
          case 1:
            value = bandPass.next(value); // Use the bandPass filter object
            break;
          case 2:
            value = highPass.next(value); // Use the highPass filter object
            break;
        }
        // Apply the envelope to the value
        value = value * envelopeValue / 4095.0f;
        // Apply the volume to the value
        value = value * volume / 4095.0f;
        analogWrite(dacPin, value); // Write the value to the DAC pin
        delayMicroseconds(1000000.0f / sampleRate); // Wait for the next sample
      }
      break;
    case 1:
      // Generate pulse wave
      for (int i = 0; i < 2000; i++) {
        int pulse = i < 1000 ? 0 : 4095; // Generate a square wave with a 50% duty cycle
        switch (filterIndex) {
          case 0:
            pulse = lowPass.next(pulse); // Use the lowPass filter object
            break;
          case 1:
            pulse = bandPass.next(pulse); // Use the bandPass filter object
            break;
          case 2:
            pulse = highPass.next(pulse); // Use the highPass filter object
            break;
        }
        // Apply the envelope to the pulse
        pulse = pulse * envelopeValue / 4095.0f;
        // Apply the volume to the pulse
        pulse = pulse * volume / 4095.0f;
        analogWrite(dacPin, pulse); // Write the pulse to the DAC pin
        delayMicroseconds(500); // Wait for the next pulse
      }
      break;
    case 2:
      // Generate triangle wave
      for (int i = 0; i < 2000; i++) {
        int triangle = i < 1000 ? map(i, 0, 1000, 0, 4095) : map(i, 1000, 2000, 4095, 0); // Generate a linear ramp up and down
        switch (filterIndex) {
          case 0:
            triangle = lowPass.next(triangle); // Use the lowPass filter object
            break;
          case 1:
            triangle = bandPass.next(triangle); // Use the bandPass filter object
            break;
          case 2:
            triangle = highPass.next(triangle); // Use the highPass filter object
            break;
        }
        // Apply the envelope to the triangle
        triangle = triangle * envelopeValue / 4095.0f;
        // Apply the volume to the triangle
        triangle = triangle * volume / 4095.0f;
        analogWrite(dacPin, triangle); // Write the triangle to the DAC pin
        delayMicroseconds(500); // Wait for the next triangle
      }
      break;
  }
}

void midiReader() {
  midiEventPacket_t rx; // A MIDI event packet
  rx = MIDI.read(); // Read a MIDI event packet
  if (rx.header != 0) { // If the packet is not empty
    switch (rx.header) { // Check the packet header
      case 0x9: // Note on message
        midiNote = rx.byte1; // Get the note number
        midiVelocity = rx.byte2; // Get the note velocity
        frequency = frequencyAtZeroOctave * pow(2, midiNote / 12.0f); // Calculate the frequency based on the note number
        envelopePhase = ATTACK; // Start the envelope from the attack phase
        phaseStartTime = millis(); // Reset the phase start time
        MIDI.sendNoteOn(midiNote, midiVelocity, 1); // Send a note on message to the output
        break;
      case 0x8: // Note off message
        midiNote = rx.byte1; // Get the note number
        midiVelocity = rx.byte2; // Get the note velocity
        envelopePhase = RELEASE; // Start the envelope from the release phase
        phaseStartTime = millis(); // Reset the phase start time
        MIDI.sendNoteOff(midiNote, midiVelocity, 1); // Send a note off message to the output
        break;
    }
  }
}

void changeWaveform() {
  waveformIndex = (waveformIndex + 1) % 3; // Increment the waveform index and wrap around if necessary
}

void changeFilter() {
  filterIndex = (filterIndex + 1) % 3;
}
