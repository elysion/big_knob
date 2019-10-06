#include <MillisTimer.h>
#include <MIDIUSB_Defs.h>
#include <MIDIUSB.h>
#include <RotaryEncoder.h>
#include <Adafruit_NeoPixel.h>
#include <stdint.h>

const byte BUTTON_PIN = 10;
const byte ENCODER_A_PIN = 9;
const byte ENCODER_B_PIN = 8;
const byte LED_PIN = 16;
const byte LED_COUNT = 16;
const byte DECK_COUNT = 2;

Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum ControlType {
  CONTROL_TYPE_ENCODER,
  CONTROL_TYPE_BUTTON
};

const byte MIDI_MAX_VALUE = 127;
const byte MIDI_CC_BACKWARD = 127;
const byte MIDI_CC_FORWARD = 1;

const byte ENCODER_DIRECTION_UP = 1;
const byte ENCODER_DIRECTION_DOWN = -1;
const byte ENCODER_NO_CHANGE = 0;

const byte BUTTON_STATE_DOWN = 0;
const byte BUTTON_STATE_UP = 1;

enum DeviceMode {
  MODE_SETUP,
  MODE_BROWSE,
  MODE_BPM,
  MODE_CUE,
  MODE_MIX
};

enum Control {
  CONTROL_BPM,
  CONTROL_BROWSE,
  CONTROL_CROSSFADER,
  COMMON_CONTROL_COUNT,
  
  CONTROL_LOAD,
  CONTROL_CUE_PLAY,
  CONTROL_PLAY,
  CONTROL_SYNC,
  BUTTON_CONTROL_COUNT, // TODO: is this really wise?
  
  CONTROL_CUE_SELECT,
  CONTROL_EQ_HIGH,
  CONTROL_EQ_MID,
  CONTROL_EQ_LOW,
  CONTROL_VOLUME,
  CONTROL_FILTER,
  CONTROL_COUNT
};

const byte MAX_CONTROL = CONTROL_COUNT * 2 - COMMON_CONTROL_COUNT + 3;

const char *CONTROL_NAMES[] = {
  "BPM",
  "BRWS",
  "CFDR",
  0,
  "LOAD",
  "CUP",
  "PLAY",
  "SYNC",
  0,
  "CUES",
  "EQHI",
  "EQMI",
  "EQLO",
  "VOL",
  "FLTR",
  "DONE",
};

enum DeckTarget {
  OUTGOING_DECK,
  INCOMING_DECK,
  GLOBAL
};

struct Transition {
  Control control;
  DeckTarget target;
  int startPosition;
  byte startValue;
  int endPosition;
  byte endValue;
  //, curve?
};

const Transition transitions[] = {
  {CONTROL_CROSSFADER, GLOBAL, 0, 0, 127, 127}, // Crossfader not needed

  {CONTROL_VOLUME, INCOMING_DECK, 0, 0, 50, 127},
  {CONTROL_EQ_HIGH, INCOMING_DECK, 0, 30, 64, 64},
  {CONTROL_EQ_MID, INCOMING_DECK, 0, 30, 64, 64},
  {CONTROL_EQ_LOW, INCOMING_DECK, 64, 0, 68, 64},
  {CONTROL_FILTER, INCOMING_DECK, 0, 64, 127, 64},

  // Previous deck transitions need to be grouped to the bottom of the list
  {CONTROL_EQ_HIGH, OUTGOING_DECK, 0, 64, 60, 50},
  {CONTROL_EQ_MID, OUTGOING_DECK, 0, 64, 60, 50},
  {CONTROL_EQ_LOW, OUTGOING_DECK, 64, 64, 68, 0},
  {CONTROL_VOLUME, OUTGOING_DECK, 70, 127, 127, 0},
  {CONTROL_FILTER, OUTGOING_DECK, 90, 64, 100, 127}
};

#define TRANSITION_COUNT (sizeof(transitions)/sizeof(Transition))
const int TRANSITION_MAX_VALUE = 127;

RotaryEncoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);
bool buttonState = 1;
bool previousButtonState = 1;

byte currentMode = MODE_SETUP;
int currentPosition = -1;
byte incomingDeck = 0;
byte currentBpm = 174;
byte targetBpm = 174;
bool isFirstTrack = true;

Transition bpmTransition = {CONTROL_BPM, GLOBAL, 0, currentBpm, 127, targetBpm};

enum LedMode {
  LED_RAINBOW,
  LED_PUMP,
  LED_FLASH
};

MillisTimer timer1 = MillisTimer();

LedMode currentLedMode = LED_RAINBOW;
uint16_t ledValue = 0;
byte ledChange = 100;
const int PUMP_TIME = 2500;
void updateLed(MillisTimer &mt)
{
  ledValue += ledChange;

  uint32_t color;
  switch (currentLedMode) {
    case LED_RAINBOW:
      color = leds.ColorHSV(ledValue, UINT16_MAX, UINT16_MAX);
      break;
    case LED_FLASH:
      color = leds.Color(0, 0, 0, ledValue % 10 == 0 ? UINT16_MAX : 0);
      break;
    case LED_PUMP:
      int value = ledValue % PUMP_TIME;
      color = leds.ColorHSV(0, UINT16_MAX, (value > PUMP_TIME/2 ? PUMP_TIME - value : value) * (UINT16_MAX / PUMP_TIME));
  }
  
  leds.fill(color, 0, LED_COUNT);
  leds.show();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Boot");
  
  noInterrupts();
  leds.begin();
  interrupts();

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  PCMSK0 |= (1 << 6) | (1 << 5) | (1 << 4);
  PCICR |= (1 << PCIE0);

  timer1.setInterval(16);
  timer1.expiredHandler(updateLed);
  timer1.start();
}

void loop() {
  delay(100);
  Serial.print(".");
  MidiUSB.flush();

  const byte direction = encoder.getDirection();
  if (direction != ENCODER_NO_CHANGE) {
    handleControlChange(CONTROL_TYPE_ENCODER, direction);
  } else if (buttonState != previousButtonState) {
    previousButtonState = buttonState;
    handleControlChange(CONTROL_TYPE_BUTTON, buttonState);
  }

  timer1.run();
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

uint32_t ledColorForControl(byte control) {
  leds.ColorHSV(control * (UINT16_MAX / 5));
}

byte ledPositionForControl(byte control) {
  
}

uint32_t colorForControl(Control control) {
  return leds.ColorHSV(control * (UINT16_MAX / 10 * 3), UINT16_MAX, UINT16_MAX);
}

void updateSetupLed() {
  leds.fill(colorForControl(getControlForPosition(currentPosition)), 0, LED_COUNT);
  leds.show();
}

Control getControlForPosition(byte position) {
  if (position < CONTROL_COUNT) {
    return position;
  } else if (position >= MAX_CONTROL) {
    return CONTROL_COUNT;
  } else {
    return position - CONTROL_COUNT;
  }
}

ControlType getTypeForControl(Control control) {
  if (control < BUTTON_CONTROL_COUNT && control > COMMON_CONTROL_COUNT) {
    return CONTROL_TYPE_BUTTON;
  } else {
    return CONTROL_TYPE_ENCODER;
  }
}

void sendControlName(Control control) {
  char *controlName = CONTROL_NAMES[control];

  Serial.print("Sending control for: ");
  Serial.println(controlName);
  Serial.print("Position: ");
  Serial.println(currentPosition);
  Serial.print("Control: ");
  Serial.println(control);
}

byte getValueForTransition(Transition transition, int position) {
  Serial.print("Got position: ");
  Serial.println(position);
  if (position < transition.startPosition) {
    return transition.startValue;
  } else if (position > transition.endPosition) {
    return transition.endValue;
  } else {
    const int distanceFromStart = position - transition.startPosition;
    const int transitionLength = transition.endPosition - transition.startPosition;
    const int transitionValueChange = transition.endValue - transition.startValue;

    return transition.startValue + (transitionValueChange * distanceFromStart / transitionLength);
  }
}

void changeMode(DeviceMode targetMode) {
  switch (currentMode) {
    case MODE_SETUP:
      currentPosition = 0;
      
      incomingDeck = 1;
      runTransitions(currentPosition, false);
      
      incomingDeck = 0;
      break;
    case MODE_MIX:
      noteOff(outgoingDeck(), CONTROL_PLAY, MIDI_MAX_VALUE);
      changeToNextDeck();
      break;
    case MODE_BPM:
      sendToAllDecks(CONTROL_SYNC, MIDI_MAX_VALUE);
      break;
  }
  
  currentMode = targetMode;

  switch (targetMode) {
    case MODE_BROWSE:
      currentLedMode = LED_RAINBOW;
      timer1.start();
      break;
    case MODE_MIX:
      currentPosition = incomingDeck == 1 ? 0 : 127;
      currentLedMode = LED_PUMP;
      timer1.start();
      break;
    case MODE_BPM:
      sendToAllDecks(CONTROL_SYNC, 0);
      timer1.stop();
      leds.fill(leds.ColorHSV(UINT16_MAX / 40 * (targetBpm - 140), UINT16_MAX, UINT16_MAX / 10), 0, LED_COUNT);
      leds.show();
      break;
  }
}

void sendToAllDecks(Control control, byte value) {
  for (byte i = 0; i < DECK_COUNT; ++i) {
    if (getTypeForControl(control) == CONTROL_TYPE_ENCODER) {
      controlChange(i, control, value);
    } else {
      noteOn(i, control, value);
    }
  }
}

void changeToNextMode() {
  changeMode(currentMode + 1);
}

void changeToNextDeck() {
  incomingDeck = nextDeck();
}

byte nextDeck() {
  return (incomingDeck + 1) % DECK_COUNT;
}

byte outgoingDeck() {
  return (incomingDeck + DECK_COUNT - 1) % DECK_COUNT;
}

const uint16_t MIDI_MIN_BPM = 40;
const uint16_t MIDI_MAX_BPM = 300;
byte bpmToMidi(byte bpm) { // TODO: with byte, max bpm = 255
  return (bpm - MIDI_MIN_BPM) * 127 / (MIDI_MAX_BPM - MIDI_MIN_BPM);
}

bool transitionDone() {
  return currentPosition % TRANSITION_MAX_VALUE == 0;
}

void runTransitions(int position, bool reverseDeckPositions) {
  int channelPosition = reverseDeckPositions ? 127 - position : position;
  
  for (byte i = 0; i < TRANSITION_COUNT; ++i) {
    const Transition transition = transitions[i];
    byte targetDeck = incomingDeck;
    
    if (transition.target == GLOBAL) {
      targetDeck = 0;
    } else if (transition.target == OUTGOING_DECK) {
      targetDeck = outgoingDeck();
      Serial.print("Deck: ");
      Serial.println(targetDeck);
      Serial.println("Changing to outgoing deck");
    }
    
    const byte valueForControl = getValueForTransition(transition, transition.target == GLOBAL ? position : channelPosition);
    controlChange(targetDeck, transition.control, valueForControl);

    Serial.print("Transition: ");
    Serial.println(i);
    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" ");
    Serial.println(channelPosition);
    Serial.print("Deck: ");
    Serial.println(targetDeck);
    Serial.print("Value for control: ");
    Serial.println(transition.control);
    sendControlName(transition.control);
    Serial.println(valueForControl);
  }

  const byte valueForControl = getValueForTransition(bpmTransition, channelPosition);
  Serial.print("BPM from: ");
  Serial.print(bpmTransition.startValue);
  Serial.print(", to: ");
  Serial.println(bpmTransition.endValue);
  Serial.println(valueForControl);
  controlChange(0, bpmTransition.control, bpmToMidi(valueForControl));
}

void handleControlChange(ControlType type, byte value) {
  Serial.println("Received event:");

  Serial.print(", Type: ");
  Serial.print(type);
  Serial.print(", Value: ");
  Serial.println(value);

  Serial.print("Current mode: ");
  Serial.println(currentMode);

  Serial.print("Current deck: ");
  Serial.println(incomingDeck);

  switch (currentMode) {
    case MODE_SETUP:
    timer1.stop();
    if (type == CONTROL_TYPE_ENCODER) {
      if (value == ENCODER_DIRECTION_UP) {
        currentPosition = min(currentPosition + 1, CONTROL_COUNT * 2);
        
        if (currentPosition % CONTROL_COUNT == BUTTON_CONTROL_COUNT || currentPosition == COMMON_CONTROL_COUNT) {
          currentPosition++;
        } else if (currentPosition == CONTROL_COUNT) {
          currentPosition += COMMON_CONTROL_COUNT + 1;
        }
      } else {
        currentPosition = max(currentPosition - 1, 0);
        if (currentPosition == CONTROL_COUNT + COMMON_CONTROL_COUNT) {
          currentPosition -= COMMON_CONTROL_COUNT + 1;
        }
      }
      
      incomingDeck = currentPosition > CONTROL_COUNT ? 1 : 0;
      updateSetupLed();
      Control control = getControlForPosition(currentPosition);
      sendControlName(control);
    } else if (type == CONTROL_TYPE_BUTTON) {
      Control control = getControlForPosition(currentPosition);
      if (currentPosition >= MAX_CONTROL && value == BUTTON_STATE_DOWN) {
        currentPosition = 0;
        changeToNextMode();
      } else if (getTypeForControl(control) == CONTROL_TYPE_ENCODER) {
        controlChange(incomingDeck, control, 1);
      } else {
        noteOn(incomingDeck, control, 127);
      }
    }
    break;
    
    case MODE_BROWSE:
    if (type == CONTROL_TYPE_ENCODER) {
      controlChange(0, CONTROL_BROWSE, value == ENCODER_DIRECTION_UP ? MIDI_CC_FORWARD : MIDI_CC_BACKWARD);
    } else if (type == CONTROL_TYPE_BUTTON && value == BUTTON_STATE_DOWN) {
      noteOn(incomingDeck, CONTROL_LOAD, MIDI_MAX_VALUE);
      noteOff(incomingDeck, CONTROL_LOAD, MIDI_MAX_VALUE);
      changeToNextMode();
      
      Serial.println("Track selected");
    }
    break;

    case MODE_BPM:
    if (type == CONTROL_TYPE_ENCODER) {
      targetBpm += value == ENCODER_DIRECTION_UP ? 1 : -1;
      targetBpm = constrain(targetBpm, 40, 245); // TODO: Traktor limits bpm to 300
      Serial.print("Trying to set bpm to: ");
      Serial.println(targetBpm);
      Serial.println(bpmToMidi(targetBpm));
      controlChange(0, CONTROL_BPM, bpmToMidi(targetBpm));
       
      leds.fill(leds.ColorHSV(UINT16_MAX / 40 * (targetBpm - 140), UINT16_MAX, UINT16_MAX / 10), 0, LED_COUNT);
      leds.show();
    } else if (type == CONTROL_TYPE_BUTTON && value == BUTTON_STATE_DOWN) {
      if (!isFirstTrack) {
        controlChange(0, CONTROL_BPM, bpmToMidi(currentBpm));
        bpmTransition = {CONTROL_BPM, GLOBAL, 56, currentBpm, 64, targetBpm};
        Serial.print("Target BPM set from ");
        Serial.print(currentBpm);
        Serial.print(" to ");
        Serial.println(targetBpm);
      }
      
      changeToNextMode();
    }
    break;
    
    case MODE_CUE:
    if (type == CONTROL_TYPE_ENCODER) {
      controlChange(incomingDeck, CONTROL_CUE_SELECT, value == ENCODER_DIRECTION_UP ? MIDI_CC_FORWARD : MIDI_CC_BACKWARD);
    } else if (type == CONTROL_TYPE_BUTTON && value == BUTTON_STATE_DOWN) {
      Serial.println("Cue selected");
      noteOn(incomingDeck, CONTROL_PLAY, MIDI_MAX_VALUE);

      if (isFirstTrack) {
        isFirstTrack = false;
        changeToNextDeck();
        changeMode(MODE_BROWSE);
      } else {
        changeMode(MODE_MIX);
      }
    }
    // TODO: How to send the CUP noteOff message?
    break;
    
    case MODE_MIX:
    if (type == CONTROL_TYPE_ENCODER) {
      currentPosition += value == ENCODER_DIRECTION_UP ? 2 : -2;
      currentPosition = constrain(currentPosition, 0, MIDI_MAX_VALUE);
      Serial.print("Current position: ");
      Serial.println(currentPosition);
            
      runTransitions(currentPosition, incomingDeck % 2 == 0);

      if (transitionDone()) {
          leds.fill(0, 0, LED_COUNT);
      }
    } else if (type == CONTROL_TYPE_BUTTON && value == BUTTON_STATE_DOWN && transitionDone()) {
      changeMode(MODE_BROWSE);
    } else if (type == CONTROL_TYPE_BUTTON && value == BUTTON_STATE_DOWN) {
      // TODO: implement cross switch 
    }
    // TODO: calculate the value to be sent using the Transitions
    break;
  }
}

void updateButtonState() {
  buttonState = digitalRead(BUTTON_PIN);
}

ISR(PCINT0_vect) {
  encoder.tick();
  updateButtonState();
}
