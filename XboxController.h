////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Xbox controller using xinput.h only for one controller of the game Pinball wizzard
//


#ifndef XBOXCONTROLLER_INCLUDED
#define XBOXCONTROLLER_INCLUDED

#include <Xinput.h>

// Array of WORDS for button checks
static const WORD BUTTONS[] = {
  XINPUT_GAMEPAD_A,
  XINPUT_GAMEPAD_B,
  XINPUT_GAMEPAD_X,
  XINPUT_GAMEPAD_Y,
  XINPUT_GAMEPAD_DPAD_UP,
  XINPUT_GAMEPAD_DPAD_DOWN,
  XINPUT_GAMEPAD_DPAD_LEFT,
  XINPUT_GAMEPAD_DPAD_RIGHT,
  XINPUT_GAMEPAD_LEFT_SHOULDER,
  XINPUT_GAMEPAD_RIGHT_SHOULDER,
  XINPUT_GAMEPAD_LEFT_THUMB,
  XINPUT_GAMEPAD_RIGHT_THUMB,
  XINPUT_GAMEPAD_START,
  XINPUT_GAMEPAD_BACK
};

struct ButtonIndex{
  // default constructor
  ButtonIndex();

  // member veriables
  int A, B, X, Y;
  int DPad_Up, DPad_Down, DPad_Left, DPad_Right;
  int L_Shoulder, R_Shoulder;
  int L_Thumbstick, R_Thumbstick;
  int Start;
  int Back;
};

class XboxController {
private:
  XINPUT_STATE state;
  DWORD dwResult;

  /// returns the gamepad's state
  XINPUT_STATE GetState(){
    XINPUT_STATE padState;
    ZeroMemory(&padState, sizeof(XINPUT_STATE));  // USB detection
    XInputGetState(0, &padState);                 // get input state
    return padState;
  }

public:
  /// default constructor
  XboxController() {
  }

  /// default destructor
  ~XboxController(){
  }

  /// returns true if controller is connected
  bool isConnected() {
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    DWORD error_state = XInputGetState(0, &state);
    return (error_state == ERROR_SUCCESS) ? true : false; // returns true if connected
  }

  /// Updates the gamepad
  void Update() {
    state = GetState();
  }

  // returns whether a button was pressed
  bool isButtonPressed(int button) {

  }

  /// These functions are for the triggers to be used as buttons
  // returns Left trigger value
  float LeftTrigger() {
    BYTE Trigger = state.Gamepad.bLeftTrigger;
    return (Trigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD) ? Trigger / 255.0f : 0.0f;
  }

  // returns right trigger value
  float RightTrigger() {
    BYTE Trigger = state.Gamepad.bLeftTrigger;
    return (Trigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD) ? Trigger / 255.0f : 0.0f;
  }

};

#endif 

// externally refer to the button indexs as XButtonIDs
// this allows for button checking within other classes
extern ButtonIndex XButtonIDs;