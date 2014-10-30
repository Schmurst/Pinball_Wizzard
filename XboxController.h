////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Xbox controller using xinput.h
//


#ifndef XBOXCONTROLLER_INCLUDED
#define XBOXCONTROLLER_INCLUDED

#include <Xinput.h>
#include <Windows.h>

class XboxController {
private:
  XINPUT_STATE state;
  DWORD dwResult;

public:

  bool getState() {
    dwResult = XInputGetState(0, &state);
    return dwResult == ERROR_SEVERITY_SUCCESS ? true : false;
  }

};

#endif 