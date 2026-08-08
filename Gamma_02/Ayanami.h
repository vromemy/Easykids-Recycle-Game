#include <Bluepad32.h>

// ---- how many gamepads you want to read at once ----
#define BP32_MAX_GAMEPADS 1

// All values are now arrays: gamepadLX[0] = gamepad 0's left-stick X, etc.
int gamepadLX[BP32_MAX_GAMEPADS];
int gamepadLY[BP32_MAX_GAMEPADS];
int gamepadRX[BP32_MAX_GAMEPADS];
int gamepadRY[BP32_MAX_GAMEPADS];
int gamepadButton[BP32_MAX_GAMEPADS];
int gamepadCross[BP32_MAX_GAMEPADS];
int gamepadCircle[BP32_MAX_GAMEPADS];
int gamepadSquare[BP32_MAX_GAMEPADS];
int gamepadTriangle[BP32_MAX_GAMEPADS];
int gamepadL1[BP32_MAX_GAMEPADS];
int gamepadL2[BP32_MAX_GAMEPADS];
int gamepadR1[BP32_MAX_GAMEPADS];
int gamepadR2[BP32_MAX_GAMEPADS];
int gamepadUp[BP32_MAX_GAMEPADS];
int gamepadDown[BP32_MAX_GAMEPADS];
int gamepadLeft[BP32_MAX_GAMEPADS];
int gamepadRight[BP32_MAX_GAMEPADS];
int gamepadStart[BP32_MAX_GAMEPADS];
int gamepadSelect[BP32_MAX_GAMEPADS];
int gamepadPad[BP32_MAX_GAMEPADS];

GamepadPtr Ayanami_joystick[BP32_MAX_GAMEPADS];

void Ayanami_onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (joystick[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      joystick[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
      "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void Ayanami_onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (joystick[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      joystick[i] = nullptr;
      // clear that slot's values so a disconnected pad doesn't leave stale/stuck input
      gamepadLX[i] = 0; gamepadLY[i] = 0; gamepadRX[i] = 0; gamepadRY[i] = 0;
      gamepadButton[i] = 0; gamepadPad[i] = 0;
      gamepadCross[i] = 0; gamepadCircle[i] = 0; gamepadSquare[i] = 0; gamepadTriangle[i] = 0;
      gamepadL1[i] = 0; gamepadL2[i] = 0; gamepadR1[i] = 0; gamepadR2[i] = 0;
      gamepadUp[i] = 0; gamepadDown[i] = 0; gamepadLeft[i] = 0; gamepadRight[i] = 0;
      gamepadStart[i] = 0; gamepadSelect[i] = 0;
      foundGamepad = true;
      break;
    }
  }
  if (!foundGamepad) {
    Serial.println(
      "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

void Ayanami_Setup(){
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
              addr[3], addr[4], addr[5]);
  long timer = 0;
  timer = millis();
  do {
    if(sw_Start() == 0){
      BP32.forgetBluetoothKeys();
      break;
    }
  } while ((millis() - timer) < 3000);
  BP32.setup(&Ayanami_onConnectedGamepad, &Ayanami_onDisconnectedGamepad);
  delay(10);
}

void Ayanami_Gamepad() {
  BP32.update();
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr joy = joystick[i];
    if (joy && joy->isConnected()) {
      gamepadLX[i] = map(joy->axisX(), -508, 512, -100, 100);
      gamepadLY[i] = map(joy->axisY(), -508, 512, 100, -100);
      gamepadRX[i] = map(joy->axisRX(), -508, 512, -100, 100);
      gamepadRY[i] = map(joy->axisRY(), -508, 512, 100, -100);
      gamepadButton[i] = joy->buttons();
      gamepadPad[i] = joy->dpad();
      int misc = joy->miscButtons();

      // Each button is checked independently with a bitwise AND, so any
      // combination can be true at the same time instead of only one
      // winning an if/else-if chain (that chain is what caused the
      // "jitter" -- a combo like Cross+Circle matched none of the exact
      // equality checks and fell into the else that zeroed everything).
      gamepadCross[i]    = (gamepadButton[i] & 0x0001) ? 1 : 0;
      gamepadCircle[i]   = (gamepadButton[i] & 0x0002) ? 1 : 0;
      gamepadSquare[i]   = (gamepadButton[i] & 0x0004) ? 1 : 0;
      gamepadTriangle[i] = (gamepadButton[i] & 0x0008) ? 1 : 0;
      gamepadL1[i]       = (gamepadButton[i] & 0x0010) ? 1 : 0;
      gamepadR1[i]       = (gamepadButton[i] & 0x0020) ? 1 : 0;
      gamepadL2[i]       = (gamepadButton[i] & 0x0040) ? 1 : 0;
      gamepadR2[i]       = (gamepadButton[i] & 0x0080) ? 1 : 0;

      gamepadUp[i]    = (gamepadPad[i] & 0x01) ? 1 : 0;
      gamepadDown[i]  = (gamepadPad[i] & 0x02) ? 1 : 0;
      gamepadRight[i] = (gamepadPad[i] & 0x04) ? 1 : 0;
      gamepadLeft[i]  = (gamepadPad[i] & 0x08) ? 1 : 0;

      // Start/Select were declared in the original file but never set;
      // they live in miscButtons(), so filling them in here.
      gamepadStart[i]  = (misc & 0x01) ? 1 : 0;
      gamepadSelect[i] = (misc & 0x02) ? 1 : 0;
    }
  }
}
