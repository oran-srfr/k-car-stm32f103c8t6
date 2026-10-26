#ifndef K_CAR_APP_MODULES_H
#define K_CAR_APP_MODULES_H

#include "app.h"

/* Nonblocking module boundaries. Implement these as drivers become available. */
void Modules_Init(void);
void Modules_Poll(uint32_t now_ms, AppInputs *inputs);
void Modules_RunStarted(AppMode mode, uint32_t now_ms);
void Modules_RunStopped(void);
void Modules_Drive(AppDrive command);
void Modules_Sound(AppSound event);
void Modules_Display(const AppStatus *status);

#endif
