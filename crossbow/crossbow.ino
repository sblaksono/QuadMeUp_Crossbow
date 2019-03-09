/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this file,
You can obtain one at http://mozilla.org/MPL/2.0/.

Copyright (c) 20xx, MPL Contributor1 contrib1@example.net
*/

#include "config.h"
#include "board.h"
#include "crossbow.h"

void setup(void)
{
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif

    Crossbow_setup();

}

void loop(void)
{

    Crossbow_loop();
    Crossbow_updateLeds();

}
