/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2016 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * Contains JavaScript interface for Puck.js
 * ----------------------------------------------------------------------------
 */
#include "jspin.h"

//void jswrap_lorab_joinotaa();
void jswrap_lorab_joinotaa(JsVar *appEui,JsVar *appKey, JsVar *devEui);
void jswrap_lorab_joinabp(JsVar *devAddr, JsVar *nwkSKey, JsVar *appSKey);
void jswrap_lorab_send(JsVar *frame, JsVar *confirmed );
