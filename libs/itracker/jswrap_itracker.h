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
 * Contains JavaScript interface for Itracker
 * ----------------------------------------------------------------------------
 */
#include "jspin.h"

//Sensor functions

JsVar *jswrap_itracker_bme280data();
JsVar *jswrap_itracker_opt3001data();
JsVar *jswrap_itracker_lis3dhdata();
JsVar *jswrap_itracker_lis2mdldata();

//GSM functions

void jswrap_itracker_gsmon();
void jswrap_itracker_gsmoff();
