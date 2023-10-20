/*********************************************************************
 *
 * $Id: MqttBridge.h 57313 2023-10-20 09:00:00Z seb $
 *
 * Application-specific data structures
 *
 * - - - - - - - - - License information: - - - - - - - - -
 *
 *  Copyright (C) 2011 and beyond by Yoctopuce Sarl, Switzerland.
 *
 *  Yoctopuce Sarl (hereafter Licensor) grants to you a perpetual
 *  non-exclusive license to use, modify, copy and integrate this
 *  file into your software for the sole purpose of interfacing
 *  with Yoctopuce products.
 *
 *  You may reproduce and distribute copies of this file in
 *  source or object form, as long as the sole purpose of this
 *  code is to interface with Yoctopuce products. You must retain
 *  this notice in the distributed source file.
 *
 *  You should refer to Yoctopuce General Terms and Conditions
 *  for additional information regarding your rights and
 *  obligations.
 *
 *  THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 *  WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING
 *  WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO
 *  EVENT SHALL LICENSOR BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 *  INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 *  COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR
 *  SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
 *  LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR
 *  CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON THE
 *  BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF
 *  WARRANTY, OR OTHERWISE.
 *
 *********************************************************************/

#ifndef MQTTBRIDGE_H
#define MQTTBRIDGE_H

#include "yapi.h"
#include "yproto.h"
#include "ostools.h"

#define VHUB_RUN_IN_DEAMON      1
#define IGNORE_CERT             2
#define MQTT_USE_SSL            4
#define MQTT_NO_INSTANT_VALUE   8
#define MQTT_NO_RETAIN          16
#define MQTT_RW_MODE            32

#define MQTT_CONNECT_TIMEOUT    10

typedef struct {
    u32         flags;
    const char* logfile;
    const char* settingsfile;
    const char* ca_certiffile;
    const char* yoctohub_url;
    const char* mqttbroker_host;
    u16         mqttbroker_port;
    const char* mqtt_port;
    const char* mqtts_port;
    const char* mqttUser;
    const char* mqttPass;
    const char* rootTopic;
    const char* clientID;
} GlobalParameters;

extern GlobalParameters Globalp;
extern u32 EnsureDaemonExitDelay;
extern yStrRef NetworkHubNameRef;

void    ylogf(const char* fmt, ...);
int     ParseArguments(int argc, char* argv[]);
void    CtrlCHandler(int dummy);
int     TunnelMain(char* errmsg);
void    EnsureDaemonExit(void);

int	    InstallService(int install, const char* exe, const char* args, char* errmsg);

#endif