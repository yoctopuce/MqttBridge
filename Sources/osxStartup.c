//
//  main.c
//  osxOSHub
//
//  Created by Sébastien Rinsoz on 07.08.11.
//  Copyright 2011 Yoctopuce. All rights reserved.
//

#define __FILENAME__   "linStartup"
#include "MqttBridge.h"

#ifdef OSX_API

#include <IOKit/hid/IOHIDLib.h>
#include <stdio.h>
#include <signal.h>


int main (int argc, char * argv[])
{
    char  errmsg[YOCTO_ERRMSG_LEN*2];
    signal(SIGPIPE,SIG_IGN);
    signal(SIGINT, CtrlCHandler);
    signal(SIGKILL, CtrlCHandler);

    ParseArguments(argc, argv);
    //if((Globalp.flags & VHUB_RUN_IN_DEAMON) && daemon(0,0) < 0)
    //    return 1;
    return  TunnelMain(errmsg);
}

#endif