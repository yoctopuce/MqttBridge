/*********************************************************************
 *
 * $Id: request.h 57313 2023-10-20 09:00:00Z seb $
 *
 *  Helpers to make requests to Yoctopuce hosts using yapi from C
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

#ifndef REQUEST_H
#define REQUEST_H

#include "yapi.h"
#include "yproto.h"

#define yFarFifoBuf         yFifoBuf
#define yFarFifoInit        yFifoInit
#define yFarFifoEmpty       yFifoEmpty
#define yPushFarFifo        yPushFifo
#define yPopFarFifo         yPopFifo
#define yPeekFarFifo        yPeekFifo
#define ySeekFarFifo        ySeekFifo
#define yFarFifoGetUsed     yFifoGetUsed
#define yFarFifoGetFree     yFifoGetFree

// Test if the registered hub is actually connected
int             isHubConnected(char** errmsg);
// When the hub has been connected at least once, return its serial number
char*           getHubSerialNumber(void);

// Functions to make a request to the yoctohub via FIFO objects
int             yoctohubOpenReq(char **errmsg);
yFarFifoBuf*    yoctohubGetInputFifo(void);
yFarFifoBuf*    yoctohubGetOutputFifo(void);
int             yoctohubReqIsDone(void);
void            yoctohubCloseReq(void);

// Issue a request to the hub, and optionally return the result as a dynamically allocated, NUL-terminated string
int yoctohubQuickGet(const char* relativeUrl, char **result, char* errmsg);

#endif
