/*********************************************************************
 *
 * $Id: request.c 57313 2023-10-20 09:00:00Z seb $
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

#define __FILENAME__   "request"
#include "request.h"
#include "yhash.h"

#include <stdio.h>
#include <string.h>

int isHubConnected(char **errmsg)
{
    int hubref = yapiGetNextHubRef(-1);
    if (hubref == -1) {
        *errmsg = "No hub was registered";
        return 0;
    }
    YRETCODE rc = yapiGetHubIntAttr(hubref, "isOnline");
    if (rc <= 0) {
        *errmsg = "Unable to attach to Yoctopuce hub";
        return 0;
    }
    return 1;
}

char* getHubSerialNumber(void)
{
    return SerialNumberStr;
}

// Functions to make a request to the yoctohub via FIFO objects
char errMsgBuf[512];
u8 inputBuffer[1024];
u8 outputBuffer[32768];
u8 fifoInitialized = 0;
yFarFifoBuf inputFifo;
yFarFifoBuf outputFifo;

int yoctohubOpenReq(char **errmsg)
{
    if (!isHubConnected(errmsg)) {
        return -1;
    }
    if (!fifoInitialized) {
        yFarFifoInit(&inputFifo, inputBuffer, sizeof(inputBuffer));
        yFarFifoInit(&outputFifo, outputBuffer, sizeof(outputBuffer));
        fifoInitialized = 1;
    } else {
        yFarFifoEmpty(&inputFifo);
        yFarFifoEmpty(&outputFifo);
    }
    return 0;
}

yFarFifoBuf* yoctohubGetInputFifo(void)
{
    return &inputFifo;
}

yFarFifoBuf* yoctohubGetOutputFifo(void)
{
    int requestSize = yFarFifoGetUsed(&inputFifo);
    if (requestSize > 0 && yFarFifoGetUsed(&outputFifo) == 0) {
        // Execute request
        YRETCODE rc;
        YIOHDL   iohdl;
        char     *reply;
        int      replySize;

        char* request = calloc(1, requestSize + 1);
        yPopFarFifo(&inputFifo, (u8 *)request, requestSize);
        rc = yapiHTTPRequestSyncStart(&iohdl, SerialNumberStr, request, &reply, &replySize, errMsgBuf);
        if (rc >= 0) {
            if (memcmp(reply, "HTTP/1", 6) == 0) {
                // transform full reply in short reply, for compatibility with our mqtt code
                int p;
                for (p = 0; reply[p] != ' ' && reply[p] != '\r'; p++);
                if (reply[p] == ' ') p++;
                if (reply[p] == '2') {
                    // success, replace by standard short reply
                    yPushFarFifo(&outputFifo, (u8*)"OK\r\n\r\n", 6);
                    // locate end of HTTP header
                    for (p = 0; p + 3 < replySize; p++) {
                        if (reply[p] == '\r' && reply[p + 1] == '\n' && reply[p + 2] == '\r' && reply[p + 3] == '\n') {
                            p += 4;
                            break;
                        }
                    }
                    // drop header
                    yPushFarFifo(&outputFifo, (u8*)reply+p, replySize-p);
                } else {
                    // request failed, keep full headers
                    yPushFarFifo(&outputFifo, (u8*)reply, replySize);
                }
            } else {
                yPushFarFifo(&outputFifo, (u8*)reply, replySize);
            }
            yapiHTTPRequestSyncDone(&iohdl, errMsgBuf);
        }
        free(request);
    }
    return &outputFifo;
}

int yoctohubReqIsDone(void)
{
    yFarFifoBuf* outfifo = yoctohubGetOutputFifo();
    if (yFifoGetUsed(outfifo) == 0) {
        return 1;
    }
    return 0;
}

void yoctohubCloseReq(void)
{
    yFarFifoEmpty(&inputFifo);
    yFarFifoEmpty(&outputFifo);
}

// Issue a request to the hub, and optionally return the result as a dynamically allocated, NUL-terminated string
int yoctohubQuickGet(const char* relativeUrl, char **output, char* errmsg)
{
    YRETCODE rc;
    YIOHDL   iohdl;
    char     *errptr, *request, *reply;
    int      requestSize, replySize;

    if (!isHubConnected(&errptr)) {
        ystrcpy(errmsg, 128, errptr);
        return YAPI_IO_ERROR;
    }
    requestSize = (int)strlen(relativeUrl) + 6;
    request = calloc(1, requestSize);
    if (!request) {
        return YAPI_EXHAUSTED;
    }
    ystrcpy(request, requestSize, "GET /");
    ystrcat(request, requestSize, relativeUrl);
    rc = yapiHTTPRequestSyncStart(&iohdl, SerialNumberStr, request, &reply, &replySize, errmsg);
    free(request);
    if (rc < 0) {
        return rc;
    }
    if (replySize < 6 || memcmp(reply, "OK\r\n\r\n", 6) != 0) {
        if (replySize < 10 || memcmp(reply, "HTTP/1.1 ", 9) != 0) {
            // expect at least HTTP/1.1 <status> 
            yapiHTTPRequestSyncDone(&iohdl, errmsg);
            return YAPI_IO_ERROR;
        }
        rc = atoi(reply + 9);
        if (rc != 200) {
            char* p = reply + 9;
            while (*p && *p != '\r' && *p != '\n') p++;
            ystrcpy(errmsg, 128, "Hub returned ");
            ystrncpy(errmsg + 13, 128 - 13, reply, (int)(p - reply));
            errmsg[13 + p - reply] = 0;
            yapiHTTPRequestSyncDone(&iohdl, errmsg);
            return (rc == 404 ? YAPI_FILE_NOT_FOUND : YAPI_IO_ERROR);
        }
    }
    if (output) {
        int p;
        for (p = 0; p+3 < replySize; p++) {
            if (reply[p] == '\r' && reply[p+1] == '\n' && reply[p+2] == '\r' && reply[p+3] == '\n') {
                // skip over HTTP header
                reply += p + 4;
                replySize -= p + 4;
                break;
            }
        }
        *output = calloc(1, replySize + 1);
        if (*output) memcpy(*output, reply, replySize);
    }
    yapiHTTPRequestSyncDone(&iohdl, errmsg);
    return 0;
}

