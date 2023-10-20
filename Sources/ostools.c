/*********************************************************************
 *
 * $Id: ostools.c 57313 2023-10-20 09:00:00Z seb $
 *
 * Generic os helpers
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

#define __FILENAME__   "ostools"
#include "ostools.h"

#include <stdio.h>
#include <string.h>

#ifdef WINDOWS_API
#include "Windows.h"
#include <direct.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif

static char* findPreviousSep(char *str,char *start)
{
    char *org=str;
    while(str > start){
        if(*str=='/' || *str=='\\'){
            return str;
        }
        str--;
    }
    return org;
}

static char* appenddir(char *path,const char *dir, int len,int sep)
{
    if(sep){
#ifdef WIN32
        *path++='\\';
#else
        *path++='/';
#endif
    }
    memcpy(path,dir,len);
    path += len;
    *path=0;
    return path;
}


char* relativeToAbsolutePath(const char* relativePath)
{

	char buffer[1024];
	char *p;
	if (
#ifdef WINDOWS_API
	relativePath[1]==':' && relativePath[2]=='\\'
#else
	relativePath[0]=='/'
#endif
	){
		return ystrdup(relativePath);
	}

#ifdef WINDOWS_API
	p=_getcwd(buffer,1023);
#else
	p=getcwd(buffer,1023);
#endif
	if(p==NULL){
		return  ystrdup(relativePath);
	}
	yStrChDir(buffer, 1023,relativePath);
	return ystrdup(buffer);
}

 int yStrChDir(char *buffer, int len, const char *path)
{
    const char *to,*from;
    char *last = buffer + strlen(buffer);

    //check lenght
    if((u32) len <strlen(buffer)+strlen(path)+1){
        return -1;
    }
    to=path;
    while(*to=='/' || *to=='\\')
        to++;
    from=to;
    while(last>buffer && (*(last-1)=='/' || *(last-1)=='\\'))
        last--;
    *last=0;
    while(*to){
        if(*to=='/' || *to=='\\'){
            if( (to-from ==1 && *from=='.')){
                //. do nothing
            }else if(to-from ==2 && *from=='.' && *(from+1)=='.'){
                // ..
                char *newpos = findPreviousSep(last,buffer);
                if(newpos != last){
                    // pop the last directory of the path
                    last=newpos;
                    *last=0;
                }else{
                    // we have nothing to pop: leave the ../ into the path
#ifdef WIN32
                    if(last==buffer){
                        last = appenddir(last,from,(int)(to-from),0);
                    }else
#endif
                    {
                        last = appenddir(last,from,(int)(to-from),1);
                    }
                }
            }else{
               // std name
#ifdef WIN32
                if(last==buffer){
                    last = appenddir(last,from,(int)(to-from),0);
                }else
#endif
                {
                    last = appenddir(last,from,(int)(to-from),1);
                }
            }
            to++;
            from=to;
        }else{
            to++;
        }
    }
     last = appenddir(last,from,(int)(to-from),1);

    return (int)(last-buffer);
}


char *loadtext(const char *filename)
{
    FILE            *file = NULL;
    char            *contents;
    int             size, rsize;

    // load text file
    if(YFOPEN(&file,filename, "rt") != 0) {
        return NULL;
    }
    fseek(file, 0, SEEK_END);
    size = (int)ftell(file);
    if(size > 0x100000){
        fclose(file);
        return NULL;
    }
    contents = calloc(1,size+1);
    if (contents) {
        fseek(file, 0, SEEK_SET);
        rsize = (int)fread(contents, 1, size, file);
        if (rsize < 0) rsize = 0;
        fclose(file);
        contents[rsize] = 0;
    }

    return contents;
}
