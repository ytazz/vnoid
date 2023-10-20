#include "debug.h"

#define _CRT_SECURE_NO_WARNINGS

namespace cnoid {
namespace vnoid {

Debug::Debug() {

}

void Debug::Out(LPCTSTR pszFormat, ...)
{
    va_list argp;
    TCHAR pszBuf[256];
    va_start(argp, pszFormat);
    _vstprintf(pszBuf, pszFormat, argp);
    va_end(argp);
    OutputDebugString(pszBuf);
}

}  // namespace vnoid
}  // namespace cnoid