#pragma once

#include <cnoid/EigenTypes>
#include <stdio.h>
#include <locale.h>
#include <tchar.h>
#include <stdlib.h>
#include <stdarg.h>

namespace cnoid {
namespace vnoid {

#include <windows.h>

#define OPD (Debug::Out)

class Debug {
public: 
    static void Out(LPCTSTR pszFormat, ...);

    Debug();
};

}  // namespace vnoid
}  // namespace cnoid