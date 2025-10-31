/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "base/DLLImports.hpp"
#include "base/String.hpp"

using namespace FW;

//------------------------------------------------------------------------

static bool s_inited = false;

//------------------------------------------------------------------------

static struct
{
    const char* name;
    HMODULE     handle;
}
s_importDLLs[] =
{
    { "kernel32.dll", NULL },
    { "winmm.dll", NULL },
    { "shlwapi.dll", NULL },
};

//------------------------------------------------------------------------

#define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)   static RET (CALL *s_ ## NAME)PARAMS = NULL;
#define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)   static RET (CALL *s_ ## NAME)PARAMS = NULL;
#define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)  static RET (CALL *s_ ## NAME)PARAMS = NULL;
#define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)  static RET (CALL *s_ ## NAME)PARAMS = NULL;
#include "base/DLLImports.inl"
#undef FW_DLL_IMPORT_RETV
#undef FW_DLL_IMPORT_VOID
#undef FW_DLL_DECLARE_RETV
#undef FW_DLL_DECLARE_VOID

//------------------------------------------------------------------------

static const struct
{
    const char* name;
    void**      ptr;
}
s_importFuncs[] =
{
#define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)   { #NAME, (void**)&s_ ## NAME },
#define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)   { #NAME, (void**)&s_ ## NAME },
#define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)  { #NAME, (void**)&s_ ## NAME },
#define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)  { #NAME, (void**)&s_ ## NAME },
#include "base/DLLImports.inl"
#undef FW_DLL_IMPORT_RETV
#undef FW_DLL_IMPORT_VOID
#undef FW_DLL_DECLARE_RETV
#undef FW_DLL_DECLARE_VOID
};

//------------------------------------------------------------------------
//------------------------------------------------------------------------

void FW::initDLLImports(void)
{
    if (s_inited)
        return;
    s_inited = true;

    for (int i = 0; i < (int)FW_ARRAY_SIZE(s_importDLLs); i++)
    {
        s_importDLLs[i].handle = LoadLibrary(s_importDLLs[i].name);
        if (s_importDLLs[i].handle)
            for (int j = 0; j < (int)FW_ARRAY_SIZE(s_importFuncs); j++)
                if (!*s_importFuncs[j].ptr)
                    *s_importFuncs[j].ptr = (void*)GetProcAddress(s_importDLLs[i].handle, s_importFuncs[j].name);
    }
}

//-----------------------------------------------------------------------

void FW::deinitDLLImports(void)
{
    if (!s_inited)
        return;
    s_inited = false;

    for (int i = 0; i < (int)FW_ARRAY_SIZE(s_importDLLs); i++)
    {
        if (s_importDLLs[i].handle)
            FreeLibrary(s_importDLLs[i].handle);
        s_importDLLs[i].handle = NULL;
    }

    for (int i = 0; i < (int)FW_ARRAY_SIZE(s_importFuncs); i++)
        *s_importFuncs[i].ptr = NULL;
}

//------------------------------------------------------------------------

#define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)       RET CALL NAME PARAMS { if (!s_inited) initDLLImports(); if (!s_ ## NAME) fail("Failed to import " #NAME "()!"); return s_ ## NAME PASS; }
#define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)       RET CALL NAME PARAMS { if (!s_inited) initDLLImports(); if (!s_ ## NAME) fail("Failed to import " #NAME "()!"); s_ ## NAME PASS; }
#define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)      RET CALL NAME PARAMS { if (!s_inited) initDLLImports(); if (!s_ ## NAME) fail("Failed to import " #NAME "()!"); return s_ ## NAME PASS; }
#define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)      RET CALL NAME PARAMS { if (!s_inited) initDLLImports(); if (!s_ ## NAME) fail("Failed to import " #NAME "()!"); s_ ## NAME PASS; }

#include "../base/DLLImports.inl"
#undef FW_DLL_IMPORT_RETV
#undef FW_DLL_IMPORT_VOID
#undef FW_DLL_DECLARE_RETV
#undef FW_DLL_DECLARE_VOID

//------------------------------------------------------------------------

#define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)       bool isAvailable_ ## NAME(void) { if (!s_inited) initDLLImports(); return (s_ ## NAME != NULL); }
#define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)       bool isAvailable_ ## NAME(void) { if (!s_inited) initDLLImports(); return (s_ ## NAME != NULL); }
#define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)      bool isAvailable_ ## NAME(void) { if (!s_inited) initDLLImports(); return (s_ ## NAME != NULL); }
#define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)      bool isAvailable_ ## NAME(void) { if (!s_inited) initDLLImports(); return (s_ ## NAME != NULL); }
#include "../base/DLLImports.inl"
#undef FW_DLL_IMPORT_RETV
#undef FW_DLL_IMPORT_VOID
#undef FW_DLL_DECLARE_RETV
#undef FW_DLL_DECLARE_VOID

//------------------------------------------------------------------------
