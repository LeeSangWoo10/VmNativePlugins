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

#pragma once
#include "base/Defs.hpp"

//------------------------------------------------------------------------

#define _WIN32_WINNT 0x0600
#define WIN32_LEAN_AND_MEAN
#define _KERNEL32_
#define _WINMM_
#include <windows.h>
#undef min
#undef max

#pragma warning(push,3)
#include <mmsystem.h>
#pragma warning(pop)

#define _SHLWAPI_
#include <shlwapi.h>

//------------------------------------------------------------------------

namespace FW
{
void    initDLLImports      (void);
void    deinitDLLImports    (void);
}

#   define FW_DLL_IMPORT_RETV(RET, CALL, NAME, PARAMS, PASS)        bool isAvailable_ ## NAME(void);
#   define FW_DLL_IMPORT_VOID(RET, CALL, NAME, PARAMS, PASS)        bool isAvailable_ ## NAME(void);
#   define FW_DLL_DECLARE_RETV(RET, CALL, NAME, PARAMS, PASS)       bool isAvailable_ ## NAME(void); RET CALL NAME PARAMS;
#   define FW_DLL_DECLARE_VOID(RET, CALL, NAME, PARAMS, PASS)       bool isAvailable_ ## NAME(void); RET CALL NAME PARAMS;
#include "../base/DLLImports.inl"
#   undef FW_DLL_IMPORT_RETV
#   undef FW_DLL_IMPORT_VOID
#   undef FW_DLL_DECLARE_RETV
#   undef FW_DLL_DECLARE_VOID
#   undef FW_DLL_IMPORT_CUDA
#   undef FW_DLL_IMPORT_CUV2

//------------------------------------------------------------------------
