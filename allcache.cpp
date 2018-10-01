/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2018 Intel Corporation. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */
/*! @file
 *  This file contains an ISA-portable PIN tool for functional simulation of
 *  instruction+data TLB+cache hierarchies
 */

//================================================================================
// Includes and type definitions
//================================================================================
#include <iostream>
#include "pin.H"
typedef UINT32 CACHE_STATS; // type of cache hit/miss counters
#include "cache.H"

//================================================================================
// Knobs and Output Related
//================================================================================
ofstream out;

KNOB<string> knob_output(KNOB_MODE_WRITEONCE, "pintool",
	"o", "memtrans.out", "specify log file name");
KNOB<UINT32> knob_size(KNOB_MODE_WRITEONCE, "pintool",
	"s", "4096", "Cache size (bytes)");
KNOB<UINT32> knob_associativity(KNOB_MODE_WRITEONCE, "pintool", 
	"a", "1", "Cache associativity");
KNOB<UINT32> knob_line_size(KNOB_MODE_WRITEONCE, "pintool",
	"l", "64", "Cache line size");

LOCALVAR CACHE_DIRECT_MAPPED* llc;

LOCALFUN VOID Fini(int code, VOID * v)
{
    out << llc;
	delete llc;
}

LOCALFUN VOID CacheLoad(ADDRINT addr, UINT32 size)
{
	ADDRINT highAddr;
	highAddr = addr + size;
	do {
		llc->LdAccessSingleLine(addr);
		addr += llc->LineSize();
	} while (addr < highAddr);
}

LOCALFUN VOID CacheStore(ADDRINT addr, UINT32 size)
{
	ADDRINT highAddr;
	highAddr = addr + size;
	do {
		llc->StAccessSingleLine(addr);
		addr += llc->LineSize();
	} while (addr < highAddr);
}

LOCALFUN VOID CacheLoadSingle(ADDRINT addr)
{
	llc->LdAccessSingleLine(addr);
}

LOCALFUN VOID CacheStoreSingle(ADDRINT addr)
{
	llc->StAccessSingleLine(addr);
}

LOCALFUN VOID Instruction(INS ins, VOID *v)
{
	// all instruction fetches access I-cache
#ifdef INS_SIM  
	INS_InsertCall(
		ins, IPOINT_BEFORE, (AFUNPTR)CacheLoadSingle,
		IARG_INST_PTR,
		IARG_END);
#endif
	if (INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins))
	{
		// only predicated-on memory instructions access D-cache
		UINT32 size = INS_MemoryReadSize(ins);
		if (size <= llc->LineSize()) {
			INS_InsertPredicatedCall(
				ins, IPOINT_BEFORE, (AFUNPTR)CacheLoadSingle,
				IARG_MEMORYREAD_EA,
				IARG_END);
		}
		else {
			INS_InsertPredicatedCall(
				ins, IPOINT_BEFORE, (AFUNPTR)CacheLoad,
				IARG_MEMORYREAD_EA,
				IARG_MEMORYREAD_SIZE,
				IARG_END);
		}
	}

	if (INS_IsMemoryWrite(ins) && INS_IsStandardMemop(ins))
	{
		// only predicated-on memory instructions access D-cache
		UINT32 size = INS_MemoryWriteSize(ins);
		if (size <= llc->LineSize()) {
			INS_InsertPredicatedCall(
				ins, IPOINT_BEFORE, (AFUNPTR)CacheStoreSingle,
				IARG_MEMORYWRITE_EA,
				IARG_END);
		}
		else {
			INS_InsertPredicatedCall(
				ins, IPOINT_BEFORE, (AFUNPTR)CacheStore,
				IARG_MEMORYWRITE_EA,
				IARG_MEMORYWRITE_SIZE,
				IARG_END);
		}
	}
}

void initCache(void)
{
	llc = new CACHE_DIRECT_MAPPED("Last Level Cache", knob_size.Value(), knob_line_size.Value(), knob_associativity.Value());
	pin_timing::start = clock();
	fill_hamming_lut();
}

GLOBALFUN int main(int argc, char *argv[])
{
	PIN_Init(argc, argv);
	initCache();

    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    // Never returns
    PIN_StartProgram();

    return 0; // make compiler happy
}
