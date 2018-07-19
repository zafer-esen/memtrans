/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2017 Intel Corporation. All rights reserved.
 
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

#include <iostream>
#include <cstdint>

#include "pin.H"
#include <time.h>

#define INS_SIM //if defined, the tool will simulate the instruction accesses

typedef UINT32 CACHE_STATS; // type of cache hit/miss counters

#include "memtrans_cache2.H"

namespace UL3
{
  // 3rd level unified cache: 16 MB, 64 B lines, direct mapped
  const UINT32 cacheSize = 16*MEGA;
  const UINT32 lineSize = 64;
  const CACHE_ALLOC::STORE_ALLOCATION allocation = CACHE_ALLOC::STORE_ALLOCATE;

  const UINT32 max_sets = cacheSize / (lineSize);
}

clock_t start;
LOCALFUN VOID Fini(int code, VOID * v)
{
  clock_t end = clock() ;
  double elapsed_time = (end-start)/(double)CLOCKS_PER_SEC ;

  //std::cerr << ul3;
  std::cout << "L3 miss count: " << L3MissCount << std::endl;
  std::cout << "L3 store evict count: " << L3EvictCount << std::endl;
  std::cout << "Total number of bit transitions: " << totalTransitions << "\n\n";
  std::cout << "Elapsed time: " << elapsed_time << std::endl;
  delete lineBytes;
}

LOCALFUN VOID CacheLoad(ADDRINT addr, UINT32 size)
{
  ADDRINT highAddr;
  highAddr = addr + size;
  do{
      ul3.LdAccessSingleLine(addr);
      addr = (addr & notLineMask) + lineSize;
    } while (addr < highAddr);
}

LOCALFUN VOID CacheStore(ADDRINT addr, UINT32 size)
{
  ADDRINT highAddr;
  highAddr = addr + size;
  do{
    ul3.StAccessSingleLine(addr);
    addr = (addr & notLineMask) + lineSize;
  } while (addr < highAddr);
}

LOCALFUN VOID CacheLoadSingle(ADDRINT addr)
{
  LdAccessSingleLine(addr);
}

LOCALFUN VOID CacheStoreSingle(ADDRINT addr)
{
  StAccessSingleLine(addr);
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
      if(size <= lineSize){
	INS_InsertPredicatedCall(
			       ins, IPOINT_BEFORE, (AFUNPTR)CacheLoadSingle,
			       IARG_MEMORYREAD_EA,
			       IARG_END);
      }
      else{
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
	if(size <= lineSize){
	  INS_InsertPredicatedCall(
				 ins, IPOINT_BEFORE, (AFUNPTR)CacheStoreSingle,
				 IARG_MEMORYWRITE_EA,
				 IARG_END);
	}
	else{
	  INS_InsertPredicatedCall(
				 ins, IPOINT_BEFORE, (AFUNPTR)CacheStore,
				 IARG_MEMORYWRITE_EA,
				 IARG_MEMORYWRITE_SIZE,
				 IARG_END);
	}
      }
}

GLOBALFUN int main(int argc, char *argv[])
{
  start = clock() ;

  PIN_Init(argc, argv);
	
  initCache("L3 Unified Cache", UL3::cacheSize, UL3::lineSize); 
  lineBytes = new UINT8[ul3.LineSize()];
  fill_hamming_lut();

  INS_AddInstrumentFunction(Instruction, 0);
  PIN_AddFiniFunction(Fini, 0);

  // Never returns
  PIN_StartProgram();

  return 0; // make compiler happy
}
