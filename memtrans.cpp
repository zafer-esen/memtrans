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

typedef UINT32 CACHE_STATS; // type of cache hit/miss counters

#include "pin_cache.H"

namespace UL3
{
    // 3rd level unified cache: 16 MB, 64 B lines, direct mapped
    const UINT32 cacheSize = 16*MEGA;
    const UINT32 lineSize = 64;
    const UINT32 associativity = 1;
    const CACHE_ALLOC::STORE_ALLOCATION allocation = CACHE_ALLOC::STORE_ALLOCATE;

    const UINT32 max_sets = cacheSize / (lineSize * associativity);

    typedef CACHE_DIRECT_MAPPED(max_sets, allocation) CACHE;
}
LOCALVAR UL3::CACHE ul3("L3 Unified Cache", UL3::cacheSize, UL3::lineSize, UL3::associativity);

UINT32 L3MissCount = 0;
UINT8 * lineBytes;
UINT8 hamming_lut[256][256];
UINT32 totalTransitions = 0;

// finds the hamming distance between two bytes
uint8_t hamming_dist(uint8_t b1, uint8_t b2)
{
  // xor to get the differing bits in a byte
  uint8_t diff = b1 ^ b2;

  // Brian Kernighan's algorithm to count the number of bits in a byte
  uint8_t count = 0;
  while (diff)
    {
      ++count;
      diff &= diff - 1;
    }
  return count;
}

void fill_hamming_lut(void)
{
  for (int i = 0; i<256; ++i)
    for (int j = 0; j < 256; ++j)
      hamming_lut[i][j] = hamming_dist(i, j);
}

clock_t start;
LOCALFUN VOID Fini(int code, VOID * v)
{
  clock_t end = clock() ;
  double elapsed_time = (end-start)/(double)CLOCKS_PER_SEC ;

  std::cerr << ul3;
  std::cout << "L3 miss count: " << L3MissCount << std::endl;
  std::cout << "Total number of bit transitions: " << totalTransitions << "\n\n";
  std::cout << "Elapsed time: " << elapsed_time << std::endl;
  delete lineBytes;
}

//assume: busWidth * N = len
UINT32 countTransitions(UINT8* startAddr, UINT32 len, UINT8 busWidth)
{
  UINT32 count = 0;
  UINT8* b1;
  b1 = startAddr;
  for (UINT32 i=0; i<(len/busWidth-1); ++i){
    for (UINT32 j=0; j<busWidth; ++j)
      count += hamming_lut[*(b1+j)][*((b1+j)+busWidth)];
    b1 += busWidth;
  }
  return count;
}

LOCALFUN VOID CacheAccess(ADDRINT addr, UINT32 size, CACHE_BASE::ACCESS_TYPE accessType)
{
    // last level unified cache
    const ADDRINT lineSize = ul3.LineSize();
    const ADDRINT notLineMask = ~(lineSize - 1);
    ADDRINT highAddr;
    highAddr = addr + size;
    if(size>1)
      std::cout<<"size: "<<size<<"\n"; 
    do{
      if(!ul3.AccessSingleLine(addr, accessType)){
	// cache miss
	// first copy the data for transition count
	/*	PIN_SafeCopy(lineBytes, (void*)addr, (UINT32)lineSize);
	totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)lineSize, 8); // bus width assumed 8 bytes
	L3MissCount++;*/
      }
      addr = (addr & notLineMask) + lineSize;
      } while (addr < highAddr);
}

LOCALFUN VOID InsRef(ADDRINT addr)
{
    const CACHE_BASE::ACCESS_TYPE accessType = CACHE_BASE::ACCESS_TYPE_LOAD;
    CacheAccess(addr, size, accessType);
}
/*
LOCALFUN VOID MemRef(ADDRINT addr, UINT32 size, CACHE_BASE::ACCESS_TYPE accessType)
{
    // second level unified Cache
    CacheAccess(addr, size, accessType);
    }*/

LOCALFUN VOID Instruction(INS ins, VOID *v)
{
    // all instruction fetches access I-cache
    INS_InsertCall(
        ins, IPOINT_BEFORE, (AFUNPTR)InsRef,
        IARG_INST_PTR,
        IARG_END);
    /*
    if (INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins))
    {
      // only predicated-on memory instructions access D-cache
      INS_InsertPredicatedCall(
			       ins, IPOINT_BEFORE, (AFUNPTR)MemRef,
			       IARG_MEMORYREAD_EA,
			       IARG_MEMORYREAD_SIZE,
			       IARG_UINT32, CACHE_BASE::ACCESS_TYPE_LOAD,
			       IARG_END);
    }

    if (INS_IsMemoryWrite(ins) && INS_IsStandardMemop(ins))
      {
        // only predicated-on memory instructions access D-cache
        INS_InsertPredicatedCall(
				 ins, IPOINT_BEFORE, (AFUNPTR)MemRef,
				 IARG_MEMORYWRITE_EA,
				 IARG_MEMORYWRITE_SIZE,
				 IARG_UINT32, CACHE_BASE::ACCESS_TYPE_STORE,
				 IARG_END);
				 }*/
}

GLOBALFUN int main(int argc, char *argv[])
{
  start = clock() ;

  PIN_Init(argc, argv);

  lineBytes = new UINT8[ul3.LineSize()];
  fill_hamming_lut();

  INS_AddInstrumentFunction(Instruction, 0);
  PIN_AddFiniFunction(Fini, 0);

  // Never returns
  PIN_StartProgram();

  return 0; // make compiler happy
}
