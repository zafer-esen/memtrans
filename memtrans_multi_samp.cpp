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
#include <fstream>
#include <cstdint>
#include "pin.H"
#include "instlib.H"
#include <time.h>

//#define INS_SIM

typedef UINT32 CACHE_STATS; // type of cache hit/miss counters

#include "memtrans_cache_multi.H"

#define SAMPLING_RATIO 2

//================================================================================
// Knobs
//================================================================================
ofstream out;
KNOB<string> knob_output(KNOB_MODE_WRITEONCE, "pintool",
			 "o", "memtrans.out", "specify log file name");
KNOB<UINT32> knob_size(KNOB_MODE_WRITEONCE, "pintool",
		       "s", "8388608", "Cache size (bytes)");
KNOB<UINT32> knob_associativity(KNOB_MODE_WRITEONCE, "pintool", 
				"a", "8", "Cache associativity");
KNOB<UINT32> knob_line_size(KNOB_MODE_WRITEONCE, "pintool",
			    "l", "64", "Cache line size");
KNOB<UINT32> knob_sim_inst(KNOB_MODE_WRITEONCE, "pintool",
			   "ic", "1", "Instruction cache simulation (default: off)");

namespace LLC
{
  UINT32 cacheSize;
  UINT32 lineSize;
  ADDRINT notLineMask;
  UINT32 max_sets;
  UINT32 associativity;
}

clock_t start;
LOCALFUN VOID Fini(int code, VOID * v)
{
  clock_t end = clock() ;
  double elapsed_time = (end-start)/(double)CLOCKS_PER_SEC ;
  double bitEntropy = calcBitEntropy(LLC::lineSize, 8);

  out << "Elapsed time: " << elapsed_time << "\n\n";

  out << "Cache size: " << LLC::cacheSize * LLC::associativity << " B\n";
  out << "Associativity: " << LLC::associativity << (LLC::associativity == 1 ? " way\n" : " ways\n");
  out << "Line size: " << LLC::lineSize << " B\n";
  out << "DRAM bus width: 8 B\n"; 
  out << "Instructions cache simulation: " << (knob_sim_inst.Value() == 0 ? "off\n\n" : "on\n\n");

  out << "LLC Load Miss Count: " << LLCMissCount[LOAD_ACCESS]*SAMPLING_RATIO << "\n";
  out << "LLC Load Hit Count: " << LLCHitCount[LOAD_ACCESS]*SAMPLING_RATIO << "\n";
  double loadAccesses =  (double)LLCHitCount[LOAD_ACCESS] + (double)LLCMissCount[LOAD_ACCESS];
  out << "LLC Load Miss Ratio: " << ((double)LLCMissCount[LOAD_ACCESS] / loadAccesses)*100 << "%\n\n";
  out << "LLC Store Miss Count: " << LLCMissCount[STORE_ACCESS]*SAMPLING_RATIO << "\n";
  out << "LLC Store Hit Count: " << LLCHitCount[STORE_ACCESS]*SAMPLING_RATIO << "\n";
  double storeAccesses =  (double)LLCHitCount[STORE_ACCESS] + (double)LLCMissCount[STORE_ACCESS];
  out << "LLC Store Evict Count: " << LLCEvictCount*SAMPLING_RATIO << "\n";
  out << "LLC Store Miss Ratio: " << ((double)LLCMissCount[STORE_ACCESS]) / storeAccesses*100 << "%\n\n";
  double totalMissCount = (double)(LLCMissCount[STORE_ACCESS] + LLCMissCount[LOAD_ACCESS])*SAMPLING_RATIO;
  double totalHitCount = (double)(LLCHitCount[STORE_ACCESS] + LLCHitCount[LOAD_ACCESS])*SAMPLING_RATIO;
  double totalAccesses = totalMissCount + totalHitCount;
  out << "LLC Total Miss Count: " << totalMissCount << "\n";
  out << "LLC Total Hit Count: " << totalHitCount << "\n";
  out << "LLC Total Miss Ratio: " << (totalMissCount / totalAccesses)*100 << "%\n\n";

  out << "Total number of bit transitions: " << totalTransitions*SAMPLING_RATIO << "\n";
  out << "Bit entropy: " << bitEntropy << "\n\n";
  
  out << "Other metrics" << "\n";
  
  // DO NOT MODIFY BELOW CODE OUTPUT STRUCTURE
  out << "Number of bytes with value:" << "\n";
  UINT64 totalBytes = 0;
  for (int i = 0; i < 256; ++i) {
    totalBytes += counts[i]*SAMPLING_RATIO;
    out << i << ": " << counts[i]*SAMPLING_RATIO << "\n";
  }
	
  out << "Number of times every byte is repeated:" << std::endl;
  for (int i = 0; i < 256; ++i)
    out << i << ": " << same_bytes[i]*SAMPLING_RATIO << std::endl;

  for (int i = 0; i < 256; ++i)
    for (int j = 0; j < 256; ++j)
      out << i << "," << j << ": " << transition_counts[i][j]*SAMPLING_RATIO << "\n";
  
  // DO NOT MODIFY ABOVE CODE OUTPUTSTRUCTURE
  
  out << "Total number of bytes transferred: " << totalBytes << "\n\n";
  out << "Elapsed time: " << elapsed_time << "\n";
  
  out.close();
  delete[] lineBytes;
  cleanupCache();
}

bool enableSampling = true;
LOCALFUN VOID CacheLoad(ADDRINT addr, UINT32 size)
{
  if(!enableSampling) return;
  ADDRINT highAddr = addr + size;
  do{
    AccessSingleLine(addr & LLC::notLineMask, LOAD_ACCESS);
    addr = (addr & LLC::notLineMask) + LLC::lineSize;
  } while (addr < highAddr);
}

LOCALFUN VOID CacheStore(ADDRINT addr, UINT32 size)
{
  if(!enableSampling) return;
  ADDRINT highAddr = addr + size;
  do{
    AccessSingleLine(addr & LLC::notLineMask, STORE_ACCESS);
    addr = (addr & LLC::notLineMask) + LLC::lineSize;
  } while (addr < highAddr);
}

int sample_counter = 0;
LOCALFUN VOID CacheLoadSingle(ADDRINT addr)
{
  sample_counter++;
  if(sample_counter >= 100){
    enableSampling = true;
    sample_counter = 0;
  }
  else if(sample_counter == 10){
    enableSampling = false;
  }
  if(!enableSampling) return;
  AccessSingleLine(addr & LLC::notLineMask, LOAD_ACCESS);
}

LOCALFUN VOID CacheStoreSingle(ADDRINT addr)
{
  if(!enableSampling) return;
  AccessSingleLine(addr & LLC::notLineMask, STORE_ACCESS);
}

LOCALFUN VOID Instruction(INS ins, VOID *v)
{
  // all instruction fetches access I-cache
  if(knob_sim_inst == 1)
    INS_InsertCall(
		   ins, IPOINT_BEFORE, (AFUNPTR)CacheLoadSingle,
		   IARG_INST_PTR,
		   IARG_END);
  if (INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins))
    {
      // only predicated-on memory instructions access D-cache
      UINT32 size = INS_MemoryReadSize(ins);
      if(size <= LLC::lineSize){
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
      if(size <= LLC::lineSize){
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

bool initCacheParams(void)
{
  start = clock();
  LLC::associativity = knob_associativity.Value();
  LLC::cacheSize = knob_size.Value()/LLC::associativity;
  LLC::lineSize = knob_line_size.Value();

  if ((LLC::cacheSize % LLC::associativity)){
    std::cout << "Error, cache size must be divisible by associativity! Aborting...\n";
    return false;
  }

  if ( !IsPower2( LLC::lineSize )){
    std::cout << "Error, line size must be a power of 2! Aborting...\n";
    return false;
  }

  if ( !IsPower2(LLC::cacheSize / LLC::lineSize) ){
    std::cout << "Error, (cache size / line size) must be a power of 2! Aborting...\n";
    return false;
  }

  LLC::notLineMask = ~(((ADDRINT)(LLC::lineSize)) - 1);
  LLC::max_sets = LLC::cacheSize / (LLC::lineSize);
  
  lineBytes = new UINT8[LLC::lineSize];
  
  out.open(knob_output.Value().c_str());

  initCache(LLC::cacheSize, LLC::lineSize, LLC::max_sets, LLC::associativity); 
  fill_hamming_lut();

  return true;
}

GLOBALFUN int main(int argc, char *argv[])
{
  PIN_Init(argc, argv);
  PIN_InitSymbols();

  if (!initCacheParams())
    return 1; //exit if could not init cache

  std::cout << "Starting simulation with:\n";
  std::cout << "Cache size: " << LLC::cacheSize*LLC::associativity << " B\n";
  std::cout << "Associativity: " << LLC::associativity << (LLC::associativity == 1 ? " way\n" : " ways\n");
  std::cout << "Line size: " << LLC::lineSize << " B\n";
  std::cout << "Instructions cache simulation: " << (knob_sim_inst.Value() == 0 ? "off\n\n" : "on\n\n");

  INS_AddInstrumentFunction(Instruction, 0);
  PIN_AddFiniFunction(Fini, 0);

  // Never returns
  PIN_StartProgram();

  return 0; // make compiler happy
}
