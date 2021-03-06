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
#include "pinplay.H"
#include "instlib.H"
#include <time.h>

//#define INS_SIM

typedef UINT32 CACHE_STATS; // type of cache hit/miss counters

#include "memtrans_cache_multi.H"

// should be linked with libpinplay.a, libzlib.a, libbz2.a
PINPLAY_ENGINE pinplay_engine;

#define KNOB_LOG_NAME "log"
#define KNOB_REPLAY_NAME "replay"
#define KNOB_FAMILY "pintool:pinplay-driver"

//================================================================================
// Knobs
//================================================================================
ofstream out;
KNOB<string> knob_output(KNOB_MODE_WRITEONCE, "pintool",
			 "statfile", "memtrans.out", "specify log file name");
KNOB<UINT32> knob_size(KNOB_MODE_WRITEONCE, "pintool",
		       "s", "8388608", "Cache size (bytes)");
KNOB<UINT32> knob_associativity(KNOB_MODE_WRITEONCE, "pintool", 
				"a", "8", "Cache associativity");
KNOB<UINT32> knob_line_size(KNOB_MODE_WRITEONCE, "pintool",
			    "l", "64", "Cache line size");
KNOB<UINT32> knob_sim_inst(KNOB_MODE_WRITEONCE, "pintool",
			   "ic", "1", "Instruction cache simulation (default: off)");

KNOB_COMMENT pinplay_driver_knob_family(KNOB_FAMILY, "PinPlay Driver Knobs");

KNOB<BOOL> knob_replayer(KNOB_MODE_WRITEONCE, KNOB_FAMILY,
			 KNOB_REPLAY_NAME, "0", "Replay a pinball");
KNOB<BOOL> knob_logger(KNOB_MODE_WRITEONCE, KNOB_FAMILY,
		       KNOB_LOG_NAME, "0", "Create a pinball");

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

  out << "LLC Load Miss Count: " << LLCMissCount[LOAD_ACCESS] << "\n";
  out << "LLC Load Hit Count: " << LLCHitCount[LOAD_ACCESS] << "\n";
  double loadAccesses =  (double)LLCHitCount[LOAD_ACCESS] + (double)LLCMissCount[LOAD_ACCESS];
  out << "LLC Load Miss Ratio: " << ((double)LLCMissCount[LOAD_ACCESS] / loadAccesses)*100 << "%\n\n";
  out << "LLC Store Miss Count: " << LLCMissCount[STORE_ACCESS] << "\n";
  out << "LLC Store Hit Count: " << LLCHitCount[STORE_ACCESS] << "\n";
  double storeAccesses =  (double)LLCHitCount[STORE_ACCESS] + (double)LLCMissCount[STORE_ACCESS];
  out << "LLC Store Evict Count: " << LLCEvictCount << "\n";
  out << "LLC Store Miss Ratio: " << ((double)LLCMissCount[STORE_ACCESS]) / storeAccesses*100 << "%\n\n";
  double totalMissCount = (double)(LLCMissCount[STORE_ACCESS] + LLCMissCount[LOAD_ACCESS]);
  double totalHitCount = (double)(LLCHitCount[STORE_ACCESS] + LLCHitCount[LOAD_ACCESS]);
  double totalAccesses = totalMissCount + totalHitCount;
  out << "LLC Total Miss Count: " << totalMissCount << "\n";
  out << "LLC Total Hit Count: " << totalHitCount << "\n";
  out << "LLC Total Miss Ratio: " << (totalMissCount / totalAccesses)*100 << "%\n\n";

  out << "Total number of bit transitions: " << totalTransitions << "\n";
  out << "Bit entropy: " << bitEntropy << "\n";

  double reuse_ratios[256];
  double total_reuse_ratio = 0.0;
  for (int i = 0; i < 256; ++i)
    reuse_ratios[i] = ((double)reuse_counts[i])/((double)counts[i]);
  for (int i = 0; i < 256; ++i){
    total_reuse_ratio += reuse_ratios[i];
  }
  total_reuse_ratio/=256;
  out << "Cache line utilization ratio: " << total_reuse_ratio << "\n\n";
  
  out << "Other metrics" << "\n";

  // DO NOT MODIFY BELOW CODE OUTPUT STRUCTURE
  out << "Sequential 0 counts, bus-wise:\n";
  for (int i = 0; i < 7; ++i)
    out << i + 2 << ": " << consecutive_zero_counts_bw[i] << "\n";
  
  out << "\nSequential 0 counts, transfer-wise:\n";
  for (int i = 0; i < 7; ++i)
    out << i + 2 << ": " << consecutive_zero_counts_tw[i] << "\n";
  
  out << "\nNumber of bytes with value:\n";
  for (int i = 0; i < 256; ++i) {
    out << i << ": " << counts[i] << "\n";
  }

  out << "\nTransition counts, bus-wise:\n";
  for (int i = 0; i < 256; ++i)
    for (int j = 0; j < 256; ++j)
      out << i << "," << j << ": " << transition_counts_bw[i][j] << "\n";

  out << "Transition counts, transfer-wise:\n";
  for (int i = 0; i < 256; ++i)
    for (int j = 0; j < 256; ++j)
      out << i << "," << j << ": " << transition_counts_tw[i][j] << "\n";

  // DO NOT MODIFY ABOVE CODE OUTPUTSTRUCTURE

  // the values inside the array are set even if the value is used only one time
  // after being brought in
  out << "\nReuse counts for values brought in to the cache:\n";
  for (int i = 0; i < 256; ++i) {
    out << i << ": " << reuse_counts[i] << "\n";
  }
  out << "\nReuse ratios for values brought in to the cache:\n";
  for (int i = 0; i < 256; ++i)
    out << i << ": " << reuse_ratios[i] << "\n";
  
  
  out.close();
  delete[] lineBytes;
  cleanupCache();
}

LOCALFUN VOID CacheLoad(ADDRINT addr, UINT32 size)
{
  LLCAccess(addr, size, LOAD_ACCESS);
}

LOCALFUN VOID CacheStore(ADDRINT addr, UINT32 size)
{
  LLCAccess(addr, size, STORE_ACCESS);
}

LOCALFUN VOID Instruction(INS ins, VOID *v)
{
  // TODO: if we are going to use SimPoint/PinPlay, we need to
  // normalize against the number of instructions, so we must
  // also have an instruction count. E.g. MPI: misses/inst.

  // all instruction fetches access I-cache
  if(knob_sim_inst == 1)
    INS_InsertCall(
		   ins, IPOINT_BEFORE, (AFUNPTR)CacheLoad,
		   IARG_INST_PTR,
		   IARG_UINT32, INS_Size(ins),
		   IARG_END);
  if (INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins))
    {
      //TODO: this part can be slightly optimized by adding another
      //analysis function where the memory access covers only a single
      //cache line (this was done before, but was implemented wrong)
      
      // only predicated-on memory instructions access D-cache
      //      UINT32 size = INS_MemoryReadSize(ins);
      INS_InsertPredicatedCall(
			       ins, IPOINT_BEFORE, (AFUNPTR)CacheLoad,
			       IARG_MEMORYREAD_EA,
			       IARG_MEMORYREAD_SIZE,
			       IARG_END);
    }

  if (INS_IsMemoryWrite(ins) && INS_IsStandardMemop(ins))
    {
      //TODO: this part can be slightly optimized by adding another
      //analysis function where the memory access covers only a single
      //cache line (this was done before, but was implemented wrong)

      // only predicated-on memory instructions access D-cache
      //      UINT32 size = INS_MemoryWriteSize(ins);
	INS_InsertPredicatedCall(
				 ins, IPOINT_BEFORE, (AFUNPTR)CacheStore,
				 IARG_MEMORYWRITE_EA,
				 IARG_MEMORYWRITE_SIZE,
				 IARG_END);
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

  pinplay_engine.Activate(argc, argv, knob_logger, knob_replayer);
  
  // Never returns
  PIN_StartProgram();

  return 0; // make compiler happy
}
