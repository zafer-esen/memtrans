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
 *  This file contains a configurable cache class
 */

#ifndef MEMTRANS_CACHE_MULTI_H
#define MEMTRANS_CACHE_MULTI_H

#include <list>
#include <iterator>
#include <bitset>

enum ACCESS_TYPE {LOAD_ACCESS=0, STORE_ACCESS};
UINT64 LLCMissCount[2] = {0, 0};
UINT64 LLCHitCount[2] = {0, 0};
UINT64 LLCEvictCount = 0;
UINT8 * lineBytes;
UINT8 hamming_lut[256][256];
UINT64 totalTransitions = 0;
UINT64 counts[256] = { 0 }; //number of times every byte value appears in transfers
UINT64 transition_counts_tw[256][256] = { 0 }; //counts the transitioning byte values (transfer-wise)
UINT64 transition_counts_bw[256][256] = { 0 }; //counts the transitioning byte values (bus-wise)
UINT64 consecutive_zero_counts_bw[7] = { 0 };
UINT64 consecutive_zero_counts_tw[7] = { 0 };
UINT64 reuse_counts[256] = { 0 }; //for each byte value, increments the count 
  //for that byte if it was reused after being brought in to the cache

/*
  -----> transfer-wise		
  t0	t1	t2	...	^ bus-wise
  b00	b01	b02	...	|
  b10	b11	b12	...	|
  ...	...	...	...	|
*/

//assume: busWidth * N = len
UINT64 countTransitionsCalled = 0;
UINT8 zero_count_tw[8] = { 0 };
static inline UINT32 countTransitions(UINT8* startAddr, UINT32 len, UINT8 busWidth)
{
  UINT32 count = 0;
  UINT8 zero_count_bw = 0;
  bool end_zero_count_tw;

  UINT8* curWord;
  UINT8 b0, b1;
  UINT32 num_words = len/busWidth;
  curWord = startAddr;
  for (UINT32 i=0; i<num_words; ++i){
    for (UINT32 j=0; j<busWidth; ++j){
      
      b0 = *(curWord + j);
            
      zero_count_bw += (b0 == 0);
      if ((b0 != 0) || (j == (busWidth - 1u))) {
	if (zero_count_bw > 1)
	  consecutive_zero_counts_bw[zero_count_bw - 2]++;
	zero_count_bw = 0;
      }
	  
      if(j > 0)
	transition_counts_bw[*(curWord+j-1)][b0]++;
      
      counts[b0]++;
      if (i != (num_words - 1)) {
	b1 = *((curWord + j) + busWidth);
	end_zero_count_tw = b0 | b1;
	zero_count_tw[j] += !end_zero_count_tw;
	if (end_zero_count_tw ) {
	  if (zero_count_tw[j] > 0)
	    consecutive_zero_counts_tw[zero_count_tw[j] - 1]++;
	  zero_count_tw[j] = 0;
	}
	count += hamming_lut[b0][b1];
	transition_counts_tw[b0][b1]++;
      }
      else {
	if (zero_count_tw[j] > 0)
	  consecutive_zero_counts_tw[zero_count_tw[j] - 1]++;
      }
      
    }
    curWord += busWidth;
  }
  countTransitionsCalled++;
  return count;
}

static inline double calcBitEntropy(UINT32 len, UINT8 busWidth)
{
  return (double)totalTransitions / ((len/busWidth-1)*busWidth*8*countTransitionsCalled);
}

#include <string>

#include "pin_util.H"

/*!
 *  @brief Checks if n is a power of 2.
 *  @returns true if n is power of 2
 */
static inline bool IsPower2(UINT32 n)
{
  return ((n & (n - 1)) == 0);
}

/*!
 *  @brief Computes floor(log2(n))
 *  Works by finding position of MSB set.
 *  @returns -1 if n == 0.
 */
static inline INT32 FloorLog2(UINT32 n)
{
  INT32 p = 0;

  if (n == 0) return -1;

  if (n & 0xffff0000) { p += 16; n >>= 16; }
  if (n & 0x0000ff00)	{ p +=  8; n >>=  8; }
  if (n & 0x000000f0) { p +=  4; n >>=  4; }
  if (n & 0x0000000c) { p +=  2; n >>=  2; }
  if (n & 0x00000002) { p +=  1; }

  return p;
}

/*!
 *  @brief Computes floor(log2(n))
 *  Works by finding position of MSB set.
 *  @returns -1 if n == 0.
 */
static inline INT32 CeilLog2(UINT32 n)
{
  return FloorLog2(n - 1) + 1;
}

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

/*!
 *  @brief Cache set direct mapped
 */

typedef struct CACHE_BLOCK
{
  ADDRINT tag;
  ADDRINT addr;
  std::bitset<8> reused; //TODO: change this to by dynamic so it can handle cache lines bigger than 8 bytes
                          //std::vector<bool> ?
  bool dirty;
}CACHE_BLOCK;

UINT32 _associativity = 1; //do not create LRU sets before changing this, if more than 1 associativity required
class LRU
{
public: 
  std::list<CACHE_BLOCK*> cb_list;
  CACHE_BLOCK * cb;
  LRU(){
    CACHE_BLOCK * cb = new CACHE_BLOCK[_associativity];
    for (UINT32 i = 0; i < _associativity; ++i){
      cb[i].tag = 0;
      cb[i].addr = 0;
      cb[i].reused.reset();
      cb[i].dirty = false;
      cb_list.push_back(&cb[i]);
    }
  }
  ~LRU(){ 
    cb_list.clear();
    delete[] cb; }

  /*
    To search for a tag in a set of cache lines (or blocks), we use the doubly linked std::list.
    - If we have a hit, we return true, if not, we return false.
    - If we do not have a hit, we evict the last cache block. 
    Also: 
    - If the evicted cacheline was dirty, we return its address in ADDRINT* evicted,
    otherwise we just throw it away over our shoulder and set ADDRINT* evicted to NULL.
    - We set the value of CACHE_BLOCK* block to point to the newly added cache block.
    - accessType parameter sent in should reflect if this func is called from a load or store instruction
  */
  bool FindReplace(ADDRINT tag, ADDRINT addr, ACCESS_TYPE accessType, ADDRINT* evicted, 
		   UINT32 accessStart, UINT32 accessSize){
    bool cacheHit;
    list<CACHE_BLOCK*>::iterator it, it_next;
    //  std::cout << "list size: " << cb_list.size() << "\n";
    
    it = cb_list.begin();
    for (UINT32 i = 0; i < _associativity;  ++i){
      //std::cout << "checking tag\n";
      cacheHit = (*it)->tag == tag;
      if(cacheHit){ //if it is a hit!
	//		std::cout << "it's a hit\n";
	(*it)->dirty |= (accessType == STORE_ACCESS); //we set the dirty bit if this is a store
	//if ((*it)->dirty && (accessType == STORE_ACCESS))
	//(*it)->dirty = true; //we set the dirty bit if this is a store
	//	std::cout << "checking if in begin\n";

	//set the reused fields for all the accessed bytes in the cache line,
	//so, the cache line utilization can be calculate.
	for(UINT32 j = accessStart; j < (accessStart+accessSize); ++j){
	  (*it)->reused[j] = true;
	}
	//TODO: here it is not known which values are reused, so we need to do a memcpy to get that info :(
	// luckily, we probably will not need to copy the whole cache line (only accessSize bytes)

	if(i != 0){ //if the hit is not at the beginning of the list
	  //	  std::cout << "not at begin!\n";
	  it_next = it; //the following two lines can be removed if C++11+ is used
	  std::advance(it_next,1);
	  cb_list.splice( cb_list.begin(), cb_list, 
			  it, it_next);  //this moves it to the beginning of the list (MRU)
	}
	break;
      }
      ++it;
    }
    if(!cacheHit){ //if we are at this point, that means we are gonna do a cache block replacement!
      //std::cout << "not hit!\n";
      //it should already be pointing to the end of the list, so we go one back
      std::advance(it, -1); //this should now point to the back of the list (LRU)
      *evicted = (*it)->dirty ? (*it)->addr : (UINT32)0;  //if what we are going to throw away is a dirty cache block
      // at this point we can safely overwrite the evicted cache block
      //std::cout << "evicted, now replacing!\n";
      (*it)->tag = tag;
      (*it)->addr = addr; // TODO: evaluate if we really need this...
      (*it)->dirty = (accessType == STORE_ACCESS); //this update ensures that if the new call was a store, we set the block to dirty
      (*it)->reused.reset(); //this resets the cache line utilization bits
      // finally move the cache block to the front, if associativity is more than 1
      //std::cout << "moving to beginning!\n";
      if (_associativity > 1)
	cb_list.splice( cb_list.begin(), cb_list, 
			it, cb_list.end());  //this moves it to the beginning of the list (MRU) 
      //std::cout << "moved to beginning after miss!\n";
      // at this point, we have successfully replaced a cache block and moved it to be the MRU in the list
    }
    return cacheHit;
  }
};
 

LRU * _sets;
UINT32 _lineSize;
// computed params
UINT32 _lineShift;
UINT32 _setIndexMask;
UINT32 _lineMask;
UINT32 _notLineMask;


void initCache(UINT32 cacheSize, UINT32 lineSize, UINT32 max_sets, UINT32 associativity)
{
  _lineSize = lineSize;
  _lineShift = FloorLog2(lineSize);
  _setIndexMask = (cacheSize / lineSize) - 1;
  _notLineMask = ~(((ADDRINT)(lineSize)) - 1);
  _lineMask = (((ADDRINT)(lineSize)) - 1);
  ASSERTX(IsPower2(_lineSize));
  ASSERTX(IsPower2(_setIndexMask + 1));
  _associativity = associativity;
  _sets = new LRU[max_sets];
}

void cleanupCache(void)
{
  delete[] _sets;
}
  
static inline void AccessSingleLine(ADDRINT addr, UINT32 accessSize, ACCESS_TYPE accessType)
{
  ADDRINT tag = addr >> _lineShift;
  ADDRINT lineStart = addr & _notLineMask; //the beginning of the accessed line, for data movement purposes
  ADDRINT accessStart = addr & _lineMask; //the beginning of the accessed data, inside the line (accessSize
  //marks the end of the accessed bytes)
  UINT32 setIndex = tag & _setIndexMask;
  LRU &set = _sets[setIndex];

  ADDRINT evicted_block_addr = 0;
  bool hit = set.FindReplace(tag, lineStart, accessType, &evicted_block_addr, accessStart, accessSize);
  if (!hit){
    if(evicted_block_addr){ //if the evicted block was dirty (i.e. writeback to memory)
      // get statistics from the evicted cache block
      PIN_SafeCopy(lineBytes, (void*)evicted_block_addr, (UINT32)_lineSize);
      totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
      LLCEvictCount++;

      //TODO: here we also need to check the cache line utilization

      /*std::cout << "Load store evict @ index: " << setIndex << "\nValues written: ";
	for (int i = 0; i<_lineSize/4 - 1; ++i)
	std::cout << ((int*)lineBytes)[i] << ", ";
	std::cout << ((int*)lineBytes)[_lineSize/4 - 1] << "\n";*/
    }
    // update the cache to hold the new tag, new addr and set it to valid
    PIN_SafeCopy(lineBytes, (void*)addr, (UINT32)_lineSize);
    totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
    LLCMissCount[accessType]++;
    /*std::cout << "Load LLC miss @ index: " << setIndex << "\nValues read: ";
      for (int i = 0; i<_lineSize / 4 - 1; ++i)
      std::cout << ((int*)lineBytes)[i] << ", ";
      std::cout << ((int*)lineBytes)[_lineSize / 4 - 1] << "\n";*/
  }
  else
    LLCHitCount[accessType]++;  
}

#endif
