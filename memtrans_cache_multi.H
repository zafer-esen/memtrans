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

UINT32 L3MissCount = 0;
UINT32 L3EvictCount = 0;
UINT8 * lineBytes;
UINT8 hamming_lut[256][256];
UINT64 totalTransitions = 0;
UINT64 counts[256] = { 0 }; //number of times every byte value appears in transfers
UINT64 same_bytes[256] = { 0 }; //number of times a byte value repeats in transfers (bus-wise)
UINT64 transition_counts[256][256]; //counts the transitioning byte values (transfer-wise)
/*
  -----> transfer-wise		
  t0	t1	t2	...	^ bus-wise
  b00	b01	b02	...	|
  b10	b11	b12	...	|
  ...	...	...	...	|
*/

//assume: busWidth * N = len
UINT64 countTransitionsCalled = 0;
static inline UINT32 countTransitions(UINT8* startAddr, UINT32 len, UINT8 busWidth)
{
  UINT32 count = 0;
  UINT8* curWord;
  UINT8 b0, b1;
  UINT32 num_words = len/busWidth-1;
  curWord = startAddr;
  for (UINT32 i=0; i<num_words; ++i){
    for (UINT32 j=0; j<busWidth; ++j){
      count += hamming_lut[*(curWord+j)][*((curWord+j)+busWidth)];
      b0 = *(curWord + j);
      b1 = *((curWord + j) + busWidth);
      counts[b0]++;
      transition_counts[b0][b1]++;
      same_bytes[b0] += (j>0) &&  (b0 == *((curWord + j - 1)));
      if (i == num_words - 1) {
	counts[b1]++;
	same_bytes[b1] += (j>0) && (b1 == *((curWord + j - 1) + busWidth));
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

typedef enum
  {
    ACCESS_TYPE_LOAD,
    ACCESS_TYPE_STORE,
    ACCESS_TYPE_NUM
  } ACCESS_TYPE;

/*!
 *  @brief Cache set direct mapped
 */

typedef struct CACHE_BLOCK
{
  ADDRINT tag;
  ADDRINT addr;
  bool isStore;
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
      cb[i].isStore = false;
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
    - isStore parameter sent in should reflect if this func is called from a load or store instruction
  */
  bool FindReplace(ADDRINT tag, ADDRINT addr, bool isStore, ADDRINT* evicted){
    bool cacheHit = false;
    list<CACHE_BLOCK*>::iterator it, it_next;
    //    std::cout << "list size: " << cb_list.size() << "\n";
    for (it = cb_list.begin(); it != cb_list.end(); ++it){
      //std::cout << "checking tag\n";
      if((*it)->tag == tag){ //if it is a hit!
	//	std::cout << "it's a hit\n";
	cacheHit = true; //return value set to true
	(*it)->isStore = isStore; //this update ensures that if the new call was a store, we set the block to dirty
	//	std::cout << "checking if in begin\n";
	if(it != cb_list.begin()){ //if the hit is not at the beginning of the list
	  //	  std::cout << "not at begin!\n";
	  it_next = it; //the following two lines can be removed if C++11+ is used
	  std::advance(it_next,1);
	  cb_list.splice( cb_list.begin(), cb_list, 
			  it, it_next);  //this moves it to the beginning of the list (MRU)
	}
	break;
      }
    }
    if(!cacheHit){ //if we are at this point, that means we are gonna do a cache block replacement!
      //std::cout << "not hit!\n";
      //it should already be pointing to the end of the list, so we go one back
      std::advance(it, -1); //this should now point to the back of the list (LRU)
      *evicted = (*it)->isStore ? (*it)->addr : (UINT32)0;  //if what we are going to throw away is a dirty cache block
      // at this point we can safely overwrite the evicted cache block
      //std::cout << "evicted, now replacing!\n";
      (*it)->tag = tag;
      (*it)->addr = addr; // TODO: evaluate if we really need this...
      (*it)->isStore = isStore; //this update ensures that if the new call was a store, we set the block to dirty
      // finally move the cache block to the front
      //std::cout << "moving to beginning!\n";
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


void initCache(UINT32 cacheSize, UINT32 lineSize, UINT32 max_sets, UINT32 associativity)
{
  _lineSize = lineSize;
  _lineShift = FloorLog2(lineSize);
  _setIndexMask = (cacheSize / lineSize) - 1;
  ASSERTX(IsPower2(_lineSize));
  ASSERTX(IsPower2(_setIndexMask + 1));
  _associativity = associativity;
  _sets = new LRU[max_sets];
}

void cleanupCache(void)
{
  delete[] _sets;
}
   
static inline void LdAccessSingleLine(ADDRINT addr)
{
  ADDRINT tag = addr >> _lineShift;
  UINT32 setIndex = tag & _setIndexMask;
  LRU &set = _sets[setIndex];

  ADDRINT evicted_block_addr = 0;
  bool hit = set.FindReplace(tag, addr, false, &evicted_block_addr);
  if (!hit){
    if(evicted_block_addr){ //if the evicted block was dirty (i.e. writeback to memory)
      // get statistics from the evicted cache block
      PIN_SafeCopy(lineBytes, (void*)evicted_block_addr, (UINT32)_lineSize);
      totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
      L3EvictCount++;
      /*std::cout << "Load store evict @ index: " << setIndex << "\nValues written: ";
	for (int i = 0; i<_lineSize/4 - 1; ++i)
	std::cout << ((int*)lineBytes)[i] << ", ";
	std::cout << ((int*)lineBytes)[_lineSize/4 - 1] << "\n";*/

    }
    // update the cache to hold the new tag, new addr and set it to valid
    PIN_SafeCopy(lineBytes, (void*)addr, (UINT32)_lineSize);
    totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
    L3MissCount++;
    /*std::cout << "Load LLC miss @ index: " << setIndex << "\nValues read: ";
      for (int i = 0; i<_lineSize / 4 - 1; ++i)
      std::cout << ((int*)lineBytes)[i] << ", ";
      std::cout << ((int*)lineBytes)[_lineSize / 4 - 1] << "\n";*/
  }
}

static inline void StAccessSingleLine(ADDRINT addr)
{
  ADDRINT tag = addr >> _lineShift;;
  UINT32 setIndex = tag & _setIndexMask;

  LRU &set = _sets[setIndex];

  ADDRINT evicted_block_addr = 0;
  bool hit = set.FindReplace(tag, addr, true, &evicted_block_addr);
  if (!hit){
    if(evicted_block_addr){ //if the evicted block was dirty (i.e. writeback to memory)
      // get statistics from the evicted cache block
      PIN_SafeCopy(lineBytes, (void*)evicted_block_addr, (UINT32)_lineSize);
      totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
      L3EvictCount++;
      /*std::cout << "Store store evict @ index: " << setIndex << "\nValues written: ";
	for (int i = 0; i<_lineSize/4 - 1; ++i)
	std::cout << ((int*)lineBytes)[i] << ", ";
	std::cout << ((int*)lineBytes)[_lineSize/4 - 1] << "\n";*/
    }
    // update the cache to hold the new tag, new addr and set it to valid
    PIN_SafeCopy(lineBytes, (void*)addr, (UINT32)_lineSize);
    totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
    L3MissCount++;
    /*std::cout << "Store LLC miss @ index: " << setIndex << "\nValues read: ";
      for (int i = 0; i<_lineSize / 4 - 1; ++i)
      std::cout << ((int*)lineBytes)[i] << ", ";
      std::cout << ((int*)lineBytes)[_lineSize / 4 - 1] << "\n";*/
  }
}

#endif // memtrans_cache2