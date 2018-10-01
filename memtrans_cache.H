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

#ifndef MEMTRANS_CACHE2_H
#define MEMTRANS_CACHE2_H

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
class DIRECT_MAPPED
{
  public:
    ADDRINT _tag;
    bool _isstore;
    ADDRINT _addr;
    DIRECT_MAPPED() { 
						_isstore = false; 
						_addr = 0;
						_tag = 0;
					}
};

DIRECT_MAPPED * _sets;

UINT32 _lineSize;
// computed params
UINT32 _lineShift;
UINT32 _setIndexMask;

void initCache(UINT32 cacheSize, UINT32 lineSize, UINT32 max_sets)
{
	_lineSize = lineSize;
	_lineShift = FloorLog2(lineSize);
	_setIndexMask = (cacheSize / lineSize) - 1;
	ASSERTX(IsPower2(_lineSize));
     ASSERTX(IsPower2(_setIndexMask + 1));
     _sets = new DIRECT_MAPPED[max_sets];
}

void cleanupCache(void)
{
  delete _sets;
}
   
static inline void LdAccessSingleLine(ADDRINT addr)
{
  ADDRINT tag = addr >> _lineShift;
  UINT32 setIndex = tag & _setIndexMask;

  DIRECT_MAPPED &set = _sets[setIndex];

  bool hit = set._tag == tag;

  if (!hit){
	if(set._isstore){ //if the set was occupied and was containing a store op, we evict the previous cacheline (i.e. memory write)
      // get statistics from set._addr here
      PIN_SafeCopy(lineBytes, (void*)set._addr, (UINT32)_lineSize);
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

    set._addr = addr;
    set._tag = tag;		
    set._isstore = false; //this is not a store operation, so we will not care when it is evicted 
  }
}

static inline void StAccessSingleLine(ADDRINT addr)
{
  ADDRINT tag = addr >> _lineShift;;
  UINT32 setIndex = tag & _setIndexMask;

  DIRECT_MAPPED &set = _sets[setIndex];

  bool hit = set._tag == tag;

  if (!hit){
    if(set._isstore){ //if the set was occupied and was containing a store op, we evict the previous cacheline (i.e. memory write)
      // get statistics from set._addr here
      PIN_SafeCopy(lineBytes, (void*)set._addr, (UINT32)_lineSize);
      totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
      L3EvictCount++;
	  /*std::cout << "Store store evict @ index: " << setIndex << "\nValues written: ";
	  for (int i = 0; i<_lineSize / 4 - 1; ++i)
		  std::cout << ((int*)lineBytes)[i] << ", ";
	  std::cout << ((int*)lineBytes)[_lineSize / 4 - 1] << "\n";*/
    }
    // update the cache to hold the new tag, new addr and set it to valid
    PIN_SafeCopy(lineBytes, (void*)addr, (UINT32)_lineSize);
    totalTransitions += countTransitions((UINT8*)lineBytes, (UINT32)_lineSize, 8); // bus width assumed 8 bytes
    L3MissCount++;

	/*std::cout << "LLC store miss @ index: " << setIndex << "\nValues read: ";
	for (int i = 0; i<_lineSize / 4 - 1; ++i)
		std::cout << ((int*)lineBytes)[i] << ", ";
	std::cout << ((int*)lineBytes)[_lineSize / 4 - 1] << "\n";*/

    set._addr = addr;
    set._tag = tag;		
    set._isstore = true; 
  }
  else{
    set._isstore = true; //this handles store after load operations
  }
}

#endif // memtrans_cache2
