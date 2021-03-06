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
 *  This file contains a configurable cache class
 */

#ifndef PIN_CACHE_H
#define PIN_CACHE_H

#include <string>
#include <list>

#define PROFILE_PIN
#ifdef PROFILE_PIN
#include <sys/time.h>
struct timeval timer_st, timer_ld, timer_stat, timer_copy; 
extern ofstream out;
#define DURATION_FILTER 10

void start_timer(struct timeval* timer)
{
  gettimeofday(timer, NULL);
}
UINT32 stop_timer(struct timeval* timer)
{
  struct timeval ct, dt;
  gettimeofday(&ct, NULL);
  
  dt.tv_sec = ct.tv_sec - timer->tv_sec;
  dt.tv_usec = ct.tv_usec - timer->tv_usec;

  return dt.tv_sec*UINT32(1e6) + dt.tv_usec;
}

#endif

#include "pin_util.H"


/*
  STATISTICS
*/

UINT32 LLCMissCount = 0; //number of times LLC misses
UINT32 LLCEvictCount = 0; //number of times CL are evicted from the LLC
UINT8 hamming_lut[256][256]; //LUT for fast calculation of hamming distances
UINT64 totalTransitions = 0; //number of transfer-wise bit transitions
UINT64 counts[256] = { 0 }; //number of times every byte value appears in transfers
UINT64 same_bytes[256] = { 0 }; //number of times a byte value repeats in transfers (bus-wise)
UINT64 transition_counts[256][256]; //counts the transitioning byte values (transfer-wise)

namespace pin_timing {
  clock_t start;
  clock_t end;
}

/*
  -----> transfer-wise		
  t0	t1	t2	...	^ bus-wise
  b00	b01	b02	...	|
  b10	b11	b12	...	|
  ...	...	...	...	|
*/

UINT8 lineBytes[1024]; //holds the byte values during transfer from/to DRAM


//assume: busWidth * N = len

/*!
 *  @brief Collects statistics when given a cache line of data, starting at startAddr. len is CL / busWidth
 *  @returns the number of bit transitions occuring in the cache line transfer-wise
 */
UINT64 countTransitionsCalled = 0;
static inline UINT32 countTransitions(UINT8* startAddr, UINT32 len, UINT8 busWidth)
{
  start_timer(&timer_stat);
  UINT32 count = 0;
  UINT8* curWord = startAddr;
  UINT8 b0, b1;
  UINT32 num_words = len/busWidth-1;
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
  UINT32 duration = stop_timer(&timer_stat);
  if (duration > DURATION_FILTER)
    out << "stat time: " << duration << "\n";
  return count;
}

/*!
 *  @brief Calculates the bit entropy
 *  @returns Returns the bit entropy as a double
 */
static inline double calcBitEntropy(UINT32 len, UINT8 busWidth)
{
  return (double)totalTransitions / ((len/busWidth-1)*busWidth*8*countTransitionsCalled);
}

/*!
 *  @brief Calculates the hamming distance using Brian Kernighan's algorithm
 *  @returns Returns the hamming distance between the two input bytes
 */
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

/*!
 *  @brief Fills the hamming LUT using Brian Kernighan's algorithm
 */
void fill_hamming_lut(void)
{
  for (int i = 0; i<256; ++i)
    for (int j = 0; j < 256; ++j)
      hamming_lut[i][j] = hamming_dist(i, j);
}

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

/*!
 *  @brief Cache tag - self clearing on creation
 */
class CACHE_LINE
{
public:
  ADDRINT tag;
  bool isStore;
  CACHE_LINE(ADDRINT tag_in = 0, bool isStore_in = false) { tag = tag_in; isStore = isStore_in;}
  void clear() { tag = 0; isStore = 0;}
};

/*!
 * Everything related to cache sets
 */
namespace CACHE_SET
{
  bool cacheHit = false;

  /*!
   *  @brief Cache set direct mapped
   */
  class DIRECT_MAPPED
  {
  public:
    CACHE_LINE _cacheline; // it is not good practice to make this public, need to deal with it
    DIRECT_MAPPED(UINT32 associativity = 1) { 
      if (associativity != 1)
	std::cerr << "associativity must be 1 if direct mapped!\n";
      ASSERTX(associativity == 1); } //TODO: deal with these asserts

    VOID SetAssociativity(UINT32 associativity) { 
      if (associativity != 1)
	std::cerr << "cannot set associativity, must be 1 if direct mapped!\n";
      ASSERTX(associativity == 1); } //TODO: deal with these asserts

    inline CACHE_LINE* Find(ADDRINT tag) { 
      cacheHit = (_cacheline.tag == tag) ? true : false;
      return &_cacheline;
    }
    inline VOID Replace(ADDRINT tag, bool isStore) { 
      _cacheline.tag = tag; 
      _cacheline.isStore = isStore; 
    }
    inline VOID Flush() { _cacheline.clear(); }
  };

  /*!
   *  @brief Cache set with round robin replacement
   */

  class ROUND_ROBIN
  {
  private: 
    UINT32 _cachelinesLastIndex;
    UINT32 _nextReplaceIndex;
    UINT32 _associativity;

  public:
    CACHE_LINE* _cachelines;
    ROUND_ROBIN(UINT32 associativity = 8)
      : _cachelinesLastIndex(associativity - 1)
    {
      _cachelines = new CACHE_LINE[associativity];

      //ASSERTX(associativity <= _associativity);
      _nextReplaceIndex = _cachelinesLastIndex;

      for (INT32 index = _cachelinesLastIndex; index >= 0; index--)
        {
	  _cachelines[index] = CACHE_LINE(0);
        }
    }
    ~ROUND_ROBIN() 
    {
      delete[] _cachelines;
    }

    VOID SetAssociativity(UINT32 associativity)
    {
      //ASSERTX(associativity <= _associativity);
      _cachelinesLastIndex = associativity - 1;
      _nextReplaceIndex = _cachelinesLastIndex;
      _associativity = associativity;
    }

    inline CACHE_LINE* Find(ADDRINT tag)
    {
      cacheHit = false;
      INT32 index;
      for (index = _cachelinesLastIndex; index >= 0; index--)
        {
	  if(_cachelines[index].tag == tag ){
	    cacheHit = true;
	    break;
	  }
        }
      return &(_cachelines[index]);
    }

    inline VOID Replace(ADDRINT tag, bool isStore)
    {
      // g++ -O3 too dumb to do CSE on following lines?!
      const UINT32 index = _nextReplaceIndex;

      _cachelines[index].tag = tag;
      _cachelines[index].isStore = isStore;
      // condition typically faster than modulo
      _nextReplaceIndex = (index == 0 ? _cachelinesLastIndex : index - 1);
    }
    VOID Flush()
    {
      for (INT32 index = _cachelinesLastIndex; index >= 0; index--)
	{
	  _cachelines[index] = 0;
	}
      _nextReplaceIndex=_cachelinesLastIndex;
    }
  };

} // namespace CACHE_SET


/*!
 *  @brief Generic cache base class; no allocate specialization, no cache set specialization
 */
class CACHE_BASE
{
public:
  // types, constants
  typedef enum
    {
      ACCESS_TYPE_LOAD,
      ACCESS_TYPE_STORE,
      ACCESS_TYPE_NUM
    } ACCESS_TYPE;
  const std::string _name;
  const UINT32 _cacheSize;
  const UINT32 _lineSize;

protected:
  static const UINT32 HIT_MISS_NUM = 2;
  CACHE_STATS _access[ACCESS_TYPE_NUM][HIT_MISS_NUM];

  // input params

  ADDRINT _notLineMask;
  UINT32 _associativity;
  UINT32 _numberOfFlushes;
  UINT32 _numberOfResets;

  // computed params
  const UINT32 _lineShift;
  const UINT32 _setIndexMask;

  CACHE_STATS SumAccess(bool hit) const
  {
    CACHE_STATS sum = 0;

    for (UINT32 accessType = 0; accessType < ACCESS_TYPE_NUM; accessType++)
	sum += _access[accessType][hit];

    return sum;
  }

public:
  // constructors/destructors
  CACHE_BASE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity);

  CACHE_STATS Hits(ACCESS_TYPE accessType) const { return _access[accessType][true];}
  CACHE_STATS Misses(ACCESS_TYPE accessType) const { return _access[accessType][false];}
  CACHE_STATS Accesses(ACCESS_TYPE accessType) const { return Hits(accessType) + Misses(accessType);}
  CACHE_STATS Hits() const { return SumAccess(true);}
  CACHE_STATS Misses() const { return SumAccess(false);}
  CACHE_STATS Accesses() const { return Hits() + Misses();}

  CACHE_STATS Flushes() const { return _numberOfFlushes;}
  CACHE_STATS Resets() const { return _numberOfResets;}

  VOID IncFlushCounter()
  {
    _numberOfFlushes += 1;
  }

  VOID IncResetCounter()
  {
    _numberOfResets += 1;
  }

  std::ostream & StatsLong(std::ostream & out) const;
};

CACHE_BASE::CACHE_BASE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity)
  : _name(name),
    _cacheSize(cacheSize),
    _lineSize(lineSize),
    _associativity(associativity),
    _lineShift(FloorLog2(lineSize)),
    _setIndexMask((cacheSize / (associativity * lineSize)) - 1)
{
  if (!IsPower2(_lineSize))
    std::cerr << "line size must be a power of 2!\n";
  ASSERTX(IsPower2(_lineSize));
  if (!IsPower2(_setIndexMask + 1))
    std::cerr << "set index mask +1 must be a power of 2!\n";
  ASSERTX(IsPower2(_setIndexMask + 1));

  _notLineMask = ~(lineSize - 1);

  for (UINT32 accessType = 0; accessType < ACCESS_TYPE_NUM; accessType++)
    {
      _access[accessType][false] = 0;
      _access[accessType][true] = 0;
    }
}

/*!
 *  @brief Stats output method
 */
std::ostream & CACHE_BASE::StatsLong(std::ostream & out) const
{
  const UINT32 headerWidth = 19;
  const UINT32 numberWidth = 10;

  out << _name << ":" << std::endl;

  for (UINT32 i = 0; i < ACCESS_TYPE_NUM; i++)
    {
      const ACCESS_TYPE accessType = ACCESS_TYPE(i);

      std::string type(accessType == ACCESS_TYPE_LOAD ? "Load" : "Store");

      out << StringString(type + " Hits:      ", headerWidth)
	  << StringInt(Hits(accessType), numberWidth) << std::endl;
      out << StringString(type + " Misses:    ", headerWidth)
	  << StringInt(Misses(accessType), numberWidth) << std::endl;
      out << StringString(type + " Accesses:  ", headerWidth)
	  << StringInt(Accesses(accessType), numberWidth) << std::endl;
      out << StringString(type + " Miss Rate: ", headerWidth)
	  << StringFlt(100.0 * Misses(accessType) / Accesses(accessType), 2, numberWidth-1) << "%" << std::endl;
      out << std::endl;
    }

  out << StringString("Total Hits:      ", headerWidth, ' ')
      << StringInt(Hits(), numberWidth) << std::endl;
  out << StringString("Total Misses:    ", headerWidth, ' ')
      << StringInt(Misses(), numberWidth) << std::endl;
  out << StringString("Total Accesses:  ", headerWidth, ' ')
      << StringInt(Accesses(), numberWidth) << std::endl;
  out << StringString("Total Miss Rate: ", headerWidth, ' ')
      << StringFlt(100.0 * Misses() / Accesses(), 2, numberWidth-1) << "%" << std::endl;

  out << StringString("Flushes:         ", headerWidth, ' ')
      << StringInt(Flushes(), numberWidth) << std::endl;
  out << StringString("Stat Resets:     ", headerWidth, ' ')
      << StringInt(Resets(), numberWidth) << std::endl;

  //our stats
  pin_timing::end = clock();
  double elapsed_time = (pin_timing::end - pin_timing::start) / (double)CLOCKS_PER_SEC;
  double bitEntropy = calcBitEntropy(_lineSize, 8);
  //std::cerr << ul3;
  out << "L3 miss count: " << LLCMissCount << std::endl;
  out << "L3 store evict count: " << LLCEvictCount << std::endl;
  out << "Total number of bit transitions: " << totalTransitions << "\n";
  out << "Bit entropy: " << bitEntropy << "\n\n";
  out << "Other metrics" << std::endl;

  // DO NOT MODIFY BELOW CODE OUTPUT STRUCTURE
  out << "Number of bytes with value:" << std::endl;
  UINT64 totalBytes = 0;
  for (int i = 0; i < 256; ++i) {
    totalBytes += counts[i];
    out << i << ": " << counts[i] << std::endl;
  }

  out << "Number of times every byte is repeated:" << std::endl;
  for (int i = 0; i < 256; ++i)
    out << i << ": " << same_bytes[i] << std::endl;

  for (int i = 0; i < 256; ++i)
    for (int j = 0; j < 256; ++j)
      out << i << "," << j << ": " << transition_counts[i][j] << std::endl;

  // DO NOT MODIFY ABOVE CODE OUTPUT STRUCTURE

  out << "Total number of bytes transferred: " << totalBytes << std::endl << std::endl;
  out << "Elapsed time: " << elapsed_time << "\n\n";
  return out;
}

/// ostream operator for CACHE_BASE
std::ostream & operator<< (std::ostream & out, const CACHE_BASE & cacheBase)
{
  return cacheBase.StatsLong(out);
}

/*!
 *  @brief Templated cache class with specific cache set allocation policies
 *
 *  All that remains to be done here is allocate and deallocate the right
 *  type of cache sets.
 */
template <class SET>
class CACHE : public CACHE_BASE
{
private:
  SET* _sets;
  UINT32 _num_sets; //TODO: set in constructor

public:
  // constructors/destructors
  CACHE(std::string name, UINT32 cacheSize, UINT32 lineSize, UINT32 associativity)
    : CACHE_BASE(name, cacheSize, lineSize, associativity)
  {
    //ASSERTX(NumSets() <= _num_sets);

    // TODO: check if _num_sets, lineSize, associativity are powers of 2?
    _num_sets = cacheSize / (lineSize * associativity);

    _sets = new SET[_num_sets]; 

    for (UINT32 i = 0; i < _setIndexMask+1; i++) //_setIndexMask+1 is NumSets
	_sets[i].SetAssociativity(associativity);
  }
  ~CACHE()
  {
    delete[] _sets;
  }

  // modifiers
  /// Cache access at addr that does not span cache lines
  void LdAccessSingleLine(ADDRINT addr);
  void StAccessSingleLine(ADDRINT addr);
  void Flush();
  void ResetStats();
};


template <class SET>
inline void CACHE<SET>::LdAccessSingleLine(ADDRINT addr)
{
  start_timer(&timer_ld);
  ADDRINT tag;
  UINT32 setIndex;
  CACHE_LINE* cacheline;
  UINT32 duration;

  addr = addr & _notLineMask; //get the start of the cacheline
  tag = addr >> _lineShift;
  setIndex = tag & _setIndexMask;

  SET & set = _sets[setIndex];

  //cacheline  = set.Find(tag);
  bool hit = (set._cacheline.tag == tag);
  cacheline = &(set._cacheline);

  // on miss, loads always allocate, stores optionally
  if (!hit){
    if(cacheline->isStore){ //if the set was occupied and was containing a store op, we evict the previous cacheline (i.e. memory write)
	// get statistics from set._addr here
        start_timer(&timer_copy);
	PIN_SafeCopy(lineBytes, (void*)(set._cacheline.tag << _lineShift), _lineSize);
	duration = stop_timer(&timer_copy);
	if (duration > DURATION_FILTER)
	  out << "LD copy time 1: " << duration << "\n";
	totalTransitions += countTransitions((UINT8*)lineBytes, _lineSize, 8); // bus width assumed 8 bytes
	LLCEvictCount++;
      }
      start_timer(&timer_copy);
      PIN_SafeCopy(lineBytes, (void*)addr, _lineSize);
      duration = stop_timer(&timer_copy);
      if (duration > DURATION_FILTER)
	out << "LD copy time 2: " << duration << "\n";
      totalTransitions += countTransitions((UINT8*)lineBytes, _lineSize, 8); // bus width assumed 8 bytes
      LLCMissCount++;
      set.Replace(tag, false); 
  }
  //_access[ACCESS_TYPE_LOAD][CACHE_SET::cacheHit]++;
  duration = stop_timer(&timer_ld);
  if (duration > DURATION_FILTER)
    out << "LD time: " << duration << ", Hit: " << hit << "\n";
}

 template <class SET> void CACHE<SET>::StAccessSingleLine(ADDRINT addr)
 {
  ADDRINT tag;
  UINT32 setIndex;
  CACHE_LINE* cacheline;
  start_timer(&timer_st);
  UINT32 duration;

  addr = addr & _notLineMask;
  tag = addr >> _lineShift;
  setIndex = tag & _setIndexMask;  
  SET & set = _sets[setIndex];

  //cacheline  = set.Find(tag);
  bool hit = (set._cacheline.tag == tag);
  cacheline = &(set._cacheline);

  if (!hit){
    if (cacheline->isStore){ //if the set was occupied and was containing a store op, we evict the previous cacheline (i.e. memory write)
      // get statistics from set._addr here
      start_timer(&timer_copy);
      PIN_SafeCopy(lineBytes, (void*)(set._cacheline.tag << _lineShift), _lineSize);
      duration = stop_timer(&timer_copy);
      if (duration > DURATION_FILTER)
	out << "ST copy time 1: " << duration << "\n";
      totalTransitions += countTransitions((UINT8*)lineBytes, _lineSize , 8); // bus width assumed 8 bytes
      LLCEvictCount++;
    }
    start_timer(&timer_copy);
    PIN_SafeCopy(lineBytes, (void*)addr, _lineSize);
    duration = stop_timer(&timer_copy);
    if (duration > DURATION_FILTER)
      out << "ST copy time 2: " << duration << "\n";
    totalTransitions += countTransitions((UINT8*)lineBytes, _lineSize, 8); // bus width assumed 8 bytes
    LLCMissCount++;
    set.Replace(tag, true); 
  }
  else{
    cacheline->isStore = true;  //this handles store after load operations
  }
  //_access[ACCESS_TYPE_STORE][CACHE_SET::cacheHit]++;
  duration = stop_timer(&timer_st);
  if (duration > DURATION_FILTER)
    out << "ST time: " << duration << ", Hit: " << hit << "\n";
}

template <class SET>
void CACHE<SET>::Flush()
{
  for (INT32 index = _setIndexMask+1; index >= 0; index--) { //_setIndexMask+1 is NumSets
    SET & set = _sets[index];
    set.Flush();
  }
  IncFlushCounter();
}

template <class SET>
void CACHE<SET>::ResetStats()
{
  for (UINT32 accessType = 0; accessType < ACCESS_TYPE_NUM; accessType++)
    {
      _access[accessType][false] = 0;
      _access[accessType][true] = 0;
    }
  IncResetCounter();
}


// define shortcuts
#define CACHE_DIRECT_MAPPED CACHE<CACHE_SET::DIRECT_MAPPED>
#define CACHE_ROUND_ROBIN CACHE<CACHE_SET::ROUND_ROBIN>

#endif // PIN_CACHE_H
