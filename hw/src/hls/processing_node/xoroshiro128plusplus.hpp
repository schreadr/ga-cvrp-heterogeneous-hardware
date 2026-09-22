#ifndef XOROSHIRO128PLUSPLUS_HPP
#define XOROSHIRO128PLUSPLUS_HPP

#include <cstdint>

/**
 * @file xoroshiro128plusplus.hpp
 * @brief Synthesizable xoroshiro128++ pseudo-random number generator for Vitis HLS.
 *
 * Designed by David Blackman and Sebastiano Vigna (2019).
 * Features:
 * - 128-bit state with period 2^128 - 1.
 * - 0 DSP / 0 hardware multipliers: state transitions use only bit-rotates (wire renaming) and XORs.
 * - Output scrambler rotl(s0 + s1, 17) + s0 uses two 64-bit carry-chain additions (~2.2 ns).
 * - Full bit independence across all 64 bits (all bits pass BigCrush and PractRand).
 * - Allows extracting two uncorrelated 32-bit random numbers (lower and upper halves)
 *   from a single 64-bit step in 1 clock cycle.
 */
class xoroshiro128plusplus {
private:
    uint64_t s[2];
    uint64_t last_seed;
    bool initialized;

    static inline uint64_t rotl(const uint64_t x, int k) {
        #pragma HLS INLINE
        return (x << k) | (x >> (64 - k));
    }

public:
    xoroshiro128plusplus() : last_seed(0), initialized(false) {
        #pragma HLS INLINE
        s[0] = 0x123456789ABCDEF0ULL;
        s[1] = 0x9E3779B97F4A7C15ULL;
    }

    explicit xoroshiro128plusplus(uint64_t seed_val) : last_seed(0), initialized(false) {
        #pragma HLS INLINE
        seed(seed_val);
    }

    /// Initialize the generator with seed_val.
    inline void init(uint64_t seed_val) {
        #pragma HLS INLINE
        seed(seed_val);
    }

    void seed(uint64_t seed_val) {
        #pragma HLS INLINE
        // Initialize state via SplitMix64 to guarantee non-zero state
        uint64_t z = seed_val + 0x9E3779B97F4A7C15ULL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        s[0] = z ^ (z >> 31);

        z += 0x9E3779B97F4A7C15ULL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        s[1] = z ^ (z >> 31);

        if (s[0] == 0ULL && s[1] == 0ULL) {
            s[0] = 1ULL;
        }
        last_seed = seed_val;
        initialized = true;
    }

    /// Generate 64 bits of uniform pseudo-random output
    inline uint64_t next() {
        #pragma HLS INLINE
        const uint64_t s0 = s[0];
        uint64_t s1 = s[1];
        const uint64_t result = rotl(s0 + s1, 17) + s0;

        s1 ^= s0;
        s[0] = rotl(s0, 49) ^ s1 ^ (s1 << 21);
        s[1] = rotl(s1, 28);

        return result;
    }

    /// Generate a 32-bit unsigned random integer
    inline uint32_t nextUInt() {
        #pragma HLS INLINE
        return static_cast<uint32_t>(next());
    }

    /// Generate a uniformly distributed 32-bit integer in [0, bound)
    inline uint32_t nextUInt(uint32_t bound) {
        #pragma HLS INLINE
        return bound > 0 ? static_cast<uint32_t>(nextUInt() % bound) : 0;
    }

    /// Extract two independent 32-bit random integers from a single 64-bit step
    inline void nextPair(uint32_t &r1, uint32_t &r2) {
        #pragma HLS INLINE
        uint64_t v = next();
        r1 = static_cast<uint32_t>(v);
        r2 = static_cast<uint32_t>(v >> 32);
    }
};

#endif // XOROSHIRO128PLUSPLUS_HPP

