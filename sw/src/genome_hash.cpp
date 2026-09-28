#include "genome_hash.hpp"
#include <cstring>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define HAS_X86_CRC32 1
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC push_options
#pragma GCC target("sse4.2")
#include <nmmintrin.h>
#pragma GCC pop_options
#elif defined(_MSC_VER)
#include <intrin.h>
#endif
#endif

const char *get_hash_backend_name() {
#if defined(HAS_X86_CRC32)
    return "Hardware CRC32 (SSE 4.2)";
#else
    return "Software FNV-1a (Portable)";
#endif
}

#if defined(HAS_X86_CRC32) && (defined(__GNUC__) || defined(__clang__))
__attribute__((target("sse4.2")))
#endif
std::uint64_t hash_genome(const std::uint8_t *bytes, std::size_t length) {
#if defined(HAS_X86_CRC32)
    std::uint64_t crc = 0xFFFFFFFFull;
    std::size_t i = 0;
    for (; i + 8 <= length; i += 8) {
        std::uint64_t chunk;
        std::memcpy(&chunk, bytes + i, sizeof(chunk));
        crc = _mm_crc32_u64(crc, chunk);
    }
    for (; i < length; ++i) {
        crc = _mm_crc32_u8(static_cast<std::uint32_t>(crc), bytes[i]);
    }
    return crc == 0 ? 1ull : crc;
#else
    // Fallback: 64-bit FNV-1a
    std::uint64_t h = 14695981039346656037ull;
    for (std::size_t i = 0; i < length; ++i) {
        h ^= static_cast<std::uint64_t>(bytes[i]);
        h *= 1099511628211ull;
    }
    return h == 0 ? 1ull : h;
#endif
}
