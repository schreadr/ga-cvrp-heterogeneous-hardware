#ifndef GENOME_HASH_HPP
#define GENOME_HASH_HPP

#include <cstddef>
#include <cstdint>

/**
 * Fast genome hasher: computes a 64-bit hash of an individual's tour bytes (excluding fitness).
 * Uses hardware CRC32 (SSE 4.2) when supported by the CPU, with fallback to 64-bit FNV-1a.
 */
std::uint64_t hash_genome(const std::uint8_t *bytes, std::size_t length);

/**
 * Returns a human-readable description of the hashing backend in use.
 */
const char *get_hash_backend_name();

#endif // GENOME_HASH_HPP

