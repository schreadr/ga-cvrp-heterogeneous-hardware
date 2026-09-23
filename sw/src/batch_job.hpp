/**
 * Descriptor for a batch of parent/offspring individuals dispatched between
 * the host Population Manager and vFPGA workers.
 * Pure host data structure - no Coyote dependency.
 */

#ifndef BATCH_JOB_HPP
#define BATCH_JOB_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

struct EncodedIndividual {
    const std::uint8_t* bytes = nullptr;
    std::uint32_t fitness = 0;
    std::uint64_t hash = 0;

    bool operator<(const EncodedIndividual& other) const {
        if (fitness != other.fitness) {
            return fitness < other.fitness;
        }
        return hash < other.hash;
    }

    bool operator==(const EncodedIndividual& other) const {
        return fitness == other.fitness && hash == other.hash;
    }

    bool operator!=(const EncodedIndividual& other) const {
        return !(*this == other);
    }
};

struct BatchBufferSlot {
    int slot_id = 0;
    int vfid = 0;
    std::size_t num_ind_per_stream = 0;

    // Source buffers (parents)
    std::uint8_t *src_buf_0 = nullptr;
    std::uint8_t *src_buf_1 = nullptr;

    // Destination buffers (offspring)
    std::uint8_t *dst_buf_0 = nullptr;
    std::uint8_t *dst_buf_1 = nullptr;
};

struct BatchJob {
    uint32_t batch_id = 0;
    int vfid = 0;
    int slot_id = 0;
    std::size_t num_ind_per_stream = 0;

    std::uint8_t *src_buf_0 = nullptr;
    std::uint8_t *src_buf_1 = nullptr;
    std::uint8_t *dst_buf_0 = nullptr;
    std::uint8_t *dst_buf_1 = nullptr;

    uint32_t batch_seed = 0;

    // Direct parent storage per batch
    std::vector<EncodedIndividual> parents1;
    std::vector<EncodedIndividual> parents2;

    bool operator>(const BatchJob &other) const {
        return batch_id > other.batch_id;
    }

    bool operator<(const BatchJob &other) const {
        return batch_id < other.batch_id;
    }
};

#endif // BATCH_JOB_HPP
