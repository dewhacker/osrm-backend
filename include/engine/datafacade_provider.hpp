#ifndef OSRM_ENGINE_DATAFACADE_PROVIDER_HPP
#define OSRM_ENGINE_DATAFACADE_PROVIDER_HPP

#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"

#include "storage/shared_barriers.hpp"

namespace osrm
{
namespace engine
{

template <typename AlgorithmT> class DataFacadeProvider
{
    using FacadeT = datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>;

  public:
    virtual ~DataFacadeProvider() = default;

    virtual std::shared_ptr<const FacadeT> Get() const = 0;
};

template <typename AlgorithmT> class ImmutableProvider final : public DataFacadeProvider<AlgorithmT>
{
    using FacadeT = datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>;

  public:
    ImmutableProvider(const storage::StorageConfig &config)
        : immutable_data_facade(std::make_shared<FacadeT>(
              std::make_shared<datafacade::ProcessMemoryAllocator>(config)))
    {
    }

    std::shared_ptr<const FacadeT> Get() const override final { return immutable_data_facade; }

  private:
    std::shared_ptr<const FacadeT> immutable_data_facade;
};
}
}

#endif
