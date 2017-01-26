#include "engine/plugins/trip.hpp"

#include "extractor/tarjan_scc.hpp"

#include "engine/api/trip_api.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/trip/trip_brute_force.hpp"
#include "engine/trip/trip_farthest_insertion.hpp"
#include "engine/trip/trip_nearest_neighbour.hpp"
#include "util/dist_table_wrapper.hpp" // to access the dist table more easily
#include "util/json_container.hpp"
#include "util/matrix_graph_wrapper.hpp" // wrapper to use tarjan scc on dist table

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

// Object to hold all strongly connected components (scc) of a graph
// to access all graphs with component ID i, get the iterators by:
// auto start = std::begin(scc_component.component) + scc_component.range[i];
// auto end = std::begin(scc_component.component) + scc_component.range[i+1];
struct SCC_Component
{
    // in_component: all NodeIDs sorted by component ID
    // in_range: index where a new component starts
    //
    // example: NodeID 0, 1, 2, 4, 5 are in component 0
    //          NodeID 3, 6, 7, 8    are in component 1
    //          => in_component = [0, 1, 2, 4, 5, 3, 6, 7, 8]
    //          => in_range = [0, 5]
    SCC_Component(std::vector<NodeID> in_component_nodes, std::vector<size_t> in_range)
        : component(std::move(in_component_nodes)), range(std::move(in_range))
    {
        BOOST_ASSERT_MSG(component.size() > 0, "there's no scc component");
        BOOST_ASSERT_MSG(*std::max_element(range.begin(), range.end()) == component.size(),
                         "scc component ranges are out of bound");
        BOOST_ASSERT_MSG(*std::min_element(range.begin(), range.end()) == 0,
                         "invalid scc component range");
        BOOST_ASSERT_MSG(std::is_sorted(std::begin(range), std::end(range)),
                         "invalid component ranges");
    }

    std::size_t GetNumberOfComponents() const
    {
        BOOST_ASSERT_MSG(range.size() > 0, "there's no range");
        return range.size() - 1;
    }

    std::vector<NodeID> component;
    std::vector<std::size_t> range;
};

// takes the number of locations and its duration matrix,
// identifies and splits the graph in its strongly connected components (scc)
// and returns an SCC_Component
SCC_Component SplitUnaccessibleLocations(const std::size_t number_of_locations,
                                         const util::DistTableWrapper<EdgeWeight> &result_table)
{

    if (std::find(std::begin(result_table), std::end(result_table), INVALID_EDGE_WEIGHT) ==
        std::end(result_table))
    {
        // whole graph is one scc
        std::vector<NodeID> location_ids(number_of_locations);
        std::iota(std::begin(location_ids), std::end(location_ids), 0);
        std::vector<size_t> range = {0, location_ids.size()};
        return SCC_Component(std::move(location_ids), std::move(range));
    }

    // Run TarjanSCC
    auto wrapper = std::make_shared<util::MatrixGraphWrapper<EdgeWeight>>(result_table.GetTable(),
                                                                          number_of_locations);
    auto scc = extractor::TarjanSCC<util::MatrixGraphWrapper<EdgeWeight>>(wrapper);
    scc.Run();

    const auto number_of_components = scc.GetNumberOfComponents();

    std::vector<std::size_t> range_insertion;
    std::vector<std::size_t> range;
    range_insertion.reserve(number_of_components);
    range.reserve(number_of_components);

    std::vector<NodeID> components(number_of_locations, 0);

    std::size_t prefix = 0;
    for (std::size_t j = 0; j < number_of_components; ++j)
    {
        range_insertion.push_back(prefix);
        range.push_back(prefix);
        prefix += scc.GetComponentSize(j);
    }
    // senitel
    range.push_back(components.size());

    for (std::size_t i = 0; i < number_of_locations; ++i)
    {
        components[range_insertion[scc.GetComponentID(i)]] = i;
        ++range_insertion[scc.GetComponentID(i)];
    }

    return SCC_Component(std::move(components), std::move(range));
}

InternalRouteResult
TripPlugin::ComputeRoute(const std::shared_ptr<const datafacade::BaseDataFacade> facade,
                         const std::vector<PhantomNode> &snapped_phantoms,
                         const std::vector<NodeID> &trip,
                         const bool roundtrip) const
{
    InternalRouteResult min_route;
    // given the final trip, compute total duration and return the route and location permutation
    PhantomNodes viapoint;
    const auto start = std::begin(trip);
    const auto end = std::end(trip);
    // computes a roundtrip from the nodes in trip
    for (auto it = start; it != end; ++it)
    {
        const auto from_node = *it;

        // if from_node is the last node and it is a fixed start and end trip,
        // break out of this loop and return the route
        if (!roundtrip && std::next(it) == end)
            break;

        // if from_node is the last node, compute the route from the last to the first location
        const auto to_node = std::next(it) != end ? *std::next(it) : *start;

        viapoint = PhantomNodes{snapped_phantoms[from_node], snapped_phantoms[to_node]};
        min_route.segment_end_coordinates.emplace_back(viapoint);
    }

    if (roundtrip)
    {
        BOOST_ASSERT(min_route.segment_end_coordinates.size() == trip.size());
    }
    else
    {
        // trip comes out to be something like 0 1 4 3 2, so the sizes don't
        BOOST_ASSERT(min_route.segment_end_coordinates.size() == trip.size() - 1);
    }

    shortest_path(facade, min_route.segment_end_coordinates, {false}, min_route);

    BOOST_ASSERT_MSG(min_route.shortest_path_length < INVALID_EDGE_WEIGHT, "unroutable route");
    return min_route;
}

Status TripPlugin::HandleRequest(const std::shared_ptr<const datafacade::BaseDataFacade> facade,
                                 const api::TripParameters &parameters,
                                 util::json::Object &json_result) const
{
    BOOST_ASSERT(parameters.IsValid());

    // enforce maximum number of locations for performance reasons
    if (max_locations_trip > 0 &&
        static_cast<int>(parameters.coordinates.size()) > max_locations_trip)
    {
        return Error("TooBig", "Too many trip coordinates", json_result);
    }

    if (!CheckAllCoordinates(parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    auto phantom_node_pairs = GetPhantomNodes(*facade, parameters);
    if (phantom_node_pairs.size() != parameters.coordinates.size())
    {
        return Error("NoSegment",
                     std::string("Could not find a matching segment for coordinate ") +
                         std::to_string(phantom_node_pairs.size()),
                     json_result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == parameters.coordinates.size());

    auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

    const auto number_of_locations = snapped_phantoms.size();

    // compute the duration table of all phantom nodes
    const auto result_table = util::DistTableWrapper<EdgeWeight>(
        duration_table(facade, snapped_phantoms, {}, {}), number_of_locations);

    if (result_table.size() == 0)
    {
        return Status::Error;
    }

    const constexpr std::size_t BF_MAX_FEASABLE = 10;
    BOOST_ASSERT_MSG(result_table.size() == number_of_locations * number_of_locations,
                     "Distance Table has wrong size");

    // The following code manipulates the table and produces the new table for
    // Trip with Fixed Start and End (TFSE)

    // Original Table
    //   a  b  c  d  e
    // a 0  15 36 34 30
    // b 15 0  25 30 34
    // c 36 25 0  18 32
    // d 34 30 18 0  15
    // e 30 34 32 15 0

    // New Table
    //   a        b         c        d         e
    // a 0 0      1 15      2 10000  3 34      4 30
    // b 5 10000  6 0       7 25     8 30      9 34
    // c 10 0     11 10000  12 0     13 10000  14 10000
    // d 15 10000 16 30     17 18    18 0      19 15
    // e 20 10000 21 34     22 32    23 15     24 0

    std::vector<EdgeWeight> tfse_table_ = result_table.GetTable();

    if (parameters.source > -1 && parameters.destination > -1)
    {
        for (std::size_t f_counter = 0; f_counter < tfse_table_.size(); f_counter++)
        {
            // parameters.source column
            if (f_counter % result_table.GetNumberOfNodes() == (unsigned long)parameters.source)
            {
                tfse_table_[f_counter] = INVALID_EDGE_WEIGHT;
            }

            // parameters.destination row
            if (f_counter >= (result_table.GetNumberOfNodes() * parameters.destination) &&
                f_counter < result_table.GetNumberOfNodes() * parameters.destination +
                                result_table.GetNumberOfNodes())
            {
                tfse_table_[f_counter] = INVALID_EDGE_WEIGHT;
            }
        }

        tfse_table_[parameters.source * result_table.GetNumberOfNodes() + parameters.source] = 0;
        tfse_table_[parameters.destination * result_table.GetNumberOfNodes() + parameters.source] =
            0;
        tfse_table_[parameters.destination * result_table.GetNumberOfNodes() +
                    parameters.destination] = 0;
        tfse_table_[parameters.source * result_table.GetNumberOfNodes() + parameters.destination] =
            INVALID_EDGE_WEIGHT;
    }

    const auto tfse_table =
        util::DistTableWrapper<EdgeWeight>(tfse_table_, result_table.GetNumberOfNodes());

    // get scc components
    SCC_Component scc = SplitUnaccessibleLocations(result_table.GetNumberOfNodes(), result_table);

    if (parameters.source > -1 && parameters.destination > -1) //check if fized start and end
    {
        std::cout << "scc.component" << std::endl;
        for (auto i : scc.component) {
            std::cout << " " << i;
        }
        std::cout << std::endl;

        std::cout << "scc.range" << std::endl;
        for (auto i : scc.range) {
            std::cout << " " << i;
        }
        std::cout << std::endl;


        // if source and destination are in different sccs then return error
        if (scc.GetNumberOfComponents() > 1)
        {
            std::size_t source_component_id = std::numeric_limits<std::size_t>::max();
            std::size_t destination_component_id = std::numeric_limits<std::size_t>::max();

            for (std::size_t k = 0; k < scc.GetNumberOfComponents(); ++k)
            {
    //          NodeID 0, 1, 2, 4, 5 are in component 0
    //          NodeID 3, 6, 7, 8    are in component 1
    //          => scc.component = [0, 1, 2, 4, 5, 3, 6, 7, 8]
    //          => scc.range = [0, 5]

                auto route_begin = std::begin(scc.component) + scc.range[k];
                auto route_end = std::begin(scc.component) + scc.range[k + 1];

                std::for_each(route_begin, route_end, [&](const NodeID &id) {
                  if (parameters.source == id) source_component_id = k;
                  if (parameters.destination == id) destination_component_id = k;
                });
            }

            if ( destination_component_id == std::numeric_limits<std::size_t>::max() ||
                source_component_id == std::numeric_limits<std::size_t>::max() ||
                source_component_id != destination_component_id)
            {
                return Error("NoTrips",
                             "There's no way to get from source to destination",
                             json_result);
            }
        }
    }

    std::vector<std::vector<NodeID>> trips;
    trips.reserve(scc.GetNumberOfComponents());
    // run Trip computation for every SCC
    for (std::size_t k = 0; k < scc.GetNumberOfComponents(); ++k)
    {
        const auto component_size = scc.range[k + 1] - scc.range[k];

        BOOST_ASSERT_MSG(component_size > 0, "invalid component size");

        std::vector<NodeID> scc_route;
        auto route_begin = std::begin(scc.component) + scc.range[k];
        auto route_end = std::begin(scc.component) + scc.range[k + 1];

        if (component_size > 1)
        {
            if (component_size < BF_MAX_FEASABLE)
            {
                if (parameters.source > -1 && parameters.destination > -1)
                {
                    scc_route = trip::BruteForceTrip(
                        route_begin, route_end, number_of_locations, tfse_table);
                }
                else
                {
                    scc_route = trip::BruteForceTrip(
                        route_begin, route_end, number_of_locations, result_table);
                }
            }
            else
            {
                if (parameters.source > -1 && parameters.destination > -1)
                {
                    scc_route = trip::FarthestInsertionTrip(
                        route_begin, route_end, number_of_locations, tfse_table);
                }
                else
                {
                    scc_route = trip::FarthestInsertionTrip(
                        route_begin, route_end, number_of_locations, result_table);
                }
            }
        }
        else
        {
            scc_route = std::vector<NodeID>(route_begin, route_end);
        }

        trips.push_back(std::move(scc_route));
    }
    if (trips.empty())
    {
        return Error("NoTrips", "Cannot find trips", json_result);
    }

    // compute all round trip routes
    std::vector<InternalRouteResult> routes;
    routes.reserve(trips.size());
    bool roundtrip = true;
    if (parameters.source > -1 && parameters.destination > -1)
    {
        roundtrip = false;
    }
    for (const auto &trip : trips)
    {
        routes.push_back(ComputeRoute(facade, snapped_phantoms, trip, roundtrip));
    }

    api::TripAPI trip_api{*facade, parameters};
    trip_api.MakeResponse(trips, routes, snapped_phantoms, json_result);

    return Status::Ok;
}
}
}
}
