#ifndef OSRM_UPDATER_UPDATER_HPP
#define OSRM_UPDATER_UPDATER_HPP

#include "updater/updater_config.hpp"

#include "extractor/edge_based_edge.hpp"
#include "extractor/query_node.hpp"

#include <boost/geometry/index/rtree.hpp>

#include <vector>

namespace osrm
{
namespace updater
{
class Timezoner
{
    public:
        Timezoner(std::string tz_filename);
        bool isLocalTimeInRange(FixedCoordinate via_coord, std::string time_range) const;
    private:
        unsigned now;
        //rtree_t timezone_shapes; ???
};

class Updater
{
  public:
      Updater(UpdaterConfig config_) : config(std::move(config_)) {}

      using NumNodesAndEdges = std::tuple<EdgeID, std::vector<extractor::EdgeBasedEdge>>;
      NumNodesAndEdges LoadAndUpdateEdgeExpandedGraph() const;

      EdgeID LoadAndUpdateEdgeExpandedGraph(
          std::vector<extractor::EdgeBasedEdge> &edge_based_edge_list,
          std::vector<EdgeWeight> &node_weights,
          std::vector<extractor::QueryNode> internal_to_external_node_map) const;

  private:
    UpdaterConfig config;
};
}
}

#endif
