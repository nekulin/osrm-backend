#ifndef OSRM_UPDATER_UPDATER_HPP
#define OSRM_UPDATER_UPDATER_HPP

#include "updater/updater_config.hpp"

#include "extractor/edge_based_edge.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction.hpp"
#include "util/coordinate.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <vector>

namespace osrm
{
namespace updater
{
class Timezoner
{
    public:
        Timezoner(std::string tz_filename)
        {
            // load tz_filename into timezone_shapes member
            // timezone_shapes = ;
            // here's where we'd call LoadLocalTimesRTree();
        };
        using point_t = boost::geometry::model::
            point<double, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>>;
        using polygon_t = boost::geometry::model::polygon<point_t>;
        using box_t = boost::geometry::model::box<point_t>;
        using rtree_t =
            boost::geometry::index::rtree<std::pair<box_t, size_t>, boost::geometry::index::rstar<8>>;
        using local_time_t = std::pair<polygon_t, struct tm>;

        bool isLocalTimeInRange(util::Coordinate via_coord, std::string time_range) const;
    private:
        unsigned now;
        rtree_t timezone_shapes;
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

      bool ValidateTurn(const extractor::InputRestrictionContainer &restriction);

  private:
      UpdaterConfig config;
};
}
}

#endif
