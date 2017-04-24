#ifndef OSRM_TIMEZONES_HPP
#define OSRM_TIMEZONES_HPP

#include "util/log.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <chrono>

// Time zone shape polygons loaded in R-tree
// local_time_t is a pair of a time zone shape polygon and the corresponding local time
// rtree_t is a lookup R-tree that maps a geographic point to an index in a local_time_t vector
using point_t = boost::geometry::model::
    point<int32_t, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>>;
using polygon_t = boost::geometry::model::polygon<point_t>;
using box_t = boost::geometry::model::box<point_t>;
using rtree_t =
    boost::geometry::index::rtree<std::pair<box_t, size_t>, boost::geometry::index::rstar<8>>;
using local_time_t = std::pair<polygon_t, struct tm>;

std::function<struct tm(const point_t &)> LoadLocalTimesRTree(const std::string &tz_shapes_filename,
                                                              std::time_t utc_time);
namespace osrm
{
namespace updater
{

inline bool SupportsShapefiles()
{
    #ifdef ENABLE_SHAPEFILE
        return true;
    #else
        return false;
    #endif
}

class Timezoner
{
  public:
    Timezoner() = default;

    #ifdef ENABLE_SHAPEFILE
    Timezoner(std::string tz_filename, std::time_t utc_time_now)
    {
        util::Log() << "Time zone validation based on UTC time : " << utc_time_now;
        GetLocalTime = LoadLocalTimesRTree(tz_filename, utc_time_now);
    }

    Timezoner(std::string tz_filename)
        : Timezoner(tz_filename,
                    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())) {
    }
    #else
        Timezoner(std::string tz_filename, std::time_t utc_time_now)
        {
        }
        Timezoner(std::string tz_filename)
        {
        }
    #endif
    std::function<struct tm(const point_t &)> GetLocalTime;
};
}
}

#endif
