#ifndef STATIC_RTREE_HPP
#define STATIC_RTREE_HPP

#include "storage/io.hpp"
#include "util/bearing.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/deallocating_vector.hpp"
#include "util/exception.hpp"
#include "util/hilbert_value.hpp"
#include "util/integer_range.hpp"
#include "util/rectangle.hpp"
#include "util/typedefs.hpp"
#include "util/vector_view.hpp"
#include "util/web_mercator.hpp"

#include "osrm/coordinate.hpp"

#include "storage/shared_memory_ownership.hpp"

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

// An extended alignment is implementation-defined, so use compiler attributes
// until alignas(LEAF_PAGE_SIZE) is compiler-independent.
#if defined(_MSC_VER)
#define ALIGNED(x) __declspec(align(x))
#elif defined(__GNUC__)
#define ALIGNED(x) __attribute__((aligned(x)))
#else
#define ALIGNED(x)
#endif

namespace osrm
{
namespace util
{

// Static RTree for serving nearest neighbour queries
// All coordinates are pojected first to Web Mercator before the bounding boxes
// are computed, this means the internal distance metric doesn not represent meters!
template <class EdgeDataT,
          storage::Ownership Ownership = storage::Ownership::Container,
          std::uint32_t BRANCHING_FACTOR = 128,
          std::uint32_t LEAF_PAGE_SIZE = 4096>
class StaticRTree
{
    template <typename T> using Vector = ViewOrVector<T, Ownership>;

  public:
    using Rectangle = RectangleInt2D;
    using EdgeData = EdgeDataT;
    using CoordinateList = Vector<util::Coordinate>;

    static_assert(LEAF_PAGE_SIZE >= sizeof(uint32_t) + sizeof(EdgeDataT), "page size is too small");
    static_assert(((LEAF_PAGE_SIZE - 1) & LEAF_PAGE_SIZE) == 0, "page size is not a power of 2");
    static constexpr std::uint32_t LEAF_NODE_SIZE =
        (LEAF_PAGE_SIZE - sizeof(uint32_t) - sizeof(Rectangle)) / sizeof(EdgeDataT);

    struct CandidateSegment
    {
        Coordinate fixed_projected_coordinate;
        EdgeDataT data;
    };

    struct TreeIndex
    {
        TreeIndex() : index(0), is_leaf(false) {}
        TreeIndex(std::size_t index, bool is_leaf) : index(index), is_leaf(is_leaf) {}
        std::uint32_t index : 31;
        std::uint32_t is_leaf : 1;
    };

    struct TreeNode
    {
        TreeNode() : child_count(0) {}
        std::uint32_t child_count;
        Rectangle minimum_bounding_rectangle;
        TreeIndex children[BRANCHING_FACTOR];
    };

    struct ALIGNED(LEAF_PAGE_SIZE) LeafNode
    {
        LeafNode() : object_count(0), objects() {}
        std::uint32_t object_count;
        Rectangle minimum_bounding_rectangle;
        std::array<EdgeDataT, LEAF_NODE_SIZE> objects;
    };
    static_assert(sizeof(LeafNode) == LEAF_PAGE_SIZE, "LeafNode size does not fit the page size");

  private:
    struct HilbertInputElement
    {
        explicit HilbertInputElement(const uint64_t _hilbert_value,
                                     const std::uint32_t _array_index)
            : m_hilbert_value(_hilbert_value), m_array_index(_array_index)
        {
        }

        HilbertInputElement() : m_hilbert_value(0), m_array_index(UINT_MAX) {}

        uint64_t m_hilbert_value;
        std::uint32_t m_array_index;

        inline bool operator<(const HilbertInputElement &other) const
        {
            return m_hilbert_value < other.m_hilbert_value;
        }
    };

    struct QueryCandidate
    {
        QueryCandidate(std::uint64_t squared_min_dist, TreeIndex tree_index)
            : squared_min_dist(squared_min_dist), tree_index(tree_index),
              segment_index(std::numeric_limits<std::uint32_t>::max())
        {
        }

        QueryCandidate(std::uint64_t squared_min_dist,
                       TreeIndex tree_index,
                       std::uint32_t segment_index,
                       const Coordinate &coordinate)
            : squared_min_dist(squared_min_dist), tree_index(tree_index),
              segment_index(segment_index), fixed_projected_coordinate(coordinate)
        {
        }

        inline bool is_segment() const
        {
            return segment_index != std::numeric_limits<std::uint32_t>::max();
        }

        inline bool operator<(const QueryCandidate &other) const
        {
            // Attn: this is reversed order. std::pq is a max pq!
            return other.squared_min_dist < squared_min_dist;
        }

        std::uint64_t squared_min_dist;
        TreeIndex tree_index;
        std::uint32_t segment_index;
        Coordinate fixed_projected_coordinate;
    };

    Vector<TreeNode> m_search_tree;
    const Vector<Coordinate> &m_coordinate_list;

    boost::iostreams::mapped_file_source m_leaves_region;
    // read-only view of leaves
    util::vector_view<const LeafNode> m_leaves;

  public:
    StaticRTree(const StaticRTree &) = delete;
    StaticRTree &operator=(const StaticRTree &) = delete;

    enum PackingMethod
    {
        Hilbert,
        STR,
        OMT
    };

    explicit StaticRTree(const std::vector<EdgeDataT> &input_data_vector,
                         const std::string &tree_node_filename,
                         const std::string &leaf_node_filename,
                         const Vector<Coordinate> &coordinate_list,
                         const PackingMethod packing_method = OMT)
        : m_coordinate_list(coordinate_list)
    {
        util::Log() << "Starting rtree";
        switch (packing_method)
        {
        case Hilbert:
            PackWithHilbert(input_data_vector, tree_node_filename, leaf_node_filename);
            break;
        case STR:
            PackWithSTR(input_data_vector, tree_node_filename, leaf_node_filename);
            break;
        case OMT:
            PackWithOMT(input_data_vector, tree_node_filename, leaf_node_filename);
            break;
        default:
            PackWithHilbert(input_data_vector, tree_node_filename, leaf_node_filename);
        }
    }

    // In-place partial sort-by-group
    template <typename Iterator, typename Compare>
    void grouped_partial_sort(Iterator left, Iterator right, std::size_t n, Compare compare)
    {
        std::stack<Iterator> stack;
        stack.push(left);
        stack.push(right);
        Iterator mid;

        while (!stack.empty())
        {
            right = stack.top();
            stack.pop();
            left = stack.top();
            stack.pop();

            if (std::distance(left, right) <= n)
                continue;

            // Important: note the use of static_cast<double>" here - we need this to be floating
            // point math, because we depend on the "ceil" behiaviour rounding up in some
            // circumstances.  If we left it as integer math, we'll sometimes round down, which
            // will put us into an infinite loop.  TODO: could this be achieved with integer math
            // and a +1 ?
            mid = left + std::ceil(static_cast<double>(std::distance(left, right)) / n / 2.) * n;

            std::partial_sort(left, mid, right, compare);

            stack.push(left);
            stack.push(mid);
            stack.push(mid);
            stack.push(right);
        }
    }

    // Constructs a packed RTree with the Lee-Lee OMT approach
    // This should minimize leaf-node overlap, which works well for the typical
    // layout of road network geometries
    void PackWithOMT(const std::vector<EdgeDataT> &input_data_vector,
                     const std::string &tree_node_filename,
                     const std::string &leaf_node_filename)
    {
        util::Log() << "Packing with OMT";
        auto leaves = const_cast<std::vector<EdgeDataT> &>(input_data_vector);

        struct Range
        {
            Range(std::size_t parent_, std::size_t left_, std::size_t right_, std::size_t height_)
                : parent{parent_}, left{left_}, right{right_}, height{height_}
            {
            }
            std::size_t parent;
            std::size_t left;
            std::size_t right;
            std::size_t height;
        };

        // We use a queue here so that we can do a breadth-first
        std::queue<Range> queue;
        queue.emplace(0, 0, leaves.size(), 0);

        // TODO: we do a lot of sorting - it would make sense to only calculate centroids once
        auto longitude_compare = [this](const EdgeDataT &a, const EdgeDataT &b) {
            auto a_centroid =
                coordinate_calculation::centroid(m_coordinate_list[a.u], m_coordinate_list[a.v]);
            auto b_centroid =
                coordinate_calculation::centroid(m_coordinate_list[b.u], m_coordinate_list[b.v]);
            return a_centroid.lon < b_centroid.lon;
        };

        auto latitude_compare = [this](const EdgeDataT &a, const EdgeDataT &b) {
            auto a_centroid =
                coordinate_calculation::centroid(m_coordinate_list[a.u], m_coordinate_list[a.v]);
            auto b_centroid =
                coordinate_calculation::centroid(m_coordinate_list[b.u], m_coordinate_list[b.v]);
            return a_centroid.lat < b_centroid.lat;
        };

        util::Log() << "LEAF_NODE_SIZE " << LEAF_NODE_SIZE;

        boost::filesystem::ofstream leaf_node_file(leaf_node_filename, std::ios::binary);

        // position of the last leaf node written to diskcountindex
        std::size_t leaf_node_count = 0;

        while (!queue.empty())
        {
            auto r = queue.front();
            queue.pop();

            auto N = r.right - r.left + 1;
            auto M = BRANCHING_FACTOR;

            // We're processing a leaf
            if (N <= M)
            {
                LeafNode current_leaf;
                current_leaf.object_count = N - 1;
                // This copies the actual EdgeDataT objects into our leaf node struct
                BOOST_ASSERT(N <= LEAF_NODE_SIZE);
                std::copy(leaves.begin() + r.left,
                          leaves.begin() + r.right,
                          current_leaf.objects.begin());
                // Now calculate the bounding-box
                std::for_each(current_leaf.objects.begin(),
                              current_leaf.objects.begin() + current_leaf.object_count,
                              [this, &current_leaf](const EdgeDataT &edge) {
                                  Coordinate projected_u{web_mercator::fromWGS84(
                                      Coordinate{m_coordinate_list[edge.u]})};
                                  Coordinate projected_v{web_mercator::fromWGS84(
                                      Coordinate{m_coordinate_list[edge.v]})};
                                  current_leaf.minimum_bounding_rectangle.Extend(projected_u.lon,
                                                                                 projected_u.lat);
                                  current_leaf.minimum_bounding_rectangle.Extend(projected_v.lon,
                                                                                 projected_v.lat);
                              });

                leaf_node_file.write((char *)&current_leaf, sizeof(current_leaf));

                std::cout << "{ "
                             "\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":"
                             "\"Polygon\",\"coordinates\":[[";
                std::cout << "[" << toFloating(current_leaf.minimum_bounding_rectangle.min_lon)
                          << "," << toFloating(current_leaf.minimum_bounding_rectangle.min_lat)
                          << "],";
                std::cout << "[" << toFloating(current_leaf.minimum_bounding_rectangle.min_lon)
                          << "," << toFloating(current_leaf.minimum_bounding_rectangle.max_lat)
                          << "],";
                std::cout << "[" << toFloating(current_leaf.minimum_bounding_rectangle.max_lon)
                          << "," << toFloating(current_leaf.minimum_bounding_rectangle.max_lat)
                          << "],";
                std::cout << "[" << toFloating(current_leaf.minimum_bounding_rectangle.max_lon)
                          << "," << toFloating(current_leaf.minimum_bounding_rectangle.min_lat)
                          << "],";
                std::cout << "[" << toFloating(current_leaf.minimum_bounding_rectangle.min_lon)
                          << "," << toFloating(current_leaf.minimum_bounding_rectangle.min_lat)
                          << "]";
                std::cout << "]]}}," << std::endl;

                BOOST_ASSERT(m_search_tree[r.parent].child_count <= BRANCHING_FACTOR);
                m_search_tree[r.parent].children[m_search_tree[r.parent].child_count].index =
                    leaf_node_count;
                m_search_tree[r.parent].children[m_search_tree[r.parent].child_count].is_leaf =
                    true;
                // Grow the parent node's bounding box'
                m_search_tree[r.parent].minimum_bounding_rectangle.Extend(
                    current_leaf.minimum_bounding_rectangle);
                ++leaf_node_count;
                ++m_search_tree[r.parent].child_count;
                continue;
            }

            TreeNode current_node;
            m_search_tree.push_back(current_node);

            // Special case for the first item (height = 0),
            if (r.height == 0)
            {
                r.height = std::ceil(std::log(N) / std::log(M));
                M = std::ceil(N / std::pow(M, r.height - 1));
            }
            else
            {
                BOOST_ASSERT(m_search_tree[r.parent].child_count < BRANCHING_FACTOR);
                m_search_tree[r.parent].children[m_search_tree[r.parent].child_count].index =
                    m_search_tree.size() - 1;
                m_search_tree[r.parent].children[m_search_tree[r.parent].child_count].is_leaf =
                    false;
                ++m_search_tree[r.parent].child_count;
            }

            std::size_t N2 = std::ceil(N / M);
            std::size_t N1 = N2 * std::ceil(std::sqrt(M));

            /*
                        grouped_partial_sort(
                            leaves.begin() + r.left, leaves.begin() + r.right, N,
               longitude_compare);
                            */

            std::sort(leaves.begin() + r.left, leaves.begin() + r.right, longitude_compare);

            // Now, for each column (there are S columns)
            for (auto i = r.left; i < r.right; i += N1)
            {
                auto right2 = std::min(i + N1 - 1, r.right);
                /*
                grouped_partial_sort(
                    leaves.begin() + i, leaves.begin() + right2, N2, latitude_compare);
                    */
                std::sort(leaves.begin() + i, leaves.begin() + right2, latitude_compare);
                for (auto j = i; j <= right2; j += N2)
                {
                    auto right3 = std::min(j + N2 - 1, right2);
                    queue.emplace(m_search_tree.size() - 1, j, right3, r.height - 1);
                }
            }
        }

        // Because we used a queue above, the m_search_tree vector is sorted in
        // the same order as a breadth-first-search of the tree.  We can iterate
        // over this in reverse and propogate node recangle sizes up the tree
        // The leaf nodes already have their bounding box set, so we just need to
        // propogate those up the tree

        std::stack<std::size_t> n;
        for (auto n = m_search_tree.rbegin(); n != m_search_tree.rend(); n++)
        {
            // Skip the bottom of the tree, the sizes are already set
            if (n->child_count == 0 || n->children[0].is_leaf)
            {
                std::cout << "{ "
                             "\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":"
                             "\"Polygon\",\"coordinates\":[[";
                std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                          << toFloating(n->minimum_bounding_rectangle.min_lat) << "],";
                std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                          << toFloating(n->minimum_bounding_rectangle.max_lat) << "],";
                std::cout << "[" << toFloating(n->minimum_bounding_rectangle.max_lon) << ","
                          << toFloating(n->minimum_bounding_rectangle.max_lat) << "],";
                std::cout << "[" << toFloating(n->minimum_bounding_rectangle.max_lon) << ","
                          << toFloating(n->minimum_bounding_rectangle.min_lat) << "],";
                std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                          << toFloating(n->minimum_bounding_rectangle.min_lat) << "]";
                std::cout << "]]}}," << std::endl;
                continue;
            }
            for (auto i = 0u; i < n->child_count; i++)
            {
                n->minimum_bounding_rectangle.Extend(
                    m_search_tree[n->children[i].index].minimum_bounding_rectangle);
            }
            if (n != m_search_tree.rbegin())
                std::cout << "," << std::endl;
            std::cout << "{ "
                         "\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":"
                         "\"Polygon\",\"coordinates\":[[";
            std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                      << toFloating(n->minimum_bounding_rectangle.min_lat) << "],";
            std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                      << toFloating(n->minimum_bounding_rectangle.max_lat) << "],";
            std::cout << "[" << toFloating(n->minimum_bounding_rectangle.max_lon) << ","
                      << toFloating(n->minimum_bounding_rectangle.max_lat) << "],";
            std::cout << "[" << toFloating(n->minimum_bounding_rectangle.max_lon) << ","
                      << toFloating(n->minimum_bounding_rectangle.min_lat) << "],";
            std::cout << "[" << toFloating(n->minimum_bounding_rectangle.min_lon) << ","
                      << toFloating(n->minimum_bounding_rectangle.min_lat) << "]";
            std::cout << "]]}}," << std::endl;
        }
        std::cout << std::endl;
        util::Log() << "There are now " << leaf_node_count << " leaf nodes and "
                    << m_search_tree.size() << " tree nodes";

        // open tree file
        storage::io::FileWriter tree_node_file(tree_node_filename,
                                               storage::io::FileWriter::HasNoFingerprint);

        std::uint64_t size_of_tree = m_search_tree.size();
        BOOST_ASSERT_MSG(0 < size_of_tree, "tree empty");

        tree_node_file.WriteOne(size_of_tree);
        tree_node_file.WriteFrom(&m_search_tree[0], size_of_tree);
    }

    // Construct a packed STR-RTree with the Luetengger-Edgington-Lopez approach
    // STR performs better than Hilbert for road-network-like data where the
    // distribution is only mildly biased
    void PackWithSTR(const std::vector<EdgeDataT> &input_data_vector,
                     const std::string &tree_node_filename,
                     const std::string &leaf_node_filename)
    {
        util::Log() << "Packing with STR";
        auto copy = const_cast<std::vector<EdgeDataT> &>(input_data_vector);
        //        copy.reserve(input_data_vector.size());
        // std::copy(input_data_vector.begin(), input_data_vector.end(), copy);

        // Sort by longitude coordinate of centroid for each segment
        // TODO: this means calculating centroids a lot, this could be moved into
        // a separate single pass
        util::Log() << "Sorting leaves by centroid longitude";
        tbb::parallel_sort(
            copy.begin(), copy.end(), [this](const EdgeDataT &a, const EdgeDataT &b) {
                auto a_centroid = coordinate_calculation::centroid(m_coordinate_list[a.u],
                                                                   m_coordinate_list[a.v]);
                auto b_centroid = coordinate_calculation::centroid(m_coordinate_list[b.u],
                                                                   m_coordinate_list[b.v]);
                return a_centroid.lon < b_centroid.lon;

            });

        // Now, break into even-sized vertical slices, and sort each slice by latitude
        // TODO: make sure this isn't 0
        const std::size_t leaf_K = std::sqrt(copy.size() / LEAF_NODE_SIZE);
        util::Log() << "leaf_K is " << leaf_K;
        tbb::parallel_for(tbb::blocked_range<uint64_t>(0, copy.size(), leaf_K),
                          [&copy, this](const tbb::blocked_range<uint64_t> &range) {
                              util::Log() << "Sorting vertical leaf slice " << range.begin() << "-"
                                          << range.end();
                              tbb::parallel_sort(
                                  copy.begin() + range.begin(),
                                  copy.begin() + range.end(),
                                  [this](const EdgeDataT &a, const EdgeDataT &b) {
                                      auto a_centroid = coordinate_calculation::centroid(
                                          m_coordinate_list[a.u], m_coordinate_list[a.v]);
                                      auto b_centroid = coordinate_calculation::centroid(
                                          m_coordinate_list[b.u], m_coordinate_list[b.v]);
                                      return a_centroid.lat < b_centroid.lat;
                                  });
                          });

        // open leaf file
        boost::filesystem::ofstream leaf_node_file(leaf_node_filename, std::ios::binary);
        std::vector<TreeNode> tree_nodes_in_level;

        // pack M elements into leaf node, write to leaf file and add child index to the parent node
        uint64_t wrapped_element_index = 0;
        for (std::uint32_t node_index = 0; wrapped_element_index < copy.size(); ++node_index)
        {
            TreeNode current_node;
            for (std::uint32_t leaf_index = 0;
                 leaf_index < BRANCHING_FACTOR && wrapped_element_index < copy.size();
                 ++leaf_index)
            {
                LeafNode current_leaf;
                Rectangle &rectangle = current_leaf.minimum_bounding_rectangle;
                for (std::uint32_t object_index = 0;
                     object_index < LEAF_NODE_SIZE && wrapped_element_index < copy.size();
                     ++object_index, ++wrapped_element_index)
                {
                    const auto &object = copy[wrapped_element_index];

                    current_leaf.object_count += 1;
                    current_leaf.objects[object_index] = object;

                    Coordinate projected_u{
                        web_mercator::fromWGS84(Coordinate{m_coordinate_list[object.u]})};
                    Coordinate projected_v{
                        web_mercator::fromWGS84(Coordinate{m_coordinate_list[object.v]})};

                    BOOST_ASSERT(std::abs(toFloating(projected_u.lon).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_u.lat).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_v.lon).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_v.lat).operator double()) <= 180.);

                    rectangle.min_lon =
                        std::min(rectangle.min_lon, std::min(projected_u.lon, projected_v.lon));
                    rectangle.max_lon =
                        std::max(rectangle.max_lon, std::max(projected_u.lon, projected_v.lon));

                    rectangle.min_lat =
                        std::min(rectangle.min_lat, std::min(projected_u.lat, projected_v.lat));
                    rectangle.max_lat =
                        std::max(rectangle.max_lat, std::max(projected_u.lat, projected_v.lat));

                    BOOST_ASSERT(rectangle.IsValid());
                }

                // append the leaf node to the current tree node
                current_node.child_count += 1;
                current_node.children[leaf_index] =
                    TreeIndex{node_index * BRANCHING_FACTOR + leaf_index, true};
                current_node.minimum_bounding_rectangle.MergeBoundingBoxes(
                    current_leaf.minimum_bounding_rectangle);

                // write leaf_node to leaf node file
                leaf_node_file.write((char *)&current_leaf, sizeof(current_leaf));
            }

            tree_nodes_in_level.emplace_back(current_node);
        }
        leaf_node_file.flush();
        leaf_node_file.close();
        util::Log(logINFO) << "There are " << tree_nodes_in_level.size() << " tree nodes now";

        // Now, repeat the STR lon/lat sorting until we reach the root node
        std::uint32_t processing_level = 0;
        while (tree_nodes_in_level.size() > 1)
        {
            // Sort boxes by centroid longitude
            tbb::parallel_sort(tree_nodes_in_level.begin(),
                               tree_nodes_in_level.end(),
                               [](const TreeNode &a, const TreeNode &b) {
                                   return a.minimum_bounding_rectangle.Centroid().lon <
                                          b.minimum_bounding_rectangle.Centroid().lon;
                               });

            // Now, break into even-sized vertical slices, and sort each slice by latitude
            const std::size_t tree_K = std::sqrt(tree_nodes_in_level.size() / BRANCHING_FACTOR);
            tbb::parallel_for(tbb::blocked_range<uint64_t>(0, tree_nodes_in_level.size(), tree_K),
                              [&](const tbb::blocked_range<uint64_t> &range) {
                                  tbb::parallel_sort(
                                      tree_nodes_in_level.begin() + range.begin(),
                                      tree_nodes_in_level.begin() + range.end(),
                                      [this](const TreeNode &a, const TreeNode &b) {
                                          return a.minimum_bounding_rectangle.Centroid().lat <
                                                 b.minimum_bounding_rectangle.Centroid().lat;
                                      });
                              });

            std::vector<TreeNode> tree_nodes_in_next_level;
            std::uint32_t processed_tree_nodes_in_level = 0;
            while (processed_tree_nodes_in_level < tree_nodes_in_level.size())
            {
                TreeNode parent_node;
                // pack BRANCHING_FACTOR elements into tree_nodes each
                for (std::uint32_t current_child_node_index = 0;
                     current_child_node_index < BRANCHING_FACTOR;
                     ++current_child_node_index)
                {
                    if (processed_tree_nodes_in_level < tree_nodes_in_level.size())
                    {
                        TreeNode &current_child_node =
                            tree_nodes_in_level[processed_tree_nodes_in_level];
                        // add tree node to parent entry
                        parent_node.children[current_child_node_index] =
                            TreeIndex{m_search_tree.size(), false};
                        m_search_tree.emplace_back(current_child_node);
                        // merge MBRs
                        parent_node.minimum_bounding_rectangle.MergeBoundingBoxes(
                            current_child_node.minimum_bounding_rectangle);
                        // increase counters
                        ++parent_node.child_count;
                        ++processed_tree_nodes_in_level;
                    }
                }
                tree_nodes_in_next_level.emplace_back(parent_node);
            }
            tree_nodes_in_level.swap(tree_nodes_in_next_level);
            ++processing_level;
        }
        BOOST_ASSERT_MSG(tree_nodes_in_level.size() == 1, "tree broken, more than one root node");
        // last remaining entry is the root node, store it
        m_search_tree.emplace_back(tree_nodes_in_level[0]);
        util::Log() << "Tree is " << processing_level << " deep";

        // reverse and renumber tree to have root at index 0
        std::reverse(m_search_tree.begin(), m_search_tree.end());
        std::uint32_t search_tree_size = m_search_tree.size();
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, search_tree_size),
            [this, &search_tree_size](const tbb::blocked_range<std::uint32_t> &range) {
                for (std::uint32_t i = range.begin(), end = range.end(); i != end; ++i)
                {
                    TreeNode &current_tree_node = this->m_search_tree[i];
                    for (std::uint32_t j = 0; j < current_tree_node.child_count; ++j)
                    {
                        if (!current_tree_node.children[j].is_leaf)
                        {
                            const std::uint32_t old_id = current_tree_node.children[j].index;
                            const std::uint32_t new_id = search_tree_size - old_id - 1;
                            current_tree_node.children[j].index = new_id;
                        }
                    }
                }
            });

        // open tree file
        storage::io::FileWriter tree_node_file(tree_node_filename,
                                               storage::io::FileWriter::HasNoFingerprint);

        std::uint64_t size_of_tree = m_search_tree.size();
        BOOST_ASSERT_MSG(0 < size_of_tree, "tree empty");

        tree_node_file.WriteOne(size_of_tree);
        tree_node_file.WriteFrom(&m_search_tree[0], size_of_tree);

        MapLeafNodesFile(leaf_node_filename);
    }

    // Construct a packed Hilbert-R-Tree with Kamel-Faloutsos algorithm [1]
    void PackWithHilbert(const std::vector<EdgeDataT> &input_data_vector,
                         const std::string &tree_node_filename,
                         const std::string &leaf_node_filename)
    {
        const uint64_t element_count = input_data_vector.size();
        std::vector<HilbertInputElement> input_wrapper_vector(element_count);

        // generate auxiliary vector of hilbert-values
        tbb::parallel_for(
            tbb::blocked_range<uint64_t>(0, element_count),
            [&input_data_vector, &input_wrapper_vector, this](
                const tbb::blocked_range<uint64_t> &range) {
                for (uint64_t element_counter = range.begin(), end = range.end();
                     element_counter != end;
                     ++element_counter)
                {
                    HilbertInputElement &current_wrapper = input_wrapper_vector[element_counter];
                    current_wrapper.m_array_index = element_counter;

                    EdgeDataT const &current_element = input_data_vector[element_counter];

                    // Get Hilbert-Value for centroid in mercartor projection
                    BOOST_ASSERT(current_element.u < m_coordinate_list.size());
                    BOOST_ASSERT(current_element.v < m_coordinate_list.size());

                    Coordinate current_centroid = coordinate_calculation::centroid(
                        m_coordinate_list[current_element.u], m_coordinate_list[current_element.v]);
                    current_centroid.lat = FixedLatitude{static_cast<std::int32_t>(
                        COORDINATE_PRECISION *
                        web_mercator::latToY(toFloating(current_centroid.lat)))};

                    current_wrapper.m_hilbert_value = GetHilbertCode(current_centroid);
                }
            });

        // open leaf file
        boost::filesystem::ofstream leaf_node_file(leaf_node_filename, std::ios::binary);

        // sort the hilbert-value representatives
        tbb::parallel_sort(input_wrapper_vector.begin(), input_wrapper_vector.end());
        std::vector<TreeNode> tree_nodes_in_level;

        // pack M elements into leaf node, write to leaf file and add child index to the parent node
        uint64_t wrapped_element_index = 0;
        for (std::uint32_t node_index = 0; wrapped_element_index < element_count; ++node_index)
        {
            TreeNode current_node;
            for (std::uint32_t leaf_index = 0;
                 leaf_index < BRANCHING_FACTOR && wrapped_element_index < element_count;
                 ++leaf_index)
            {
                LeafNode current_leaf;
                Rectangle &rectangle = current_leaf.minimum_bounding_rectangle;
                for (std::uint32_t object_index = 0;
                     object_index < LEAF_NODE_SIZE && wrapped_element_index < element_count;
                     ++object_index, ++wrapped_element_index)
                {
                    const std::uint32_t input_object_index =
                        input_wrapper_vector[wrapped_element_index].m_array_index;
                    const EdgeDataT &object = input_data_vector[input_object_index];

                    current_leaf.object_count += 1;
                    current_leaf.objects[object_index] = object;

                    Coordinate projected_u{
                        web_mercator::fromWGS84(Coordinate{m_coordinate_list[object.u]})};
                    Coordinate projected_v{
                        web_mercator::fromWGS84(Coordinate{m_coordinate_list[object.v]})};

                    BOOST_ASSERT(std::abs(toFloating(projected_u.lon).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_u.lat).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_v.lon).operator double()) <= 180.);
                    BOOST_ASSERT(std::abs(toFloating(projected_v.lat).operator double()) <= 180.);

                    rectangle.min_lon =
                        std::min(rectangle.min_lon, std::min(projected_u.lon, projected_v.lon));
                    rectangle.max_lon =
                        std::max(rectangle.max_lon, std::max(projected_u.lon, projected_v.lon));

                    rectangle.min_lat =
                        std::min(rectangle.min_lat, std::min(projected_u.lat, projected_v.lat));
                    rectangle.max_lat =
                        std::max(rectangle.max_lat, std::max(projected_u.lat, projected_v.lat));

                    BOOST_ASSERT(rectangle.IsValid());
                }

                // append the leaf node to the current tree node
                current_node.child_count += 1;
                current_node.children[leaf_index] =
                    TreeIndex{node_index * BRANCHING_FACTOR + leaf_index, true};
                current_node.minimum_bounding_rectangle.MergeBoundingBoxes(
                    current_leaf.minimum_bounding_rectangle);

                // write leaf_node to leaf node file
                leaf_node_file.write((char *)&current_leaf, sizeof(current_leaf));
            }

            tree_nodes_in_level.emplace_back(current_node);
        }
        leaf_node_file.flush();
        leaf_node_file.close();

        std::uint32_t processing_level = 0;
        while (1 < tree_nodes_in_level.size())
        {
            std::vector<TreeNode> tree_nodes_in_next_level;
            std::uint32_t processed_tree_nodes_in_level = 0;
            while (processed_tree_nodes_in_level < tree_nodes_in_level.size())
            {
                TreeNode parent_node;
                // pack BRANCHING_FACTOR elements into tree_nodes each
                for (std::uint32_t current_child_node_index = 0;
                     current_child_node_index < BRANCHING_FACTOR;
                     ++current_child_node_index)
                {
                    if (processed_tree_nodes_in_level < tree_nodes_in_level.size())
                    {
                        TreeNode &current_child_node =
                            tree_nodes_in_level[processed_tree_nodes_in_level];
                        // add tree node to parent entry
                        parent_node.children[current_child_node_index] =
                            TreeIndex{m_search_tree.size(), false};
                        m_search_tree.emplace_back(current_child_node);
                        // merge MBRs
                        parent_node.minimum_bounding_rectangle.MergeBoundingBoxes(
                            current_child_node.minimum_bounding_rectangle);
                        // increase counters
                        ++parent_node.child_count;
                        ++processed_tree_nodes_in_level;
                    }
                }
                tree_nodes_in_next_level.emplace_back(parent_node);
            }
            tree_nodes_in_level.swap(tree_nodes_in_next_level);
            ++processing_level;
        }
        BOOST_ASSERT_MSG(tree_nodes_in_level.size() == 1, "tree broken, more than one root node");
        // last remaining entry is the root node, store it
        m_search_tree.emplace_back(tree_nodes_in_level[0]);

        // reverse and renumber tree to have root at index 0
        std::reverse(m_search_tree.begin(), m_search_tree.end());

        std::uint32_t search_tree_size = m_search_tree.size();
        tbb::parallel_for(
            tbb::blocked_range<std::uint32_t>(0, search_tree_size),
            [this, &search_tree_size](const tbb::blocked_range<std::uint32_t> &range) {
                for (std::uint32_t i = range.begin(), end = range.end(); i != end; ++i)
                {
                    TreeNode &current_tree_node = this->m_search_tree[i];
                    for (std::uint32_t j = 0; j < current_tree_node.child_count; ++j)
                    {
                        if (!current_tree_node.children[j].is_leaf)
                        {
                            const std::uint32_t old_id = current_tree_node.children[j].index;
                            const std::uint32_t new_id = search_tree_size - old_id - 1;
                            current_tree_node.children[j].index = new_id;
                        }
                    }
                }
            });

        // open tree file
        storage::io::FileWriter tree_node_file(tree_node_filename,
                                               storage::io::FileWriter::HasNoFingerprint);

        std::uint64_t size_of_tree = m_search_tree.size();
        BOOST_ASSERT_MSG(0 < size_of_tree, "tree empty");

        tree_node_file.WriteOne(size_of_tree);
        tree_node_file.WriteFrom(&m_search_tree[0], size_of_tree);

        MapLeafNodesFile(leaf_node_filename);
    }

    explicit StaticRTree(const boost::filesystem::path &node_file,
                         const boost::filesystem::path &leaf_file,
                         const Vector<Coordinate> &coordinate_list)
        : m_coordinate_list(coordinate_list)
    {
        storage::io::FileReader tree_node_file(node_file,
                                               storage::io::FileReader::HasNoFingerprint);

        const auto tree_size = tree_node_file.ReadElementCount64();

        m_search_tree.resize(tree_size);
        tree_node_file.ReadInto(&m_search_tree[0], tree_size);

        MapLeafNodesFile(leaf_file);
    }

    explicit StaticRTree(TreeNode *tree_node_ptr,
                         const uint64_t number_of_nodes,
                         const boost::filesystem::path &leaf_file,
                         const Vector<Coordinate> &coordinate_list)
        : m_search_tree(tree_node_ptr, number_of_nodes), m_coordinate_list(coordinate_list)
    {
        MapLeafNodesFile(leaf_file);
    }

    void MapLeafNodesFile(const boost::filesystem::path &leaf_file)
    {
        // open leaf node file and return a pointer to the mapped leaves data
        try
        {
            m_leaves_region.open(leaf_file);
            std::size_t num_leaves = m_leaves_region.size() / sizeof(LeafNode);
            auto data_ptr = m_leaves_region.data();
            BOOST_ASSERT(reinterpret_cast<uintptr_t>(data_ptr) % alignof(LeafNode) == 0);
            m_leaves.reset(reinterpret_cast<const LeafNode *>(data_ptr), num_leaves);
        }
        catch (const std::exception &exc)
        {
            throw exception(boost::str(boost::format("Leaf file %1% mapping failed: %2%") %
                                       leaf_file % exc.what()) +
                            SOURCE_REF);
        }
    }

    /* Returns all features inside the bounding box.
       Rectangle needs to be projected!*/
    std::vector<EdgeDataT> SearchInBox(const Rectangle &search_rectangle) const
    {
        const Rectangle projected_rectangle{
            search_rectangle.min_lon,
            search_rectangle.max_lon,
            toFixed(FloatLatitude{
                web_mercator::latToY(toFloating(FixedLatitude(search_rectangle.min_lat)))}),
            toFixed(FloatLatitude{
                web_mercator::latToY(toFloating(FixedLatitude(search_rectangle.max_lat)))})};
        std::vector<EdgeDataT> results;

        std::queue<TreeIndex> traversal_queue;
        traversal_queue.push(TreeIndex{});

        while (!traversal_queue.empty())
        {
            auto const current_tree_index = traversal_queue.front();
            traversal_queue.pop();

            if (current_tree_index.is_leaf)
            {
                const LeafNode &current_leaf_node = m_leaves[current_tree_index.index];

                for (const auto i : irange(0u, current_leaf_node.object_count))
                {
                    const auto &current_edge = current_leaf_node.objects[i];

                    // we don't need to project the coordinates here,
                    // because we use the unprojected rectangle to test against
                    const Rectangle bbox{std::min(m_coordinate_list[current_edge.u].lon,
                                                  m_coordinate_list[current_edge.v].lon),
                                         std::max(m_coordinate_list[current_edge.u].lon,
                                                  m_coordinate_list[current_edge.v].lon),
                                         std::min(m_coordinate_list[current_edge.u].lat,
                                                  m_coordinate_list[current_edge.v].lat),
                                         std::max(m_coordinate_list[current_edge.u].lat,
                                                  m_coordinate_list[current_edge.v].lat)};

                    // use the _unprojected_ input rectangle here
                    if (bbox.Intersects(search_rectangle))
                    {
                        results.push_back(current_edge);
                    }
                }
            }
            else
            {
                const TreeNode &current_tree_node = m_search_tree[current_tree_index.index];

                // If it's a tree node, look at all children and add them
                // to the search queue if their bounding boxes intersect
                for (std::uint32_t i = 0; i < current_tree_node.child_count; ++i)
                {
                    const TreeIndex child_id = current_tree_node.children[i];
                    const auto &child_rectangle =
                        child_id.is_leaf ? m_leaves[child_id.index].minimum_bounding_rectangle
                                         : m_search_tree[child_id.index].minimum_bounding_rectangle;

                    if (child_rectangle.Intersects(projected_rectangle))
                    {
                        traversal_queue.push(child_id);
                    }
                }
            }
        }
        return results;
    }

    // Override filter and terminator for the desired behaviour.
    std::vector<EdgeDataT> Nearest(const Coordinate input_coordinate,
                                   const std::size_t max_results) const
    {
        return Nearest(input_coordinate,
                       [](const CandidateSegment &) { return std::make_pair(true, true); },
                       [max_results](const std::size_t num_results, const CandidateSegment &) {
                           return num_results >= max_results;
                       });
    }

    // Override filter and terminator for the desired behaviour.
    template <typename FilterT, typename TerminationT>
    std::vector<EdgeDataT> Nearest(const Coordinate input_coordinate,
                                   const FilterT filter,
                                   const TerminationT terminate) const
    {
        std::vector<EdgeDataT> results;
        auto projected_coordinate = web_mercator::fromWGS84(input_coordinate);
        Coordinate fixed_projected_coordinate{projected_coordinate};

        // initialize queue with root element
        std::priority_queue<QueryCandidate> traversal_queue;
        traversal_queue.push(QueryCandidate{0, TreeIndex{}});

        while (!traversal_queue.empty())
        {
            QueryCandidate current_query_node = traversal_queue.top();
            traversal_queue.pop();

            const TreeIndex &current_tree_index = current_query_node.tree_index;
            if (!current_query_node.is_segment())
            { // current object is a tree node
                if (current_tree_index.is_leaf)
                {
                    ExploreLeafNode(current_tree_index,
                                    fixed_projected_coordinate,
                                    projected_coordinate,
                                    traversal_queue);
                }
                else
                {
                    ExploreTreeNode(
                        current_tree_index, fixed_projected_coordinate, traversal_queue);
                }
            }
            else
            { // current candidate is an actual road segment
                auto edge_data =
                    m_leaves[current_tree_index.index].objects[current_query_node.segment_index];
                const auto &current_candidate =
                    CandidateSegment{current_query_node.fixed_projected_coordinate, edge_data};

                // to allow returns of no-results if too restrictive filtering, this needs to be
                // done here even though performance would indicate that we want to stop after
                // adding the first candidate
                if (terminate(results.size(), current_candidate))
                {
                    break;
                }

                auto use_segment = filter(current_candidate);
                if (!use_segment.first && !use_segment.second)
                {
                    continue;
                }
                edge_data.forward_segment_id.enabled &= use_segment.first;
                edge_data.reverse_segment_id.enabled &= use_segment.second;

                // store phantom node in result vector
                results.push_back(std::move(edge_data));
            }
        }

        return results;
    }

  private:
    template <typename QueueT>
    void ExploreLeafNode(const TreeIndex &leaf_id,
                         const Coordinate &projected_input_coordinate_fixed,
                         const FloatCoordinate &projected_input_coordinate,
                         QueueT &traversal_queue) const
    {
        const LeafNode &current_leaf_node = m_leaves[leaf_id.index];

        // current object represents a block on disk
        for (const auto i : irange(0u, current_leaf_node.object_count))
        {
            const auto &current_edge = current_leaf_node.objects[i];
            const auto projected_u = web_mercator::fromWGS84(m_coordinate_list[current_edge.u]);
            const auto projected_v = web_mercator::fromWGS84(m_coordinate_list[current_edge.v]);

            FloatCoordinate projected_nearest;
            std::tie(std::ignore, projected_nearest) =
                coordinate_calculation::projectPointOnSegment(
                    projected_u, projected_v, projected_input_coordinate);

            const auto squared_distance = coordinate_calculation::squaredEuclideanDistance(
                projected_input_coordinate_fixed, projected_nearest);
            // distance must be non-negative
            BOOST_ASSERT(0. <= squared_distance);
            traversal_queue.push(
                QueryCandidate{squared_distance, leaf_id, i, Coordinate{projected_nearest}});
        }
    }

    template <class QueueT>
    void ExploreTreeNode(const TreeIndex &parent_id,
                         const Coordinate &fixed_projected_input_coordinate,
                         QueueT &traversal_queue) const
    {
        const TreeNode &parent = m_search_tree[parent_id.index];
        for (std::uint32_t i = 0; i < parent.child_count; ++i)
        {
            const TreeIndex child_id = parent.children[i];
            const auto &child_rectangle =
                child_id.is_leaf ? m_leaves[child_id.index].minimum_bounding_rectangle
                                 : m_search_tree[child_id.index].minimum_bounding_rectangle;
            const auto squared_lower_bound_to_element =
                child_rectangle.GetMinSquaredDist(fixed_projected_input_coordinate);
            traversal_queue.push(QueryCandidate{squared_lower_bound_to_element, child_id});
        }
    }
};

//[1] "On Packing R-Trees"; I. Kamel, C. Faloutsos; 1993; DOI: 10.1145/170088.170403
//[2] "Nearest Neighbor Queries", N. Roussopulos et al; 1995; DOI: 10.1145/223784.223794
//[3] "Distance Browsing in Spatial Databases"; G. Hjaltason, H. Samet; 1999; ACM Trans. DB Sys
// Vol.24 No.2, pp.265-318
}
}

#endif // STATIC_RTREE_HPP
