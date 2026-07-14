// f4flight - digi/atc/taxi_graph.h
//
// TaxiGraph — data-driven taxi route network for an airbase.
//
// FreeFalcon hardcodes taxi logic in landme.cpp (ChooseNextPoint,
// CheckTaxiTrackPoint, DealWithBlocker — ~500 LOC of procedural routing).
// We replace this with a data-driven graph: the host defines taxi nodes
// (parking spots, runway thresholds, hold-short points, intersections) and
// edges (taxiway segments). The AI just follows the shortest path.
//
// Design:
//   - A TaxiNode has a position (world ft) and a type (parking, holdshort,
//     runway threshold, intersection, runway exit).
//   - Edges are bidirectional, with a distance (for speed planning).
//   - The graph is a simple adjacency list — no routing library dependency.
//   - Path finding is BFS (shortest hop count) or Dijkstra (shortest
//     distance). For airbase taxi networks (typically <50 nodes), BFS is
//     sufficient and simpler.

#pragma once

#include "f4flight/flight/core/types.h"

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <optional>
#include <cstdint>

namespace f4flight {
namespace digi {

// RunwayId is defined in atc_messages.h, which is included transitively.
// Forward-declare here so taxi_graph.h can be used standalone if needed.
using RunwayId = int;

namespace atc {

using TaxiNodeId = int;
constexpr TaxiNodeId kInvalidTaxiNode = -1;

enum class TaxiNodeType {
    ParkingSpot,    // aircraft start position
    HoldShort,      // hold short of runway
    RunwayThreshold,// runway entry point
    RunwayExit,     // runway exit point
    Intersection,   // taxiway intersection
    Apron,          // apron/ramp area
};

struct TaxiNode {
    TaxiNodeId id{kInvalidTaxiNode};
    Vec3 position{};  // world frame (ft), z = -altitude
    TaxiNodeType type{TaxiNodeType::Intersection};
    RunwayId runway{0};  // for holdshort/threshold/exit nodes
};

struct TaxiEdge {
    TaxiNodeId from{kInvalidTaxiNode};
    TaxiNodeId to{kInvalidTaxiNode};
    double distance{0.0};  // ft (for speed planning)
};

class TaxiGraph {
public:
    // Add a node to the graph. Nodes are indexed by their `id` field.
    // If a node with the same id already exists, this overwrites it and
    // returns false (caller can detect the duplicate). Returns true on a
    // fresh insertion. Negative ids are rejected (returns false).
    bool addNode(const TaxiNode& node) {
        if (node.id < 0) return false;
        const bool isDuplicate =
            (static_cast<int>(nodes_.size()) > node.id) &&
            (nodes_[node.id].id == node.id);  // sentinel check (default-constructed placeholders have id == 0)
        if (static_cast<int>(nodes_.size()) <= node.id) {
            nodes_.resize(node.id + 1);
        }
        nodes_[node.id] = node;
        return !isDuplicate;
    }

    void addEdge(TaxiNodeId a, TaxiNodeId b) {
        if (a < 0 || b < 0) return;
        const double dx = nodes_[a].position.x - nodes_[b].position.x;
        const double dy = nodes_[a].position.y - nodes_[b].position.y;
        const double dz = nodes_[a].position.z - nodes_[b].position.z;
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        edges_[a].push_back({a, b, dist});
        edges_[b].push_back({b, a, dist});  // bidirectional
    }

    const TaxiNode& node(TaxiNodeId id) const {
        return nodes_[id];
    }

    const std::vector<TaxiEdge>& edgesFrom(TaxiNodeId id) const {
        static const std::vector<TaxiEdge> empty;
        auto it = edges_.find(id);
        return (it != edges_.end()) ? it->second : empty;
    }

    std::size_t nodeCount() const { return nodes_.size(); }

    // Find nearest node of a specific type to a world position.
    TaxiNodeId findNearest(const Vec3& pos, TaxiNodeType type) const {
        TaxiNodeId best = kInvalidTaxiNode;
        double bestDist = 1e18;
        for (const auto& n : nodes_) {
            if (n.type != type) continue;
            const double dx = n.position.x - pos.x;
            const double dy = n.position.y - pos.y;
            const double d = dx*dx + dy*dy;
            if (d < bestDist) {
                bestDist = d;
                best = n.id;
            }
        }
        return best;
    }

    // Find a runway threshold node by runway ID.
    TaxiNodeId findRunwayThreshold(RunwayId rwy) const {
        for (const auto& n : nodes_) {
            if (n.type == TaxiNodeType::RunwayThreshold && n.runway == rwy) {
                return n.id;
            }
        }
        return kInvalidTaxiNode;
    }

    // Find a hold-short node for a runway.
    TaxiNodeId findHoldShort(RunwayId rwy) const {
        for (const auto& n : nodes_) {
            if (n.type == TaxiNodeType::HoldShort && n.runway == rwy) {
                return n.id;
            }
        }
        return kInvalidTaxiNode;
    }

    // BFS shortest path (by hop count). Returns the sequence of nodes
    // from start to goal (inclusive), or empty if no path.
    std::vector<TaxiNodeId> findPath(TaxiNodeId start, TaxiNodeId goal) const {
        if (start < 0 || goal < 0) return {};
        if (start == goal) return {start};

        std::unordered_map<TaxiNodeId, TaxiNodeId> prev;
        std::vector<TaxiNodeId> queue = {start};
        prev[start] = start;

        while (!queue.empty()) {
            TaxiNodeId cur = queue.front();
            queue.erase(queue.begin());

            if (cur == goal) {
                // Reconstruct path
                std::vector<TaxiNodeId> path;
                TaxiNodeId n = goal;
                while (n != start) {
                    path.push_back(n);
                    n = prev[n];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            auto it = edges_.find(cur);
            if (it == edges_.end()) continue;
            for (const auto& edge : it->second) {
                if (prev.find(edge.to) == prev.end()) {
                    prev[edge.to] = cur;
                    queue.push_back(edge.to);
                }
            }
        }
        return {};  // no path
    }

private:
    std::vector<TaxiNode> nodes_;
    std::unordered_map<TaxiNodeId, std::vector<TaxiEdge>> edges_;
};

} // namespace atc
} // namespace digi
} // namespace f4flight
