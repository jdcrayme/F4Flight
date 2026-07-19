// f4flight - campaign/include/f4flight/campaign/road_network.h
//
// RoadNetwork represents the network of roads and rail networks connecting objectives.
// Includes BFS pathfinding for ground troop navigation.

#pragma once

#include "f4flight/flight/core/types.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace f4flight {
namespace campaign {

struct RoadNode {
    int id{-1};
    Vec3 position{};
};

struct RoadEdge {
    int fromNode{-1};
    int toNode{-1};
    double cost{0.0}; // distance in ft
};

class RoadNetwork {
public:
    void addNode(int id, const Vec3& pos) {
        nodes_[id] = {id, pos};
    }

    void addEdge(int fromNode, int toNode) {
        auto it1 = nodes_.find(fromNode);
        auto it2 = nodes_.find(toNode);
        if (it1 == nodes_.end() || it2 == nodes_.end()) return;

        double dx = it1->second.position.x - it2->second.position.x;
        double dy = it1->second.position.y - it2->second.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        edges_[fromNode].push_back({fromNode, toNode, dist});
        edges_[toNode].push_back({toNode, fromNode, dist}); // bidirectional
    }

    const std::unordered_map<int, RoadNode>& nodes() const { return nodes_; }

    int findNearestNode(const Vec3& pos) const {
        int bestId = -1;
        double minDist = 1e18;
        for (const auto& [id, node] : nodes_) {
            double dx = node.position.x - pos.x;
            double dy = node.position.y - pos.y;
            double distSq = dx * dx + dy * dy;
            if (distSq < minDist) {
                minDist = distSq;
                bestId = id;
            }
        }
        return bestId;
    }

    // BFS Shortest Path of positions.
    std::vector<Vec3> findPath(const Vec3& start, const Vec3& end) const {
        int startNode = findNearestNode(start);
        int endNode = findNearestNode(end);
        if (startNode == -1 || endNode == -1) return {};
        if (startNode == endNode) return {nodes_.at(startNode).position};

        std::unordered_map<int, int> parent;
        std::vector<int> queue = {startNode};
        parent[startNode] = startNode;

        bool found = false;
        while (!queue.empty()) {
            int cur = queue.front();
            queue.erase(queue.begin());

            if (cur == endNode) {
                found = true;
                break;
            }

            auto it = edges_.find(cur);
            if (it == edges_.end()) continue;

            for (const auto& edge : it->second) {
                if (parent.find(edge.toNode) == parent.end()) {
                    parent[edge.toNode] = cur;
                    queue.push_back(edge.toNode);
                }
            }
        }

        if (!found) return {};

        std::vector<Vec3> path;
        int cur = endNode;
        while (cur != startNode) {
            path.push_back(nodes_.at(cur).position);
            cur = parent[cur];
        }
        path.push_back(nodes_.at(startNode).position);
        std::reverse(path.begin(), path.end());

        // Append actual start/end coordinates at boundary if they are far from the nearest nodes
        if (path.empty() || (path.front() - start).norm() > 10.0) {
            path.insert(path.begin(), start);
        }
        if (path.empty() || (path.back() - end).norm() > 10.0) {
            path.push_back(end);
        }

        return path;
    }

private:
    std::unordered_map<int, RoadNode> nodes_;
    std::unordered_map<int, std::vector<RoadEdge>> edges_;
};

} // namespace campaign
} // namespace f4flight
