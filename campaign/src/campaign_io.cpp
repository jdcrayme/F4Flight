// f4flight - campaign/src/campaign_io.cpp
//
// Implementation of JSON serialization and deserialization for CampaignState.

#include "f4flight/campaign/campaign_io.h"
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <iomanip>
#include <cmath>

namespace f4flight {
namespace campaign {

namespace {

std::string escape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else out += c;
    }
    return out;
}

// Simple recursive-descent parser helper for campaign JSON
class CampaignReader {
public:
    explicit CampaignReader(const std::string& s) : s_(s), p_(0) {}

    void skipWs() {
        while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\t' ||
               s_[p_] == '\n' || s_[p_] == '\r')) ++p_;
    }

    char peek() { skipWs(); return (p_ < s_.size()) ? s_[p_] : '\0'; }
    char next() { skipWs(); return (p_ < s_.size()) ? s_[p_++] : '\0'; }

    void expect(char c) {
        if (next() != c) throw std::runtime_error(std::string("Expected '") + c + "' at index " + std::to_string(p_));
    }

    std::string readString() {
        skipWs();
        if (next() != '"') throw std::runtime_error("Expected string starting with '\"'");
        std::string out;
        while (p_ < s_.size() && s_[p_] != '"') {
            if (s_[p_] == '\\') {
                p_++;
                if (p_ >= s_.size()) break;
                out += s_[p_++];
            } else {
                out += s_[p_++];
            }
        }
        expect('"');
        return out;
    }

    double readNumber() {
        skipWs();
        size_t start = p_;
        if (p_ < s_.size() && (s_[p_] == '-' || s_[p_] == '+')) ++p_;
        while (p_ < s_.size() && (std::isdigit(s_[p_]) || s_[p_] == '.' ||
               s_[p_] == 'e' || s_[p_] == 'E' || s_[p_] == '+' || s_[p_] == '-')) ++p_;
        std::string tok = s_.substr(start, p_ - start);
        try {
            return std::stod(tok);
        } catch (...) {
            throw std::runtime_error("Bad number: " + tok);
        }
    }

    bool readBool() {
        skipWs();
        if (s_.compare(p_, 4, "true") == 0) { p_ += 4; return true; }
        if (s_.compare(p_, 5, "false") == 0) { p_ += 5; return false; }
        throw std::runtime_error("Expected boolean");
    }

    void readNull() {
        skipWs();
        if (s_.compare(p_, 4, "null") == 0) { p_ += 4; return; }
        throw std::runtime_error("Expected null");
    }

    void readVec3(Vec3& v) {
        expect('[');
        v.x = readNumber();
        expect(',');
        v.y = readNumber();
        expect(',');
        v.z = readNumber();
        expect(']');
    }

    void readInto(double& out) {
        skipWs();
        if (peek() == 'n') { readNull(); out = 0.0; return; }
        out = readNumber();
    }

    void readInto(bool& out) {
        skipWs();
        if (peek() == 't' || peek() == 'f') { out = readBool(); return; }
        out = (readNumber() != 0.0);
    }

    void readInto(int& out) {
        double d;
        readInto(d);
        out = static_cast<int>(d);
    }

    void readInto(std::string& out) {
        skipWs();
        if (peek() == '"') { out = readString(); return; }
        readNull(); out.clear();
    }

    void skipValue() {
        skipWs();
        char c = peek();
        if (c == '"') { readString(); return; }
        if (c == '{') {
            expect('{');
            if (peek() == '}') { next(); return; }
            while (true) {
                readString(); // key
                expect(':');
                skipValue();
                if (peek() == ',') { next(); continue; }
                break;
            }
            expect('}');
            return;
        }
        if (c == '[') {
            expect('[');
            if (peek() == ']') { next(); return; }
            while (true) {
                skipValue();
                if (peek() == ',') { next(); continue; }
                break;
            }
            expect(']');
            return;
        }
        if (c == 't' || c == 'f') { readBool(); return; }
        if (c == 'n') { readNull(); return; }
        readNumber();
    }

    template <typename F>
    void readObject(F&& cb) {
        expect('{');
        if (peek() == '}') { next(); return; }
        while (true) {
            std::string key = readString();
            expect(':');
            cb(key);
            if (peek() == ',') { next(); continue; }
            break;
        }
        expect('}');
    }

    template <typename F>
    void readArray(F&& cb) {
        expect('[');
        if (peek() == ']') { next(); return; }
        while (true) {
            cb();
            if (peek() == ',') { next(); continue; }
            break;
        }
        expect(']');
    }

private:
    const std::string& s_;
    size_t p_;
};

CoverType stringToCoverType(const std::string& s) {
    if (s == "Plains")     return CoverType::Plains;
    if (s == "Forest")     return CoverType::Forest;
    if (s == "Mountains")  return CoverType::Mountains;
    if (s == "Water")      return CoverType::Water;
    if (s == "Swamp")      return CoverType::Swamp;
    if (s == "City")       return CoverType::City;
    if (s == "Coast")      return CoverType::Coast;
    if (s == "Desert")     return CoverType::Desert;
    if (s == "Hills")      return CoverType::Hills;
    return CoverType::Plains;
}

ObjectiveType stringToObjectiveType(const std::string& s) {
    if (s == "Airbase")      return ObjectiveType::Airbase;
    if (s == "City")         return ObjectiveType::City;
    if (s == "Depot")        return ObjectiveType::Depot;
    if (s == "Port")         return ObjectiveType::Port;
    if (s == "RadarStation") return ObjectiveType::RadarStation;
    if (s == "Headquarters") return ObjectiveType::Headquarters;
    if (s == "Factory")      return ObjectiveType::Factory;
    if (s == "Fort")         return ObjectiveType::Fort;
    return ObjectiveType::City;
}

UnitType stringToUnitType(const std::string& s) {
    if (s == "GroundBattalion") return UnitType::GroundBattalion;
    if (s == "AirSquadron")     return UnitType::AirSquadron;
    if (s == "NavalTaskForce")   return UnitType::NavalTaskForce;
    return UnitType::GroundBattalion;
}

FormationState stringToFormationState(const std::string& s) {
    if (s == "Column") return FormationState::Column;
    if (s == "Wedge")  return FormationState::Wedge;
    if (s == "Line")   return FormationState::Line;
    return FormationState::Column;
}

} // namespace

std::string CampaignIO::serialize(const CampaignState& state) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "{\n";
    ss << "  \"time\": " << state.time() << ",\n";

    // Serialize Terrain
    const auto& ter = state.terrain();
    ss << "  \"terrain\": {\n";
    ss << "    \"cols\": " << ter.cols() << ",\n";
    ss << "    \"rows\": " << ter.rows() << ",\n";
    ss << "    \"tileSizeFt\": " << ter.tileSizeFt() << ",\n";
    ss << "    \"tiles\": [\n";
    for (int r = 0; r < ter.rows(); ++r) {
        for (int c = 0; c < ter.cols(); ++c) {
            const auto& tile = ter.tile(c, r);
            ss << "      {\"c\": \"" << coverTypeToString(tile.cover)
               << "\", \"e\": " << tile.elevation
               << ", \"road\": " << (tile.hasRoad ? "true" : "false")
               << ", \"rail\": " << (tile.hasRail ? "true" : "false")
               << ", \"w\": " << tile.wangId << "}";
            if (r < ter.rows() - 1 || c < ter.cols() - 1) ss << ",";
            ss << "\n";
        }
    }
    ss << "    ]\n";
    ss << "  },\n";

    // Serialize Road Network
    ss << "  \"roadNetwork\": {\n";
    ss << "    \"nodes\": [\n";
    const auto& rn = state.roadNetwork();
    bool first = true;
    for (const auto& [id, node] : rn.nodes()) {
        if (!first) ss << ",\n";
        first = false;
        ss << "      {\"id\": " << id << ", \"pos\": [" << node.position.x << ", " << node.position.y << ", " << node.position.z << "]}";
    }
    ss << "\n    ]\n";
    ss << "  },\n";

    // Serialize Objectives
    ss << "  \"objectives\": [\n";
    for (size_t i = 0; i < state.objectives().size(); ++i) {
        const auto& obj = state.objectives()[i];
        ss << "    {\n";
        ss << "      \"id\": " << obj.id << ",\n";
        ss << "      \"name\": \"" << escape(obj.name) << "\",\n";
        ss << "      \"type\": \"" << objectiveTypeToString(obj.type) << "\",\n";
        ss << "      \"faction\": " << obj.faction << ",\n";
        ss << "      \"health\": " << obj.health << ",\n";
        ss << "      \"pos\": [" << obj.position.x << ", " << obj.position.y << ", " << obj.position.z << "],\n";

        // Structures
        ss << "      \"structures\": [\n";
        for (size_t s = 0; s < obj.structures.size(); ++s) {
            const auto& st = obj.structures[s];
            ss << "        {\"id\": " << st.id << ", \"name\": \"" << escape(st.name) << "\", \"h\": " << st.health
               << ", \"offset\": [" << st.offset.x << ", " << st.offset.y << ", " << st.offset.z << "]}";
            if (s < obj.structures.size() - 1) ss << ",";
            ss << "\n";
        }
        ss << "      ]";

        if (obj.type == ObjectiveType::Airbase) {
            ss << ",\n";
            // Runways
            ss << "      \"runways\": [\n";
            for (size_t r = 0; r < obj.runways.size(); ++r) {
                const auto& rwy = obj.runways[r];
                ss << "        {\"id\": " << rwy.id << ", \"name\": \"" << escape(rwy.name) << "\", \"width\": " << rwy.width
                   << ", \"heading\": " << rwy.heading << ", \"h\": " << rwy.health
                   << ", \"start\": [" << rwy.start.x << ", " << rwy.start.y << ", " << rwy.start.z << "]"
                   << ", \"end\": [" << rwy.end.x << ", " << rwy.end.y << ", " << rwy.end.z << "]}";
                if (r < obj.runways.size() - 1) ss << ",";
                ss << "\n";
            }
            ss << "      ],\n";

            // Taxi Nodes
            ss << "      \"taxiNodes\": [\n";
            for (size_t tn = 0; tn < obj.taxiNodes.size(); ++tn) {
                const auto& node = obj.taxiNodes[tn];
                ss << "        {\"id\": " << node.id << ", \"pos\": [" << node.position.x << ", " << node.position.y << ", " << node.position.z << "], \"label\": \"" << escape(node.label) << "\"}";
                if (tn < obj.taxiNodes.size() - 1) ss << ",";
                ss << "\n";
            }
            ss << "      ],\n";

            // Taxi Edges
            ss << "      \"taxiEdges\": [\n";
            for (size_t te = 0; te < obj.taxiEdges.size(); ++te) {
                const auto& edge = obj.taxiEdges[te];
                ss << "        {\"from\": " << edge.fromId << ", \"to\": " << edge.toId << ", \"dist\": " << edge.distance << "}";
                if (te < obj.taxiEdges.size() - 1) ss << ",";
                ss << "\n";
            }
            ss << "      ]\n";
        } else {
            ss << "\n";
        }

        ss << "    }";
        if (i < state.objectives().size() - 1) ss << ",";
        ss << "\n";
    }
    ss << "  ],\n";

    // Serialize Units
    ss << "  \"units\": [\n";
    for (size_t i = 0; i < state.units().size(); ++i) {
        const auto& unit = state.units()[i];
        ss << "    {\n";
        ss << "      \"id\": " << unit.id << ",\n";
        ss << "      \"name\": \"" << escape(unit.name) << "\",\n";
        ss << "      \"type\": \"" << unitTypeToString(unit.type) << "\",\n";
        ss << "      \"faction\": " << unit.faction << ",\n";
        ss << "      \"health\": " << unit.health << ",\n";
        ss << "      \"speed\": " << unit.speed << ",\n";
        ss << "      \"pos\": [" << unit.position.x << ", " << unit.position.y << ", " << unit.position.z << "],\n";
        ss << "      \"dest\": [" << unit.destination.x << ", " << unit.destination.y << ", " << unit.destination.z << "],\n";
        ss << "      \"isActive\": " << (unit.isActive ? "true" : "false") << ",\n";
        ss << "      \"hasTargetObjective\": " << (unit.hasTargetObjective ? "true" : "false") << ",\n";
        ss << "      \"targetObjectiveId\": " << unit.targetObjectiveId << ",\n";
        ss << "      \"formation\": \"" << formationStateToString(unit.formation) << "\",\n";

        // SubUnits
        ss << "      \"subUnits\": [\n";
        for (size_t s = 0; s < unit.subUnits.size(); ++s) {
            const auto& su = unit.subUnits[s];
            ss << "        {\"id\": " << su.id << ", \"name\": \"" << escape(su.name) << "\", \"h\": " << su.health
               << ", \"pos\": [" << su.position.x << ", " << su.position.y << ", " << su.position.z << "], \"hdg\": " << su.heading << "}";
            if (s < unit.subUnits.size() - 1) ss << ",";
            ss << "\n";
        }
        ss << "      ],\n";

        // Path
        ss << "      \"path\": [\n";
        for (size_t p = 0; p < unit.path.size(); ++p) {
            ss << "        [" << unit.path[p].x << ", " << unit.path[p].y << ", " << unit.path[p].z << "]";
            if (p < unit.path.size() - 1) ss << ",";
            ss << "\n";
        }
        ss << "      ],\n";
        ss << "      \"currentPathIndex\": " << unit.currentPathIndex << "\n";
        ss << "    }";
        if (i < state.units().size() - 1) ss << ",";
        ss << "\n";
    }
    ss << "  ]\n";

    ss << "}\n";
    return ss.str();
}

bool CampaignIO::serializeToFile(const CampaignState& state, const std::string& filepath) {
    std::ofstream f(filepath, std::ios::binary);
    if (!f) return false;
    f << serialize(state);
    return f.good();
}

bool CampaignIO::deserialize(const std::string& json, CampaignState& state) {
    try {
        CampaignReader r(json);
        state = CampaignState{};

        r.readObject([&](const std::string& key) {
            if (key == "time") {
                double t;
                r.readInto(t);
                state.setTime(t);
            } else if (key == "terrain") {
                int cols = 0, rows = 0;
                double tileSizeFt = 0.0;
                r.readObject([&](const std::string& tk) {
                    if (tk == "cols") r.readInto(cols);
                    else if (tk == "rows") r.readInto(rows);
                    else if (tk == "tileSizeFt") r.readInto(tileSizeFt);
                    else if (tk == "tiles") {
                        state.terrain() = TerrainMap(cols, rows, tileSizeFt);
                        int col = 0, row = 0;
                        r.readArray([&]() {
                            TerrainTile tile;
                            r.readObject([&](const std::string& tileK) {
                                if (tileK == "c") {
                                    std::string c;
                                    r.readInto(c);
                                    tile.cover = stringToCoverType(c);
                                } else if (tileK == "e") r.readInto(tile.elevation);
                                else if (tileK == "road") r.readInto(tile.hasRoad);
                                else if (tileK == "rail") r.readInto(tile.hasRail);
                                else if (tileK == "w") r.readInto(tile.wangId);
                                else r.skipValue();
                            });
                            if (state.terrain().isValid(col, row)) {
                                state.terrain().tile(col, row) = tile;
                            }
                            col++;
                            if (col >= cols) {
                                col = 0;
                                row++;
                            }
                        });
                    } else r.skipValue();
                });
            } else if (key == "roadNetwork") {
                r.readObject([&](const std::string& rnk) {
                    if (rnk == "nodes") {
                        r.readArray([&]() {
                            int id = 0;
                            Vec3 pos{};
                            r.readObject([&](const std::string& nk) {
                                if (nk == "id") r.readInto(id);
                                else if (nk == "pos") r.readVec3(pos);
                                else r.skipValue();
                            });
                            state.roadNetwork().addNode(id, pos);
                        });
                    } else r.skipValue();
                });
            } else if (key == "objectives") {
                r.readArray([&]() {
                    Objective obj;
                    r.readObject([&](const std::string& ok) {
                        if (ok == "id") r.readInto(obj.id);
                        else if (ok == "name") r.readInto(obj.name);
                        else if (ok == "type") {
                            std::string t;
                            r.readInto(t);
                            obj.type = stringToObjectiveType(t);
                        } else if (ok == "faction") r.readInto(obj.faction);
                        else if (ok == "health") r.readInto(obj.health);
                        else if (ok == "pos") r.readVec3(obj.position);
                        else if (ok == "structures") {
                            r.readArray([&]() {
                                ObjectiveStructure s;
                                r.readObject([&](const std::string& sk) {
                                    if (sk == "id") r.readInto(s.id);
                                    else if (sk == "name") r.readInto(s.name);
                                    else if (sk == "h") r.readInto(s.health);
                                    else if (sk == "offset") r.readVec3(s.offset);
                                    else r.skipValue();
                                });
                                obj.structures.push_back(s);
                            });
                        } else if (ok == "runways") {
                            r.readArray([&]() {
                                Runway rwy;
                                r.readObject([&](const std::string& rk) {
                                    if (rk == "id") r.readInto(rwy.id);
                                    else if (rk == "name") r.readInto(rwy.name);
                                    else if (rk == "width") r.readInto(rwy.width);
                                    else if (rk == "heading") r.readInto(rwy.heading);
                                    else if (rk == "h") r.readInto(rwy.health);
                                    else if (rk == "start") r.readVec3(rwy.start);
                                    else if (rk == "end") r.readVec3(rwy.end);
                                    else r.skipValue();
                                });
                                obj.runways.push_back(rwy);
                            });
                        } else if (ok == "taxiNodes") {
                            r.readArray([&]() {
                                TaxiNode node;
                                r.readObject([&](const std::string& tnk) {
                                    if (tnk == "id") r.readInto(node.id);
                                    else if (tnk == "pos") r.readVec3(node.position);
                                    else if (tnk == "label") r.readInto(node.label);
                                    else r.skipValue();
                                });
                                obj.taxiNodes.push_back(node);
                            });
                        } else if (ok == "taxiEdges") {
                            r.readArray([&]() {
                                TaxiEdge edge;
                                r.readObject([&](const std::string& tek) {
                                    if (tek == "from") r.readInto(edge.fromId);
                                    else if (tek == "to") r.readInto(edge.toId);
                                    else if (tek == "dist") r.readInto(edge.distance);
                                    else r.skipValue();
                                });
                                obj.taxiEdges.push_back(edge);
                            });
                        } else r.skipValue();
                    });
                    state.addObjective(obj);
                });
            } else if (key == "units") {
                r.readArray([&]() {
                    Unit unit;
                    r.readObject([&](const std::string& uk) {
                        if (uk == "id") r.readInto(unit.id);
                        else if (uk == "name") r.readInto(unit.name);
                        else if (uk == "type") {
                            std::string t;
                            r.readInto(t);
                            unit.type = stringToUnitType(t);
                        } else if (uk == "faction") r.readInto(unit.faction);
                        else if (uk == "health") r.readInto(unit.health);
                        else if (uk == "speed") r.readInto(unit.speed);
                        else if (uk == "pos") r.readVec3(unit.position);
                        else if (uk == "dest") r.readVec3(unit.destination);
                        else if (uk == "isActive") r.readInto(unit.isActive);
                        else if (uk == "hasTargetObjective") r.readInto(unit.hasTargetObjective);
                        else if (uk == "targetObjectiveId") r.readInto(unit.targetObjectiveId);
                        else if (uk == "formation") {
                            std::string f;
                            r.readInto(f);
                            unit.formation = stringToFormationState(f);
                        } else if (uk == "subUnits") {
                            r.readArray([&]() {
                                SubUnit su;
                                r.readObject([&](const std::string& suk) {
                                    if (suk == "id") r.readInto(su.id);
                                    else if (suk == "name") r.readInto(su.name);
                                    else if (suk == "h") r.readInto(su.health);
                                    else if (suk == "pos") r.readVec3(su.position);
                                    else if (suk == "hdg") r.readInto(su.heading);
                                    else r.skipValue();
                                });
                                unit.subUnits.push_back(su);
                            });
                        } else if (uk == "path") {
                            r.readArray([&]() {
                                Vec3 p{};
                                r.readVec3(p);
                                unit.path.push_back(p);
                            });
                        } else if (uk == "currentPathIndex") {
                            int idx;
                            r.readInto(idx);
                            unit.currentPathIndex = static_cast<size_t>(idx);
                        } else r.skipValue();
                    });
                    state.addUnit(unit);
                });
            } else {
                r.skipValue();
            }
        });
        return true;
    } catch (...) {
        return false;
    }
}

bool CampaignIO::deserializeFromFile(const std::string& filepath, CampaignState& state) {
    std::ifstream f(filepath, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    return deserialize(ss.str(), state);
}

} // namespace campaign
} // namespace f4flight
