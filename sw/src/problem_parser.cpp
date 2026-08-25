/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "problem_parser.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

std::string trim(const std::string &text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string value_after_colon(const std::string &line) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
        return trim(line);
    }
    return trim(line.substr(colon + 1));
}

ProblemInstance parse_vrp(const std::string &path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Could not open VRP file: " + path);
    }

    ProblemInstance problem;
    std::vector<ProblemNode> nodes;
    int expected_dimension = -1;

    enum class Section {
        Header,
        NodeCoords,
        Demands,
        Depot
    } section = Section::Header;

    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        if (line == "NODE_COORD_SECTION") {
            section = Section::NodeCoords;
            continue;
        }
        if (line == "DEMAND_SECTION") {
            section = Section::Demands;
            continue;
        }
        if (line == "DEPOT_SECTION") {
            section = Section::Depot;
            continue;
        }
        if (line == "EOF") {
            break;
        }

        if (section == Section::Header) {
            if (line.rfind("NAME", 0) == 0) {
                problem.name = value_after_colon(line);
            } else if (line.rfind("DIMENSION", 0) == 0) {
                expected_dimension = std::stoi(value_after_colon(line));
                nodes.resize(static_cast<std::size_t>(expected_dimension) + 1);
            } else if (line.rfind("CAPACITY", 0) == 0) {
                problem.capacity = std::stoi(value_after_colon(line));
            }
            continue;
        }

        if (section == Section::NodeCoords) {
            std::istringstream iss(line);
            int id = 0;
            int x = 0;
            int y = 0;
            if (!(iss >> id >> x >> y)) {
                throw std::runtime_error("Failed to parse NODE_COORD_SECTION line: " + line);
            }
            if (nodes.empty()) {
                throw std::runtime_error("DIMENSION must appear before NODE_COORD_SECTION");
            }
            nodes.at(static_cast<std::size_t>(id)) = ProblemNode{id, x, y, 0};
            continue;
        }

        if (section == Section::Demands) {
            std::istringstream iss(line);
            int id = 0;
            int demand = 0;
            if (!(iss >> id >> demand)) {
                throw std::runtime_error("Failed to parse DEMAND_SECTION line: " + line);
            }
            if (nodes.empty()) {
                throw std::runtime_error("DIMENSION must appear before DEMAND_SECTION");
            }
            nodes.at(static_cast<std::size_t>(id)).demand = demand;
            continue;
        }

        if (section == Section::Depot) {
            int depot_id = 0;
            std::istringstream iss(line);
            if (!(iss >> depot_id)) {
                throw std::runtime_error("Failed to parse DEPOT_SECTION line: " + line);
            }
            if (depot_id == -1) {
                section = Section::Header;
                continue;
            }
            problem.depotId = depot_id;
            continue;
        }
    }

    if (expected_dimension <= 0 || nodes.size() != static_cast<std::size_t>(expected_dimension) + 1) {
        throw std::runtime_error("VRP file did not provide a valid DIMENSION");
    }
    if (problem.capacity <= 0) {
        throw std::runtime_error("VRP file did not provide a valid CAPACITY");
    }

    problem.dimension = expected_dimension;
    problem.nodes = std::move(nodes);

    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    for (int id = 1; id <= problem.dimension; ++id) {
        min_x = std::min(min_x, problem.nodes.at(static_cast<std::size_t>(id)).x);
        min_y = std::min(min_y, problem.nodes.at(static_cast<std::size_t>(id)).y);
    }
    for (int id = 1; id <= problem.dimension; ++id) {
        problem.nodes.at(static_cast<std::size_t>(id)).x -= min_x;
        problem.nodes.at(static_cast<std::size_t>(id)).y -= min_y;
    }

    // TODO check constant bit constraint against the problem instance (max x/y, max demand, max capacity)
    // For example, ensure that the maximum x and y coordinates fit within COORD_BITS
    // and that the maximum demand and capacity fit within their respective bit widths.

    return problem;
}
