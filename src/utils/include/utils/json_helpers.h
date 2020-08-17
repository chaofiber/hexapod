#pragma once

#include <nlohmann/json.hpp>
#include <Eigen/Core>

namespace Eigen {

void to_json(nlohmann::json &j, const Eigen::Vector2d &v) {
	j.push_back(v[0]);
	j.push_back(v[1]);
}

void from_json(const nlohmann::json &j, Eigen::Vector2d &v) {
	v << j[0], j[1];
}
}
