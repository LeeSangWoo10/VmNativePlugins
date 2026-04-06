#pragma once
#include "../vismtv_modeling_vera/launch_header.h"

__vmstatic bool __core_function_launcher(VmFnContainer* _fncontainer, helpers_lamda& helpers, LocalProgress* _progress);

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

void fill_organized_pointset_buffers(float* depthmap, int* indexmap, const int w, const int h, const glm::dmat4x4& mat_p, const std::vector<__float3>& pos_pts);
void fill_rgb_matching_map(byte* rgbmap, const int* indexmap, const int w, const int h,
	const byte* rgbimg, const int w1, const int h1,
	const glm::dmat4x4& mat_p, const std::vector<__float3>& pos_pts);
void compute_camera_matrix_p(glm::dmat4x4& mat_p, const double fx, const double fy, const double ppx, const double ppy, const glm::dmat3x3& rm, const glm::dvec3& tv);

void normal_estimation(__float3* normalmap, __float3* eigenvaluemap, const int* indexmap, const byte* mask,
	const int w, const int h, const std::vector<__float3>& pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt);

void curvature_estimation_ver1(__float2* curvaturemap, const int* indexmap, const __float3* normalmap, const byte* mask,
	const int w, const int h, const std::vector<__float3>& pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt, const bool use_numerical);
