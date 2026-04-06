#pragma once

#ifndef __dojo_exoprt
#define __dojo_export extern "C" __declspec(dllexport)
#endif

// do not use this function
__dojo_export void function_launcher(void* param_container, void* helpers);

// please set openmp option

// :: compute camera matrix from intrinsic and extrinsic parameter
// mat_p : glm::dmat4x4, camera metrix (output)
// fx, fy : focusing parameters
// ppx, ppy : principal point position
// rm : glm::dmat3x3, rotation matrix (col major)
// tv : glm::dvec3, translation vector
__dojo_export void compute_camera_matrix_p(void* mat_p, const double fx, const double fy, const double ppx, const double ppy,
	const void* rm, const void* tv);

// :: make depth map from organized point set
// depthmap : depth (already allocated with w and h) buffer, 
// indexmap : index (already allocated with w and h) buffer indexing pos_pts, 
// w, h : buffer width and height,
// mat_p : glm::dmat4x4, camera metrix (intrinsic and extrinsic), col-major,
// pos_pts : organized point set ex. float3 array, 
__dojo_export void fill_organized_pointset_buffers(float* depthmap, int* indexmap, const int w, const int h, 
	const void* mat_p, const void* pos_pts);

// :: make rgb map from rgb captured image 
// rgbmap : rgb map buffer (already allocated with w and h) (output)
// indexmap : index (already set) buffer indexing pos_pts, 
// w, h : buffer width and height,
// rgbimg : rgb captured image, 
// w1, h1 : rgb captured image's width and height,
// mat_p : glm::dmat4x4, camera metrix (intrinsic and extrinsic), col-major,
// pos_pts : organized point set ex. float3 array, 
__dojo_export void fill_rgb_matching_map(unsigned char* rgbmap, const int* indexmap, const int w, const int h,
	const unsigned char* rgbimg, const int w1, const int h1, const void* mat_p, const void* pos_pts);

// :: normal estimation through PCA 
// normalmap : normal buffer (already allocated with w and h), glm::fvec3 (output)
// eigenvaluemap : eigen value buffer (already allocated with w and h), glm::fvec3 (output), x<y<z
// indexmap : index (already set) buffer indexing pos_pts, (skip at minus index pixels)
// mask : flag map. skip 0-value pixels
// w, h : buffer width and height,
// pos_pts : organized point set ex. float3 array, 
// kernel_radius : kernel size for KNN, 
// nbr_pixel_offset : image (pixels) space's neighborhood kernel size, 
// use_kdt : true ==> just use KNN without nbr_pixel_offset, false ==> do not use KNN, use neighbor pixels, 
__dojo_export void normal_estimation(void* normalmap, void* eigenvaluemap, const int* indexmap, const unsigned char* mask,
	const int w, const int h, const void* pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt);

// :: curvature estimation through numerical quadratic-fitting
// curvaturemap : curvature buffer (already allocated with w and h), glm::fvec2 (output), x : principal curvature 1, y : principal curvature 2
// indexmap : index (already set) buffer indexing pos_pts, (skip at minus index pixels)
// normalmap : normal buffer (already set), glm::fvec3
// mask : flag map. skip 0-value pixels
// w, h : buffer width and height,
// pos_pts : organized point set ex. float3 array, 
// kernel_radius : kernel size for KNN, 
// nbr_pixel_offset : image (pixels) space's neighborhood kernel size, 
// use_kdt : true ==> just use KNN without nbr_pixel_offset, false ==> do not use KNN, use neighbor pixels, 
// use_numerical : false ==> use finite difference for derivaties leading to smaller curvature-scales, 
__dojo_export void curvature_estimation_ver1(void* curvaturemap, const int* indexmap, const void* normalmap, const unsigned char* mask,
	const int w, const int h, const void* pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt, const bool use_numerical);