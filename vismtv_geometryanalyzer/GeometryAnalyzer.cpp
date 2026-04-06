#include "GeometryAnalyzer.h"
#include "CoreRelated.h"
#include "../vismtv_modeling_vera/launch_header.h" // including nanoflann.hpp

void function_launcher(void* param_container, void* helpers)
{
	__core_function_launcher((VmFnContainer*)param_container, *(helpers_lamda*)helpers, NULL);
}

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

auto ___fill_jet_colormap = [](int* colorarray, int ary_size)
{
	int prev_idx = 0;

	float gap = (float)ary_size / 4.f;
	vmfloat3 redf = vmfloat3(1.f, 0, 0);
	vmfloat3 greenf = vmfloat3(0, 1.f, 0);
	vmfloat3 bluef = vmfloat3(0, 0, 1.f);

	int index_1 = (int)(gap / 2.f + 0.5f);
	int index_2 = (int)(gap / 2.f + gap + 0.5f);
	int index_3 = (int)(gap / 2.f + 2 * gap + 0.5f);
	int index_4 = (int)(gap / 2.f + 3 * gap + 0.5f);
	int index_5 = ary_size;

	auto float3_2_int = [&](vmfloat3& c)
	{
		byte r = (byte)__min(c.x * 255.f, 255.f);
		byte g = (byte)__min(c.y * 255.f, 255.f);
		byte b = (byte)__min(c.z * 255.f, 255.f);
		return (r << 16) | (g << 8) | (b);
	};

	for (int i = 0; i <= index_1; i++)
	{
		vmfloat3 clr = bluef * (0.5f + 0.5f * (float)i / (float)index_1);
		colorarray[i] = float3_2_int(clr);
	}
	for (int i = index_1; i <= index_2; i++)
	{
		float ratio = (float)(i - index_1) / (float)(index_2 - index_1);
		vmfloat3 clr = bluef + greenf * ratio;
		colorarray[i] = float3_2_int(clr);
	}
	for (int i = index_2; i <= index_3; i++)
	{
		float ratio = (float)(i - index_2) / (float)(index_3 - index_2);
		vmfloat3 clr = greenf + bluef * (1.f - ratio) + redf * ratio;
		colorarray[i] = float3_2_int(clr);
	}
	for (int i = index_3; i <= index_4; i++)
	{
		float ratio = (float)(i - index_3) / (float)(index_4 - index_3);
		vmfloat3 clr = redf + greenf * (1.f - ratio);
		colorarray[i] = float3_2_int(clr);
	}
	for (int i = index_4; i <= index_5; i++)
	{
		vmfloat3 clr = redf * (1.f - 0.5f * (float)(i - index_4) / (float)(index_5 - index_4));
		colorarray[i] = float3_2_int(clr);
	}
};

void fill_organized_pointset_buffers(float* depthmap, int* indexmap, const int w, const int h,
	const void* mat_p, const void* pos_pts)
{	   
	const glm::dmat4x4& m_p = *(const glm::dmat4x4*)mat_p;
	const std::vector<__float3>& p_pts = *(const std::vector<__float3>*)pos_pts;
	fill_organized_pointset_buffers(depthmap, indexmap, w, h, m_p, p_pts);

	//___fill_jet_colormap(_color_array, __COLOR_ARRAY_SIZE);
	//using namespace cv;
	//Mat img_depthmap = Mat(h, w, CV_8UC3);
	//byte* buffer = (byte*)img_depthmap.data;
	//const float max_v = 2.f, min_v = 0.3f;
	//for (int i = 0; i < w * h; i++)
	//{
	//	float _value_sat = std::max(std::min(depthmap[i], max_v), min_v);
	//
	//	float ary_idx = (_value_sat - min_v) / (max_v - min_v) * (float)(__COLOR_ARRAY_SIZE - 1);
	//	vmbyte4 color = color_int_2_byte4(_color_array[(uint)ary_idx]);
	//	buffer[3 * i + 0] = color.z;
	//	buffer[3 * i + 1] = color.y;
	//	buffer[3 * i + 2] = color.x;
	//}
	//
	//imwrite("d:\\window_result\\depthmap_rgb_from_realsense.jpg", img_depthmap);
}

void fill_rgb_matching_map(unsigned char* rgbmap, const int* indexmap, const int w, const int h,
	const unsigned char* rgbimg, const int w1, const int h1, const void* mat_p, const void* pos_pts)
{
	const glm::dmat4x4& m_p = *(const glm::dmat4x4*)mat_p;
	const std::vector<__float3>& p_pts = *(const std::vector<__float3>*)pos_pts;
	fill_rgb_matching_map(rgbmap, indexmap, w, h, rgbimg, w1, h1, m_p, p_pts);

	//using namespace cv;
	//Mat rgb_map = Mat(h, w, CV_8UC3, rgbmap);
	//imwrite("d:\\window_result\\rgbmap_from_realsense.jpg", rgb_map);
}

void compute_camera_matrix_p(void* mat_p, const double fx, const double fy, const double ppx, const double ppy,
	const void* rm, const void* tv)
{
	glm::dmat4x4& m_p = *(glm::dmat4x4*)mat_p;
	const glm::dmat3x3& _rm = *(const glm::dmat3x3*)rm;
	const glm::dvec3& _tv = *(const glm::dvec3*)tv;
	compute_camera_matrix_p(m_p, fx, fy, ppx, ppy, _rm, _tv);
}

void normal_estimation(void* normalmap, void* eigenvaluemap, const int* indexmap, const unsigned char* mask,
	const int w, const int h, const void* pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt)
{
	__float3* _normalmap = (__float3*)normalmap;
	__float3* _eigenvaluemap = (__float3*)eigenvaluemap;
	const std::vector<__float3>& p_pts = *(const std::vector<__float3>*)pos_pts;
	normal_estimation(_normalmap, _eigenvaluemap, indexmap, mask, w, h, p_pts, kernel_radius, nbr_pixel_offset, use_kdt);
}

void curvature_estimation_ver1(void* curvaturemap, const int* indexmap, const void* normalmap, const unsigned char* mask,
	const int w, const int h, const void* pos_pts,
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt, const bool use_numerical)
{
	__float2* _curvaturemap = (__float2*)curvaturemap;
	const __float3* _normalmap = (const __float3*)normalmap;
	const std::vector<__float3>& p_pts = *(const std::vector<__float3>*)pos_pts;
	curvature_estimation_ver1(_curvaturemap, indexmap, _normalmap, mask, w, h, p_pts, kernel_radius, nbr_pixel_offset, use_kdt, use_numerical);
}

void fill_jet_colormap(int* clrmap, int size)
{
	___fill_jet_colormap(clrmap, size);
}