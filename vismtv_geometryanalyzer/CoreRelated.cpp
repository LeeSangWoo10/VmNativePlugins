#include "CoreRelated.h"
#include "../vismtv_modeling_vera/launch_header.h"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Householder>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigen>

#define SELF_OP(a, b, c) a=c(a, b)

using namespace std;
using namespace vmmath;

bool __core_function_launcher(VmFnContainer* _fncontainer, helpers_lamda& helpers, LocalProgress* _progress)
{
	string fnGeoAnalysis = _fncontainer->fnParams.GetParam("_string_FunctionGeoAnalysis", string("NONE"));

	VmVObjectPrimitive* pCVtxIn = _fncontainer->fnParams.GetParam("_VmVObjectPrimitive*_InputPointCloud", (VmVObjectPrimitive*)NULL);

	vector<VmVObjectPrimitive*>* pCVtxOut_Tests = _fncontainer->fnParams.GetParamPtr<vector<VmVObjectPrimitive*>>("_vlist_VmObject_TestPrims");
	// 주소의 배열이므로 sizeof(VmVObjectPrimitive) * __NUM_PTX_OBJECTS 안 됨
	//ZeroMemory(pCVtxOut_Tests, sizeof(VmVObjectPrimitive) * __NUM_PTX_OBJECTS); 

	PrimitiveData* meshArchive = pCVtxIn->GetPrimitiveData();
	vector<glm::fvec3> _pos_pts(meshArchive->num_vtx);
	// unit : meter (m)
	memcpy(&_pos_pts[0], meshArchive->GetVerticeDefinition("POSITION"), sizeof(__float3) * meshArchive->num_vtx);

	vector<__float3>& pos_pts = *(vector<__float3>*)&_pos_pts;

	using namespace glm;

	// parameters
	int w = 1280, h = 720;
	double fx0 = 901.736, fy0 = 901.736, ppx0 = 638.137, ppy0 = 361.349; // depth camera (ir r (카메라를 보는 방향에서)) ==> ws 기준
	double fx1 = 927.048, fy1 = 926.007, ppx1 = 632.936, ppy1 = 372.943; // rgb camera
	dmat3x3 rt(0.999985,
		-0.00161912,
		-0.00518555,
		0.00161944,
		0.999999,
		5.85964e-05,
		0.00518545,
		-6.69933e-05,
		0.999987
	);
	dvec3 tv(0.0150511,
		-5.58825e-05,
		-0.000315386
	);

	dmat4x4 mat_p_ir, mat_p_rgb;
	compute_camera_matrix_p(mat_p_ir, fx0, fy0, ppx0, ppy0, dmat3x3(1), dvec3(0));
	compute_camera_matrix_p(mat_p_rgb, fx1, fy1, ppx1, ppy1, rt, tv);

	float* depthmap = new float[w*h];
	int* indexmap = new int[w*h];
	fill_organized_pointset_buffers(depthmap, indexmap, w, h, mat_p_ir, pos_pts);

	using namespace cv;
	Mat rgb_map = Mat(h, w, CV_8UC3);
	//Mat rgb_img = imread("D:\\Data\\현대중공업\\0709_data\\rgb_capture_4.png");
	Mat rgb_img = imread("D:\\Data\\현대중공업\\0624_data\\rgb_capture_4.png");
	//Mat rgb_img = imread("D:\\window_result\\0704\\rgb_capture_1.png");
	fill_rgb_matching_map(rgb_map.data, indexmap, w, h, rgb_img.data, w, h, mat_p_rgb, pos_pts);


	__float3* normalmap = new __float3[w*h];
	__float3* eigenvaluemap = new __float3[w*h];
	__float2* curvmap = new __float2[w*h];
	byte* mask = new byte[w*h];
	memset(mask, 1, w*h);

	double kernelnormal = _fncontainer->fnParams.GetParam("_double_kernelnormal", 0.01);
	int nbrpixsnormal = _fncontainer->fnParams.GetParam("_int_nbrpixnormal", (int)5);
	bool use_kdtnormal = _fncontainer->fnParams.GetParam("_bool_usekdtnormal", true);
	normal_estimation(normalmap, eigenvaluemap, indexmap, mask, w, h, pos_pts, kernelnormal, nbrpixsnormal, use_kdtnormal);

	double kernelcurv = _fncontainer->fnParams.GetParam("_double_kernelcurv", 0.02);
	int nbrpixscurv = _fncontainer->fnParams.GetParam("_int_nbrpixcurv", (int)10);
	bool use_kdtcurv = _fncontainer->fnParams.GetParam("_bool_usekdtcurv", true);
	bool use_nurmcurv = _fncontainer->fnParams.GetParam("_bool_usekdtnumerical", false);
	curvature_estimation_ver1(curvmap, indexmap, normalmap, mask, w, h, pos_pts, kernelcurv, nbrpixscurv, use_kdtcurv, use_nurmcurv);

	/////////////////////////////
	// ui out
	helpers.___fill_jet_colormap(_color_array, __COLOR_ARRAY_SIZE);
	auto draw_color_map = [](const int w, const int h, const float* pixels, const string filename, const float min_v, const float max_v)
	{
		Mat img_colormap = Mat(h, w, CV_8UC3);
		byte* buffer = (byte*)img_colormap.data;
		for (int i = 0; i < w * h; i++)
		{
			float _value_sat = std::max(std::min(pixels[i], max_v), min_v);

			float ary_idx = (_value_sat - min_v) / (max_v - min_v) * (float)(__COLOR_ARRAY_SIZE - 1);
			vmbyte4 color = color_int_2_byte4(_color_array[(uint)ary_idx]);
			buffer[3 * i + 0] = color.z;
			buffer[3 * i + 1] = color.y;
			buffer[3 * i + 2] = color.x;
		}
		imwrite(filename, img_colormap);
	};
	auto draw_normal_map = [](const int w, const int h, const __float3* normalmap, const string filename)
	{
		Mat img_colormap = Mat(h, w, CV_8UC3);
		byte* buffer = (byte*)img_colormap.data;
		for (int i = 0; i < w * h; i++)
		{
			if (normalmap[i].x == 0 && normalmap[i].y == 0 && normalmap[i].z == 0)
			{
				buffer[3 * i + 0] = 0;
				buffer[3 * i + 1] = 0;
				buffer[3 * i + 2] = 0;
				continue;
			}
			vmfloat3 c = (*(vmfloat3*)&normalmap[i] + vmfloat3(1.f, 1.f, 1.f)) / 2.f;

			buffer[3 * i + 0] = byte(c.z * 255.f);
			buffer[3 * i + 1] = byte(c.y * 255.f);
			buffer[3 * i + 2] = byte(c.x * 255.f);
		}
		imwrite(filename, img_colormap);
	};
	auto draw_eigenv_map = [&](const int w, const int h, const __float3* curvaturemap, const string filename)
	{
		double threshold_eigen = _fncontainer->fnParams.GetParam("_double_thresholdeigen", 0.333);
		Mat img_colormap = Mat(h, w, CV_8UC3);
		byte* buffer = (byte*)img_colormap.data;
		for (int i = 0; i < w * h; i++)
		{
			__float3 ev = curvaturemap[i];
			if (ev.x == 0 && ev.y == 0 && ev.z == 0)
			{
				buffer[3 * i + 0] = 0;
				buffer[3 * i + 1] = 0;
				buffer[3 * i + 2] = 0;
				continue;
			}
			float sum = ev.x + ev.y + ev.z;
			float shapev = ev.x / sum;

			float min_v = 0;
			float max_v = (float)threshold_eigen;
			float _value_sat = std::max(std::min(shapev, max_v), min_v);
			float ary_idx = (_value_sat - min_v) / (max_v - min_v) * (float)(__COLOR_ARRAY_SIZE - 1);
			vmbyte4 color = color_int_2_byte4(_color_array[(uint)ary_idx]);
			buffer[3 * i + 0] = color.z;
			buffer[3 * i + 1] = color.y;
			buffer[3 * i + 2] = color.x;
		}
		imwrite(filename, img_colormap);
	};
	auto draw_curvature_map = [&](const int w, const int h, const __float2* curvmap, const string filename)
	{
		double threshold_cv = _fncontainer->fnParams.GetParam("_double_thresholdcurvature", 5.0);
		Mat img_colormap = Mat(h, w, CV_8UC3);
		byte* buffer = (byte*)img_colormap.data;
		for (int i = 0; i < w * h; i++)
		{
			buffer[3 * i + 0] = 0;
			buffer[3 * i + 1] = 0;
			buffer[3 * i + 2] = 0;
			if (curvmap[i].x == -1.f && curvmap[i].y == -1.f)
				continue;

			float k1 = curvmap[i].x;
			float k2 = curvmap[i].y;
			float r = sqrt(k1*k1 + k2 * k2);
			if (r < (float)threshold_cv)
			{
				buffer[3 * i + 0] = 30;
				buffer[3 * i + 1] = 30;
				buffer[3 * i + 2] = 30;
				continue;
			}
			r = std::min(r, 1.f);

			float theta = VM_fPI / 4.f - atan2(k2, k1);

			//vmfloat3 c = vmfloat3(r, theta, 0);

			vmfloat3 c = vmfloat3(std::min(std::max(-r * cos(theta), 0.0f), 1.f), std::min(std::max(r*sin(theta), 0.0f), 1.f), std::min(std::max(r*cos(theta), 0.0f), 1.f));

			buffer[3 * i + 0] = byte(c.z * 255.f);
			buffer[3 * i + 1] = byte(c.y * 255.f);
			buffer[3 * i + 2] = byte(c.x * 255.f);
		}
		imwrite(filename, img_colormap);
	};

	draw_color_map(w, h, depthmap, "d:\\window_result\\depthmap_rgb.jpg", 0.3f, 2.f);
	//draw_color_map(w, h, depthgrad, "d:\\window_result\\depthgradmap_rgb.jpg", 0, 0.05f);
	draw_normal_map(w, h, normalmap, "d:\\window_result\\normalmap_rgb.jpg");
	draw_eigenv_map(w, h, eigenvaluemap, "d:\\window_result\\shapevmap_rgb.jpg");
	draw_curvature_map(w, h, curvmap, "d:\\window_result\\curvaturemap_rgb.jpg");

	imwrite("d:\\window_result\\test.jpg", rgb_img);
	imwrite("d:\\window_result\\rgbmap.jpg", rgb_map);
	//\\0704
	// cam center --> (0, 0, 0)
	// view dir

	// 1. 영상 depth 만들기.
	// 2. rgb 영상 읽기.
	// 3. rgb2depth map 만들기
	// 6. depth 에서 pca
	// 7. depth 에서 laplacian
	// 8. rgb 에서 line ... detection... 1) canny, 2) 허프, 3) ld

	// 2장 구조
	// 전체 구조 프레임...
	VMSAFE_DELETEARRAY(depthmap);
	VMSAFE_DELETEARRAY(indexmap);
	VMSAFE_DELETEARRAY(normalmap);
	VMSAFE_DELETEARRAY(eigenvaluemap);
	VMSAFE_DELETEARRAY(curvmap);
	return true;
}

void compute_camera_matrix_p(glm::dmat4x4& mat_p, const double fx, const double fy, const double ppx, const double ppy, const glm::dmat3x3& rm, const glm::dvec3& tv)
{
	using namespace glm;
	// 보통 column 단위로 채운다.
	dmat4x4 mat_k(fx, 0, 0, 0/**/, 0, fy, 0, 0/**/, ppx, ppy, 1, 0/**/, 0, 0, 0, 1); // ignore 4th row 

	const double *rv = (const double*)glm::value_ptr(rm);
	dmat4x4 mat_rt(rv[0], rv[1], rv[2], 0, rv[3], rv[4], rv[5], 0, rv[6], rv[7], rv[8], 0, tv.x, tv.y, tv.z, 1); // ignore 4th row 
	mat_p = mat_k * mat_rt;
}
void fill_organized_pointset_buffers(float* depthmap, int* indexmap, const int w, const int h,
	const glm::dmat4x4& mat_p, const std::vector<__float3>& pos_pts)
{
	const float max_depth = 1000000.f;
	for (int i = 0; i < w*h; i++) depthmap[i] = max_depth, indexmap[i] = -1;

	__float3 max_pos(-FLT_MAX, -FLT_MAX, -FLT_MAX), min_pos(FLT_MAX, FLT_MAX, FLT_MAX);
	for (int i = 0; i < (int)pos_pts.size(); i++)
	{
		const __float3& p = pos_pts[i];
		SELF_OP(max_pos.x, p.x, max);
		SELF_OP(max_pos.y, p.y, max);
		SELF_OP(max_pos.z, p.z, max);
		SELF_OP(min_pos.x, p.x, min);
		SELF_OP(min_pos.y, p.y, min);
		SELF_OP(min_pos.z, p.z, min);
	}

	cout << max_pos.x << ", " << max_pos.y << ", " << max_pos.z << endl;
	cout << min_pos.x << ", " << min_pos.y << ", " << min_pos.z << endl;

	// 방법 1 (역 추산)
	//const float fovy = 43.6551f;
	//vmmat44 mat_ws2cs, mat_cs2ps, mat_ps2ss;
	//MatrixWS2CS(&mat_ws2cs, &vmfloat3(0, 0, 0), &vmfloat3(0, -1.f, 0), &vmfloat3(0, 0, 1.f));
	//VXHMMatrixPerspectiveCS2PS(&mat_cs2ps, VM_PI / 180.f * fovy, (float)w / (float)h, 0.3f, 7.f);
	//fMatrixPS2SS(&mat_ps2ss, (float)w, (float)h);
	//vmmat44 mat_ws2ss = mat_ws2cs * mat_cs2ps * mat_ps2ss;
	// 방법 2 (카메라 파라미터로 바로..)
	//double fx, fy, ppx, ppy;
	//glm::dmat4x4 mat_k(fx, 0, ppx, 0, 0, fy, ppy, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	//glm::dmat4x4 mat_rt(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	//glm::dmat4x4 mat_p = mat_k * mat_rt;

	int outside_pixels = 0;
	//float min_d(FLT_MAX), max_d(0);
	for (int i = 0; i < (int)pos_pts.size(); i++)
	{
		const __float3& fp = pos_pts[i];
		glm::dvec4 p((double)fp.x, (double)fp.y, (double)fp.z, 1.0);
		p = mat_p * p;

		// it is noted that p.z is 2d homogeneous term as mat_p is 3x4 matrix setting that p.w always becomes 1
		float fx = (float)(p.x / p.z);
		float fy = (float)(p.y / p.z);
		int x = (int)fx;
		int y = (int)fy;

		auto store_dencemap = [&outside_pixels, &w, &h, &depthmap, &indexmap](int x, int y, float fz, int idx)
		{
			if (x < 0 || y < 0 || x >= w || y >= h)
				outside_pixels++;
			else
			{
				float d = depthmap[x + y * w];
				if (d > fz)
				{
					depthmap[x + y * w] = fz;
					indexmap[x + y * w] = idx;
				}
			}
		};
		store_dencemap(x, y, fp.z, i);
		store_dencemap(x + 1, y, fp.z, i);
		store_dencemap(x, y + 1, fp.z, i);
		store_dencemap(x + 1, y + 1, fp.z, i);

		//SELF_OP(min_d, fp.z, min);
		//SELF_OP(max_d, fp.z, max);
		//SELF_OP(depthmap[x + y * w], p.z, min);
	}
	cout << "outside pixels : " << outside_pixels << endl;
	//cout << "min d pixels : " << min_d << endl;
	//cout << "max d pixels : " << max_d << endl;
}
void fill_rgb_matching_map(byte* rgbmap, const int* indexmap, const int w, const int h,
	const byte* rgbimg, const int w1, const int h1,
	const glm::dmat4x4& mat_p, const std::vector<__float3>& pos_pts)
{
	auto load_byte_buffer = [&](int x, int y) -> vmfloat3
	{
		if (x < 0 || y < 0 || x >= w || y >= h) return vmfloat3(0);
		byte r = rgbimg[(x + y * w) * 3 + 0];
		byte g = rgbimg[(x + y * w) * 3 + 1];
		byte b = rgbimg[(x + y * w) * 3 + 2];
		return vmfloat3((float)r, (float)g, (float)b);
	};
	auto bilinear = [](vmfloat3 clrs[4], float ratio_x, float ratio_y) -> vmfloat3
	{
		vmfloat3 vclr0 = clrs[0] * (1.f - ratio_x) + clrs[1] * ratio_x;
		vmfloat3 vclr1 = clrs[2] * (1.f - ratio_x) + clrs[3] * ratio_x;
		return vclr0 * (1.f - ratio_y) + vclr1 * ratio_y;
	};

	// rgb cam 에서 occlusion 되는 거 나중에 고려해 보기!
	int testcnt = 0;
	for (int i = 0; i < w*h; i++)
	{
		int idx = indexmap[i];
		if (idx < 0) continue;

		const __float3& fp = pos_pts[idx];
		glm::dvec4 p((double)fp.x, (double)fp.y, (double)fp.z, 1.0);
		p = mat_p * p;
		float px = (float)p.x / (float)p.z;
		float py = (float)p.y / (float)p.z;
		float x = floor(px);
		float y = floor(py);
		float ratio_x = (float)px - x;
		float ratio_y = (float)py - y;
		if (ratio_x < 0) testcnt++;
		if (ratio_y < 0) testcnt++;
		int ix = (int)x;
		int iy = (int)y;

		vmfloat3 clrs[4];
		clrs[0] = load_byte_buffer(ix, iy);
		clrs[1] = load_byte_buffer(ix + 1, iy);
		clrs[2] = load_byte_buffer(ix, iy + 1);
		clrs[3] = load_byte_buffer(ix + 1, iy + 1);
		vmfloat3 clr = bilinear(clrs, ratio_x, ratio_y);
		rgbmap[i * 3 + 0] = min((byte)clr.x, (byte)255);
		rgbmap[i * 3 + 1] = min((byte)clr.y, (byte)255);
		rgbmap[i * 3 + 2] = min((byte)clr.z, (byte)255);
	}
	cout << "test pixels : " << testcnt << endl;
}

#define _DIM_ 3
auto __construct_covariance_matrix = [](const Eigen::VectorXd& cov) -> Eigen::MatrixXd
{
	Eigen::MatrixXd m(_DIM_, _DIM_);

	for (std::size_t i = 0; i < _DIM_; ++i)
	{
		for (std::size_t j = i; j < _DIM_; ++j)
		{
			m(i, j) = static_cast<float>(cov[(_DIM_ * i) + j - ((i * (i + 1)) / 2)]);

			if (i != j)
				m(j, i) = m(i, j);
		}
	}

	return m;
};
auto __diagonalize_selfadjoint_matrix = [](Eigen::MatrixXd& m, Eigen::MatrixXd& eigenvectors, Eigen::VectorXd& eigenvalues) -> bool
{
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver;

	//eigensolver.computeDirect(m);
	eigensolver.compute(m);

	if (eigensolver.info() != Eigen::Success)
		return false;

	eigenvalues = eigensolver.eigenvalues();
	eigenvectors = eigensolver.eigenvectors();

	return true;
};
auto __diagonalize_selfadjoint_covariance_matrix = [&]
(const Eigen::VectorXd& cov, float* eigenvalues, __float3* eigenvectors)
{
	Eigen::MatrixXd m = __construct_covariance_matrix(cov);

	// Diagonalizing the matrix
	Eigen::VectorXd eigenvalues_;
	Eigen::MatrixXd eigenvectors_;
	bool res = __diagonalize_selfadjoint_matrix(m, eigenvectors_, eigenvalues_);

	if (res)
	{
		for (std::size_t i = 0; i < _DIM_; ++i)
		{
			eigenvalues[i] = static_cast<float>(eigenvalues_[i]);

			for (std::size_t j = 0; j < _DIM_; ++j)
				((float*)eigenvectors)[_DIM_*i + j] = static_cast<float>(eigenvectors_(j, i));
		}
	}

	return res;
};

void normal_estimation(__float3* normalmap, __float3* eigenvaluemap, const int* indexmap, const byte* mask,
	const int w, const int h, const std::vector<__float3>& pos_pts, 
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt)
{
	const int offset = nbr_pixel_offset;
	memset(normalmap, 0, sizeof(__float3) * w*h);
	memset(eigenvaluemap, 0, sizeof(__float3) * w*h);

	DWORD dwTime;
	nanoflann::SearchParams params;
	PointCloud<float> pc_kdt(pos_pts);
	kd_tree_t kdt_index(3, pc_kdt, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	if (use_kdt)
	{
		dwTime = timeGetTime();
		kdt_index.buildIndex();
		cout << "==> kd tree build : " << timeGetTime() - dwTime << " ms" << endl;
		params.sorted = false;
	}

	int nb_cnt_min = 10000;
	int nb_cnt_max = 0;
	dwTime = timeGetTime();
	float r_sq = (float)(kernel_radius * kernel_radius);
	int procs_cnt = omp_get_num_procs();
#pragma omp parallel for num_threads( procs_cnt )
	for (int y = 0; y < h; y++)
	{
		int tid = omp_get_thread_num();
		if (tid == procs_cnt - 1)
		{
			float total_range = (float)h / (float)procs_cnt;
			float total_idx = (float)y - total_range * (float)tid;
			std::cout << "\rnormal estimation processing : " << (int)((total_idx) / total_range * 100.f) << " %" << std::flush;
		}

		for (int x = 0; x < w; x++)
		{
			int ipix = x + y * w;
			int idx = indexmap[ipix];
			byte _flag = mask ? mask[ipix] : 1;
			if (idx < 0 || _flag == 0) continue;

			__float3 pos_src = pos_pts[idx];
			// just use knn search!!!
			// 나중에 2D 연결된 것만 처리하는 것 시도 (이렇게 하는 건 좋은 아이디어가 아닐 듯... 거리에 따라 주변 kernel size 가 변한다..)

			std::vector<std::pair<size_t, float>> ret_matches;
			int nMatches = 0;
			if (use_kdt)
				nMatches = (int)kdt_index.radiusSearch((float*)&pos_src, r_sq, ret_matches, params);
			else
			{
				ret_matches.assign(((offset * 2) + 1) * ((offset * 2) + 1), std::pair<size_t, float>());
				for (int oy = max(y - offset, 0); oy <= min(y + offset, h - 1); oy++)
					for (int ox = max(x - offset, 0); ox <= min(x + offset, w - 1); ox++)
					{
						int idx_nb = indexmap[ox + oy * w];
						if (idx_nb < 0) continue;
						if (fLengthVectorSq(&(*(vmfloat3*)&pos_src - *(vmfloat3*)&pos_pts[idx_nb])) > r_sq) continue;
						ret_matches[nMatches++] = std::pair<size_t, float>((size_t)idx_nb, 0.f);
					}
			}
			if (nMatches < 3) continue;

			SELF_OP(nb_cnt_min, nMatches, min);
			SELF_OP(nb_cnt_max, nMatches, max);

			vmdouble3 pos_centroid = vmdouble3(0);
			double ft_num_ptns = (double)nMatches;
			for (int k = 0; k < nMatches; k++)
			{
				int idx_nb = (int)ret_matches[k].first;
				__float3 pos_nb = pos_pts[idx_nb];
				pos_centroid.x += ((double)pos_nb.x / ft_num_ptns);
				pos_centroid.y += ((double)pos_nb.y / ft_num_ptns);
				pos_centroid.z += ((double)pos_nb.z / ft_num_ptns);
			}

			__float3 pos_centroid_f = __float3((float)pos_centroid.x, (float)pos_centroid.y, (float)pos_centroid.z);
			//Eigen::VectorXd evecs(6); evecs << 0, 0, 0, 0, 0, 0;
			Eigen::VectorXd evecs = Eigen::VectorXd::Zero(6);
			for (int k = 0; k < nMatches; k++)
			{
				int idx_nb = (int)ret_matches[k].first;
				__float3 pos_nb = pos_pts[idx_nb];
				__float3 diff = __float3(pos_centroid_f.x - pos_nb.x, pos_centroid_f.y - pos_nb.y, pos_centroid_f.z - pos_nb.z);
				evecs(0) += diff.x * diff.x;
				evecs(1) += diff.x * diff.y;
				evecs(2) += diff.x * diff.z;
				evecs(3) += diff.y * diff.y;
				evecs(4) += diff.y * diff.z;
				evecs(5) += diff.z * diff.z;
			}

			float eigenvalues[3];
			__float3 eigenvectors[3];
			if (__diagonalize_selfadjoint_covariance_matrix(evecs, eigenvalues, eigenvectors))
			{
				// eigenvalues[0] is smallest
				if (eigenvaluemap)
					eigenvaluemap[ipix] = __float3(eigenvalues[0], eigenvalues[1], eigenvalues[2]);
				fNormalizeVector((vmfloat3*)&normalmap[ipix], (vmfloat3*)&eigenvectors[0]);
				if (fDotVector((vmfloat3*)&normalmap[ipix], &vmfloat3(0, 0, 1.f)) > 0)
					*(vmfloat3*)&normalmap[ipix] = -*(vmfloat3*)&normalmap[ipix];
			}
		}
	}
	std::cout << "\rnormal estimation processing : 100 %                   " << std::endl;
	cout << "==> PCA processing : " << timeGetTime() - dwTime << " ms" << endl;
	cout << "==> Min/Max # of Valid Neighbors : " << nb_cnt_min << ", " << nb_cnt_max << endl;
}

inline double b_spline_weight(double d, double R)
{
	if (d > R)
		return 0;

	//B-spline (degree = 2)
	else {
		d = 1.5*(d / R);
		if (d < 0.5)
			return (-d * d + 0.75);
		else
			return (0.5*(1.5 - d)*(1.5 - d));
	}
}

void compute_coeff_bi_variate(double uvw_coeff[7], vmmat44f& mat_frame,
	const __float3& pos_center, const __float3& nrl_center,
	const std::vector<__float3>& pos_pts, const std::vector<float>& weight_pts)
{
	vmfloat3 vec_proj_w = *(vmfloat3*)&nrl_center, vec_proj_u, vec_proj_v;
	if (vec_proj_w.x < vec_proj_w.y) vec_proj_u = vmfloat3(1.f, 0, 0);
	else vec_proj_u = vmfloat3(0, 1.f, 0);
	fCrossDotVector(&vec_proj_v, &vec_proj_w, &vec_proj_u);
	fCrossDotVector(&vec_proj_u, &vec_proj_v, &vec_proj_w);
	fNormalizeVector(&vec_proj_u, &vec_proj_u);
	fNormalizeVector(&vec_proj_v, &vec_proj_v);
	vmfloat3 vec_proj_up = vec_proj_v;
	fMatrixWS2CS(&mat_frame, (vmfloat3*)&pos_center, &vec_proj_v, &vec_proj_w);

	int num_pts = (int)pos_pts.size();
	std::vector<__float3> pos_frame_pts(num_pts);
	for (int i = 0; i < num_pts; i++)
		fTransformPoint((vmfloat3*)&pos_frame_pts[i], (vmfloat3*)&pos_pts[i], &mat_frame);

	vmfloat3 pos_frame_center;
	fTransformPoint(&pos_frame_center, (vmfloat3*)&pos_center, &mat_frame);

	const int dim = 6;
	int solve_degree = num_pts;
	Eigen::VectorXd F(solve_degree);
	Eigen::MatrixXd B(dim, solve_degree);
	//const double cell_radius = width_cell * 0.5 * 1.732;
	double w_sum = 0;

	// typeB (6 dim): a0 + a1*u + a2*v + a3*u*v + a4*u*u + a5*v*v;
	for (int i = 0; i < solve_degree; i++)
	{
		vmdouble3 uvw = *(vmfloat3*)&pos_frame_pts[i];

		double w = weight_pts[i];// b_spline_weight((double)fLengthVector(&(uvw - pos_frame_center)), cell_radius);

		F(i) = uvw.z * w;

		B(0, i) = w * 1.;
		B(1, i) = w * uvw.x;
		B(2, i) = w * uvw.y;
		B(3, i) = w * uvw.x * uvw.y;
		B(4, i) = w * uvw.x * uvw.x;
		B(5, i) = w * uvw.y * uvw.y;
		w_sum += w;
	}

	for (int i = 0; i < (int)pos_pts.size(); i++)
	{
		F(i) /= w_sum;
		for (int j = 0; j < dim; j++) B(j, i) /= w_sum;
	}

	Eigen::MatrixXd BT = B;
	BT.transposeInPlace();

	const double regulaized_lamda = 0;// 0.00000000001;
	//Eigen::MatrixXd M = B * BT;
	Eigen::MatrixXd M = B * BT + regulaized_lamda * Eigen::MatrixXd::Identity(dim, dim);
	Eigen::VectorXd _B = B * F;

	Eigen::VectorXd eB(dim);
	Eigen::MatrixXd eM(dim, dim);
	for (int i = 0; i < dim; i++)
	{
		eB(i) = _B(i);
		for (int j = 0; j < dim; j++)
		{
			eM(i, j) = M(i, j);
		}
	}

	//Eigen::JacobiSVD<Eigen_matrix::EigenType> eigenSvd(eM.eigen_object(), ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	Eigen::BDCSVD<Eigen::MatrixXd> eigenSvd(eM, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);
	eB = eigenSvd.solve(eB);
	//cout << "SVD solver for quadratic fitting : " << eigenSvd.singularValues().array().abs().maxCoeff() /
	//	eigenSvd.singularValues().array().abs().minCoeff() << endl;

	// typeB (6 dim): b0 + b1*u + b2*v + b3*u*v + b4*u*u + b5*v*v + "b6*w";
	//double uvw_coeff[7];
	for (int i = 0; i < dim; i++)
		uvw_coeff[i] = eB[i];
	uvw_coeff[6] = -1.f;

	// convert (u, v, w) to (x, y, z)
	// a0 + a1*x + a2*y + a3*z + a4*x*y + a5*y*z + a6*x*z + a7*x*x + a8*y*y + a9*z*z;
	//double t0 = mat_frame[3][0];
	//double t1 = mat_frame[3][1];
	//double t2 = mat_frame[3][2];
	//double m00 = mat_frame[0][0];
	//double m01 = mat_frame[1][0];
	//double m02 = mat_frame[2][0];
	//double m10 = mat_frame[0][1];
	//double m11 = mat_frame[1][1];
	//double m12 = mat_frame[2][1];
	//double m20 = mat_frame[0][2];
	//double m21 = mat_frame[1][2];
	//double m22 = mat_frame[2][2];
	//
	//coeff[0] = uvw_coeff[0] + uvw_coeff[1] * t0 + uvw_coeff[2] * t1 + uvw_coeff[3] * t0*t1 + uvw_coeff[4] * t0*t0 + uvw_coeff[5] * t1*t1 + uvw_coeff[6] * t2;
	//coeff[1] = uvw_coeff[1] * m00 + uvw_coeff[2] * m10 + uvw_coeff[3] * (t0*m10 + t1 * m00) + 2.*uvw_coeff[4] * m00*t0 + 2.*uvw_coeff[5] * m10*t1 + uvw_coeff[6] * m20;
	//coeff[2] = uvw_coeff[1] * m01 + uvw_coeff[2] * m11 + uvw_coeff[3] * (t0*m11 + t1 * m01) + 2.*uvw_coeff[4] * m01*t0 + 2.*uvw_coeff[5] * m11*t1 + uvw_coeff[6] * m21;
	//coeff[3] = uvw_coeff[1] * m02 + uvw_coeff[2] * m12 + uvw_coeff[3] * (t0*m12 + t1 * m02) + 2.*uvw_coeff[4] * m02*t0 + 2.*uvw_coeff[5] * m12*t1 + uvw_coeff[6] * m22;
	//coeff[4] = uvw_coeff[3] * (m00*m11 + m01 * m10) + 2.*uvw_coeff[4] * m00*m01 + 2.*uvw_coeff[5] * m10*m11;
	//coeff[5] = uvw_coeff[3] * (m01*m12 + m02 * m11) + 2.*uvw_coeff[4] * m01*m02 + 2.*uvw_coeff[5] * m11*m12;
	//coeff[6] = uvw_coeff[3] * (m02*m10 + m00 * m12) + 2.*uvw_coeff[4] * m02*m00 + 2.*uvw_coeff[5] * m12*m10;
	//coeff[7] = uvw_coeff[3] * m00*m10 + uvw_coeff[4] * m00*m00 + uvw_coeff[5] * m10*m10;
	//coeff[8] = uvw_coeff[3] * m01*m11 + uvw_coeff[4] * m01*m01 + uvw_coeff[5] * m11*m11;
	//coeff[9] = uvw_coeff[3] * m02*m12 + uvw_coeff[4] * m02*m02 + uvw_coeff[5] * m12*m12;
}

enum ApproxType
{
	__UNDEFINED_APPROX,
	__BI_VARIATE_APPROX,
	__TRI_VARIATE_APPROX,
	__TRI_VARIATE_COMPLEX
};

ApproxType estimate_localfit_type(const vmdouble3& pos_center_cell, const std::vector<__float3>& pos_cell_pts, const std::vector<__float3>& nrl_cell_pts, 
	const kd_tree_t& kdt_index, const std::vector<__float3>& pos_aux_pts, const int min_num_approx_pts, vmmat44f& mat_frame, vector<pair<__float3, float>>& valid_aux_pts)
{
	int num_pts = (int)nrl_cell_pts.size();
	if (num_pts < min_num_approx_pts) return ApproxType::__UNDEFINED_APPROX;

	nanoflann::SearchParams params;
	params.sorted = false;

	vmdouble3 _vec_proj_w = vmdouble3(0, 0, 0);
	for (int i = 0; i < num_pts; i++)
	{
		const __float3& _nrl = nrl_cell_pts[i];
		vmdouble3 nrl(_nrl.x, _nrl.y, _nrl.z);
		_vec_proj_w += nrl / (double)num_pts;
	}

	vmfloat3 vec_proj_w(_vec_proj_w), vec_proj_u, vec_proj_v;
	bool exist_opposite = false;
	for (int i = 0; i < num_pts; i++)
	{
		const __float3& _nrl = nrl_cell_pts[i];
		vmfloat3 nrl(_nrl.x, _nrl.y, _nrl.z);
		if (fDotVector(&vec_proj_w, (vmfloat3*)&nrl) < 0)
		{
			exist_opposite = true;
			break;
		}
	}
	if (!exist_opposite)
	{
		fNormalizeVector(&vec_proj_w, &vec_proj_w);
		if (vec_proj_w.x < vec_proj_w.y) vec_proj_u = vmfloat3(1.f, 0, 0);
		else vec_proj_u = vmfloat3(0, 1.f, 0);
		fCrossDotVector(&vec_proj_v, &vec_proj_w, &vec_proj_u);
		fCrossDotVector(&vec_proj_u, &vec_proj_v, &vec_proj_w);
		fNormalizeVector(&vec_proj_u, &vec_proj_u);
		fNormalizeVector(&vec_proj_v, &vec_proj_v);
		vmfloat3 vec_proj_up = vec_proj_v;
		fMatrixWS2CS(&mat_frame, &vmfloat3(pos_center_cell), &vec_proj_v, &vec_proj_w);
		return ApproxType::__BI_VARIATE_APPROX;
	}

	// range point set 에서는 __TRI_VARIATE_COMPLEX 가 발생하지 않는다!
	// check ApproxType::__TRI_VARIATE_COMPLEX 
	for (int i = 0; i < (int)pos_aux_pts.size(); i++)
	{
		vmfloat3 pos_src = *(vmfloat3*)&pos_aux_pts[i];

		size_t nb_idx[6];
		float nb_distsq[6];
		const int nMatches = (int)kdt_index.knnSearch((float*)&pos_src, 6, nb_idx, nb_distsq);

		double dot_dirs_prev = 0;
		double dist = 0;
		int valid_nbr_count = 0;
		for (int j = 0; j < nMatches; j++)
		{
			const int idx = (const int)nb_idx[j];
			vmfloat3 pos_nbr = *(vmfloat3*)&pos_cell_pts[idx];
			const __float3& nrl_bnr = nrl_cell_pts[idx];

			double dot_dirs = fDotVector((vmfloat3*)&(nrl_cell_pts[idx]), &(pos_src - pos_nbr));
			if (dot_dirs_prev * dot_dirs < 0)
				break;

			dot_dirs_prev = dot_dirs;
			dist += (double)dot_dirs;
			valid_nbr_count++;
		}
		if (valid_nbr_count == 6)
			valid_aux_pts.push_back(pair<__float3, float>(*(__float3*)&pos_src, (float)(dist / 6.)));
	}

	if (valid_aux_pts.size() == 0)
		return ApproxType::__TRI_VARIATE_COMPLEX;

	return ApproxType::__TRI_VARIATE_APPROX;
}

inline double compute_approx_frame_f_value(const __float3& _pos, const double coeff[7])
{
	// typeB (6 dim): b0 + b1*u + b2*v + b3*u*v + b4*u*u + b5*v*v + "b6*w";
	vmdouble3 pos(_pos.x, _pos.y, _pos.z);
	return coeff[0] + coeff[1] * pos.x + coeff[2] * pos.y
		+ coeff[3] * pos.x * pos.y + coeff[4] * pos.x * pos.x + coeff[5] * pos.y * pos.y + coeff[6] * pos.z;
}
inline double compute_approx_frame_f_value(const vmfloat3& pos, const double coeff[7])
{
	return compute_approx_frame_f_value(*(__float3*)&pos, coeff);
}

void convert_uvw_xyz_coeffs(double xyz_coeff[10], const double uvw_coeff[7], const vmmat44f& mat_frame)
{
	// convert (u, v, w) to (x, y, z)
	// a0 + a1*x + a2*y + a3*z + a4*x*y + a5*y*z + a6*x*z + a7*x*x + a8*y*y + a9*z*z;
	double t0 = mat_frame[3][0];
	double t1 = mat_frame[3][1];
	double t2 = mat_frame[3][2];
	double m00 = mat_frame[0][0];
	double m01 = mat_frame[1][0];
	double m02 = mat_frame[2][0];
	double m10 = mat_frame[0][1];
	double m11 = mat_frame[1][1];
	double m12 = mat_frame[2][1];
	double m20 = mat_frame[0][2];
	double m21 = mat_frame[1][2];
	double m22 = mat_frame[2][2];
	//double uvw_coeff[7] = { _uvw_coeff[0], _uvw_coeff[1], _uvw_coeff[2], _uvw_coeff[3], _uvw_coeff[4], _uvw_coeff[5], _uvw_coeff[6] };

	xyz_coeff[0] = uvw_coeff[0] + uvw_coeff[1] * t0 + uvw_coeff[2] * t1 + uvw_coeff[3] * t0*t1 + uvw_coeff[4] * t0*t0 + uvw_coeff[5] * t1*t1 + uvw_coeff[6] * t2;
	xyz_coeff[1] = uvw_coeff[1] * m00 + uvw_coeff[2] * m10 + uvw_coeff[3] * (t0*m10 + t1 * m00) + 2.*uvw_coeff[4] * m00*t0 + 2.*uvw_coeff[5] * m10*t1 + uvw_coeff[6] * m20;
	xyz_coeff[2] = uvw_coeff[1] * m01 + uvw_coeff[2] * m11 + uvw_coeff[3] * (t0*m11 + t1 * m01) + 2.*uvw_coeff[4] * m01*t0 + 2.*uvw_coeff[5] * m11*t1 + uvw_coeff[6] * m21;
	xyz_coeff[3] = uvw_coeff[1] * m02 + uvw_coeff[2] * m12 + uvw_coeff[3] * (t0*m12 + t1 * m02) + 2.*uvw_coeff[4] * m02*t0 + 2.*uvw_coeff[5] * m12*t1 + uvw_coeff[6] * m22;
	xyz_coeff[4] = uvw_coeff[3] * (m00*m11 + m01 * m10) + 2.*uvw_coeff[4] * m00*m01 + 2.*uvw_coeff[5] * m10*m11;
	xyz_coeff[5] = uvw_coeff[3] * (m01*m12 + m02 * m11) + 2.*uvw_coeff[4] * m01*m02 + 2.*uvw_coeff[5] * m11*m12;
	xyz_coeff[6] = uvw_coeff[3] * (m02*m10 + m00 * m12) + 2.*uvw_coeff[4] * m02*m00 + 2.*uvw_coeff[5] * m12*m10;
	xyz_coeff[7] = uvw_coeff[3] * m00*m10 + uvw_coeff[4] * m00*m00 + uvw_coeff[5] * m10*m10;
	xyz_coeff[8] = uvw_coeff[3] * m01*m11 + uvw_coeff[4] * m01*m01 + uvw_coeff[5] * m11*m11;
	xyz_coeff[9] = uvw_coeff[3] * m02*m12 + uvw_coeff[4] * m02*m02 + uvw_coeff[5] * m12*m12;
}

void curvature_estimation_ver1(__float2* curvaturemap, const int* indexmap, const __float3* normalmap, const byte* mask,
	const int w, const int h, const std::vector<__float3>& pos_pts, 
	const double kernel_radius, const int nbr_pixel_offset, const bool use_kdt, const bool use_numerical)
{
	// umbrella curvature
	const int offset = nbr_pixel_offset;
	for (int i = 0; i < w*h; i++) curvaturemap[i] = __float2(-1.f, -1.f);

	DWORD dwTime;
	nanoflann::SearchParams params;
	PointCloud<float> pc_kdt(pos_pts);
	kd_tree_t kdt_index(3, pc_kdt, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	if (use_kdt)
	{
		dwTime = timeGetTime();
		kdt_index.buildIndex();
		cout << "==> kd tree build : " << timeGetTime() - dwTime << " ms" << endl;
		params.sorted = false;
	}

	int nb_cnt_min = 10000;
	int nb_cnt_max = 0;
	dwTime = timeGetTime();
	float r_sq = (float)(kernel_radius * kernel_radius);
	int procs_cnt = omp_get_num_procs();
#pragma omp parallel for num_threads( procs_cnt )
	for (int y = 0; y < h; y++)
	{
		int tid = omp_get_thread_num();
		if (tid == procs_cnt - 1)
		{
			float total_range = (float)h / (float)procs_cnt;
			float total_idx = (float)y - total_range * (float)tid;
			std::cout << "\rcurvature estimation processing : " << (int)((total_idx) / total_range * 100.f) << " %" << std::flush;
		}

		for (int x = 0; x < w; x++)
		{
			int ipix = x + y * w;
			int idx = indexmap[ipix];
			byte _flag = mask ? mask[ipix] : 1;
			if (idx < 0 || _flag == 0) continue;

			__float3 pos_src = pos_pts[idx];
			// just use knn search!!!
			std::vector<std::pair<size_t, float>> ret_matches;
			int nMatches = 0;
			if (use_kdt)
				nMatches = (int)kdt_index.radiusSearch((float*)&pos_src, r_sq, ret_matches, params);
			else
			{
				ret_matches.assign(((offset * 2) + 1) * ((offset * 2) + 1), std::pair<size_t, float>());
				for (int oy = max(y - offset, 0); oy <= min(y + offset, h - 1); oy++)
					for (int ox = max(x - offset, 0); ox <= min(x + offset, w - 1); ox++)
					{
						int idx_nb = indexmap[ox + oy * w];
						if (idx_nb < 0) continue;
						if (fLengthVectorSq(&(*(vmfloat3*)&pos_src - *(vmfloat3*)&pos_pts[idx_nb])) > r_sq) continue;
						ret_matches[nMatches++] = std::pair<size_t, float>((size_t)idx_nb, 0.f);
					}
			}

			if (nMatches < 3) continue;

			SELF_OP(nb_cnt_min, nMatches, min);
			SELF_OP(nb_cnt_max, nMatches, max);

			std::vector<__float3> pos_nbrs(nMatches);
			std::vector<float> weight_nbrs(nMatches);
			for (int k = 0; k < nMatches; k++)
			{
				pos_nbrs[k] = pos_pts[(int)ret_matches[k].first];
				weight_nbrs[k] = 1.f;
			}

			double uvw_coeff[7] = {0, 0, 0, 0, 0, 0, 0};
			vmmat44f mat_frame;
			compute_coeff_bi_variate(uvw_coeff, mat_frame, pos_src, normalmap[ipix], pos_nbrs, weight_nbrs);
			vmfloat3 pos_src_frame;
			fTransformPoint(&pos_src_frame, (vmfloat3*)&pos_src, &mat_frame);

			float fxx, fxy, fyy, fxz, fyz, fzz, gm;
			vmfloat3 g, n;
			if (use_numerical)
			{
				// numerical analysis
				auto compute_grad = [&](vmfloat3 p)
				{
					vmfloat3 g;
					g.x = (float)(uvw_coeff[1] + uvw_coeff[3] * p.y + 2.f * uvw_coeff[4] * p.x);
					g.y = (float)(uvw_coeff[2] + uvw_coeff[3] * p.x + 2.f * uvw_coeff[5] * p.y);
					g.z = (float)(uvw_coeff[6]);
					return g;
				};
				g = compute_grad(pos_src_frame);
				gm = fLengthVector(&g);

				// find hit surface //
				for (int k = 0; k < 5; k++)
				{
					pos_src_frame -= ((float)compute_approx_frame_f_value(*(__float3*)&pos_src_frame, uvw_coeff) / gm) * (g / gm);
					g = compute_grad(pos_src_frame);
					gm = fLengthVector(&g);
				}
				n = g / gm;

				fxx = (float)(2. * uvw_coeff[4]);
				fxy = (float)uvw_coeff[3];
				fyy = (float)(2. * uvw_coeff[5]);
				fxz = 0;
				fyz = 0;
				fzz = 0;
			}
			else
			{
				const float vd_offset = 0.01f;
				vmfloat3 dirs[3] = { vmfloat3(vd_offset, 0, 0) , vmfloat3(0, vd_offset, 0) , vmfloat3(0, 0, vd_offset) };
				fTransformVector(&dirs[0], &dirs[0], &mat_frame);
				fTransformVector(&dirs[1], &dirs[1], &mat_frame);
				fTransformVector(&dirs[2], &dirs[2], &mat_frame);
				//fNormalizeVector(&dirs[0], &dirs[0]);
				//fNormalizeVector(&dirs[1], &dirs[1]);
				//fNormalizeVector(&dirs[2], &dirs[2]);
				double fv = compute_approx_frame_f_value(pos_src_frame, uvw_coeff);
				double fv_XXR = compute_approx_frame_f_value(pos_src_frame + 2.f * dirs[0], uvw_coeff);
				double fv_XXL = compute_approx_frame_f_value(pos_src_frame - 2.f * dirs[0], uvw_coeff);
				double fv_YYR = compute_approx_frame_f_value(pos_src_frame + 2.f * dirs[1], uvw_coeff);
				double fv_YYL = compute_approx_frame_f_value(pos_src_frame - 2.f * dirs[1], uvw_coeff);
				double fv_ZZR = compute_approx_frame_f_value(pos_src_frame + 2.f * dirs[2], uvw_coeff);
				double fv_ZZL = compute_approx_frame_f_value(pos_src_frame - 2.f * dirs[2], uvw_coeff);
				double fv_XR = compute_approx_frame_f_value(pos_src_frame + dirs[0], uvw_coeff);
				double fv_XL = compute_approx_frame_f_value(pos_src_frame - dirs[0], uvw_coeff);
				double fv_YR = compute_approx_frame_f_value(pos_src_frame + dirs[1], uvw_coeff);
				double fv_YL = compute_approx_frame_f_value(pos_src_frame - dirs[1], uvw_coeff);
				double fv_ZR = compute_approx_frame_f_value(pos_src_frame + dirs[2], uvw_coeff);
				double fv_ZL = compute_approx_frame_f_value(pos_src_frame - dirs[2], uvw_coeff);
				double fv_XRYR = compute_approx_frame_f_value(pos_src_frame + dirs[0] + dirs[1], uvw_coeff);
				double fv_XRYL = compute_approx_frame_f_value(pos_src_frame + dirs[0] - dirs[1], uvw_coeff);
				double fv_XLYR = compute_approx_frame_f_value(pos_src_frame - dirs[0] + dirs[1], uvw_coeff);
				double fv_XLYL = compute_approx_frame_f_value(pos_src_frame - dirs[0] - dirs[1], uvw_coeff);
				double fv_YRZR = compute_approx_frame_f_value(pos_src_frame + dirs[1] + dirs[2], uvw_coeff);
				double fv_YRZL = compute_approx_frame_f_value(pos_src_frame + dirs[1] - dirs[2], uvw_coeff);
				double fv_YLZR = compute_approx_frame_f_value(pos_src_frame - dirs[1] + dirs[2], uvw_coeff);
				double fv_YLZL = compute_approx_frame_f_value(pos_src_frame - dirs[1] - dirs[2], uvw_coeff);
				double fv_XRZR = compute_approx_frame_f_value(pos_src_frame + dirs[0] + dirs[2], uvw_coeff);
				double fv_XRZL = compute_approx_frame_f_value(pos_src_frame + dirs[0] - dirs[2], uvw_coeff);
				double fv_XLZR = compute_approx_frame_f_value(pos_src_frame - dirs[0] + dirs[2], uvw_coeff);
				double fv_XLZL = compute_approx_frame_f_value(pos_src_frame - dirs[0] - dirs[2], uvw_coeff);

				g = vmfloat3((float)(fv_XR - fv_XL), (float)(fv_YR - fv_YL), (float)(fv_ZR - fv_ZL));
				gm = fLengthVector(&g);
				n = g / gm;

				fxx = (float)(fv_XXR - 2. * fv + fv_XXL);
				fxy = (float)(fv_XRYR - fv_XLYR - fv_XRYL + fv_XLYL);
				fyy = (float)(fv_YYR - 2. * fv + fv_YYL);
				fxz = (float)(fv_XRZR - fv_XLZR - fv_XRZL + fv_XLZL);
				fyz = (float)(fv_YRZR - fv_YLZR - fv_YRZL + fv_YLZL);
				fzz = (float)(fv_ZZR - 2. * fv + fv_ZZL);
			}
	
			vmmat44f matH, P;
			matH[0][0] = fxx;
			matH[0][1] = fxy;
			matH[0][2] = fxz;
			matH[1][0] = matH[0][1];
			matH[1][1] = fyy;
			matH[1][2] = fyz;
			matH[2][0] = matH[0][2];
			matH[2][1] = matH[1][2];
			matH[2][2] = fzz;
			matH[3][3] = 1.f;
			P[0][0] = 1.0f;
			P[1][1] = 1.0f;
			P[2][2] = 1.0f;
			P[3][3] = 1.0f;

			vmmat44f nnT;
			nnT[0][0] = n.x*n.x;
			nnT[0][1] = n.x*n.y;
			nnT[0][2] = n.x*n.z;
			nnT[1][0] = n.y*n.x;
			nnT[1][1] = n.y*n.y;
			nnT[1][2] = n.y*n.z;
			nnT[2][0] = n.z*n.x;
			nnT[2][1] = n.z*n.y;
			nnT[2][2] = n.z*n.z;
			nnT[3][3] = 1.f;
			P = P - nnT;

			vmmat44f G = (P * matH) * P / gm;
			float T = G[0][0] + G[1][1] + G[2][2]; // trace of G
			float F = sqrt(G[0][0] * G[0][0] + G[0][1] * G[0][1] + G[0][2] * G[0][2]
				+ G[1][0] * G[1][0] + G[1][1] * G[1][1] + G[1][2] * G[1][2]
				+ G[2][0] * G[2][0] + G[2][1] * G[2][1] + G[2][2] * G[2][2]);

			float k1 = (T + sqrt(2.f * F*F - T * T)) / 2.f;
			float k2 = (T - sqrt(2.f * F*F - T * T)) / 2.f;
			curvaturemap[ipix] = __float2(k1, k2);
		}
	}
	std::cout << "\rcurvature estimation processing : 100 %                   " << std::endl;
	cout << "==> Curvature processing : " << timeGetTime() - dwTime << " ms" << endl;
	cout << "==> Min/Max # of Valid Neighbors : " << nb_cnt_min << ", " << nb_cnt_max << endl;
}

void curvature_estimation_ver2(__float3* curvaturemap, const int* indexmap,
	const int w, const int h, const double kernel_radius, const std::vector<__float3>& pos_pts)
{

}

//void grad_depthmap(float* metricmap, const float* depthmap, const int* indexmap, 
//	const int w, const int h, const int kernel_offset, const std::vector<__float3>& pos_pts)
//{
//	auto loadpixel_clamp = [&](int x, int y) -> float
//	{
//		int sx = max(min(x, w - 1), 0);
//		int sy = max(min(y, h - 1), 0);
//		return depthmap[sx + sy * w];
//	};
//	auto grad = [&](int x, int y, int offset) -> glm::dvec2
//	{
//		float dx = loadpixel_clamp(x + offset, y) - loadpixel_clamp(x - offset, y);
//		float dy = loadpixel_clamp(x, y + offset) - loadpixel_clamp(x, y - offset);
//		return glm::dvec2(dx, dy);
//	};
//
//	for(int y = 0; y < h; y++)
//		for (int x = 0; x < w; x++)
//		{
//			metricmap[x + y * w] = (float)glm::length(grad(x, y, kernel_offset));
//		}
//}
void laplacian_depthmap(float* metricmap, const float* depthmap, const int w, const int h, const int kernel_offset)
{

}