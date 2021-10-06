#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <mutex>
#include <thread>
#include "ros/ros.h"
#include <unordered_map>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>

#define MIN_PS 5
#define SMALL_EPS 1e-10
#define HASH_P 116101
#define MAX_N 10000000019

class VOXEL_LOC
{
public:
	int64_t x, y, z;

	VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0): x(vx), y(vy), z(vz){}

	bool operator== (const VOXEL_LOC &other) const
	{
		return (x == other.x && y == other.y && z == other.z);
	}
};

namespace std
{
	template<>
	struct hash<VOXEL_LOC>
	{
		size_t operator() (const VOXEL_LOC &s) const
		{
			using std::size_t;
			using std::hash;
			long long index_x, index_y, index_z;
			double cub_len = 1.0/8;
			index_x = int(round(floor((s.x)/cub_len + SMALL_EPS))); 
			index_y = int(round(floor((s.y)/cub_len + SMALL_EPS))); 
			index_z = int(round(floor((s.z)/cub_len + SMALL_EPS))); 
			return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) + index_x) % MAX_N;
		}
	};
}

struct M_POINT
{
	float xyz[3];
	float intensity;
	int count = 0;
};

void downsample_voxel(pcl::PointCloud<PointType>& pc, double voxel_size)
{
	if (voxel_size < 0.01)
		return;

	std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
	size_t pt_size = pc.size();

	for (size_t i = 0; i < pt_size; i++)
	{
		PointType &pt_trans = pc[i];
		float loc_xyz[3];
		for (int j = 0; j < 3; j++)
		{
			loc_xyz[j] = pt_trans.data[j] / voxel_size;
			if (loc_xyz[j] < 0)
				loc_xyz[j] -= 1.0;
		}

		VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
		auto iter = feature_map.find(position);
		if (iter != feature_map.end())
		{
			iter->second.xyz[0] += pt_trans.x;
			iter->second.xyz[1] += pt_trans.y;
			iter->second.xyz[2] += pt_trans.z;
			iter->second.intensity += pt_trans.intensity;
			iter->second.count++;
		}
		else
		{
			M_POINT anp;
			anp.xyz[0] = pt_trans.x;
			anp.xyz[1] = pt_trans.y;
			anp.xyz[2] = pt_trans.z;
			anp.intensity = pt_trans.intensity;
			anp.count = 1;
			feature_map[position] = anp;
		}
	}

	pt_size = feature_map.size();
	pc.clear();
	pc.resize(pt_size);

	size_t i = 0;
	for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
	{
		pc[i].x = iter->second.xyz[0] / iter->second.count;
		pc[i].y = iter->second.xyz[1] / iter->second.count;
		pc[i].z = iter->second.xyz[2] / iter->second.count;
		pc[i].intensity = iter->second.intensity / iter->second.count;
		i++;
	}
}

Eigen::Matrix3d wedge(const Eigen::Vector3d& v)
{
	Eigen::Matrix3d V;
	V << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return V;
}

Eigen::Quaterniond exp(const Eigen::Vector3d& omega)
{
	double theta = omega.norm();
	double half_theta = 0.5 * theta;

	double imag_factor;
	double real_factor = cos(half_theta);
	if (theta < SMALL_EPS)
	{
		double theta_sq = theta * theta;
		double theta_po4 = theta_sq * theta_sq;
		imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
	}
	else
	{
		double sin_half_theta = sin(half_theta);
		imag_factor = sin_half_theta / theta;
	}

	return Eigen::Quaterniond(real_factor, imag_factor * omega.x(),
					   imag_factor * omega.y(), imag_factor * omega.z());
}

void assign_qt(Eigen::Quaterniond& q, Eigen::Vector3d& t,
               const Eigen::Quaterniond& q_, const Eigen::Vector3d& t_)
{
    q.w() = q_.w(); q.x() = q_.x(); q.y() = q_.y(); q.z() = q_.z();
    t(0) = t_(0); t(1) = t_(1); t(2) = t_(2);
}

void assign_q(Eigen::Quaterniond& q, const Eigen::Quaterniond& q_)
{
    q.w() = q_.w(); q.x() = q_.x(); q.y() = q_.y(); q.z() = q_.z();
}

void assign_t(Eigen::Vector3d& t, const Eigen::Vector3d& t_)
{
    t(0) = t_(0); t(1) = t_(1); t(2) = t_(2);
}

class LM_OPTIMIZER
{
public:
	int pose_size, ref_size, jacob_len;
	vector_quad poses, posesTmp, refQs, refQsTmp;
	vector_vec3d ts, tsTmp, refTs, refTsTmp;
	std::vector<vector_vec3d*> baseOriginPts;
	std::vector<std::vector<int>*> baseWinNums;
	std::vector<std::vector<vector_vec3d*>> refOriginPts;
	std::vector<std::vector<std::vector<int>*>> refWinNums;
	ros::NodeHandle _nh;
    ros::Publisher pub_surf;
    ros::Publisher pub_surf_debug;
	
	LM_OPTIMIZER(size_t win_sz, size_t r_sz) : pose_size(win_sz), ref_size(r_sz)
	{
		jacob_len = (pose_size + ref_size) * 6;
		poses.resize(pose_size);
		ts.resize(pose_size);
		posesTmp.resize(pose_size);
		tsTmp.resize(pose_size);
		refQs.resize(ref_size);
		refQsTmp.resize(ref_size);
		refTs.resize(ref_size);
		refTsTmp.resize(ref_size);
		refOriginPts.resize(ref_size);
		refWinNums.resize(ref_size);
		pub_surf = _nh.advertise<sensor_msgs::PointCloud2>("/map_surf", 100);
		pub_surf_debug = _nh.advertise<sensor_msgs::PointCloud2>("/debug_surf", 100);
	};

	void get_center(vector_vec3d& originPc, int cur_frame, vector_vec3d& originPt,
					std::vector<int>& winNum, int filterNum)
	{
		size_t pt_size = originPc.size();
		if (pt_size <= (size_t)filterNum)
		{
			for (size_t i = 0; i < pt_size; i++)
			{
				originPt.emplace_back(originPc[i]);
				winNum.emplace_back(cur_frame);
			}
			return;
		}

		Eigen::Vector3d center;
		double part = 1.0 * pt_size / filterNum;

		for (int i = 0; i < filterNum; i++)
		{
			size_t np = part * i;
			size_t nn = part * (i + 1);
			center.setZero();
			for (size_t j = np; j < nn; j++)
				center += originPc[j];

			center = center / (nn - np);
			originPt.emplace_back(center);
			winNum.emplace_back(cur_frame);
		}
	}

	void push_voxel(std::vector<vector_vec3d*>& baseOriginPc,
					std::vector<std::vector<vector_vec3d*>>& refOriginPc)
	{
		int baseLidarPc = 0;
		for (int i = 0; i < pose_size; i++)
			if (!baseOriginPc[i]->empty())
				baseLidarPc++;
		
		int refLidarPc = 0;
		for (int j = 0; j < ref_size; j++)
			for (int i = 0; i < pose_size; i++)
				if (!refOriginPc[j][i]->empty())
					refLidarPc++;

		if (refLidarPc + baseLidarPc <= 1)
			return;

		int filterNum = 4;
		int baseLidarPt = 0;
		int refLidarPt = 0;
		
		vector_vec3d* baseOriginPt = new vector_vec3d();
		std::vector<int>* baseWinNum = new std::vector<int>();
		baseWinNum->reserve(filterNum * pose_size);
		baseOriginPt->reserve(filterNum * pose_size);
		for (int i = 0; i < pose_size; i++)
			if (!baseOriginPc[i]->empty())
				get_center(*baseOriginPc[i], i, *baseOriginPt, *baseWinNum, filterNum);

		baseLidarPt += baseOriginPt->size();
		baseOriginPts.emplace_back(baseOriginPt); // Note they might be empty
		baseWinNums.emplace_back(baseWinNum);
		assert(baseOriginPt->size()==baseWinNum->size());

		for (int j = 0; j < ref_size; j++)
		{
			vector_vec3d* refOriginPt = new vector_vec3d();
			std::vector<int>* refWinNum = new std::vector<int>();
			refWinNum->reserve(filterNum * pose_size);
			refOriginPt->reserve(filterNum * pose_size);
			for (int i = 0; i < pose_size; i++)
				if (!refOriginPc[j][i]->empty())
					get_center(*refOriginPc[j][i], i, *refOriginPt, *refWinNum, filterNum);

			refLidarPt += refOriginPt->size();
			refOriginPts[j].emplace_back(refOriginPt);
			refWinNums[j].emplace_back(refWinNum);
		}

		for (int j = 0; j < ref_size; j++)
		{
			assert(refOriginPts[j].size()==baseOriginPts.size());
			assert(refWinNums[j].size()==baseWinNums.size());
		}
	}

	void optimize()
	{
		double u = 0.001, v = 2;
		Eigen::MatrixXd D(jacob_len, jacob_len), Hess(jacob_len, jacob_len), _Hess(ref_size*6, ref_size*6);
		Eigen::VectorXd JacT(jacob_len), JacT3(jacob_len), dxi(jacob_len), debug_H(jacob_len);

		Eigen::MatrixXd Hess2(jacob_len, jacob_len);
		Eigen::VectorXd JacT2(jacob_len);

		vector_quad RQs; RQs.resize(ref_size);
		vector_vec3d RTs; RTs.resize(ref_size);
		double delta = 1e-8;
		Eigen::Vector3d dx(delta, 0, 0), dy(0, delta, 0), dz(0, 0, delta);

		D.setIdentity();
		double residual1, residual2, q;
		bool is_calc_hess = true;

		cv::Mat matA(jacob_len, jacob_len, CV_64F, cv::Scalar::all(0));
		cv::Mat matB(jacob_len, 1, CV_64F, cv::Scalar::all(0));
		cv::Mat matX(jacob_len, 1, CV_64F, cv::Scalar::all(0));

		for (int loop = 0; loop < 20; loop++)
		{
			if (is_calc_hess)
				divide_thread(poses, ts, refQs, refTs, Hess, JacT, residual1);

			D = Hess.diagonal().asDiagonal();
			Hess2 = Hess + u * D;

			for (int j = 0; j < jacob_len; j++)
			{
				matB.at<double>(j, 0) = -JacT(j, 0);
				for (int f = 0; f < jacob_len; f++)
					matA.at<double>(j, f) = Hess2(j, f);
			}
			cv::solve(matA, matB, matX, cv::DECOMP_QR);

			for (int j = 0; j < jacob_len; j++)
				dxi(j, 0) = matX.at<double>(j, 0);

			for (int i = 0; i < pose_size; i++)
			{
				Eigen::Quaterniond q_tmp(exp(dxi.block<3, 1>(6*i, 0)) * poses[i]);
				Eigen::Vector3d t_tmp(dxi.block<3, 1>(6*i+3, 0) + ts[i]);
				assign_qt(posesTmp[i], tsTmp[i], q_tmp, t_tmp);
			}
			for (int i = pose_size; i < pose_size+ref_size; i++)
			{
				Eigen::Quaterniond q_tmp(exp(dxi.block<3, 1>(6*i, 0)) * refQs[i-pose_size]);
				Eigen::Vector3d t_tmp(dxi.block<3, 1>(6*i+3, 0) + refTs[i-pose_size]);
				assign_qt(refQsTmp[i-pose_size], refTsTmp[i-pose_size], q_tmp, t_tmp);
			}

			double q1 = 0.5 * (dxi.transpose() * (u * D * dxi - JacT))[0];
			evaluate_only_residual(posesTmp, tsTmp, refQsTmp, refTsTmp, residual2);

			q = (residual1 - residual2);
			// printf("residual %d: %lf %lf u: %lf v: %lf q: %lf %lf %lf\n",
			// 	   loop, residual1, residual2, u, v, q/q1, q1, q);
			assert(!std::isnan(residual1));
			assert(!std::isnan(residual2));

			if (q > 0)
			{
				for (int i = 0; i < pose_size; i++)
					assign_qt(poses[i], ts[i], posesTmp[i], tsTmp[i]);
				for (int i = 0; i < ref_size; i++)
					assign_qt(refQs[i], refTs[i], refQsTmp[i], refTsTmp[i]);
				q = q / q1;
				v = 2;
				q = 1 - pow(2*q-1, 3);
				u *= (q < 1.0/3 ? 1.0/3:q);
				is_calc_hess = true;
			}
			else
			{
				u = u * v;
				v = 2 * v;
				is_calc_hess = false;
			}

			if (fabs(residual1 - residual2) < 1e-9)
				break;
		}
	}

	void divide_thread(vector_quad& poses, vector_vec3d& ts, vector_quad& refQs, vector_vec3d& refTs,
					   Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual)
	{
		Hess.setZero(); JacT.setZero(); residual = 0;

		std::vector<Eigen::MatrixXd> hessians(4, Hess);
		std::vector<Eigen::VectorXd> jacobians(4, JacT);
		std::vector<double> resis(4, 0);

		uint gps_size = baseOriginPts.size();
		if (gps_size < (uint)4)
		{
			calculate_HJ(poses, ts, refQs, refTs, 0, gps_size, Hess, JacT, residual);
			return;
		}

		std::vector<std::thread*> mthreads(4);

		double part = 1.0*(gps_size)/4;
		for (int i=0; i<4; i++)
		{
			int np = part*i;
			int nn = part*(i+1);
			
			mthreads[i] = new std::thread(&LM_OPTIMIZER::calculate_HJ, this, std::ref(poses), std::ref(ts),
				std::ref(refQs), std::ref(refTs), np, nn,
				std::ref(hessians[i]), std::ref(jacobians[i]), std::ref(resis[i]));
		}

		for (int i=0; i<4; i++)
		{
			mthreads[i]->join();
			Hess += hessians[i];
			JacT += jacobians[i];
			residual += resis[i];
			delete mthreads[i];
		}
	}

	void calculate_HJ(vector_quad& poses, vector_vec3d& ts, vector_quad& refQs, vector_vec3d& refTs,
					  int head, int end, Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual)
	{
		Hess.setZero();
		JacT.setZero();
		residual = 0;
		Eigen::MatrixXd _jact(JacT);

		for (size_t i = head; i < end; i++)
		{
			vector_vec3d& baseorigin_pts = *baseOriginPts[i];
			std::vector<int>& basewin_num = *baseWinNums[i];
			size_t basepts_size = baseorigin_pts.size();

			size_t refpts_size_all = 0;
			std::vector<int>refpts_size, refpts_size_part;
			refpts_size.reserve(ref_size);
			refpts_size_part.reserve(ref_size+1);
			refpts_size_part.emplace_back(0);
			for (int j = 0; j < ref_size; j++)
			{
				vector_vec3d& reforigin_pts = *refOriginPts[j][i];
				refpts_size.emplace_back(reforigin_pts.size());
				refpts_size_all += reforigin_pts.size();
				int temp = 0;
				for (int k = 0; k <= j; k++)
					temp += refOriginPts[k][i]->size();
				refpts_size_part.emplace_back(temp);
			}

			double N = refpts_size_all + basepts_size;
			int _N = (int)N;
			Eigen::MatrixXd _H(3*_N, 3*_N);
			_H.setZero();
			Eigen::MatrixXd _D(3*_N, jacob_len);
			_D.setZero();

			Eigen::Vector3d vec_tran;
			vector_vec3d pt_trans(basepts_size + refpts_size_all);
			std::vector<Eigen::Matrix3d> point_xis(basepts_size + refpts_size_all);
			std::vector<Eigen::Matrix3d> point_xic(basepts_size + refpts_size_all);
			Eigen::Vector3d center(Eigen::Vector3d::Zero());
			Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());

			for (size_t j = 0; j < basepts_size; j++)
			{
				vec_tran = poses[basewin_num[j]] * baseorigin_pts[j];
				point_xis[j] = -wedge(vec_tran);
				pt_trans[j] = vec_tran + ts[basewin_num[j]];
				center += pt_trans[j];
				covMat += pt_trans[j] * pt_trans[j].transpose();
			}
			
			size_t cnt = 0;
			for (int k = 0; k < ref_size; k++)
			{
				vector_vec3d& reforigin_pts = *refOriginPts[k][i];
				std::vector<int>& refwin_num = *refWinNums[k][i];
				for (size_t j = 0; j < reforigin_pts.size(); j++)
				{
					vec_tran = poses[refwin_num[j]] * (refQs[k] * reforigin_pts[j] + refTs[k]);
					point_xis[cnt+basepts_size] = -wedge(vec_tran);
					pt_trans[cnt+basepts_size] = vec_tran + ts[refwin_num[j]];
					center += pt_trans[cnt+basepts_size];
					covMat += pt_trans[cnt+basepts_size] * pt_trans[cnt+basepts_size].transpose();
					cnt++;
				}
			}
			
			cnt = 0;
			for (size_t j = 0; j < basepts_size; j++)
				point_xic[j] = Eigen::MatrixXd::Zero(3, 3);
			for (int k = 0; k < ref_size; k++)
			{
				vector_vec3d& reforigin_pts = *refOriginPts[k][i];
				for (size_t j = 0; j < reforigin_pts.size(); j++)
				{
					vec_tran = refQs[k] * reforigin_pts[j];
					point_xic[cnt+basepts_size] = -wedge(vec_tran);
					cnt++;
				}
			}
			
			covMat = covMat - center * center.transpose() / N;
			covMat = covMat / N;
			center = center / N;

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
			Eigen::Vector3d eigen_value = saes.eigenvalues();

			Eigen::Matrix3d U = saes.eigenvectors();
			Eigen::Vector3d u[3];
			for (int j = 0; j < 3; j++)
				u[j] = U.block<3, 1>(0, j);

			Eigen::Matrix3d ukukT = u[0] * u[0].transpose();
			Eigen::Vector3d vec_Jt;
			for (size_t j = 0; j < basepts_size; j++)
			{
				pt_trans[j] = pt_trans[j] - center;
				vec_Jt = 2.0 / N * ukukT * pt_trans[j];
				_jact.block<3, 1>(6*basewin_num[j], 0) -= point_xis[j] * vec_Jt;
				_jact.block<3, 1>(6*basewin_num[j]+3, 0) += vec_Jt;
			}

			cnt = 0;
			for (int k = 0; k < ref_size; k++)
			{
				vector_vec3d& reforigin_pts = *refOriginPts[k][i];
				std::vector<int>& refwin_num = *refWinNums[k][i];
				for (size_t j = 0; j < reforigin_pts.size(); j++)
				{
					pt_trans[cnt+basepts_size] = pt_trans[cnt+basepts_size] - center;
					vec_Jt = 2.0 / N * ukukT * pt_trans[cnt+basepts_size];
					_jact.block<3, 1>(6*refwin_num[j], 0) -= point_xis[cnt+basepts_size] * vec_Jt;
					_jact.block<3, 1>(6*refwin_num[j]+3, 0) += vec_Jt;

					_jact.block<3, 1>(6*pose_size+6*k, 0) -= point_xic[cnt+basepts_size] * 
						poses[refwin_num[j]].toRotationMatrix().transpose() * vec_Jt;
					_jact.block<3, 1>(6*pose_size+3+6*k, 0) +=
						poses[refwin_num[j]].toRotationMatrix().transpose() * vec_Jt;
					cnt++;
				}
			}
			assert(cnt==refpts_size_all);

			Eigen::Matrix3d Hessian33;
			Eigen::Matrix3d F;
			std::vector<Eigen::Matrix3d> F_(3);
			for (size_t j = 0; j < 3; j++)
			{
				if (j == 0)
				{
					F_[j].setZero();
					continue;
				}
				Hessian33 = u[j] * u[0].transpose();
				F_[j] = 1.0 / N / (eigen_value[0] - eigen_value[j]) * 
					(Hessian33 + Hessian33.transpose());
			}

			size_t rownum_s, colnum_s;

			for (size_t j = 0; j < N; j++)
			{
				for (int f = 0; f < 3; f++)
					F.block<1, 3>(f, 0) = pt_trans[j].transpose() * F_[f];
				F = U * F;

				for (size_t k = 0; k < N; k++)
				{
					Hessian33 = u[0] * (pt_trans[k]).transpose() * F + u[0].dot(pt_trans[k]) * F;
					if (k == j)
						Hessian33 += (N-1) / N * ukukT;
					else
						Hessian33 -= 1.0 / N * ukukT;
					
					Hessian33 = 2.0 / N * Hessian33;
					_H.block<3, 3>(3*k, 3*j) = Hessian33;
				}

				if (j < basepts_size)
				{
					colnum_s = 6 * basewin_num[j];
					_D.block<3, 3>(3*j, colnum_s) = point_xis[j];
					_D.block<3, 3>(3*j, colnum_s+3) = Eigen::MatrixXd::Identity(3, 3);
				}
				else
				{
					int base_pose;
					for (int a = 0; a < ref_size; a++)
						if (j-basepts_size < refpts_size_part[a+1])
						{
							std::vector<int>& refwin_num = *refWinNums[a][i];
							colnum_s = 6 * refwin_num[j-basepts_size-refpts_size_part[a]];
							base_pose = refwin_num[j-basepts_size-refpts_size_part[a]];
							_D.block<3, 3>(3*j, colnum_s) = point_xis[j];
							_D.block<3, 3>(3*j, colnum_s+3) = Eigen::MatrixXd::Identity(3, 3);
							colnum_s = 6 * (pose_size + a);
							_D.block<3, 3>(3*j, colnum_s) = poses[base_pose] * point_xic[j];
							_D.block<3, 3>(3*j, colnum_s+3) = poses[base_pose].toRotationMatrix();
							break;
						}
				}
			}

			residual += eigen_value[0];
			Hess += _D.transpose() * _H * _D;
			JacT += _jact;
			_jact.setZero();
		}
	}

	void evaluate_only_residual(vector_quad& poses_, vector_vec3d& ts_,
								vector_quad& refposes_, vector_vec3d& refts_, double& residual)
	{
		residual = 0;
		size_t voxel_size = baseOriginPts.size();
		// std::cout<<"voxel_size "<<voxel_size<<std::endl;
		Eigen::Vector3d pt_trans, new_center;
		Eigen::Matrix3d new_A;

		for (size_t i = 0; i < voxel_size; i++)
		{
			new_center.setZero();
			new_A.setZero();
			for (size_t j = 0; j < baseWinNums[i]->size(); j++)
			{
				pt_trans = poses_[(*baseWinNums[i])[j]] * (*baseOriginPts[i])[j] + 
					ts_[(*baseWinNums[i])[j]];
				new_A += pt_trans * pt_trans.transpose();
				new_center += pt_trans;				
			}
			size_t refpts_size = 0;
			for (int k = 0; k < ref_size; k++)
			{
				for (size_t j = 0; j < refWinNums[k][i]->size(); j++)
				{
					pt_trans = refposes_[k] * (*refOriginPts[k][i])[j] + refts_[k];
					pt_trans = poses_[(*refWinNums[k][i])[j]] * pt_trans + ts_[(*refWinNums[k][i])[j]];
					new_A += pt_trans * pt_trans.transpose();
					new_center += pt_trans;				
				}
				refpts_size += refOriginPts[k][i]->size();
			}
			size_t pt_size = baseOriginPts[i]->size() + refpts_size;
			new_center /= pt_size;
			new_A /= pt_size;
			new_A -= new_center * new_center.transpose();
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(new_A);
			Eigen::Vector3d eigen_value = saes.eigenvalues();
			residual += eigen_value(0);
		}
	}

	void free_voxel()
	{
		size_t voxel_size = baseOriginPts.size();
		for (size_t i = 0; i < voxel_size; i++)
		{
			delete (baseOriginPts[i]);
			delete (baseWinNums[i]);
		}
		baseOriginPts.clear();
		baseWinNums.clear();

		for (int j = 0; j < ref_size; j++)
		{
			voxel_size = refOriginPts[j].size();
			for (size_t i = 0; i < voxel_size; i++)
			{
				delete (refOriginPts[j][i]);
				delete (refWinNums[j][i]);
			}
			refOriginPts[j].clear();
			refWinNums[j].clear();
		}
		refOriginPts.clear();
		refWinNums.clear();
	}
};

enum OT_STATE {END_OF_TREE, NOT_TREE_END};
class OCTO_TREE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static int voxel_windowsize;
	std::vector<vector_vec3d*> baseOriginPc;
	std::vector<vector_vec3d*> baseTransPc;
	std::vector<std::vector<vector_vec3d*>> refOriginPc; // store the point cloud collected at pose i
	std::vector<std::vector<vector_vec3d*>> refTransPc;
	OCTO_TREE* leaves[8];

	int pose_size, ref_size, feat_eigen_limit;
	OT_STATE octo_state; // 0 is end of tree, 1 is not
	int points_size, sw_points_size;
	
	double feat_eigen_ratio, feat_eigen_ratio_test;
	double voxel_center[3]; // x, y, z
	double quater_length;
	bool is2opt;

	OCTO_TREE(int capa, int refsz) : pose_size(capa), ref_size(refsz)
	{
		octo_state = END_OF_TREE;
		for (int i = 0; i < 8; i++)
			leaves[i] = nullptr;

		for (int i = 0; i < pose_size; i++)
		{
			baseOriginPc.emplace_back(new vector_vec3d());
			baseTransPc.emplace_back(new vector_vec3d());

		}
		for (int j = 0; j < ref_size; j++)
		{
			std::vector<vector_vec3d*> refOriginPc_, refTransPc_;
			for (int i = 0; i < pose_size; i++)
			{
				refOriginPc_.emplace_back(new vector_vec3d());
				refTransPc_.emplace_back(new vector_vec3d());
			}
			refOriginPc.emplace_back(refOriginPc_);
			refTransPc.emplace_back(refTransPc_);
		}
		is2opt = true;
		feat_eigen_limit = 25;
	}

	~OCTO_TREE()
	{
		for (int i = 0; i < pose_size; i++)
		{
			delete (baseOriginPc[i]);
			delete (baseTransPc[i]);
		}
		baseOriginPc.clear();
		baseTransPc.clear();
		for (int i = 0; i < ref_size; i++)
		{
			for (int j = 0; j < pose_size; j++)
			{
				delete refOriginPc[i][j];
				delete refTransPc[i][j];
			}
			refOriginPc[i].clear();
			refTransPc[i].clear();
		}
		refOriginPc.clear();
		refTransPc.clear();
		for (int i = 0; i < 8; i++)
			if (leaves[i] != nullptr)
				leaves[i]->~OCTO_TREE();
	}

	void recut(int layer, size_t frame_head)
	{
		if (octo_state == END_OF_TREE)
		{
			points_size = 0;
			for (int i = 0; i < pose_size; i++)
			{
				points_size += baseOriginPc[i]->size();
				for (int j = 0; j < ref_size; j++)
					points_size += refOriginPc[j][i]->size();
			}	
			
			if (points_size < MIN_PS)
			{
				feat_eigen_ratio = -1;
				return;
			}

			calc_eigen();
			
			if (std::isnan(feat_eigen_ratio))
			{
				feat_eigen_ratio = -1;
				return;
			}

			if (feat_eigen_ratio >= feat_eigen_limit)
				return;

			if (layer == 4)
				return;

			octo_state = NOT_TREE_END;
		}

		int leafnum;
		size_t pt_size;

		for (int i = frame_head; i < OCTO_TREE::voxel_windowsize; i++)
		{
			pt_size = baseTransPc[i]->size();
			for (size_t j = 0; j < pt_size; j++)
			{
				int xyz[3] = {0, 0, 0};
				for (size_t k = 0; k < 3; k++)
					if ((*baseTransPc[i])[j][k] > voxel_center[k])
						xyz[k] = 1;
				leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
				if (leaves[leafnum] == nullptr)
				{
					leaves[leafnum] = new OCTO_TREE(pose_size, ref_size);
					leaves[leafnum]->voxel_center[0] =
						voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
					leaves[leafnum]->voxel_center[1] =
						voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
					leaves[leafnum]->voxel_center[2] =
						voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
					leaves[leafnum]->quater_length = quater_length / 2;
				}
				leaves[leafnum]->baseOriginPc[i]->emplace_back((*baseOriginPc[i])[j]);
				leaves[leafnum]->baseTransPc[i]->emplace_back((*baseTransPc[i])[j]);
			}
			for (int k = 0; k < ref_size; k++)
			{
				pt_size = refTransPc[k][i]->size();
				for (size_t j = 0; j < pt_size; j++)
				{
					int xyz[3] = {0, 0, 0};
					for (size_t a = 0; a < 3; a++)
						if ((*refTransPc[k][i])[j][a] > voxel_center[a])
							xyz[a] = 1;
					leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
					if (leaves[leafnum] == nullptr)
					{
						leaves[leafnum] = new OCTO_TREE(pose_size, ref_size);
						leaves[leafnum]->voxel_center[0] =
							voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
						leaves[leafnum]->voxel_center[1] =
							voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
						leaves[leafnum]->voxel_center[2] =
							voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
						leaves[leafnum]->quater_length = quater_length / 2;
					}
					leaves[leafnum]->refOriginPc[k][i]->emplace_back((*refOriginPc[k][i])[j]);
					leaves[leafnum]->refTransPc[k][i]->emplace_back((*refTransPc[k][i])[j]);
				}
			}
		}

		if (layer != 0)
			for (int i = frame_head; i < OCTO_TREE::voxel_windowsize; i++)
			{
				if (baseOriginPc[i]->size() != 0)
				{
					vector_vec3d().swap(*baseOriginPc[i]);
					vector_vec3d().swap(*baseTransPc[i]);
				}
				for (int j = 0; j < ref_size; j++)
					if (refOriginPc[j][i]->size() != 0)
					{
						vector_vec3d().swap(*refOriginPc[j][i]);
						vector_vec3d().swap(*refTransPc[j][i]);
					}
			}

		layer++;
		for (size_t i = 0; i < 8; i++)
			if (leaves[i] != nullptr)
				leaves[i]->recut(layer, frame_head);
	}

	void calc_eigen()
	{
		Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
		Eigen::Vector3d center(0, 0, 0);

		size_t pt_size;
		for (int i = 0; i < pose_size; i++)
		{
			pt_size = baseTransPc[i]->size();
			for (size_t j = 0; j < pt_size; j++)
			{
				covMat += (*baseTransPc[i])[j] * (*baseTransPc[i])[j].transpose();
				center += (*baseTransPc[i])[j];
			}
			for (int k = 0; k < ref_size; k++)
			{
				pt_size = refTransPc[k][i]->size();
				for (size_t j = 0; j < pt_size; j++)
				{
					covMat += (*refTransPc[k][i])[j] * (*refTransPc[k][i])[j].transpose();
					center += (*refTransPc[k][i])[j];
				}
			}
		}
		center /= points_size;
		covMat = covMat / points_size - center * center.transpose();
		/* saes.eigenvalues()[2] is the biggest */
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
		feat_eigen_ratio = saes.eigenvalues()[2] / saes.eigenvalues()[0];
	}

	void feed_pt(LM_OPTIMIZER& lm_opt)
	{
		if (octo_state == END_OF_TREE)
		{
			sw_points_size = 0;
			for (int i = 0; i < pose_size; i++)
			{
				sw_points_size += baseOriginPc[i]->size();
				for (int j = 0; j < ref_size; j++)
					sw_points_size += refOriginPc[j][i]->size();
			}
			
			if (sw_points_size < MIN_PS)
				return;
			
			traversal_opt_calc_eigen();

			if (std::isnan(feat_eigen_ratio_test))
				return;

			if (feat_eigen_ratio_test > feat_eigen_limit)
				lm_opt.push_voxel(baseOriginPc, refOriginPc);
		}
		else
			for (int i = 0; i < 8; i++)
				if (leaves[i] != nullptr)
					leaves[i]->feed_pt(lm_opt);
	}

	void traversal_opt_calc_eigen()
	{
		Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
		Eigen::Vector3d center(0, 0, 0);

		size_t pt_size;
		for (int i = 0; i < pose_size; i++)
		{
			pt_size = baseTransPc[i]->size();
			for (size_t j = 0; j < pt_size; j++)
			{
				covMat += (*baseTransPc[i])[j] * (*baseTransPc[i])[j].transpose();
				center += (*baseTransPc[i])[j];
			}
		}
		for (int i = 0; i < pose_size; i++)
		{
			for (int k = 0; k < ref_size; k++)
			{
				pt_size = refTransPc[k][i]->size();
				for (size_t j = 0; j < pt_size; j++)
				{
					covMat += (*refTransPc[k][i])[j] * (*refTransPc[k][i])[j].transpose();
					center += (*refTransPc[k][i])[j];
				}
			}
		}

		covMat -= center * center.transpose() / sw_points_size; 
		covMat /= sw_points_size;

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
		feat_eigen_ratio_test = (saes.eigenvalues()[2] / saes.eigenvalues()[0]);
	}
};

int OCTO_TREE::voxel_windowsize = 0;

#endif