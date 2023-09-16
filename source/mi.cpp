#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "common.h"

using namespace std;
using namespace Eigen;

double myrd(double a)
{
  double a_ = (int)(a * 10 + 0.5);
  return a_ / 10;
}

int main(int argc, char** argv)
{
  Vector3d r0(0.012301,-1.552941,1.563441);
  Matrix3d R0;
  R0 = AngleAxisd(r0(2), Vector3d::UnitZ()) *
       AngleAxisd(r0(1), Vector3d::UnitY()) *
       AngleAxisd(r0(0), Vector3d::UnitX());
  Vector3d t0(0.115689,-0.157345,0.455022);

  Vector3d r1(-0.015447,-1.551050,1.570723);
  Matrix3d R1;
  R1 = AngleAxisd(r1(2), Vector3d::UnitZ()) *
        AngleAxisd(r1(1), Vector3d::UnitY()) *
        AngleAxisd(r1(0), Vector3d::UnitX());
  Vector3d t1(-0.009478,-0.185336,0.343020);

  Eigen::Matrix3d Rgt;
  Rgt << 0.999824645293243, -0.0183851254185105, 0.00355890820136192,
          0.0183851559551168, 0.999830978444791, 2.41378898009210e-05,
          -0.00355875044729421, 4.12974252036677e-05, 0.999993666774833;

  Eigen::Quaterniond q2(Rgt.transpose());
  Eigen::Quaterniond qme(R1 * R0.inverse());
  cout << qme.toRotationMatrix() << endl;
  cout << "angular error " << qme.angularDistance(q2) * 57.3 << endl;
  cout << "baseline error " << fabs(t0(0) - (qme * t1)(0) - 0.1072) << endl;

  /* For TRO LiDAR LiDAR extrinsic calibration
  Matrix3d R21, R23;
  R21 << 0.9971,-0.0757,-0.0000,0.0757,0.9971,0.0005,-0.0000,-0.0005,1.0000;
  R23 << 1.0000,-0.0085,0.0000,0.0085,1.0000,-0.0048,0.0000,0.0048,1.0000;
  Vector3d t21(0.0841,0,-0.4837);
  Vector3d t23(-0.0198,0,0.0363);
  double q_err = Quaterniond(R21).angularDistance(Quaterniond(1,0,0,0));
  cout<<"q_left "<<q_err<<" "<<q_err*57.3<<endl;
  cout<<"q_right "<<Quaterniond(R23).angularDistance(Quaterniond(1,0,0,0))<<" "
      <<Quaterniond(R23).angularDistance(Quaterniond(1,0,0,0))*57.3<<endl;
  cout<<"t_left "<<t21.norm()<<endl;
  cout<<"t_right "<<t23.norm()<<endl;

  Matrix3d Ra1, Ra2, Ra3;
  Vector3d ta1(-0.0375,0,-1.9234), ta2(0.0052,0,-0.5734), ta3(-0.0348,0,0.8168);
  Ra1 << -0.9977,0.0657,-0.0151,-0.0657,-0.9978,-0.0005,-0.0151,0.0005,0.9999;
  Ra2 << -0.9999,0.0062,-0.0138,-0.0062,-1.0000,-0.0000,-0.0138,0.0000,0.9999;
  Ra3 << -0.9997,0.0130,-0.0184,-0.0130,-0.9999,-0.0001,-0.0184,0.0001,0.9998;
  double q_err1 = Quaterniond(Ra2).angularDistance(Quaterniond(Ra1));
  double q_err2 = Quaterniond(Ra2).angularDistance(Quaterniond(Ra3));
  double t_err1 = (ta1-ta2).norm();
  double t_err2 = (ta3-ta2).norm();
  cout<<"q_left "<<q_err1<<" "<<q_err1*57.3<<endl;
  cout<<"q_right "<<q_err2<<" "<<q_err2*57.3<<endl;
  cout<<"t_left "<<t_err1<<endl;
  cout<<"t_right "<<t_err2<<endl;
  **/

  // /* For MI based LiDAR camera extrinsic calibration comparision
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  std::ofstream file;
  file.open(
    "/home/sam/catkin_ws/src/mlcc/result/init_for_mi.txt",
    std::ofstream::trunc);
  file.close();
  file.open(
    "/home/sam/catkin_ws/src/mlcc/result/init_for_us.txt",
    std::ofstream::trunc);
  file.close();
  file.open(
    "/home/sam/catkin_ws/src/mlcc/result/init_for_mlcc.txt",
    std::ofstream::trunc);
  file.close();

  int valid_cnt = 0;
  double angle = 2;
  for (int a = 0; a < 1000; a++)
  {
    double roll = 0;
    double pitch = 90;  // 90 for mid100, -90 for avia
    double yaw = -90;     // -90 for mid100, 90 for avia
    double _roll = myrd((float)rand() / RAND_MAX * angle * 2 - angle);
    double _pitch = myrd((float)rand() / RAND_MAX * angle * 2 - angle);
    double _yaw = myrd((float)rand() / RAND_MAX * angle * 2 - angle);

    Matrix3d R_left, R_right, R, R_err;
    R = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ()) *
        AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY()) *
        AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
    R_left =
      Eigen::AngleAxisd(DEG2RAD(yaw + _yaw), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(DEG2RAD(pitch + _pitch), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DEG2RAD(roll + _roll), Eigen::Vector3d::UnitX());

    _roll = myrd((float)rand() / RAND_MAX * angle * 2 - angle);
    _pitch = myrd((float)rand() / RAND_MAX * angle * 2 - angle);
    _yaw = myrd((float)rand() / RAND_MAX * angle * 2 - angle);
    R_right =
      Eigen::AngleAxisd(DEG2RAD(yaw + _yaw), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(DEG2RAD(pitch + _pitch), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DEG2RAD(roll + _roll), Eigen::Vector3d::UnitX());

    double tx = 0.0536; // MID-100
    double ty = 0.1532;
    double tz = -0.2252;
    // double tx = 0.0536; // AVIA
    // double ty = 0.0185;
    // double tz = 0.02943;

    double _tx = (float)rand() / RAND_MAX * 20 - 10;
    double _ty = (float)rand() / RAND_MAX * 20 - 10;
    double _tz = (float)rand() / RAND_MAX * 20 - 10;
    Vector3d t_left(_tx, _ty, _tz);
    _tx = (float)rand() / RAND_MAX * 20 - 10;
    _ty = (float)rand() / RAND_MAX * 20 - 10;
    _tz = (float)rand() / RAND_MAX * 20 - 10;
    Vector3d t_right(_tx, _ty, _tz);

    Eigen::Quaterniond q_err(R_right * R_left.inverse());
    double angular_err = q_err.angularDistance(q2) * 57.3;
    double baseline_err = fabs(t_left(0) / 100 - (q_err * t_right)(0) / 100 - 0.1072);
    if (angular_err < 4 || angular_err > 5) continue;
    // if (baseline_err > 0.15) continue;

    file.open(
      "/home/sam/catkin_ws/src/mlcc/result/"
      "init_for_mi.txt",
      std::ofstream::app);
    Vector3d euler_angle = R_left.eulerAngles(2, 1, 0);
    euler_angle *= 57.3;
    file << valid_cnt << "\n";
    file << tx + t_left(0) / 100 << ", " << ty + t_left(1) / 100 << ", "
         << tz + t_left(2) / 100 << ", " << euler_angle(2) << ", "
         << euler_angle(1) << ", " << euler_angle(0) << "\n";
    euler_angle = R_right.eulerAngles(2, 1, 0);
    euler_angle *= 57.3;
    file << -tx + t_right(0) / 100 << ", " << ty + t_right(1) / 100 << ", "
         << tz + t_right(2) / 100 << ", " << euler_angle(2) << ", "
         << euler_angle(1) << ", " << euler_angle(0) << "\n\n";
    file.close();

    file.open(
      "/home/sam/catkin_ws/src/mlcc/result/"
      "init_for_us.txt",
      std::ofstream::app);
    file << valid_cnt << "\n";
    file << R_left(0, 0) << "," << R_left(0, 1) << "," << R_left(0, 2) << ","
         << tx + t_left(0) / 100 << ",\n";
    file << R_left(1, 0) << "," << R_left(1, 1) << "," << R_left(1, 2) << ","
         << ty + t_left(1) / 100 << ",\n";
    file << R_left(2, 0) << "," << R_left(2, 1) << "," << R_left(2, 2) << ","
         << tz + t_left(2) / 100 << ",\n";
    file << 0 << "," << 0 << "," << 0 << "," << 1 << "\n";

    file << R_right(0, 0) << "," << R_right(0, 1) << "," << R_right(0, 2) << ","
         << -tx + t_right(0) / 100 << ",\n";
    file << R_right(1, 0) << "," << R_right(1, 1) << "," << R_right(1, 2) << ","
         << ty + t_right(1) / 100 << ",\n";
    file << R_right(2, 0) << "," << R_right(2, 1) << "," << R_right(2, 2) << ","
         << tz + t_right(2) / 100 << ",\n";
    file << 0 << "," << 0 << "," << 0 << "," << 1 << "\n";

    file << "angular error " << angular_err << "\n";
    file << "baseline error " << baseline_err << "\n\n\n";
    file.close();

    file.open(
      "/home/sam/catkin_ws/src/mlcc/result/init_for_mlcc.txt",
      std::ofstream::app);
    file << R_left(0, 0) << " " << R_left(0, 1) << " " << R_left(0, 2) << " "
         << tx + t_left(0) / 100 << " "
         << R_left(1, 0) << " " << R_left(1, 1) << " " << R_left(1, 2) << " "
         << ty + t_left(1) / 100 << " "
         << R_left(2, 0) << " " << R_left(2, 1) << " " << R_left(2, 2) << " "
         << tz + t_left(2) / 100 << "\n";
        //  << 0 << " " << 0 << " " << 0 << " " << 1 << "\n";
    file << R_right(0, 0) << " " << R_right(0, 1) << " " << R_right(0, 2) << " "
         << tx + t_right(0) / 100 << " "
         << R_right(1, 0) << " " << R_right(1, 1) << " " << R_right(1, 2) << " "
         << ty + t_right(1) / 100 << " "
         << R_right(2, 0) << " " << R_right(2, 1) << " " << R_right(2, 2) << " "
         << tz + t_right(2) / 100 << "\n";
        //  << 0 << " " << 0 << " " << 0 << " " << 1 << "\n";
    file.close();

    valid_cnt++;
    if (valid_cnt == 30) break;
  }
  // **/
}