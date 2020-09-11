#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Householder>

using namespace std;

string scan_match_file = "./scan_match.txt";
string odom_file = "./odom.txt";

int main(int argc, char** argv)
{
    // laser data: time stamp, x, y, theta
    vector<vector<double>> s_data;
    // wheel odometry data: time stamp, left angular velocity, right angualr velocity
    vector<vector<double>> r_data;

    ifstream fin_s(scan_match_file);
    ifstream fin_r(odom_file);
    if (!fin_s || !fin_r)
    {
        cerr << "please run the code under the directory containing scan_match.txt and odom.txt" << endl;
        return 1;
    }

    // read laser data
    while (!fin_s.eof()) {
        double s_t, s_x, s_y, s_th;
        fin_s >> s_t >> s_x >> s_y >> s_th;
        s_data.push_back(vector<double>({s_t, s_x, s_y, s_th}));
    }
    fin_s.close();

    // read angular velocity of wheels
    while (!fin_r.eof()) {
        double t_r, w_L, w_R;
        fin_r >> t_r >> w_L >> w_R;
        r_data.push_back(vector<double>({t_r, w_L, w_R}));
    }
    fin_r.close();

    // step1: compute J21, J22
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    A.conservativeResize(5000, 2);
    b.conservativeResize(5000);
    A.setZero();
    b.setZero();

    size_t id_r = 0;
    size_t id_s = 0;
    double last_rt = r_data[0][0];
    double w_Lt = 0;
    double w_Rt = 0;
    while (id_s < 5000)
    {
        // laser data
        const double &s_t = s_data[id_s][0];
        const double &s_th = s_data[id_s][3];
        // wheel odometry data
        const double &r_t = r_data[id_r][0];
        const double &w_L = r_data[id_r][1];
        const double &w_R = r_data[id_r][2];
        ++id_r;

        // compute the angular integration of wheel odometry between two laser data
        // make sure time stamp is synchronized
        if (r_t < s_t)
        {
            double dt = r_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = r_t;
        }
        else
        {
            double dt = s_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = s_t;

            A(id_s,0) = w_Lt;
            A(id_s,1) = w_Rt;
            b(id_s) = s_th;

            w_Lt = 0;
            w_Rt = 0;
            ++id_s;
        }
    }

    Eigen::Vector2d J21J22;

    J21J22 = A.colPivHouseholderQr().solve(b);

    const double &J21 = J21J22(0);
    const double &J22 = J21J22(1);
    cout << "J21: " << J21 << endl;
    cout << "J22: " << J22 << endl;

    // step2: compute baseline b
    Eigen::VectorXd C;
    Eigen::VectorXd S;

    C.conservativeResize(10000);
    S.conservativeResize(10000);
    C.setZero();
    S.setZero();

    id_r = 0;
    id_s = 0;
    last_rt = r_data[0][0];
    double th = 0;
    double cx = 0;
    double cy = 0;
    while (id_s < 5000)
    {
        // laser data
        const double &s_t = s_data[id_s][0];
        const double &s_x = s_data[id_s][1];
        const double &s_y = s_data[id_s][2];
        // wheel odometry data
        const double &r_t = r_data[id_r][0];
        const double &w_L = r_data[id_r][1];
        const double &w_R = r_data[id_r][2];
        ++id_r;
        // compute odometry position integration between two laser frame
        if (r_t < s_t)
        {
            double dt = r_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = r_t;
        }
        else
        {
            double dt = s_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = s_t;

            C(id_s*2) = cx;
            C(id_s*2+1) = cy;
            S(id_s*2) = s_x;
            S(id_s*2+1) = s_y;

            cx = 0;
            cy = 0;
            th = 0;
            ++id_s;
        }
    }
    // compute b, r_L, r_R
    double b_wheel;
    double r_L;
    double r_R;

    Eigen::MatrixXd b_wheel_mat = C.colPivHouseholderQr().solve(S);
    b_wheel = b_wheel_mat(0,0);
    r_L = -b_wheel*J21;
    r_R = b_wheel*J22;

    cout << "b: " << b_wheel << endl;
    cout << "r_L: " << r_L << endl;
    cout << "r_R: " << r_R << endl;

    cout << "ground truth: b is about 0.6m, the radius of wheel is about 0.1m" << endl;

    return 0;
}

