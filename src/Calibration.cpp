#include "Calibration.hpp"

//STL
#include <cassert>
#include <cmath>

//Debug only
#include <iostream>
#include <fstream>

#include "auxiliar.h"

inline Eigen::Vector4f getHomogeneousPoint(const Eigen::Vector3f &v)
{
    return Eigen::Vector4f(v(0), v(1), v(2), 1.f);
}

inline Eigen::Vector3f undoHomogeneousPoint(const Eigen::Vector4f &h)
{
    return Eigen::Vector3f(h(0)/h(3), h(1)/h(3), h(2)/h(3));
}

inline Eigen::Vector4f getHomogeneousVector(const Eigen::Vector3f &v)
{
    return Eigen::Vector4f(v(0), v(1), v(2), 0.f);
}

inline Eigen::Vector3f undoHomogeneousVector(const Eigen::Vector4f &h)
{
    Eigen::Vector3f v(h(0), h(1), h(2));
    return v/v.norm();
}

Calibration::Calibration(const ParametersPtr &params)
    : outliers(0)
{
    verbose = params->verbose;

    max_angle = params->max_angle;
    max_distance = params->max_distance;

    approx_rotation = params->approx_rotation;
    approx_translation = params->approx_translation;

    if (verbose)
    {
        std::cout << "max_rotation: " << (max_angle + approx_rotation) << std::endl;
        std::cout << "max_translation: " << (max_distance + approx_translation) << std::endl;
    }

    save_dir = params->output_dir;

    const std::size_t N = params->sensor_labels.size();

    assert(N > 1);

    init_pose = params->init_pose;
    for (std::size_t k = 0; k < N; ++k)
        sensor_id[params->sensor_labels[k]] = k;

    all_matches.resize(N*(N-1)/2);

    poses.resize(N);
    labels = params->sensor_labels;
}

Correspondences Calibration::findCorrespondences(const ObservationPair &obs2, const PlanesList &p1, const PlanesList &p2, bool use_initial_est)
{
    if (verbose)
    {
        std::cout << obs2.first->sensorLabel << ": " << sensor_id[obs2.first->sensorLabel] << std::endl;
        std::cout << obs2.second->sensorLabel << ": " << sensor_id[obs2.second->sensorLabel] << std::endl;

//        std::cout << "Pose1: " << obs2.first->sensorPose << std::endl;
//        std::cout << "Pose2: " << obs2.second->sensorPose << std::endl;
    }

    std::size_t id_i = sensor_id[obs2.first->sensorLabel];
    std::size_t id_j = sensor_id[obs2.second->sensorLabel];

    assert(id_i != id_j);

    PlanesList p_i = p1;
    PlanesList p_j = p2;

    if (id_i > id_j)
    {
        std::swap(id_i, id_j);
        std::swap(p_i, p_j);
    }

    Correspondences matches;
    for (std::size_t i = 0; i < p_i.size(); ++i)
    {
        Eigen::Vector3f n_i = undoHomogeneousVector(init_pose[id_i]*getHomogeneousVector(p_i[i].n)); //Transform normal vector
        Eigen::Vector3f x_i = -p_i[i].n*p_i[i].d; //A point on the plane (assuming norm(n) == 1)
        float d_i = -n_i.dot(undoHomogeneousPoint(init_pose[id_i]*getHomogeneousPoint(x_i))); //Calculate new distance to origin
        for (std::size_t j = 0; j < p_j.size(); ++j)
        {
            Eigen::Vector3f n_j = undoHomogeneousVector(init_pose[id_j]*getHomogeneousVector(p_j[j].n)); //Transform normal vector
            Eigen::Vector3f x_j = -p_j[j].n*p_j[j].d; //A point on the plane (assuming norm(n) == 1)
            float d_j = -n_j.dot(undoHomogeneousPoint(init_pose[id_j]*getHomogeneousPoint(x_j))); //Calculate new distance to origin

            float a = std::abs(mrpt::utils::RAD2DEG(std::acos(n_i.dot(n_j))));
            float d = std::abs(d_i-d_j);

            if (verbose)
            {
                std::cout << "angle: " << a << std::endl;
                std::cout << "dist: " << d << std::endl;
            }

            //TODO: the propagation of the estimated error is not really rigurous...
            if ((a < (max_angle + approx_rotation)) && (d < (max_distance + approx_translation)))
                matches.push_back(PlaneMatch(p_i[i], p_j[j]));
            else if (!use_initial_est)
            {
                outliers++;
                matches.push_back(PlaneMatch(p_i[i], p_j[j]));
            }
        }
    }

    //TODO: test inserting empty correspondences
    std::size_t idx = this->getPairId(id_i, id_j);
    if (verbose)
        std::cout << "idx: " << idx << std::endl;
    all_matches[idx].insert(all_matches[idx].end(), matches.begin(), matches.end());

    return matches;
}

//TODO: if id_i > id_j -> return inverse rotation
Eigen::Matrix3f Calibration::calibrateRotation(std::size_t id_i, std::size_t id_j)
{
    // Calibration system
    Eigen::Matrix3f rotationCov = Eigen::Matrix3f::Zero();
    float accum_error = 0.f;

    std::size_t idx = this->getPairId(id_i, id_j);
    Correspondences matches = all_matches[idx];
    PlaneMatch match;

    std::size_t N = matches.size();

    if (verbose)
        std::cout << "Number of matches: " << N << std::endl;

    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        Eigen::Vector3f n_j = match.second.n;
        Eigen::Vector3f n_j_ = undoHomogeneousVector((init_pose[id_i].inverse()*init_pose[id_j])*getHomogeneousVector(n_j)); //Rotation from id_i to id_j

        Eigen::Vector3f rot_error = (n_i - n_j_);
        accum_error += rot_error.dot(rot_error);

        // From first to second
        rotationCov += n_j*n_i.transpose();
    }

    if (verbose)
        std::cout << "Rotation error (initial estimation): " << accum_error << std::endl;

    // Calculate calibration Rotation
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();

    if (verbose)
    {
        std::cout << "Conditioning: " << conditioning;
        if (conditioning > 100)
            std::cout << " (bad)";
        std::cout << std::endl;
    }

    Eigen::Matrix3f rotation = svd.matrixV()*svd.matrixU().transpose();
    float det = rotation.determinant();
    if(det != 1)
    {
        Eigen::Matrix3f aux = Eigen::Matrix3f::Identity();
        aux(2, 2) = det;
        rotation = svd.matrixV()*aux*svd.matrixU().transpose();
    }

    accum_error = 0.f;
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        Eigen::Vector3f n_j_ = rotation*match.second.n;
        n_j_ = n_j_/n_j_.norm();

        Eigen::Vector3f rot_error = (n_i - n_j_);
        accum_error += rot_error.dot(rot_error);
    }

    if (verbose)
        std::cout << "Rotation error (after calibration): " << accum_error << std::endl;

    return rotation;
}

Eigen::Vector3f Calibration::calibrateTranslation(std::size_t id_i, std::size_t id_j)
{
    // Calibration system
    Eigen::Matrix3f translationHessian = Eigen::Matrix3f::Zero();
    Eigen::Vector3f translationGradient = Eigen::Vector3f::Zero();

    std::size_t idx = this->getPairId(id_i, id_j);
    Correspondences matches = all_matches[idx];
    PlaneMatch match;

    float accum_error = 0.f;

    std::size_t N = matches.size();
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        float trans_error = match.second.d - match.first.d;

        translationHessian += (n_i*n_i.transpose());
        translationGradient += (n_i*trans_error);

        Eigen::Vector3f t ((init_pose[id_i].inverse()*init_pose[id_j]).block(0, 3, 3, 1));
        float r = trans_error - t.dot(n_i);
        accum_error += r*r;
    }

    if (verbose)
        std::cout << "Translation error (initial estimation): " << accum_error << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(translationHessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "FIM translation " << svd.singularValues().transpose() << std::endl;

    Eigen::Vector3f translation = translationHessian.inverse()*translationGradient;

    accum_error = 0.f;
    for(std::size_t k = 0; k < N; ++k)
    {
        match = matches[k];
        Eigen::Vector3f n_i = match.first.n;
        float trans_error = match.second.d - match.first.d;

        float r = trans_error - translation.dot(n_i);
        accum_error += r*r;
    }

    if (verbose)
        std::cout << "Translation error (after calibration): " << accum_error << std::endl;

    return translation;
}

Eigen::Matrix4f Calibration::runCalibration()
{
    Eigen::Matrix3f rot = this->calibrateRotation(0, 1);
    Eigen::Vector3f t = this->calibrateTranslation(0, 1);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block(0, 0, 3, 3) = rot;
    T.block(0, 3, 3, 1) = t;

    return T;
}

/*  Iterative Solutions */

Eigen::Matrix4f Calibration::runCalibrationGN(std::size_t id_i, std::size_t id_j, bool robust)
{
    // define initial value, and update increment [TODO: define initial value, otherwise 0]
    Vector6d X, Xinc;
    Matrix4d T, Tinc, Tgt;
    T = Tini;  //[TODO: T = Tini]

    // if available, load GT
//    Tgt = Matrix4d::Identity();
//    Tgt.block(0,0,1,4) <<	0.6316, -0.0318, 0.7746, 0.2558;
//    Tgt.block(1,0,1,4) <<	0.3198, 0.9209, -0.2229, 0.4039;
//    Tgt.block(2,0,1,4) <<	-0.7062, 0.3885, 0.5919, 0.3731;

    std::size_t idx = this->getPairId(id_i, id_j);
    Correspondences matches = all_matches[idx];

    // define hessians, gradients, and residuals
    int N = matches.size();
    Matrix6d Hi, H;
    Vector6d gi, g;
    Vector4d ei;
    double   e = 0.0, ei_sq;
    H   = Matrix6d::Zero();
    g   = Vector6d::Zero();

    // define GN/LM parameters
    double delta          = 0.00001f;
    double err_prev       = 9999999999999.9;
    int max_iters         = 30;
    double min_err        = 0.0000001;
    double min_err_change = 0.0000001;

    // start GN loop (if LM we have to first estimate the order of lambdaa)
    for( int i = 0; i < max_iters; i++ )
    {

        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;

        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
            // update Hessian, gradient (neg), and error
            H += J.transpose() * J  * w;
            g -= J.transpose() * ei * w;
            e += ei_sq * w;
        }

        // if the difference is very small stop
        if( ( abs(e-err_prev) < min_err_change ) || ( e < min_err ) )
            break;

        // update step
        LDLT<Matrix6d> solver(H);
        Xinc = solver.solve(g);
        T = T * expmap_se3(Xinc);

//        cout << endl << "Iteration " << i << "\t" << e;

        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( Xinc.norm() < numeric_limits<double>::epsilon() )
            break;

        // update previous values
        err_prev = e;

    }

    return T.cast<float>();

    // show solution (DBG)
//    Vector6d calib_err = logmap_se3( T * inverse_se3(Tgt) );
//    cout << endl << endl;
//    cout << "Final residue: " << err_prev << endl << endl << endl;
//    cout << "Rotation error (rad)  = " << calib_err.tail(3).norm() << endl ;
//    cout << "Translation error (m) = " << calib_err.head(3).norm() << endl << endl << endl;
//    cout << "Calibration: " << endl << "---------------------------------------" << endl << endl;
//    cout << T << endl << endl << endl;
//    cout << "Uncertainty matrix: " << endl << "---------------------------------------" << endl << endl;
//    cout << H.inverse() << endl << endl;

}

Eigen::Matrix4f Calibration::runCalibrationLM(std::size_t id_i, std::size_t id_j, bool robust)
{

    // define initial value, and update increment [TODO: define initial value, otherwise 0]
    Vector6d X, Xinc;
    Matrix4d T, Tinc, Tgt;
    T = Tini;  //[TODO: T = Tini]

    // if available, load GT
//    Tgt = Matrix4d::Identity();
//    Tgt.block(0,0,1,4) <<	0.6316, -0.0318, 0.7746, 0.2558;
//    Tgt.block(1,0,1,4) <<	0.3198, 0.9209, -0.2229, 0.4039;
//    Tgt.block(2,0,1,4) <<	-0.7062, 0.3885, 0.5919, 0.3731;

    std::size_t idx = this->getPairId(id_i, id_j);
    Correspondences matches = all_matches[idx];

    // define hessians, gradients, and residuals
    int N = matches.size();
    Matrix6d Hi, H;
    Vector6d gi, g;
    Vector4d ei;
    double   e = 0.0, ei_sq;
    H   = Matrix6d::Zero();
    g   = Vector6d::Zero();

    // define GN/LM parameters
    double delta          = 0.00001;
    double err_prev       = 9999999999999.9;
    int max_iters         = 30;
    double min_err        = 0.0000001;
    double min_err_change = 0.0000001;
    double lambda         = 0.001; // we pre-calculate it later
    double lambda_k       = 5.0;
    bool precalculate_lambda = true;

    // pre-calculate lambda value
    if( precalculate_lambda )
    {
        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;
        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
        }
        // initial guess of lambda
        double Hmax = 0.0;
        for( int i = 0; i < 6; i++)
        {
            if( H(i,i) > Hmax || H(i,i) < -Hmax )
                Hmax = fabs( H(i,i) );
        }
        lambda = Hmax;
    }

    // start GN loop (if LM we have to first estimate the order of lambdaa)
    for( int i = 0; i < max_iters; i++ )
    {

        // reset gradient, Hessan, and error
        H   = Matrix6d::Zero();
        g   = Vector6d::Zero();
        e   = 0.0;

        // estimate hessian and gradient
        for( int j = 0; j < N; j++ )
        {
            // grab observation
            PlaneMatch match = matches[j];
            Vector3d ni = match.first.n.cast<double>();
            Vector3d nj = match.second.n.cast<double>();
            double   di = match.first.d;
            double   dj = match.second.d;
            // define intermediate variables
            Vector4d u; u.head(3) = ni; u(3) = di-dj;
            Vector4d v; v.head(3) = nj; v(3) = 0;
            // error
            ei    = u - inverse_se3(T).transpose() * v;
            ei_sq = ei.transpose() * ei;
            // Jacobians
            MatrixXd J = MatrixXd::Zero(4,6);
            for( int k = 0; k < 6; k++ )
            {
                Vector6d xk_inc  = Vector6d::Zero(); xk_inc(k) = delta;
                Matrix4d Tk_inc  = T * expmap_se3(xk_inc) ;
                Vector4d xk_err  = u - inverse_se3(Tk_inc).transpose() * v;
                J.block(0,k,4,1) = ( xk_err - ei ) / delta;
            }
            // if employing robust function
            double w = 1.0;
            if( robust )
                w = robustKernel( ei_sq );
            // update Hessian, gradient (neg), and error
            H += J.transpose() * J  * w;
            g -= J.transpose() * ei * w;
            e += ei_sq * w;
        }

        // if the difference is very small stop
        if( ( abs(e-err_prev) < min_err_change ) || ( e < min_err ) )
            break;

        // update H, estimate increment
        for( int j = 0; j < 6; j++ )
            H(j,j) += lambda * H(j,j);
        LDLT<Matrix6d> solver(H);
        Xinc = solver.solve(g);

        // update LM
        if( e > err_prev )
        {
            lambda /= lambda_k;
        }
        else
        {
            lambda *= lambda_k;
            T = T * expmap_se3(Xinc);
        }
//        cout << endl << "Iteration " << i << "\t" << e;

        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( Xinc.norm() < numeric_limits<double>::epsilon() )
            break;

        // update previous values
        err_prev = e;

    }

    return T.cast<float>();

    // show solution (DBG)
//    Vector6d calib_err = logmap_se3( T * inverse_se3(Tgt) );
//    cout << endl << endl;
//    cout << "Final residue: " << err_prev << endl << endl << endl;
//    cout << "Rotation error (rad)  = " << calib_err.tail(3).norm() << endl ;
//    cout << "Translation error (m) = " << calib_err.head(3).norm() << endl << endl << endl;
//    cout << "Calibration: " << endl << "---------------------------------------" << endl << endl;
//    cout << T << endl << endl << endl;
//    cout << "Uncertainty matrix: " << endl << "---------------------------------------" << endl << endl;
//    cout << H.inverse() << endl << endl;

}

double Calibration::robustKernel(double e)
{
    return 1.0 / ( 1.0 + e ) ;
}
