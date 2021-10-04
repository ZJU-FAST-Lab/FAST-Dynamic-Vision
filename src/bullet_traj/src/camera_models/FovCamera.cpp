#include <camera_models/FovCamera.h>

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <code_utils/eigen_utils.h>
#include <gpl/gpl.h>

namespace camera_model
{

FovCamera::FovCamera( )
: m_inv_K11( 1.0 )
, m_inv_K13( 0.0 )
, m_inv_K22( 1.0 )
, m_inv_K23( 0.0 )
{
}

FovCamera::FovCamera( const std::string& cameraName, int imageWidth, int imageHeight, double omg, double fx, double fy, double u0, double v0 )
: mParameters( cameraName, imageWidth, imageHeight, omg, fx, fy, u0, v0 )
{
}

FovCamera::FovCamera( const FovCamera::Parameters& params )
: mParameters( params )
{
    calcKinvese( params.fx( ), params.fy( ), params.u0( ), params.v0( ) );
}

Camera::ModelType
FovCamera::modelType( ) const
{
    return mParameters.modelType( );
}

const std::string&
FovCamera::cameraName( ) const
{
    return mParameters.cameraName( );
}

int
FovCamera::imageWidth( ) const
{
    return mParameters.imageWidth( );
}

int
FovCamera::imageHeight( ) const
{
    return mParameters.imageHeight( );
}

void
FovCamera::spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) const
{
    //    double theta = acos(P(2) / P.norm());
    //    double phi   = atan2(P(1), P(0));

    double ru_p = ru( P );
    double rd_p = rd( mParameters.omg( ), ru_p );

    Eigen::Vector2d p_u( P( 0 ) / P( 2 ) * rd_p / ru_p, P( 1 ) / P( 2 ) * rd_p / ru_p );

    // Apply generalised projection matrix
    p( 0 ) = mParameters.fx( ) * p_u( 0 ) + mParameters.u0( );
    p( 1 ) = mParameters.fy( ) * p_u( 1 ) + mParameters.v0( );
    //    std::cout<<"p " <<p.transpose()<<std::endl;
}

void
FovCamera::spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p, float image_scalse ) const
{
    Eigen::Vector2d p_tmp;
    spaceToPlane( P, p_tmp );
    p = p_tmp * image_scalse;
}

/**
 * \brief Project a 3D point to the image plane and calculate Jacobian
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
FovCamera::spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p, Eigen::Matrix< double, 2, 3 >& J ) const
{
    double ru_p = ru( P );
    double rd_p = rd( mParameters.omg( ), ru_p );

    Eigen::Vector2d p_u( P( 0 ) / P( 2 ) * rd_p / ru_p, P( 1 ) / P( 2 ) * rd_p / ru_p );

    // Apply generalised projection matrix
    p << mParameters.fx( ) * p_u( 0 ) + mParameters.u0( ), mParameters.fy( ) * p_u( 1 ) + mParameters.v0( );
}

void
FovCamera::estimateIntrinsics( const cv::Size& boardSize,
                               const std::vector< std::vector< cv::Point3f > >& objectPoints,
                               const std::vector< std::vector< cv::Point2f > >& imagePoints )
{
    // Z. Zhang, A Flexible New Technique for Camera Calibration, PAMI 2000

    Parameters params = getParameters( );

    params.omg( ) = 0.0;

    double cx    = params.imageWidth( ) / 2.0;
    double cy    = params.imageHeight( ) / 2.0;
    params.u0( ) = cx;
    params.v0( ) = cy;

    size_t nImages = imagePoints.size( );

    cv::Mat A( nImages * 2, 2, CV_64F );
    cv::Mat b( nImages * 2, 1, CV_64F );

    for ( size_t i = 0; i < nImages; ++i )
    {
        const std::vector< cv::Point3f >& oPoints = objectPoints.at( i );

        std::vector< cv::Point2f > M( oPoints.size( ) );
        for ( size_t j = 0; j < M.size( ); ++j )
        {
            M.at( j ) = cv::Point2f( oPoints.at( j ).x, oPoints.at( j ).y );
        }

        cv::Mat H = cv::findHomography( M, imagePoints.at( i ) );

        H.at< double >( 0, 0 ) -= H.at< double >( 2, 0 ) * cx;
        H.at< double >( 0, 1 ) -= H.at< double >( 2, 1 ) * cx;
        H.at< double >( 0, 2 ) -= H.at< double >( 2, 2 ) * cx;
        H.at< double >( 1, 0 ) -= H.at< double >( 2, 0 ) * cy;
        H.at< double >( 1, 1 ) -= H.at< double >( 2, 1 ) * cy;
        H.at< double >( 1, 2 ) -= H.at< double >( 2, 2 ) * cy;

        double h[3], v[3], d1[3], d2[3];
        double n[4] = { 0, 0, 0, 0 };

        for ( int j = 0; j < 3; ++j )
        {
            double t0 = H.at< double >( j, 0 );
            double t1 = H.at< double >( j, 1 );
            h[j]      = t0;
            v[j]      = t1;
            d1[j]     = ( t0 + t1 ) * 0.5;
            d2[j]     = ( t0 - t1 ) * 0.5;
            n[0] += t0 * t0;
            n[1] += t1 * t1;
            n[2] += d1[j] * d1[j];
            n[3] += d2[j] * d2[j];
        }

        for ( int j = 0; j < 4; ++j )
        {
            n[j] = 1.0 / sqrt( n[j] );
        }

        for ( int j = 0; j < 3; ++j )
        {
            h[j] *= n[0];
            v[j] *= n[1];
            d1[j] *= n[2];
            d2[j] *= n[3];
        }

        A.at< double >( i * 2, 0 )     = h[0] * v[0];
        A.at< double >( i * 2, 1 )     = h[1] * v[1];
        A.at< double >( i * 2 + 1, 0 ) = d1[0] * d2[0];
        A.at< double >( i * 2 + 1, 1 ) = d1[1] * d2[1];
        b.at< double >( i * 2, 0 )     = -h[2] * v[2];
        b.at< double >( i * 2 + 1, 0 ) = -d1[2] * d2[2];
    }

    cv::Mat f( 2, 1, CV_64F );
    cv::solve( A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU );

    params.fx( ) = sqrt( fabs( 1.0 / f.at< double >( 0 ) ) );
    params.fy( ) = sqrt( fabs( 1.0 / f.at< double >( 1 ) ) );

    params.omg( ) = 2 * atan( sqrt( params.u0( ) * params.u0( ) + params.v0( ) * params.v0( ) ) / params.fx( ) );

    setParameters( params );
}

void
FovCamera::liftSphere( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const
{
    liftProjective( p, P );
}

void
FovCamera::rayToPlane( const Ray& ray, Eigen::Vector2d& p ) const
{
    Eigen::Vector3d P( sin( ray.theta( ) ) * cos( ray.phi( ) ), sin( ray.theta( ) ) * sin( ray.phi( ) ),
                       cos( ray.theta( ) ) );

    double ru_p = ru( P );
    double rd_p = rd( mParameters.omg( ), ru_p );

    Eigen::Vector2d p_u( P( 0 ) / P( 2 ) * rd_p / ru_p, P( 1 ) / P( 2 ) * rd_p / ru_p );

    // Apply generalised projection matrix
    p << mParameters.fx( ) * p_u( 0 ) + mParameters.u0( ), mParameters.fy( ) * p_u( 1 ) + mParameters.v0( );
}

void
FovCamera::liftProjectiveToRay( const Eigen::Vector2d& p, Ray& ray ) const
{
    //    backprojectSymmetric(
    //                Eigen::Vector2d(m_inv_K11 * p(0) + m_inv_K13,
    //                                m_inv_K22 * p(1) + m_inv_K23),
    //                ray.theta(), ray.phi());
}

void
FovCamera::liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const
{
    // Lift oints to normalised plane
    double theta, phi;
    // Obtain a projective ray

    Eigen::Vector2d p_u( ( p( 0 ) - mParameters.u0( ) ) / mParameters.fx( ),
                         ( p( 1 ) - mParameters.v0( ) ) / mParameters.fy( ) );
    double rd = p_u.norm( );

    //    if (rd < 1e-10)
    //        phi = 0.0;
    //    else
    //        phi = atan2(p_u(1), p_u(0));
    double ru_p = rd_inverse( mParameters.omg( ), rd );

    P = Eigen::Vector3d( ( p_u( 0 ) ) * ru_p / rd, ( p_u( 1 ) ) * ru_p / rd, 1.0 );
    //    std::cout<<"liftProjective " <<P.transpose()<<std::endl;
}

void
FovCamera::liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P, float image_scale ) const
{
    Eigen::Vector2d p_tmp = p / image_scale; // p_tmp is without resize, p is with resize
    liftProjective( p_tmp, P );              // p_tmp is without resize
}

void
FovCamera::undistToPlane( const Eigen::Vector2d& p_u, Eigen::Vector2d& p ) const
{
}

cv::Mat
FovCamera::initUndistortRectifyMap(
cv::Mat& map1, cv::Mat& map2, float fx, float fy, cv::Size imageSize, float cx, float cy, cv::Mat rmat ) const
{
    if ( imageSize == cv::Size( 0, 0 ) )
    {
        imageSize = cv::Size( mParameters.imageWidth( ), mParameters.imageHeight( ) );
    }

    cv::Mat mapX = cv::Mat::zeros( imageSize.height, imageSize.width, CV_32F );
    cv::Mat mapY = cv::Mat::zeros( imageSize.height, imageSize.width, CV_32F );

    Eigen::Matrix3f K_rect;

    if ( cx == -1.0f && cy == -1.0f )
    {
        K_rect << fx, 0, imageSize.width / 2, 0, fy, imageSize.height / 2, 0, 0, 1;
    }
    else
    {
        K_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    }

    if ( fx == -1.0f || fy == -1.0f )
    {
        K_rect( 0, 0 ) = mParameters.fx( );
        K_rect( 1, 1 ) = mParameters.fy( );
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse( );

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen( rmat, R );
    R_inv = R.inverse( );

    for ( int v = 0; v < imageSize.height; ++v )
    {
        for ( int u = 0; u < imageSize.width; ++u )
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            // TODO FIXME
            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane( uo.cast< double >( ), p );

            mapX.at< float >( v, u ) = p( 0 );
            mapY.at< float >( v, u ) = p( 1 );
        }
    }

    cv::convertMaps( mapX, mapY, map1, map2, CV_32FC1, false );

    cv::Mat K_rect_cv;
    cv::eigen2cv( K_rect, K_rect_cv );
    return K_rect_cv;
}

int
FovCamera::parameterCount( ) const
{
    return FOV_PARAM_NUM;
}

const FovCamera::Parameters&
FovCamera::getParameters( ) const
{
    return mParameters;
}

void
FovCamera::setParameters( const FovCamera::Parameters& parameters )
{
    mParameters = parameters;

    calcKinvese( parameters.fx( ), parameters.fy( ), parameters.u0( ), parameters.v0( ) );

    //    std::cout<<"setParameters " <<this->parametersToString()<<std::endl;
}

void
FovCamera::readParameters( const std::vector< double >& parameterVec )
{
    if ( int( parameterVec.size( ) ) != parameterCount( ) )
    {
        return;
    }

    Parameters params = getParameters( );

    params.omg( ) = parameterVec.at( 0 );
    params.fx( )  = parameterVec.at( 1 );
    params.fy( )  = parameterVec.at( 2 );
    params.u0( )  = parameterVec.at( 3 );
    params.v0( )  = parameterVec.at( 4 );

    setParameters( params );
}

void
FovCamera::writeParameters( std::vector< double >& parameterVec ) const
{
    parameterVec.resize( parameterCount( ) );

    parameterVec.at( 0 ) = mParameters.omg( );
    parameterVec.at( 1 ) = mParameters.fx( );
    parameterVec.at( 2 ) = mParameters.fy( );
    parameterVec.at( 3 ) = mParameters.u0( );
    parameterVec.at( 4 ) = mParameters.v0( );
}

void
FovCamera::writeParametersToYamlFile( const std::string& filename ) const
{
    mParameters.writeToYamlFile( filename );
}

std::string
FovCamera::parametersToString( ) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str( );
}

bool
FovCamera::calcKinvese( double fx, double fy, double u0, double v0 )
{
    eigen_utils::Matrix K( 3, 3 );
    K << fx, 0, u0, 0, fy, v0, 0, 0, 1;

    eigen_utils::Matrix K_inv = K.inverse( );
    //  std::cout << " K file: "<< K << std::endl;
    //  std::cout << " K_inv file: "<< K_inv << std::endl;

    // Inverse camera projection matrix parameters
    m_inv_K11 = K_inv( 0, 0 );
    m_inv_K13 = K_inv( 0, 2 );
    m_inv_K22 = K_inv( 1, 1 );
    m_inv_K23 = K_inv( 1, 2 );
}

double
FovCamera::getInv_K23( ) const
{
    return m_inv_K23;
}

double
FovCamera::getInv_K22( ) const
{
    return m_inv_K22;
}

double
FovCamera::getInv_K13( ) const
{
    return m_inv_K13;
}

double
FovCamera::getInv_K11( ) const
{
    return m_inv_K11;
}

FovCamera::Parameters::Parameters( )
: Camera::Parameters( FOV )
, m_omg( 0.0 )
, m_fx( 1.0 )
, m_fy( 1.0 )
, m_u0( 0.0 )
, m_v0( 0.0 )
{
}

FovCamera::Parameters::Parameters( const std::string& cameraName, int w, int h, double omg, double fx, double fy, double u0, double v0 )
: Camera::Parameters( FOV, cameraName, w, h )
, m_omg( omg )
, m_fx( fx )
, m_fy( fy )
, m_u0( u0 )
, m_v0( v0 )
{
}

double&
FovCamera::Parameters::omg( )
{
    return m_omg;
}

double&
FovCamera::Parameters::fx( )
{
    return m_fx;
}

double&
FovCamera::Parameters::fy( )
{
    return m_fy;
}

double&
FovCamera::Parameters::u0( )
{
    return m_u0;
}

double&
FovCamera::Parameters::v0( )
{
    return m_v0;
}

double
FovCamera::Parameters::omg( ) const
{
    return m_omg;
}

double
FovCamera::Parameters::fx( ) const
{
    return m_fx;
}

double
FovCamera::Parameters::fy( ) const
{
    return m_fy;
}

double
FovCamera::Parameters::u0( ) const
{
    return m_u0;
}

double
FovCamera::Parameters::v0( ) const
{
    return m_v0;
}

bool
FovCamera::Parameters::readFromYamlFile( const std::string& filename )
{
    cv::FileStorage fs( filename, cv::FileStorage::READ );

    if ( !fs.isOpened( ) )
    {
        return false;
    }

    if ( !fs["model_type"].isNone( ) )
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if ( sModelType.compare( "FOV" ) != 0 )
        {
            return false;
        }
    }

    m_modelType = FOV;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth  = static_cast< int >( fs["image_width"] );
    m_imageHeight = static_cast< int >( fs["image_height"] );

    cv::FileNode n = fs["projection_parameters"];

    m_omg = static_cast< double >( n["omg"] );
    m_fx  = static_cast< double >( n["fx"] );
    m_fy  = static_cast< double >( n["fy"] );
    m_u0  = static_cast< double >( n["u0"] );
    m_v0  = static_cast< double >( n["v0"] );

    return true;
}

void
FovCamera::Parameters::writeToYamlFile( const std::string& filename ) const
{
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    fs << "model_type"
       << "FOV";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;

    // projection:  k2, k3, k4, k5, k6, k7, A11, A22, u0, v0
    fs << "projection_parameters";
    fs << "{";

    fs << "m_omg" << m_omg;
    fs << "m_fx " << m_fx;
    fs << "m_fy " << m_fy;
    fs << "m_u0 " << m_u0;
    fs << "m_v0 " << m_v0;

    fs << "}";

    fs.release( );
}

FovCamera::Parameters&
FovCamera::Parameters::operator=( const FovCamera::Parameters& other )
{
    if ( this != &other )
    {
        m_modelType   = other.m_modelType;
        m_cameraName  = other.m_cameraName;
        m_imageWidth  = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;

        m_omg = other.m_omg;
        m_fx  = other.m_fx;
        m_fy  = other.m_fy;
        m_u0  = other.m_u0;
        m_v0  = other.m_v0;
    }

    return *this;
}

std::ostream&
operator<<( std::ostream& out, const FovCamera::Parameters& params )
{
    out << "Camera Parameters:" << std::endl;
    out << "|    model_type| "
        << "FOV" << std::endl;
    out << "|   camera_name| " << params.m_cameraName << std::endl;
    out << "|   image_width| " << params.m_imageWidth << std::endl;
    out << "|  image_height| " << params.m_imageHeight << std::endl;

    // projection:  k2, k3, k4, k5, k6, k7, A11, A22, u0, v0
    out << "Projection Parameters" << std::endl;
    out << "|            m_omg| " << params.m_omg << std::endl;
    out << "|            m_fx | " << params.m_fx << std::endl;
    out << "|            m_fy | " << params.m_fy << std::endl;
    out << "|            m_u0 | " << params.m_u0 << std::endl;
    out << "|            m_v0 | " << params.m_v0 << std::endl;

    return out;
}
}
