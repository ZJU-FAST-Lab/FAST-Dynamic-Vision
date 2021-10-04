#ifndef FOV_CAMERA_H
#define FOV_CAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"

#include "Camera.h"

#define FOV_PARAM_NUM 5

namespace camera_model
{

class FovCamera : public Camera
{
    public:
    class Parameters : public Camera::Parameters
    {
        public:
        Parameters( );
        Parameters( const std::string& cameraName, int w, int h, double omg, double fx, double fy, double u0, double v0 );

        double& omg( void );
        double& fx( void );
        double& fy( void );
        double& u0( void );
        double& v0( void );

        double omg( void ) const;
        double fx( void ) const;
        double fy( void ) const;
        double u0( void ) const;
        double v0( void ) const;

        bool readFromYamlFile( const std::string& filename );

        void writeToYamlFile( const std::string& filename ) const;

        Parameters& operator=( const Parameters& other );

        friend std::ostream& operator<<( std::ostream& out, const Parameters& params );

        private:
        // projection parameters
        double m_omg;
        double m_fx;
        double m_fy;
        double m_u0;
        double m_v0;
    };

    FovCamera( );

    FovCamera( const std::string& cameraName,
               int imageWidth,
               int imageHeight,
               double omg,
               double fx,
               double fy,
               double u0,
               double v0 );

    FovCamera( const Parameters& params );

    Camera::ModelType modelType( void ) const;
    const std::string& cameraName( void ) const;

    int imageWidth( void ) const;
    int imageHeight( void ) const;
    cv::Size imageSize( ) const { return cv::Size( imageWidth( ), imageHeight( ) ); }
    cv::Point2f getPrinciple( ) const
    {
        return cv::Point2f( mParameters.u0( ), mParameters.v0( ) );
    }

    void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) const;
    void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p, float image_scalse ) const;

    void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p, Eigen::Matrix< double, 2, 3 >& J ) const;

    void estimateIntrinsics( const cv::Size& boardSize,
                             const std::vector< std::vector< cv::Point3f > >& objectPoints,
                             const std::vector< std::vector< cv::Point2f > >& imagePoints );

    void setInitIntrinsics( const std::vector< std::vector< cv::Point3f > >& objectPoints,
                            const std::vector< std::vector< cv::Point2f > >& imagePoints )
    {
    }

    void liftSphere( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const;

    void rayToPlane( const Ray& ray, Eigen::Vector2d& p ) const;

    // Lift points from the image plane to the projective space
    void liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const;

    void liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P, float image_scale ) const;

    void liftProjectiveToRay( const Eigen::Vector2d& p, Ray& ray ) const;

    void undistToPlane( const Eigen::Vector2d& p_u, Eigen::Vector2d& p ) const;

    template< typename T >
    static void spaceToPlane( const T* const params,
                              const T* const q,
                              const T* const t,
                              const Eigen::Matrix< T, 3, 1 >& P,
                              Eigen::Matrix< T, 2, 1 >& p );

    // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale
    // =
    // 1.0) const = 0;
    cv::Mat initUndistortRectifyMap( cv::Mat& map1,
                                     cv::Mat& map2,
                                     float fx           = -1.0f,
                                     float fy           = -1.0f,
                                     cv::Size imageSize = cv::Size( 0, 0 ),
                                     float cx           = -1.0f,
                                     float cy           = -1.0f,
                                     cv::Mat rmat = cv::Mat::eye( 3, 3, CV_32F ) ) const;

    int parameterCount( void ) const;

    const Parameters& getParameters( void ) const;

    void setParameters( const Parameters& parameters );

    void readParameters( const std::vector< double >& parameterVec );

    void writeParameters( std::vector< double >& parameterVec ) const;

    void writeParametersToYamlFile( const std::string& filename ) const;

    std::string parametersToString( void ) const;

    double getInv_K11( ) const;
    double getInv_K12( ) const;
    double getInv_K13( ) const;
    double getInv_K22( ) const;
    double getInv_K23( ) const;

    private:
    template< typename T >
    static T rd( T omg, T ru );

    template< typename T >
    static T rd_inverse( T omg, T rd );

    template< typename T >
    static T ru( const Eigen::Matrix< T, 3, 1 >& P );

    bool calcKinvese( double fx, double fy, double u0, double v0 );

    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
};

typedef boost::shared_ptr< FovCamera > FovCameraPtr;
typedef boost::shared_ptr< const FovCamera > FovCameraConstPtr;

template< typename T >
T
FovCamera::rd( T omg, T ru )
{
    return T( 1 ) / ( omg )*atan( T( 2 ) * ru * tan( omg * 0.5 ) );
}

template< typename T >
T
FovCamera::rd_inverse( T omg, T rd )
{
    return tan( rd * omg ) / ( T( 2 ) * tan( omg * 0.5 ) );
}

template< typename T >
T
FovCamera::ru( const Eigen::Matrix< T, 3, 1 >& P )
{
    return sqrt( ( P( 0 ) * P( 0 ) + P( 1 ) * P( 1 ) ) / ( P( 2 ) * P( 2 ) ) );
}

template< typename T >
void
FovCamera::spaceToPlane( const T* const params,
                         const T* const q,
                         const T* const t,
                         const Eigen::Matrix< T, 3, 1 >& P,
                         Eigen::Matrix< T, 2, 1 >& p )
{
    T P_w[3];
    P_w[0] = T( P( 0 ) );
    P_w[1] = T( P( 1 ) );
    P_w[2] = T( P( 2 ) );

    // Eigen convention (x, y, z, w)
    // Ceres convention (w, x, y, z)
    T q_ceres[4] = { q[3], q[0], q[1], q[2] };

    T P_c[3];
    ceres::QuaternionRotatePoint( q_ceres, P_w, P_c );

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    T omg = params[0];
    T fx  = params[1];
    T fy  = params[2];
    T u0  = params[3];
    T v0  = params[4];

    T ru_p = sqrt( ( P_c[0] * P_c[0] + P_c[1] * P_c[1] ) / ( P_c[2] * P_c[2] ) );
    T rd_p = rd( omg, ru_p );

    Eigen::Matrix< T, 2, 1 > p_u
    = Eigen::Matrix< T, 2, 1 >( P( 0 ) / P( 2 ) * rd_p / ru_p, P( 1 ) / P( 2 ) * rd_p / ru_p );

    p( 0 ) = fx * p_u( 0 ) + u0;
    p( 1 ) = fy * p_u( 1 ) + v0;
}
}
#endif // FOV_CAMERA_H
