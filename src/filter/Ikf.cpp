/**\file Ikf.cpp
 */

#include <iostream> /**< IO C++ Standard library */
#include <algorithm> /**< Algorithm C++ Standard library */
#include <Eigen/LU> /**< Lineal algebra of Eigen */
#include <Eigen/SVD> /**< Singular Value Decomposition (SVD) of Eigen */
#include <base/Pose.hpp>
#include "Ikf.hpp" /**< Indirect Kalman Filter */

/** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
#ifndef Re
#define Re	6378137 /**< Equatorial radius in meters **/
#endif
#ifndef Rp
#define Rp	6378137 /**< Polar radius in meters **/
#endif
#ifndef ECC
#define ECC  0.0818191908426 /**< First eccentricity **/
#endif
#ifndef GRAVITY
#define GRAVITY 9.79766542 /**< Mean value of gravity value in m/s^2 **/
#endif
#ifndef GWGS0
#define GWGS0 9.7803267714 /**< Gravity value at the equator in m/s^2 **/
#endif
#ifndef GWGS1
#define GWGS1 0.00193185138639 /**< Gravity formula constant **/
#endif
#ifndef EARTHW
#define EARTHW  7.292115e-05 /**< Earth angular velocity in rad/s **/
#endif


//#define DEBUG_PRINTS 1

namespace stim300
{

    Ikf::~Ikf()
    {
        adapAttAcc.reset();
        adapAttIncl.reset();
    }

    /**
    * @brief This function Initialize the vectors and matrix of the IKF
    */
    void Ikf::Init(const Eigen::Matrix <double,Ikf::IKFSTATEVECTORSIZE,Ikf::IKFSTATEVECTORSIZE> &P_0,
                    const Eigen::Matrix <double,Ikf::NUMAXIS,Ikf::NUMAXIS> &Ra,
                    const Eigen::Matrix <double,NUMAXIS,NUMAXIS> &Rg,
                    const Eigen::Matrix <double,NUMAXIS,NUMAXIS> &Rm,
                    const Eigen::Matrix <double,NUMAXIS,NUMAXIS> &Ri,
                    const Eigen::Matrix <double,Ikf::NUMAXIS,Ikf::NUMAXIS> &Qbg,
                    const Eigen::Matrix <double,Ikf::NUMAXIS,Ikf::NUMAXIS> &Qba,
                    const Eigen::Matrix <double,Ikf::NUMAXIS,Ikf::NUMAXIS> &Qbi,
                    double g, double alpha,
                    unsigned int am1, unsigned int am2, double agamma,
                    unsigned int im1, unsigned int im2, double igamma)
    {

      /** Gravitation acceleration **/
      gtilde << 0, 0, g;

      /** Dip angle (alpha is in rad) **/
      mtilde(0) = cos(alpha);
      mtilde(1) = 0;
      mtilde(2) = -sin(alpha);


      /** Kalman filter state, error covariance and process noise covariance **/
      x = Eigen::Matrix <double,Ikf::IKFSTATEVECTORSIZE,1>::Zero();

      Q = Eigen::Matrix <double,Ikf::IKFSTATEVECTORSIZE,Ikf::IKFSTATEVECTORSIZE>::Zero();
      Q.block <NUMAXIS, NUMAXIS> (0,0) = 0.25 * (Rg);
      Q.block <NUMAXIS, NUMAXIS> (3,3) = (Qbg);
      Q.block <NUMAXIS, NUMAXIS> (6,6) = (Qba);
      Q.block <NUMAXIS, NUMAXIS> (9,9) = (Qbi);

      /** Initial error covariance **/
      P = (P_0);

      H1 = Eigen::Matrix <double,NUMAXIS,Ikf::IKFSTATEVECTORSIZE>::Zero();
      H2 = Eigen::Matrix <double,NUMAXIS,Ikf::IKFSTATEVECTORSIZE>::Zero();
      H3 = Eigen::Matrix <double,NUMAXIS,Ikf::IKFSTATEVECTORSIZE>::Zero();
      H1(0,6) = 1; H1(1,7) = 1; H1(2,8) = 1;
      H3(0,9) = 1; H3(1,10) = 1; H3(2,11) = 1;

      /** System matrix A **/
      A = Eigen::Matrix <double,Ikf::IKFSTATEVECTORSIZE,Ikf::IKFSTATEVECTORSIZE>::Zero();
      A(0,3) = -0.5;A(1,4) = -0.5;A(2,5) = -0.5;

      /** Initial bias **/
      bghat = Eigen::Matrix <double,NUMAXIS,1>::Zero();
      bahat = Eigen::Matrix <double,NUMAXIS,1>::Zero();
      bihat = Eigen::Matrix <double,NUMAXIS,1>::Zero();

      /** Default omega matrix **/
      oldomega4 << 0 , 0 , 0 , 0,
	  0 , 0 , 0 , 0,
          0 , 0 , 0 , 0,
          0 , 0 , 0 , 0;

	
      /** Initial quaternion in Init**/
      q4.w() = 1.00;
      q4.x() = 0.00;
      q4.y() = 0.00;
      q4.z() = 0.00;

      /** Default initial bias **/
      bghat << 0.00, 0.00, 0.00;
      bahat << 0.00, 0.00, 0.00;
      bihat << 0.00, 0.00, 0.00;

      /** Fill matrix Rg, Ra, Rm and Ri **/
      this->Ikf::Ra = Ra;
      this->Ikf::Rg = Rg;
      this->Ikf::Rm = Rm;
      this->Ikf::Ri = Ri;

      /** Initialize adaptive object **/
      initAdaptiveAttitude(am1, am2, agamma, im1, im2, igamma);

      /** Print filter information **/
      #ifdef DEBUG_PRINTS
      std::cout<< "P:\n"<<P<<"\n";
      std::cout<< "Q:\n"<<Q<<"\n";
      std::cout<< "H1:\n"<<H1<<"\n";
      std::cout<< "H2:\n"<<H2<<"\n";
      std::cout<< "H3:\n"<<H3<<"\n";
      std::cout<< "A:\n"<<A<<"\n";
      std::cout<< "mtilde:\n"<<mtilde<<"\n";
      std::cout<< "gtilde:\n"<<gtilde<<"\n";
      std::cout<< "Ra:\n"<<Ra<<"\n";
      std::cout<< "Rg:\n"<<Rg<<"\n";
      std::cout<< "Rm:\n"<<Rm<<"\n";
      std::cout<< "Ri:\n"<<Ri<<"\n";
      #endif

      return;
    }

    void Ikf::initAdaptiveAttitude(const unsigned int accM1, const unsigned int accM2, const double accGamma,
                                const unsigned int incM1, const unsigned int incM2, const double incGamma)
    {
        this->adapAttAcc.reset(new stim300::AdaptiveAttitudeCov (accM1, accM2, accGamma));
        this->adapAttIncl.reset(new stim300::AdaptiveAttitudeCov (incM1, incM2, incGamma));
    }

    void Ikf::setInitBias(const Eigen::Matrix<double, NUMAXIS, 1> &gbias,
            const Eigen::Matrix<double, NUMAXIS, 1> &abias,
            const Eigen::Matrix<double, NUMAXIS, 1> &ibias)
    {
        this->bghat = gbias;
        this->bahat = abias;
        this->bihat = ibias;

        return;
    }

    /**
    * @brief Set the current state vector of the filter
    */
    void Ikf::setState(const Eigen::Matrix< double, Ikf::IKFSTATEVECTORSIZE , 1  > &x_0)
    {
      x = x_0;

      return;
    }

    /**
    * @brief This function Initialize Attitude
    */
    bool Ikf::setAttitude(const Eigen::Quaternion< double > &initq)
    {
      if (&initq != NULL)
      {
	/** Initial orientation **/
	q4 = initq;
	
	return true;
      }

      return false;
    }

    /**
    * @brief This function set the initial Omega matrix
    */
    bool Ikf::setOmega(const Eigen::Matrix< double, Ikf::NUMAXIS , 1  > &u)
    {
      if (&u != NULL)
      {
	/** Initialization for quaternion integration **/
	oldomega4 << 0,-(u)(0), -(u)(1), -(u)(2),
	  (u)(0), 0, (u)(2), -(u)(1),
	  (u)(1), -(u)(2), 0, (u)(0),
	  (u)(2), (u)(1), -(u)(0), 0;

	return true;
      }
      return false;
    }

    /**
    * @brief Gravity
    */
    void Ikf::setGravity(const double gravity)
    {
        gtilde << 0.00, 0.00, gravity;
        return;
    }

    /**
    * @brief Set Noise covariance matrix
    */
    void Ikf::setCovariance(const Eigen::Matrix< double, Ikf::IKFSTATEVECTORSIZE , Ikf::IKFSTATEVECTORSIZE> &Pk)
    {
        P = Pk;
        return;
    }

    /**
    * @brief Gets the current orientation in Quaternion
    */
    Eigen::Quaternion< double > Ikf::getAttitude()
    {
      return q4;
    }

    /**
    * @brief Gets the current gyroscopes bias
    */
    Eigen::Matrix<double, Ikf::NUMAXIS, 1> Ikf::getGyroBias()
    {
        return this->bghat;
    }

    /**
    * @brief Gets the current accelerometers bias
    */
    Eigen::Matrix<double, Ikf::NUMAXIS, 1> Ikf::getAccBias()
    {
        return this->bahat;
    }

     /**
    * @brief Gets the current inclinometers bias
    */
    Eigen::Matrix<double, Ikf::NUMAXIS, 1> Ikf::getInclBias()
    {
        return this->bihat;
    }


    /**
    * @brief Gets the current state vector of the filter
    */
    Eigen::Matrix< double, Ikf::IKFSTATEVECTORSIZE , 1  > Ikf::getState()
    {
      return x;

    }

    /**
    * @brief Gets gravity in IMU body frame
    */
    Eigen::Matrix<double, Ikf::NUMAXIS, 1> Ikf::getGravityinBody()
    {
        return q4.inverse() * gtilde;
    }

    /**
    * @brief Gets Noise covariance matrix
    */
    Eigen::Matrix< double, Ikf::IKFSTATEVECTORSIZE , Ikf::IKFSTATEVECTORSIZE> Ikf::getCovariance()
    {
	return P;
    }


    /**
    * @brief Performs the prediction step of the filter.
    */
    void Ikf::predict(const Eigen::Matrix< double, Ikf::NUMAXIS , 1  > &u, double dt)
    {
      Eigen::Matrix <double,NUMAXIS,NUMAXIS> vec2product; /**< Vec 2 product  matrix */
      Eigen::Matrix <double,NUMAXIS,1> angvelo; /**< Vec 2 product  matrix */
      Eigen::Matrix <double,QUATERSIZE,QUATERSIZE> omega4; /**< Quaternion integration matrix */
      Eigen::Matrix <double,QUATERSIZE,1> quat; /**< Quaternion integration matrix */
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> dA; /**< Discrete System matrix */
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> Qd; /**< Discrete Q matrix */

      /** Compute the vector2product matrix with the angular velocity **/
      angvelo = (u) - bghat; /** Eliminate the Bias **/

      vec2product << 0, -angvelo(2), angvelo(1),
		    angvelo(2), 0, -angvelo(0),
		    -angvelo(1), angvelo(0), 0;

      /** Compute the dA Matrix **/
      A.block<NUMAXIS, NUMAXIS> (0,0) = -vec2product;
      dA = Eigen::Matrix<double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Identity() + A * dt + 0.5 *A * A * pow(dt,2);

      /** Propagate the vector through the system **/
      x = dA * x;
      Qd = Q*dt + 0.5*dt*A*Q + 0.5*dt*Q*A.transpose();
      Qd = 0.5*(Qd + Qd.transpose());//Guarantee symmetry
      P = dA*P*dA.transpose() + Qd;

      omega4 << 0,-angvelo(0), -angvelo(1), -angvelo(2),
		angvelo(0), 0, angvelo(2), -angvelo(1),
		angvelo(1), -angvelo(2), 0, angvelo(0),
		angvelo(2), angvelo(1), -angvelo(0), 0;
	
      quat(0) = q4.w();
      quat(1) = q4.x();
      quat(2) = q4.y();
      quat(3) = q4.z();

      /** Third-order gyroscopes integration accuracy **/
      quat = (Eigen::Matrix<double,QUATERSIZE,QUATERSIZE>::Identity() +(0.75 * omega4 *dt)-(0.25 * oldomega4 * dt) -
      ((1.0/6.0) * angvelo.squaredNorm() * pow(dt,2) *  Eigen::Matrix<double,QUATERSIZE,QUATERSIZE>::Identity()) -
      ((1.0/24.0) * omega4 * oldomega4 * pow(dt,2)) - ((1.0/48.0) * angvelo.squaredNorm() * omega4 * pow(dt,3))) * quat;

      q4.w() = quat(0);
      q4.x() = quat(1);
      q4.y() = quat(2);
      q4.z() = quat(3);
      q4.normalize();

      oldomega4 = omega4;

      return;

    }


    /**
    * @brief Performs the measurement and correction steps of the filter.
    */
    void Ikf::update(const Eigen::Matrix< double, Ikf::NUMAXIS , 1  > &acc,
            const Eigen::Matrix< double, Ikf::NUMAXIS , 1  > &incl, bool incl_on,
            const Eigen::Matrix< double, Ikf::NUMAXIS , 1  > &mag, bool magn_on)
    {
      Eigen::Matrix <double,NUMAXIS,NUMAXIS> vec2product; /**< Vec 2 product  matrix */
      Eigen::Matrix <double,NUMAXIS,NUMAXIS> fooR2; /**<  Measurement noise matrix from accelerometers matrix Ra*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> P1; /**< Error convariance matrix for measurement 1*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> P2; /**< Error convariance matrix for measurement 2*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> P3; /**< Error convariance matrix for measurement 3*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> auxM; /**< Auxiliar matrix for computing Kalman gain in measurement*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE, NUMAXIS> K1; /**< Kalman Gain matrix for measurement 1*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE, NUMAXIS> K2; /**< Kalman Gain matrix for measurement 2*/
      Eigen::Matrix <double,IKFSTATEVECTORSIZE, NUMAXIS> K3; /**< Kalman Gain matrix for measurement 3*/
      Eigen::Matrix <double,NUMAXIS,NUMAXIS> R1; /**< Acceleration covariance matrix */
      Eigen::Matrix <double,NUMAXIS,NUMAXIS> R3; /**< Inclinometers covariance matrix */
      Eigen::Quaternion <double> qe;  /**< Attitude error quaternion */
      Eigen::Matrix <double,NUMAXIS,1> gtilde_body; /**< Gravitation in the body frame */
      Eigen::Matrix <double,NUMAXIS,1> mtilde_body; /**< Magnetic field in the body frame */
      Eigen::Matrix <double,NUMAXIS,1> z1; /**< Measurement vector 1 Acc */
      Eigen::Matrix <double,NUMAXIS,1> z2; /**< Measurement vector 2 Mag */
      Eigen::Matrix <double,NUMAXIS,1> z3; /**< Measurement vector 3 Incl */
      Eigen::Matrix <double,NUMAXIS,1> auxvector; /**< Auxiliar vector variable */


      /**----------------------- **/
      /** Measurement step 1 Acc **/
      /**----------------------- **/

      /** First measurement step (Pitch and Roll correction from Acc) **/
      gtilde_body = q4.inverse() * gtilde;
      vec2product << 0, -gtilde_body(2), gtilde_body(1),
		    gtilde_body(2), 0, -gtilde_body(0),
		    -gtilde_body(1), gtilde_body(0), 0;

      H1.block<NUMAXIS, NUMAXIS> (0,0) = 2*vec2product;

      /** Measurement **/
      z1 = (acc) - bahat - gtilde_body;

      #ifdef DEBUG_PRINTS
      std::cout<<"acc:\n"<<acc<<"\n";
      std::cout<<"z1:\n"<<z1<<"\n";
      std::cout<<"g_body:\n"<<gtilde_body<<"\n";
      #endif

      /** The adaptive algorithm **/
      R1 = adapAttAcc->matrix<IKFSTATEVECTORSIZE> (x, P, z1, H1, Ra);

      /** Compute the Kalman Gain Matrix **/
      P1 = P;
      Eigen::Matrix<double, NUMAXIS, NUMAXIS> S1, S1_inverse;
      S1 = H1 * P1 * H1.transpose() + R1;
      S1_inverse = S1.inverse();
      K1 = P1 * H1.transpose() * S1_inverse;
      Eigen::Matrix<double, NUMAXIS, 1> innovation = (z1 - H1 * x);

      /** Update the state vector and the covariance matrix **/
      x = x + K1 * innovation;
      P = (Eigen::Matrix<double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Identity()
              -K1*H1)*P*(Eigen::Matrix<double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Identity()
              -K1*H1).transpose() + K1*R1*K1.transpose();
      P = 0.5 * (P + P.transpose());//Guarantee symmetry

      #ifdef DEBUG_PRINTS
      std::cout<<"x(k+1|k+1):\n"<<x<<"\n";
      std::cout<<"P(k+1|k+1):\n"<<P<<"\n";
      std::cout<<"innovation:\n"<<innovation<<"\n";
      std::cout<<"K1:\n"<<K1<<"\n";
      std::cout<<"R1:\n"<<R1<<"\n";
      #endif

      /** Update the quaternion with the Indirect approach **/
      /** This is necessary mainly because after(in the 2 measurement) C(q) is computed **/
      qe.w() = 1;
      qe.x() = x(0);
      qe.y() = x(1);
      qe.z() = x(2);
      q4 = q4 * qe;

      /** Normalize quaternion **/
      q4.normalize();

      /** Reset the quaternion part of the state vector **/
      x.block<NUMAXIS,1>(0,0) = Eigen::Matrix<double, NUMAXIS, 1>::Zero();


      /**------------------------- **/
      /** Measurement step 2 Mag   **/
      /** It only updates Yaw angle**/
      /**------------------------- **/

      if (magn_on)
      {

	/** Second measurement step **/
	mtilde_body = q4.inverse() * mtilde;
	vec2product << 0, -mtilde_body(2), mtilde_body(1),
		    mtilde_body(2), 0, -mtilde_body(0),
		    -mtilde_body(1), mtilde_body(0), 0;

	/** Observation matrix **/
	H2.block<NUMAXIS, NUMAXIS> (0,0) = 2*vec2product;

	/** Measurement vector **/
	z2 = (mag) - mtilde_body;

	P2 = Eigen::Matrix<double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE>::Zero();
	P2.block<NUMAXIS, NUMAXIS>(0,0) = P.block<NUMAXIS, NUMAXIS>(0,0);

	auxvector << 0, 0, 1;
	auxvector = q4.inverse() * auxvector;

	/** Compute Kalman Gain **/
	auxM = Eigen::Matrix<double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE>::Zero();
	auxM.block<NUMAXIS, NUMAXIS>(0,0) = auxvector * auxvector.transpose();
	K2 = auxM * P2 * H2.transpose() * (H2*P2*H2.transpose() + Rm).inverse();

	/** Update the state vector and the covariance matrix **/
	x = x + K2*(z2 - (H2*x));
	P = P - K2 * H2 * P - P * H2.transpose() * K2.transpose() + K2*(H2*P*H2.transpose() + Rm)*K2.transpose();
	P = 0.5 * (P + P.transpose());

	/** Update the quaternion with the Indirect approach **/
	qe.w() = 1;
	qe.x() = x(0);
	qe.y() = x(1);
	qe.z() = x(2);
	q4 = q4 * qe;

	/** Normalize quaternion **/
	q4.normalize();

	/** Reset the quaternion part of the state vector **/
	x.block<NUMAXIS,1>(0,0) = Eigen::Matrix<double, NUMAXIS, 1>::Zero();
      }

      if (incl_on)
      {
          /**--------------------------------- **/
          /** Measurement step 3 Inclinometers **/
          /**--------------------------------- **/

          /** Measurement step (Pitch and Roll correction from Inclinometers) **/
          gtilde_body = q4.inverse() * gtilde;
          vec2product << 0, -gtilde_body(2), gtilde_body(1),
                        gtilde_body(2), 0, -gtilde_body(0),
                        -gtilde_body(1), gtilde_body(0), 0;

          H3.block<NUMAXIS, NUMAXIS> (0,0) = 2*vec2product;

          /** Measurement **/
          z3 = (incl) - bihat - gtilde_body;

          #ifdef DEBUG_PRINTS
          std::cout<<"incl:\n"<<incl<<"\n";
          std::cout<<"z3:\n"<<z3<<"\n";
          std::cout<<"g_body:\n"<<gtilde_body<<"\n";
          #endif

          /** The adaptive algorithm **/
          R3 = adapAttIncl->matrix<IKFSTATEVECTORSIZE> (x, P, z3, H3, Ri);

          /** Compute the Kalman Gain Matrix **/
          P3 = P;
          Eigen::Matrix<double, NUMAXIS, NUMAXIS> S3, S3_inverse;
          S3 = H3 * P3 * H3.transpose() + R3;
          S3_inverse = S3.inverse();
          K3 = P3 * H3.transpose() * S3_inverse;
          innovation = (z3 - H3 * x);

          /** Update the state vector and the covariance matrix **/
          x = x + K3 * innovation;
          P = (Eigen::Matrix<double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Identity()
                  -K3*H3)*P*(Eigen::Matrix<double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Identity()
                  -K3*H3).transpose() + K3*R3*K3.transpose();
          P = 0.5 * (P + P.transpose());//Guarantee symmetry

          #ifdef DEBUG_PRINTS
          std::cout<<"x(k+1|k+1):\n"<<x<<"\n";
          std::cout<<"P(k+1|k+1):\n"<<P<<"\n";
          std::cout<<"innovation:\n"<<innovation<<"\n";
          std::cout<<"K3:\n"<<K3<<"\n";
          std::cout<<"R3:\n"<<R3<<"\n";
          #endif

          /** Update the quaternion with the Indirect approach **/
          /** This is necessary mainly because after(in the 2 measurement) C(q) is computed **/
          qe.w() = 1;
          qe.x() = x(0);
          qe.y() = x(1);
          qe.z() = x(2);
          q4 = q4 * qe;

          /** Normalize quaternion **/
          q4.normalize();

          /** Reset the quaternion part of the state vector **/
          x.block<NUMAXIS,1>(0,0) = Eigen::Matrix<double, NUMAXIS, 1>::Zero();
      }

      /**---------------------------- **/
      /** Reset the rest of the state **/
      /**---------------------------- **/
      bghat = bghat + x.block<NUMAXIS, 1> (3,0);
      x.block<NUMAXIS, 1> (3,0) = Eigen::Matrix <double, NUMAXIS, 1>::Zero();

      bahat = bahat + x.block<NUMAXIS, 1> (6,0);
      x.block<NUMAXIS, 1> (6,0) = Eigen::Matrix <double, NUMAXIS, 1>::Zero();

      bihat = bihat + x.block<NUMAXIS, 1> (9,0);
      x.block<NUMAXIS, 1> (9,0) = Eigen::Matrix <double, NUMAXIS, 1>::Zero();

      #ifdef DEBUG_PRINTS
      std::cout<<"bahat:\n"<<bahat<<"\n";
      std::cout<<"bghat:\n"<<bghat<<"\n";
      std::cout<<"bihat:\n"<<bihat<<"\n";
      #endif


      return;
    }


    /**
    * @brief Conversion Quaternion to DCM (Direct Cosine Matrix) (Alternative to Eigen)
    */
    void Ikf::Quaternion2DCM(Eigen::Quaternion< double >* q, Eigen::Matrix< double, Ikf::NUMAXIS, Ikf::NUMAXIS  >*C)
    {
      double q0, q1, q2, q3;

      if (C != NULL)
      {
	/** Take the parameters of the quaternion */
	q0 = q->w();
	q1 = q->x();
	q2 = q->y();
	q3 = q->z();
	
	/** Create the DCM matrix from the actual quaternion */
	(*C)(0,0) = 2 * q0 * q0 + 2 * q1 * q1 - 1;
	(*C)(0,1) = 2 * q1 * q2 + 2 * q0 * q3;
	(*C)(0,2) = 2 * q1 * q3 - 2 * q0 * q2;
	(*C)(1,0) = 2 * q1 * q2 - 2 * q0 * q3;
	(*C)(1,1) = 2 * q0 * q0 + 2 * q2 * q2 - 1;
	(*C)(1,2) = 2 * q2 * q3 + 2 * q0 * q1;
	(*C)(2,0) = 2 * q1 * q3 + 2 * q0 * q2;
	(*C)(2,1) = 2 * q2 * q3 - 2 * q0 * q1;
	(*C)(2,2) = 2 * q0 * q0 + 2 * q3 * q3 - 1;	
      }

      return;
    }

}
