/*
 * inverse_kinematics.cpp
 *
 *  Created on: 31-Aug-2016
 *      Author: rohin
 */

#include <five_dof_arm_kinematics/inverse_kinematics.h>

#include <iostream>
#include <sstream>

using namespace five_dof_arm_kinematics;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double InverseKinematics::ALMOST_PLUS_ONE = 0.9999999;
const double InverseKinematics::ALMOST_MINUS_ONE = -0.9999999;

std::stringstream sstr;

InverseKinematics::InverseKinematics(
        const std::vector<double> &min_angles,
        const std::vector<double> &max_angles,
        Logger &logger) : logger_(logger)
{
    min_angles_ = min_angles;
    max_angles_ = max_angles;
}

InverseKinematics::~InverseKinematics()
{
}

int InverseKinematics::CartToJnt(const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        std::vector<KDL::JntArray> &q_out)
{
    KDL::JntArray solution;
    bool bools[] = { true, false };

    // there are no solutions available yet
    q_out.clear();

    // iterate over all redundant solutions
    solution = ik(p_in);
    if (isSolutionValid(solution)) q_out.push_back(solution);

    if (q_out.size() > 0) {
        logger_.write("Inverse kinematics found a solution",
                __FILE__, __LINE__);

        return 1;
    } else {
        logger_.write("Inverse kinematics found no solution",
                __FILE__, __LINE__);

        return -1;
    }
}

KDL::JntArray InverseKinematics::ik(const KDL::Frame& g0)
{
	double j1      = 0.0;
	double j2,j2_2 = 0.0;
	double j3,j3_2 = 0.0;
	double j4      = 0.0;
	double j5      = 0.0;

	double h1 = 0.183 ;
	double l2 = 0.152 ;
	double l3 = 0.152 ;

	double R,P,Y, pitch ;
	KDL::Frame goal = g0 ;
	KDL::Rotation rot ;

	//goal.p.x(0.0621796) ;
	//goal.p.y(0.0485451) ;
	//goal.p.z(0.296658) ;

	//goal.M = rot.Quaternion( 0.639923, 0.317976, 0.136204, 0.686176) ;
    sstr << "Goal Position is: "
    		<< goal.p[0] << ","
    		<< goal.p[1] << ","
			<< goal.p[2]
			<< std::endl ;

    goal.M.GetRPY(R,P,Y) ;
    sstr << "RPY is: "
        		<< R << ","
        		<< P << ","
    			<< Y
    			<< std::endl ;

    // Joint 1
	KDL::Vector pos_d = goal.p ;

	double p_x = pos_d[0] ;
	double p_y = pos_d[1] ;
	double p_z = pos_d[2] ;

	j1 = atan2( p_y , p_x ) ;
	j5 = R ; // Last joint is Roll

	// Transform from frame 1 to frame 2
	KDL::Frame frame1_to_frame2(
			goal.M ,
			KDL::Vector(-0.14825, 0, 0));
	KDL::Frame g2 = goal * frame1_to_frame2 ;

	double xc = g2.p[0]  ;
	double yc = g2.p[1]  ;
	double zc = g2.p[2]  ;
	sstr << "xc: " << xc << std::endl ;
	sstr << "yc: " << yc << std::endl ;
	sstr << "zc: " << zc << std::endl ;
	
	pitch = atan2( (p_z - zc) , ((p_x - xc)/cos(j1))) ;
	sstr << "Pitch: " << pitch << std::endl ;

	double B1 =  xc  / (cos(j1))  ;
	sstr << "B1: " << B1 << std::endl ;
	double B2 = zc - h1 ;
	sstr << "B2: " << B2 << std::endl ;


	//Checking validity for theta2
	double arg2 = ( B1*B1 + B2*B2 + l2*l2 - l3*l3 )/( 2*l2*(sqrt(B1*B1 + B2*B2))) ;
	sstr << "arg2: " << arg2 << std::endl ;

	if ( arg2  <= 1 )
	{
		j2 = acos( arg2  ) + atan2(B2,B1) ;
		j2_2 = -acos( arg2  ) + atan2(B2,B1) ;
	}
	else
	{
		sstr << "Argument for j2 is out of range. No solution exists:Point is unreachable" << std::endl ;
	}

	//Checking validity for theta3
	double arg3 = (B2 - (l2 * sin(j2))) / l3 ;
	double arg3_2 = (B2 - (l2 * sin(j2_2))) / l3 ;

	if  ( arg3  <= 1 )
	{
		if ( l2*cos(j2) <= B1 )
		{
			j3 = asin( arg3 ) - j2 ;
		}
		if ( l2*cos(j2_2) > B1 )
		{
			j3_2 = ( 3.14 - asin( arg3_2 ) ) - j2_2 ;
		}
	}
	else
	{
		sstr << "Argument for j3 is out of range. No solution exists:Point is unreachable" << std::endl ;
	}

	//****************//

	if  ( arg3_2  <= 1 )
	{
		if ( l2*cos(j2_2) <= B1 )
		{
			j3_2 = asin( arg3_2 ) - j2_2 ;
		}
		if ( l2*cos(j2_2) > B1 )
		{
			j3_2 = ( 3.14 - asin( arg3_2 ) ) - j2_2 ;
		}
	}
	else
	{
		sstr << "Argument for j3_2 is out of range. No solution exists:Point is unreachable" << std::endl ;
	}

	j4 = pitch - (j2 + j3) ;
	j5 = R ;

	sstr << "j1: " << j1 << std::endl ;
	sstr << "j2: " << j2 << std::endl ;
	sstr << "j2_2: " << j2_2 << std::endl ;
	sstr << "j3: " << j3 << std::endl ;
	sstr << "j3_2: " << j3_2 << std::endl ;
	sstr << "j4: " << j4 << std::endl ;
	sstr << "j5: " << j5 << std::endl ;

	KDL::JntArray solution(5);
	solution(0) = j1;
	solution(1) = j2;
	solution(2) = j3;
	solution(3) = j4;
	solution(4) = j5;

	logger_.write(sstr.str(), __FILE__, __LINE__);

	return solution ;


}



KDL::Frame InverseKinematics::projectGoalOrientationIntoArmSubspace(
        const KDL::Frame &goal) const
{
    KDL::Vector y_t_hat = goal.M.UnitY();   // y vector of the rotation matrix
    KDL::Vector z_t_hat = goal.M.UnitZ();   // z vector of the rotation matrix

    // m_hat is the normal of the "arm plane"
    KDL::Vector m_hat(0, -1, 0);

    // k_hat is the vector about which rotation of the goal frame is performed
    KDL::Vector k_hat = m_hat * z_t_hat;        // cross product

    // z_t_hat_tick is the new pointing direction of the arm
    KDL::Vector z_t_hat_tick = k_hat * m_hat;   // cross product

    // the amount of rotation
    double cos_theta = KDL::dot(z_t_hat, z_t_hat_tick);
    // first cross product then dot product
    double sin_theta = KDL::dot(z_t_hat * z_t_hat_tick, k_hat);

    // use Rodriguez' rotation formula to perform the rotation
    KDL::Vector y_t_hat_tick = (cos_theta * y_t_hat)
            // k_hat * y_t_hat is cross product
            + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta)
            * (KDL::dot(k_hat, y_t_hat)) * k_hat;
    KDL::Vector x_t_hat_tick = y_t_hat_tick * z_t_hat_tick; // cross product

    KDL::Rotation rot(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick);

    // the frame uses the old position but has the new, projected orientation
    return KDL::Frame(rot, goal.p);
}


bool InverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
    bool valid = true;

    if (solution.rows() != 5) return false;

    for (unsigned int i = 0; i < solution.rows(); i++) {
        if ((solution(i) < min_angles_[i]) || (solution(i) > max_angles_[i])) {
            valid = false;
        }
    }

    return valid;
}

KDL::Frame projectGoalOrientationIntoArmSubspace1(const KDL::Frame &goal) ;

KDL::Frame projectGoalOrientationIntoArmSubspace1(const KDL::Frame &goal)
{
    KDL::Vector y_t_hat = goal.M.UnitY();   // y vector of the rotation matrix
    KDL::Vector z_t_hat = goal.M.UnitZ();   // z vector of the rotation matrix

    // m_hat is the normal of the "arm plane"
    KDL::Vector m_hat(0, -1, 0);

    // k_hat is the vector about which rotation of the goal frame is performed
    KDL::Vector k_hat = m_hat * z_t_hat;        // cross product

    // z_t_hat_tick is the new pointing direction of the arm
    KDL::Vector z_t_hat_tick = k_hat * m_hat;   // cross product

    // the amount of rotation
    double cos_theta = KDL::dot(z_t_hat, z_t_hat_tick);
    // first cross product then dot product
    double sin_theta = KDL::dot(z_t_hat * z_t_hat_tick, k_hat);

    // use Rodriguez' rotation formula to perform the rotation
    KDL::Vector y_t_hat_tick = (cos_theta * y_t_hat)
            // k_hat * y_t_hat is cross product
            + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta)
            * (KDL::dot(k_hat, y_t_hat)) * k_hat;
    KDL::Vector x_t_hat_tick = y_t_hat_tick * z_t_hat_tick; // cross product

    KDL::Rotation rot(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick);

    // the frame uses the old position but has the new, projected orientation
    return KDL::Frame(rot, goal.p);
}


