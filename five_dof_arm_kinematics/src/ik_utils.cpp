#include <five_dof_arm_kinematics/ik_utils.h>
#include <kdl_conversions/kdl_msg.h>

void initializeLimits()
{
	min_angles.resize(5,0.0) ;
	min_angles[0] = -3.14159 ;
	min_angles[1] = 0.72 ;
	min_angles[2] = -2.0 ;
	min_angles[3] =	-2.5 ;
	min_angles[4] = -3.14159 ;

	max_angles.resize(5,0.0);
	max_angles[0] = 3.14159 ;
	max_angles[1] = 2.42 ;
	max_angles[2] = 2.0 ;
	max_angles[3] =	2.5 ;
	max_angles[4] = 3.14159 ;

}

std::vector<double> checkIK(geometry_msgs::Pose pose ,bool& b)
{
	std::stringstream sstr;
	std::cout.precision(5);
	std::vector<double> solution(5,0.0) ;
	KDL::Frame goal ;
	
	tf::poseMsgToKDL(pose,goal) ;
	
	double j1      = 0.0;
	double j2,j2_2 = 0.0;
	double j3,j3_2 = 0.0;
	double j4,j4_2 = 0.0;
	double j5      = 0.0;

	double h1 = 0.183 ;
	double l2 = 0.152 ;
	double l3 = 0.152 ;

	double R,P,Y, pitch ;
	
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
		b = false ;
		sstr << "Argument for j2 is out of range. No solution exists:Point is unreachable" << std::endl ;
		return solution ;
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
		b = false ;	
		sstr << "Argument for j3 is out of range. No solution exists:Point is unreachable" << std::endl ;
		return solution ;
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
		b = false ;
		sstr << "Argument for j3_2 is out of range. No solution exists:Point is unreachable" << std::endl ;
		return solution ;
	}

	j4   = pitch - (j2 + j3) ;
	j4_2 = pitch - (j2_2 + j3_2) ;
	j5 = R ;
	

	solution[0] = j1;
	solution[1] = j2;
	solution[2] = j3;
	solution[3] = j4;
	solution[4] = j5;
	
	sstr << "j1: " << solution[0] << std::endl ;
	sstr << "j2: " << solution[1] << std::endl ;
	sstr << "j2_2: " << j2_2 << std::endl ;
	sstr << "j3: " << solution[2] << std::endl ;
	sstr << "j3_2: " << j3_2 << std::endl ;
	sstr << "j4: " << solution[3] << std::endl ;
	sstr << "j4_2: " << j4_2 << std::endl ;
	sstr << "j5: " << solution[4] << std::endl ;
		
	if (isSolutionValid(solution))
	{
		b = true ;
		return solution ;
	}
	else
	{
		solution[1] = j2_2 ;
		solution[2] = j3_2 ;
		solution[3] = j4_2 ;
		if (isSolutionValid(solution))
		{
			b = true ;
			return solution ;
		}
	}

	b = false ;
	std::cout << sstr.str() << std::endl ;
	return solution ;
}

bool isSolutionValid(const std::vector<double> &solution)
{
    bool valid = true;

    if (solution.size() != 5) return false;

    for (unsigned int i = 0; i < solution.size(); i++) {
        if ((solution[i] < min_angles[i]) || (solution[i] > max_angles[i])) {
            valid = false;
        }
    }

    return valid;
}
