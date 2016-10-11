/*
 * test_kdl.cpp
 *
 *  Created on: 01-Sep-2016
 *      Author: rohin
 */


#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;

std::stringstream sstr;

int main()
{
	    double j1      = 0.0;
		double j2,j2_2 = 0.0;
		double j3,j3_2 = 0.0;
		double j4      = 0.0;
		double j5      = 0.0;

		double h1 = 0.183 ;
		double l2 = 0.152 ;
		double l3 = 0.152 ;

		double R,P,Y,pitch ;
		KDL::Frame goal ;
		KDL::Rotation rot ;

		goal.p.x(-0.116082) ;
		goal.p.y(0.0881662) ;
		goal.p.z(0.462849) ;

		goal.M = rot.Quaternion( 0.181643 , 0.545043 , 0.775306 , 0.262365 ) ;
		
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

		std::cout << sstr.str() << endl ;

        return 0 ;

}

