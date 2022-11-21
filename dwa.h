#ifndef __DWA_H
#define __DWA_H

#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include "data_container.h"

#include <limits>

namespace dwa
{


template<typename T>
class DWA
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
	using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;
	using Vector4 = typename Eigen::Matrix<DataType, 4, 1>;
	using Vector5 = typename Eigen::Matrix<DataType, 5, 1>;
	using Vector6 = typename Eigen::Matrix<DataType, 6, 1>;

	DWA()
	{

	}

	~DWA()
	{

	}

	const Vector2 processOnce( const Vector5& curr, const Vector2& goal, const sensor::ScanContainer& scan, std::vector<std::vector<Vector2>>& traj_vec )
	{
		Vector2 u = Vector2::Zero();		

		Vector4 window = cacuDynamicWindow( curr );
		std::cout<<"window = "<<window.transpose()<<std::endl;		

		std::vector<Vector6> eval_vec;
		evaluation( curr, window, goal, scan, eval_vec, traj_vec );
		
		//std::cout<<"traj_vec.size = "<<traj_vec.size()<<std::endl;
		//for( auto& traj : traj_vec ){
		//	std::cout<<"traj.size = "<<traj.size()<<std::endl;
		//}

		if( eval_vec.empty() ){
			std::cout<<"No path to planer !"<<std::endl;
			return u;
		}

		normalizeEval( eval_vec );

		for( int i = 0; i < eval_vec.size(); i ++ ){
			eval_vec[i][5] = 0.5 * eval_vec[i][2] + 0.5 * eval_vec[i][3] + 0.3 * eval_vec[i][4];
		}		

		DataType max_score = std::numeric_limits<DataType>::min();
		int index = -1;
		for( int i = 0; i < eval_vec.size(); i ++ ){
			std::cout<<"score ["<<i<<"] = "<<eval_vec[i][5]<<std::endl;
			if( max_score < eval_vec[i][5] ){
				max_score = eval_vec[i][5];
				index = i;
			}
		}
		std::cout<<"max socore index = "<<index<<std::endl;

		if( index != -1 ){
			u[0] = eval_vec[index][0];
			u[1] = eval_vec[index][1];
		}

		std::cout<<"max score = "<<max_score<<std::endl;
		
		return u;
	}	

private:
	// curr : current state
	// u : (v, w)
	const Vector5 motionModel( const Vector5& curr, const Vector2& u )
	{
		Vector5 next;
		next[0] = curr[0] + u[0] * dt * ::cos( curr[2] );
		next[1] = curr[1] + u[0] * dt * ::sin( curr[2] );
		next[2] = curr[2] + u[1] * dt;
	
		next[3] = u[0];
		next[4] = u[1];

		return next;
	} 

	// state : ( x, y, theta, v, w )
	const Vector4 cacuDynamicWindow( const Vector5& state )
	{
		Vector4 velocity_window;
		
		velocity_window[0] = std::max( 0.03f, state[3] - linear_acc * dt ); // minimum linear velocity
		velocity_window[1] = std::min( max_linear_velo, state[3] + linear_acc * dt ); // maxmum linear velocity
		velocity_window[2] = std::max( -max_angular_velo, state[4] - angular_acc * dt ); // minimum angular velocity
		velocity_window[3] = std::min( max_angular_velo, state[4] + angular_acc * dt ); // maxmum angular velocity

		return velocity_window;
	}

	const Vector5 generateTrajectory( const Vector5& curr, std::vector<Vector2>& traj, const DataType vt, const DataType wt )
	{
		DataType time = 0.0;

		traj.clear();
		traj.push_back( Vector2( curr[0], curr[1] ) );
	
		Vector5 predict_state = curr;
		while( time < prediction_time ){
			time += dt;
			predict_state = motionModel( predict_state, Vector2( vt, wt ) );
			//std::cout<<"predicted state = "<<predict_state.transpose()<<std::endl;
	
			traj.push_back( Vector2( predict_state[0], predict_state[1] ) );
		}	
	
		return predict_state;
	}

	const DataType cacuHeadingEval( const Vector5& state, const Vector2& goal )
	{
		//DataType robot_heading = state[2] / M_PI * 180;
		//DataType goal_heading = ::atan2( goal[1] - state[1], goal[0] - state[0] ) / M_PI * 180;
		DataType robot_heading = state[2];	
		DataType goal_heading = ::atan2( goal[1] - state[1], goal[0] - state[0] );
		//std::cout<<"robot_heading : "<<robot_heading<<", goal_heading : "<<goal_heading<<std::endl;	

		DataType target_heading = std::abs( goal_heading - robot_heading );
		return M_PI - target_heading;
	}

	const DataType cacuDistEval( const Vector5& state, const sensor::ScanContainer& scan )
	{
		DataType dist_min = std::numeric_limits<DataType>::max();
		int index = -1;
		for( int i = 0; i < scan.getSize(); i ++ ){
			Vector2 tmp = scan.getIndexData(i);
			Vector2 pose( state[0], state[1] );
			//std::cout<<"pose = "<<pose.transpose()<<std::endl;
			DataType dist = (pose - tmp).norm();
		
			if( dist_min > dist ){
				dist_min = dist;
				index = i;
			}
		}
		std::cout<<"dist min = "<<dist_min<<", index = "<<index<<", scan[index] = "<<scan.getIndexData(index).transpose()<<std::endl;

		//if( dist_min > 2 * R ){
		//	dist_min = 2 * R;
		//}

		return dist_min;
	}

	const DataType cacuBreakingDist(DataType& vel, const DataType& acc)
        {
        	DataType stop_dist = 0;
        	while (vel > 0){
        		stop_dist += vel * dt; //制动距离的计算
        		vel -= acc * dt;
        	}
	
        	return stop_dist;
        }
	
	void evaluation( const Vector5& state, const Vector4& window, const Vector2& goal, const sensor::ScanContainer& scan, std::vector<Vector6>& eval_vec  )
	{
		std::vector<Vector2> traj;
		for( DataType v = window[0]; v <= window[1]; v += linear_velo_resolution ){
			for( DataType w = window[2]; w <= window[3]; w += angular_velo_resolution ){
		
				Vector5 x_predict = generateTrajectory( state, traj, v, w );
				DataType heading = cacuHeadingEval( x_predict, goal );		
				DataType dist = cacuDistEval( x_predict, scan );
				DataType vel = std::abs( v );
				DataType stop_dist = cacuBreakingDist( vel, linear_acc );
				std::cout<<"heading = "<<heading<<", dist = "<<dist<<", vel = "<<vel<<", stop_dist = "<<stop_dist<<std::endl;				

				if( dist > stop_dist ){
					Vector6 eval;
					eval << v, w, heading, dist, vel, 0;
					eval_vec.push_back( eval );
				}
			}
		}
	}
	
	void evaluation( const Vector5& state, const Vector4& window, const Vector2& goal, const sensor::ScanContainer& scan, std::vector<Vector6>& eval_vec, std::vector<std::vector<Vector2>>& traj_vec  )
        {
                std::vector<Vector2> traj;
                for( DataType v = window[0]; v <= window[1]; v += linear_velo_resolution ){
                        for( DataType w = window[2]; w <= window[3]; w += angular_velo_resolution ){
				std::cout<<"--------- v = "<<v<<", w = "<<w<<"----------"<<std::endl;
				
                                Vector5 x_predict = generateTrajectory( state, traj, v, w );
				traj_vec.push_back( traj );
				//std::cout<<"traj_vec.size = "<<traj_vec.size()<<std::endl;
			
				std::cout<<"x_predict = "<<x_predict.transpose()<<std::endl;
                                DataType heading = cacuHeadingEval( x_predict, goal );
                                DataType dist = cacuDistEval( x_predict, scan );
                                DataType vel = std::abs( v );
                                DataType stop_dist = cacuBreakingDist( vel, linear_acc );
				std::cout<<"heading = "<<heading<<", dist = "<<dist<<", vel = "<<vel<<", stop_dist = "<<stop_dist<<std::endl<<std::endl;

                                if( dist > stop_dist ){
                                        Vector6 eval;
                                        eval << v, w, heading, dist, vel, 0;
                                        eval_vec.push_back( eval );
                                }
                        }
                }
        }


	void normalizeEval( std::vector<Vector6>& eval_vec )
	{
		DataType sum_heading = 0;
		DataType sum_dist = 0;
		DataType sum_vel = 0;

		for( int i = 0; i < eval_vec.size(); i ++ ){
			sum_heading += eval_vec[i][2];
			sum_dist += eval_vec[i][3];
			sum_vel += eval_vec[i][4];
		}
		
		if( sum_heading != 0 ){
			for( int i = 0; i < eval_vec.size(); i ++ ){
				eval_vec[i][2] = eval_vec[i][2] / sum_heading;
			}
		}

		if( sum_dist != 0 ){
			for( int i = 0; i < eval_vec.size(); i ++ ){
                                eval_vec[i][3] = eval_vec[i][3] / sum_dist;
			}
		}
		
		if( sum_vel != 0 ){
			for( int i = 0; i < eval_vec.size(); i ++ ){
                                eval_vec[i][4] = eval_vec[i][4] / sum_vel;
                        }

		}
	}	 

private:

	// paramters
	static const DataType max_linear_velo;
	static const DataType max_angular_velo;
	static const DataType linear_acc;
	static const DataType angular_acc;

	static const DataType linear_velo_resolution;
	static const DataType angular_velo_resolution;

	static const DataType dt;
	static const DataType prediction_time;
	static const DataType R;
};

template<typename T>
const typename DWA<T>::DataType DWA<T>::max_linear_velo = 0.06;

template<typename T>
const typename DWA<T>::DataType DWA<T>::max_angular_velo = 1.0;

template<typename T>
const typename DWA<T>::DataType DWA<T>::linear_acc = 0.5;

template<typename T>
const typename DWA<T>::DataType DWA<T>::angular_acc = 5;

template<typename T>
const typename DWA<T>::DataType DWA<T>::linear_velo_resolution = 0.01;

template<typename T>
const typename DWA<T>::DataType DWA<T>::angular_velo_resolution = 0.1;

template<typename T>
const typename DWA<T>::DataType DWA<T>::dt = 0.1;

template<typename T>
const typename DWA<T>::DataType DWA<T>::prediction_time = 3.0;

template<typename T>
const typename DWA<T>::DataType DWA<T>::R = 0.01;

}

#endif
