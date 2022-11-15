#ifndef __SIMULATOR_H
#define __SIMULATOR_H

#include <opencv2/opencv.hpp>

#include "dwa.h"
#include "data_container.h"

#include "file_read.h"

#define WORLD_WIDTH 800
#define WORLD_HEIGHT 800

#define IMAGE_WIDTH 900
#define IMAGE_HEIGHT 900
#define CENTER_X 450
#define CENTER_Y 450

#define SCALE_FACTOR 1000

namespace dwa
{

template<typename T>
class Simulator
{
public:
        using DataType = T;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
	using Vector5 = typename Eigen::Matrix<DataType, 5, 1>;

	Simulator()
	{

	}

	~Simulator()
	{

	}

	void initObtacles()
	{
		file::FileRead file_read;
		file_read.openFile( "test_data1" );
		sensor::RecordData data;
		file_read.readAFrame( data );
		//displayAScan( data.scan_data );
		lidarData2DataContainer( data.scan_data, scan );

		// 
		initial_pose << 0.0, 0.0, 0.0, 0.0, 0.0;
	
		drawTarget();
		drawObstacles();
	}

	void dwaProcess()
	{
		std::vector<std::vector<Vector2>> traj_vec;
		Vector2 u = dwa.processOnce( initial_pose, target_pose, scan, traj_vec );
		drawTrajectories( traj_vec );

		std::cout<<"u = "<<u.transpose()<<std::endl;
		motionModel( initial_pose, u);
		std::cout<<"robot pose = "<<initial_pose.transpose()<<std::endl;
		drawRobotPose();
	}


private:
	void drawRobotPose(  )
	{
		DataType img_x = initial_pose[0] * SCALE_FACTOR + 400;
                DataType img_y = initial_pose[1] * SCALE_FACTOR + 400;
		cv::circle(map, cv::Point2f(img_x, img_y), 3, cv::Scalar(0,  0, 255), -1);
		cv::imshow("map", map);
	}

	void drawTrajectories( const std::vector<std::vector<Vector2>>& traj_vec )
	{
		for( auto& traj : traj_vec ){
			for( auto& point : traj ){
				DataType img_x = point[0] * SCALE_FACTOR + 400;
				DataType img_y = point[1] * SCALE_FACTOR + 400;
				cv::circle(map, cv::Point2f(img_x, img_y), 2, cv::Scalar(0,  255, 0), -1);
			}
		}
		cv::imshow("map", map);
	}

	void drawTarget()
	{
		cv::circle(map, cv::Point2f(target_pose[0] * SCALE_FACTOR + 400, target_pose[1] * SCALE_FACTOR + 400), 3, cv::Scalar(0,  255, 0), -1);
	}

	void drawObstacles( )
	{
		for( int i = 0; i < scan.getSize(); i ++ ){
			DataType img_x = scan[i][0] * SCALE_FACTOR + 400;
			DataType img_y = scan[i][1] * SCALE_FACTOR + 400;
			cv::circle(map, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 0, 0), -1);
		}
		cv::imshow("map", map);
	}

	void lidarData2DataContainer( const sensor::LaserScan& scan, sensor::ScanContainer& scan_container )
        {
                for(size_t i = 0; i < 160; i ++){
                        DataType dist = static_cast<DataType>( scan.dists[i] ) * 0.001; // mm -> m
                        // judgement 
                        if( dist <= 0.005 || dist >= 0.3 ) {
                                DataType theta = scan.angles[i];
                        }else{

                                DataType theta = scan.angles[i];
                                DataType lidar_x = dist * cos( (M_PI / 180) * theta );
                                DataType lidar_y = dist * sin( (M_PI / 180) * theta );
                                std::cout << "dist : ( " << lidar_x << ", " << lidar_y << " )" << std::endl;
                                scan_container.addData( Eigen::Matrix<DataType, 2, 1>( lidar_x, lidar_y ));
                        }
                }
        }

	void displayAScan( const sensor::LaserScan& scan )
        {
                cv::Mat image = cv::Mat::zeros(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3);
                cv::line(image, cv::Point2f(CENTER_X, CENTER_Y), cv::Point2f(CENTER_X + 40, CENTER_Y), cv::Scalar(255, 255, 0), 4);
                cv::line(image, cv::Point2f(CENTER_X, CENTER_Y), cv::Point2f(CENTER_X, CENTER_Y - 40), cv::Scalar(0, 0, 255), 4);
                cv::line(image, cv::Point2f(0, CENTER_Y), cv::Point2f(IMAGE_WIDTH, CENTER_Y), cv::Scalar(0, 255, 0), 1);
                cv::line(image, cv::Point2f(CENTER_X, 0), cv::Point2f(CENTER_X, IMAGE_HEIGHT), cv::Scalar(0, 255, 0), 1);

                for(int i = 0; i < 160; i ++){
                        DataType dist = static_cast<DataType>( scan.dists[i] );
                        DataType theta = scan.angles[i];
                        DataType lidar_x = dist * cos( (M_PI / 180) * theta );
                        DataType lidar_y = dist * sin( (M_PI / 180) * theta );

                        //std::cout << "dist : ( " << lidar_x << ", " << lidar_y << " )" << std::endl;

                        DataType img_x = CENTER_X + lidar_x;
                        DataType img_y = CENTER_Y - lidar_y;

                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(255, 0, 0), -1);
                }

                cv::imshow( "scan", image );
                cv::waitKey(10);
        }
	
	void motionModel( Vector5& pose, const Vector2& u )
	{
		pose[0] += u[0] * dt * ::cos( pose[2] );
		pose[1] += u[0] * dt * ::sin( pose[2] );
		pose[2] += u[1] * dt;
	}

private:
	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT + 200, CV_8UC3, cv::Scalar(255, 255, 255 ) );

	DWA<DataType> dwa;
	sensor::ScanContainer scan;

	Vector5 initial_pose;	
	Vector2 target_pose = Vector2( 0.5, 0.0 );

	const DataType dt = 2;
};

}

#endif
