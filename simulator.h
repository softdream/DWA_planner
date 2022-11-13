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


namespace dwa
{

template<typename T>
class Simulator
{
public:
        using DataType = T;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

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
		
		displayAScan( data.scan_data );
		lidarData2DataContainer( data.scan_data, scan );
	}

private:
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
                                //std::cout << "dist : ( " << lidar_x << ", " << lidar_y << " )" << std::endl;
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


private:
	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );

	DWA<DataType> dwa;
	sensor::ScanContainer scan;
};

}

#endif
