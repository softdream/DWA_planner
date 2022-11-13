#ifndef __OBSTACLES_H
#define __OBSTACLES_H

#include <Eigen/Dense>
#include <vector>

namespace dwa
{

template<typename T>
struct Obstacle
{
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

	Obstacle()
	{

	}

	Obstacle( const Vector2& pose, const DataType& radius )
		: pose_(pose_), radius_(radius_)
	{

	}

	~Obstacle()
	{

	}

	DataType radius_ = 0;
	Vector2 pose_ = Vector2::Zero();
	
};

class Obstacles
{
public:
	using DataType = T;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

	Obstacles()
	{

	}

	~Obstacles()
	{

	}

	void addObstacle( const Obstacle<DataType>& obs ){
		return obs_vec.push_back( obs );
	}

	void addObstacle( const Vector2& pose, const DataType& radius )
	{
		return obs_vec.push_back( Obstacle<DataType>( pose, radius ) );
	}

	void clearAll()
	{
		return obs_vec.clear();
	}

	const std:vector<Obstacle<DataType>>& getObstacles() const
	{
		return obs_vec;
	}

	void updateObstacles( const std:vector<Obstacle<DataType>>& obs_new )
	{
		obs_vec = obs_new;
	}

	const int getSize() const
	{
		return obs_vec.size();
	}

	const boool isEmpty() const
	{
		return obs_vec.empty();
	}

	const Obstacle<DataType>& operator[]( const int i ) const
	{
		return obs_vec[i];
	}

private:
	std:vector<Obstacle<DataType>> obs_vec;

};

}

#endif
