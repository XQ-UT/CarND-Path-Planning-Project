# CarND-Path-Planning-Project

This project creates a simple motion planner that helps self-driving car driving on highway. The planner should
satisfies the following requirement:

- Obey speed limit.
- Avoid collision.
- Do lane change in slow traffic.
- Avoid jerk.

## Algorithm

The planner behaves as following:
1. We will have `curr_lane` and `curr_speed_mph` variable to track AV status. 

2. The planner tracks all the vehicle around it through sensor fusion data. It will use two helper functions as below. 
```c++
/// Returns true when other car is in the same lane with my car.
bool in_same_lane(double my_car_d, double other_car_d){
	return other_car_d >= my_car_d - LANE_WIDTH_METERS / 2 
									&& other_car_d <= my_car_d + LANE_WIDTH_METERS / 2;
}
```
The first function checks if another car is in same lane with the AV.

```c++
/// Returns true when other car is in in front and close of my car.
bool close_and_in_front(double my_car_s, double other_car_s, double too_close_distance_meters){
	return other_car_s >= my_car_s && other_car_s - my_car_s <= too_close_distance_meters;
}
```
The second function checks if the other car is in front of AV within a short distance.

We use the two functions to check every car in sensor fusion both current and future status, to determine whether it is `too_close`, `left_lane_change_safe` and `right_lane_change_safe`.

2. After having `too_close`, `left_lane_change_safe` and `right_lane_change_safe` information. We change our `curr_lane` and `curr_speed_mph` accordingly.

```c++
if(too_close)
{
    if(left_lane_change_safe)
    {
        --curr_lane;
    } 
    else if (right_lane_change_safe)
    {
        ++curr_lane;
    }
    else 
    {
        curr_speed_mph = max(0.0, curr_speed_mph - 0.2);
    }
}
else if(curr_speed_mph < TARGET_SPEED_MPH)
{
    curr_speed_mph += 0.2;		
}
```

3. Then we will choose 5 anchor points. 2 from previous path, 3 from future path.

The following block choose 3 future waypoints, which will adapts to lane change trajectory.
```c++
double new_d = curr_lane * LANE_WIDTH_METERS + 0.5 * LANE_WIDTH_METERS;
vector<double> new_wp1 = getXY(car_s + 50, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> new_wp2 = getXY(car_s + 70, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> new_wp3 = getXY(car_s + 90, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

4. The next step is to convert anchor points to vehicle coordinates system, using following function.
```c++
/// Converts points in global coordinates to vehicle coordinates.
void convert_points_to_vehicle_coordinates(vector<double>& pts_x, vector<double>& pts_y, 
			const double car_x, const double car_y, const double car_yaw)
{
	for(int i = 0; i < pts_x.size(); ++i)
	{
		double shift_x = pts_x[i] - car_x;
		double shift_y = pts_y[i] - car_y;
		pts_x[i] = shift_x * cos(0 - car_yaw) - shift_y * sin(0 - car_yaw);
		pts_y[i] = shift_x * sin(0 - car_yaw) + shift_y * cos(0 - car_yaw);
	}
}
```


## Result

<p align="center">
  <img src="report_images/result.png" width="500" height="300"/>
  <br>
  <em>Figure 1: Driving on highway</em>
</p>

<p align="center">
  <img src="report_images/lane_change.png" width="500" height="300"/>
  <br>
  <em>Figure 2: Lane change</em>
</p>
