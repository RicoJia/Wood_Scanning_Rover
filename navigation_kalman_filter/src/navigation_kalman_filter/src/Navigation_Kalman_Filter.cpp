//
// Created by rico on 20/04/19.
//
#define USE_MATH_DEFINES   //use math library

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <sstream>
#include <ros/callback_queue.h>

const float angle_tolerance = 20;
const int d_buffer_size = 1;//distance buffers
const float a = 0.28;
const float l = 0.26;
const float velocity = 0.25;

enum class states
{
    INITIALIZATION,
    OPERATION
};

std_msgs::Float32 theta_msg;
std_msgs::Float32 distance_msg;
std_msgs::Bool initialization_flag_msg;

template <typename T,int size,int column = 1>      // remember when you use it, you have to pass a const int as non-type parameter!
void vector_add(T* a[size][column],T* b[size][column],T* result[size][column],T a_co, T b_co)
{
    for(int i=0;i<size;i++)
    {
        *(result[i][column-1]) = a_co*(*a[i][column-1])+b_co*(*b[i][column-1]);         //result matrix needs to be a double array of ptrs.
    }
}


template <class T,int row_a,int column_a,int row_b,int column_b>
void matrix_multiply(T* a[row_a][column_a],T* b[row_b][column_b],T* result[row_a][column_b])
{
    // if columns don't match then
    T result_temp[row_a][row_b] = {0};
    if(column_a==row_b)
    {
        for (int row=0;row<row_a;row++)
        {
            for(int column=0;column<column_b;column++)
            {
                for(int index=0;index<column_a;index++)
                {(result_temp[row][column]) += (*a[row][index])*(*b[index][column]);
                }
            }
        }
    }

    for (int row=0;row<row_a;row++)
    {
        for(int column=0;column<column_b;column++)
        {
            *(result[row][column]) = result_temp[row][column];
        }
    }
}

template <class T,int row_a,int column_a,int row_b,int column_b>    // this function for overloading
void matrix_multiply(const T a[row_a][column_a],T* b[row_b][column_b],T* result[row_a][column_b])
{
    // if columns don't match then
    T result_temp[row_a][row_b] = {0};
    if(column_a==row_b)
    {
        for (int row=0;row<row_a;row++)
        {
            for(int column=0;column<column_b;column++)
            {
                for(int index=0;index<column_a;index++)
                {(result_temp[row][column]) += a[row][index]*(*b[index][column]);
                }
            }
        }
    }

    for (int row=0;row<row_a;row++)
    {
        for(int column=0;column<column_b;column++)
        {
            *(result[row][column]) = result_temp[row][column];
        }
    }
}

class kf
{
private:
    ros::Publisher theta_filtered_pub;
    ros::Publisher distance_pub;
    ros::Publisher initialization_flag_pub;

    ros::Subscriber imu_sub;
    ros::Subscriber d1_sub;
    ros::Subscriber d2_sub;
    ros::Subscriber tan_theta_sub;
    ros::Subscriber reset_sub;
    ros::Subscriber u_sub;
    states state;


    double    d1_buffer [d_buffer_size];
    double    d1_init_buffer_value;
    int       d1_buffer_pos;
    float     d1;
    double    d2_buffer [d_buffer_size];
    double    d2_init_buffer_value;
    int       d2_buffer_pos;
    float     d2;
    float     distance;
    bool     reset;                                    //flags
    bool     initialization_flag;
    ros::Time start;                                   //initialization state variables
    int      avg_count;
    float    theta_sum;
    float    theta_reading;                            //theta variables are in radians!
    float    theta_update;
    float    theta_prediction;
    float    angular_velocity_reading;             //angular velocity variables
    float    angular_velocity_prediction;
    float    angular_velocity_update;
    const float    A[2][2];                                        //Kalman Filter Formula Variables and Constants
    const float    B[2][1];
    const float    delta_t;
    float    u;//tan_ksi
    const float    angle_kalman_gain_big[2][2];
    const float    angle_kalman_gain_small[2][2];
    const float    angle_kalman_gain_initialization[2][2];
    const float    Rv[2][2];
    float*   update[2][1]                     ={{&theta_update},{&angular_velocity_update}};
    float*   prediction[2][1]                 ={{&theta_prediction},{&angular_velocity_prediction}};
    float*   measurement[2][1]                = {{&theta_reading},{&angular_velocity_reading}};
    float*   U[1][1]                          ={{&angular_velocity_update}};// ={{&u}}; TODO: to delete
    ros::NodeHandle nh;
    // temp array for B
    float B1;
    float B2;
    float*B_temp[2][1]                        ={{&B1},{&B2}};;

public:
    kf(ros::NodeHandle n):
    nh(n),
    A({{1,0},{0,0}}),  //Kalman Filter Formula Variables and Constants
    B({{delta_t},{1}}),
    delta_t(0.008),
    angle_kalman_gain_big({{0.0000,0.3},{0.00000,0.9998}}),
    angle_kalman_gain_small({{0.0119,0.2964},{0.0005,0.9997}}),
    angle_kalman_gain_initialization({{0.4464,0.1661},{0.6642,0.8006}}),
    Rv({{0.01,0},{0,0.04}})
    {
        theta_filtered_pub =nh.advertise<std_msgs::Float32>("theta_filtered",15);
        distance_pub = nh.advertise<std_msgs::Float32>("distance_filtered",15);
        initialization_flag_pub = nh.advertise<std_msgs::Bool>("kalman_filter_initialization_flag",15);

        imu_sub = nh.subscribe("imu/data_raw",15,&kf::imu_callback,this);   //without this you can't assign a regular function ptr to a member function for its additional info
        d1_sub = nh.subscribe("d1",15,&kf::d1_callback,this);
        d2_sub = nh.subscribe("d2",15,&kf::d2_callback,this);
        tan_theta_sub = nh.subscribe("tanTheta",15,&kf::tan_theta_callback,this);
        //reset_sub = nh.subscribe("kalman_filter_reset_flag",1000,&kf::reset_sub_callback,this);
        //TODO: add reset topic in control node!!
        u_sub = nh.subscribe("tan_ksi",15,&kf::u_callback,this);

        state = states::INITIALIZATION;

        d1_init_buffer_value             = 0.35;
        d1_buffer_pos                    = 0;
        d1                               = 0.35;
        d2_init_buffer_value             = 0.35;
        d2_buffer_pos                    = 0;
        d2                               = 0.35;
        distance                         = 0.35;
        reset                            =1;        //reset = 1 means the state machine will reset
        initialization_flag              =1;        //initialization = 1 means it is already in initialization mode.
        avg_count                        =0;
        theta_sum                        =0;
        ros::Time start                  =ros::Time::now();
        theta_reading                    =0;
        theta_update                     =0;
        theta_prediction                 =0;
        angular_velocity_reading         =0;
        angular_velocity_prediction      =0;
        angular_velocity_update          =0;
        u                                =0;//tan_ksi
        //NOTE: define your matrix like this as a double array that stores ptrs.
        B1 = 0;
        B2 = 0;

        // Initializing distance buffer
        for (int i = 0; i < d_buffer_size; i++)
        {
            d1_buffer[i] = d1_init_buffer_value;
            d2_buffer[i] = d2_init_buffer_value;
        }
    };

    void filter_control()
    {
        double start_sec = start.toSec();
       // ROS_INFO("Start time is %f\n",start_sec);
      //  ROS_INFO("Initialization flag that is being published %d\n",initialization_flag);
        if (reset == 1)         // reset can be zero only by commands from the control package. Otherwise it will remain in 1
        {
            state=states::INITIALIZATION;
            start= ros::Time::now();
            avg_count = 0;
            theta_sum=0;
            theta_prediction=0;      // we just need to reset theta, since reset is for clearing drift in theta from IMU!
            reset = 0; // TODO: to delete!!	
	}     // control wants to reset, then it will reset. Else, let each state decide their state
        if (state == states::INITIALIZATION) initialization();
        else operation();

        initialization_flag_msg.data = initialization_flag;
        initialization_flag_pub.publish(initialization_flag_msg);
    }
    void initialization()
    //vector_add(T* a[size],T* b[size],T* result[size],T a_co, T b_co)
    // matrix_multiply(T a[row_a][column_a],T b[row_b][column_b],T* result[row_a][column_b])
    {
        if (ros::Time::now()<(start+ros::Duration(1)))        //supposed to be 1 second of average
        {
           theta_sum += theta_reading;//keep adding to the sum
           avg_count++;// updating avg_count
           theta_prediction = theta_sum/avg_count;
        }
        else if ((ros::Time::now()<((start+ros::Duration(3))))&&(ros::Time::now()>=(start+ros::Duration(1))) ) //kalman_filter for 2 seconds
        {
            //calculating update
            vector_add<float,2>(measurement,prediction,update,1,-1);
            matrix_multiply<float,2,2,2,1>(angle_kalman_gain_initialization,update,update);
            vector_add<float,2>(prediction,update,update,1,1);
            // calculating prediction
            matrix_multiply<float,2,2,2,1>(A,update,prediction);
            matrix_multiply<float,2,1,1,1>(B,U,B_temp);
            vector_add<float,2>(prediction,B_temp,prediction,1,1);
        }
        else
        {
            state = states::OPERATION;// state
        }
        initialization_flag = 1;//update initialization flag here
    }
    void operation()
    {
        /*Kalman Formula:
         * update = prediction + kalman_gain*(measurement - prediction)
         * prediction = A*update + B*u
         *
         */
        if(reset!=1)
        {
            // calculating update
            vector_add<float,2>(measurement,prediction,update,1,-1);
            if(abs(theta_update)>angle_tolerance) //if theta_prediction is greater than 30 deg, then switch to kalman_filter_big mode
            {
               matrix_multiply<float,2,2,2,1>(angle_kalman_gain_big,update,update);
 //matrix_multiply<float,2,2,2,1>(angle_kalman_gain_small,update,update); //TODO: to modify
            }
            else
            {
                //matrix_multiply<float,2,2,2,1>(angle_kalman_gain_small,update,update);
 matrix_multiply<float,2,2,2,1>(angle_kalman_gain_initialization,update,update);// TODO: to modify
            }
            vector_add<float,2>(prediction,update,update,1,1);
            //calculating predictions here
            matrix_multiply<float,2,2,2,1>(A,update,prediction);
            matrix_multiply<float,2,1,1,1>(B,U,B_temp);
            vector_add<float,2>(prediction,B_temp,prediction,1,1);
            //update distance here
            distance = cos(theta_update)*(d1+d2)/2;
            state = states::OPERATION;
        }
        else
        {
            state = states::INITIALIZATION;
        }
	

	//TODO: CONVERT BACK TO RADIANS!!
        theta_msg.data = theta_update;// publish theta here
        theta_filtered_pub.publish(theta_msg);
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);//publish distance here.
        initialization_flag = 0;    //update initialization flag here
    }
    void imu_callback(const sensor_msgs::Imu& imu_input)
    {
        angular_velocity_reading = imu_input.angular_velocity.z*180/3.1415926;
    }
    void d1_callback(const std_msgs::Float32& d1_input)
    {
        if (d1_buffer_pos == d_buffer_size) {
            d1_buffer_pos = 0;
        }
        d1_buffer[d1_buffer_pos] = ((double) d1_input.data)/100;
        d1_buffer_pos++;
        // Compute Average Theta value from Theta Buffer
        double result      = 0.0;
        int    buffer_valid = 0;

        for (int i = 0; i < d_buffer_size; i++) {
            if (d1_buffer[i] != d1_init_buffer_value) {
                result += d1_buffer[i];
                buffer_valid++;
            }
        }

        if (buffer_valid == 0)
            d1 = 0.0;
        else
            d1 = (float)(result / buffer_valid);

    }
    void d2_callback(const std_msgs::Float32& d2_input)
    {
        if (d2_buffer_pos == d_buffer_size) {
            d2_buffer_pos = 0;
        }
        d2_buffer[d2_buffer_pos] = ((double) d2_input.data)/100;
        d2_buffer_pos++;
        // Compute Average Theta value from Theta Buffer
        double result      = 0.0;
        int    buffer_valid = 0;

        for (int i = 0; i < d_buffer_size; i++) {
            if (d2_buffer[i] != d2_init_buffer_value) {
                result += d2_buffer[i];
                buffer_valid++;
            }
        }

        if (buffer_valid == 0)
            d2 = 0.0;
        else
            d2 = (float)(result / buffer_valid);

    }
    void tan_theta_callback(const std_msgs::Float32& tan_theta_input)
    {
        theta_reading = atan(tan_theta_input.data)*180/3.1415926;
	//ROS_INFO("theta: %f",theta_reading);
	//ROS_INFO("theta_update: %f",theta_update);
    }
    void u_callback(const std_msgs::UInt16& u_input)
    {
        u = 0*velocity/sqrt((a*a+l*l/(u_input.data*u_input.data)))/3.1415926;
    }
    void reset_sub_callback(const std_msgs::Bool& kalman_filter_reset_input)
    {
        reset = kalman_filter_reset_input.data;
        //ROS_INFO("The reset value is %d\n",reset);
    }
};

int main (int argc, char **argv)        // what is this?
{
    ros::init(argc,argv,"Navigation_Kalman_Filter");
    // Initializing handles after ros::init
    ros::NodeHandle nh;
    kf kalman_filter(nh);
    ros::Rate loop_rate(90);     //8Hz, so 0.125s.

    while(ros::ok())
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
        kalman_filter.filter_control();
        loop_rate.sleep();              // if my control function takes too long, then will it crash?
    }
}

//

