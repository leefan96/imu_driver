#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <serial/serial.h>

#include <vector>
#include <deque>
#include <cmath>
#include <cstring>
#include <algorithm>



#define FRAME_LEN 134








// ============================
// IMU帧结构（最小必要字段）
// ============================
#pragma pack(push,1)
struct ImuFrame
{
    uint8_t head1;
    uint8_t head2;
    uint32_t cnt1ms;

    int32_t gyro[3];
    int32_t acc[3];

    int16_t temp;

    uint32_t gps_time;

    double lat;
    double lon;

    int32_t height;

    int32_t vel_e;
    int32_t vel_n;
    int32_t vel_u;

    int32_t roll;
    int32_t pitch;
    uint32_t yaw;

    uint8_t nav_state;
    uint32_t flags;
    uint16_t detaT;

    double gps_lat;
    double gps_lon;

    int32_t gps_h;
    uint32_t gps_t;

    uint8_t sat;

    int32_t gps_e;
    int32_t gps_n;
    int32_t gps_u;

    int32_t odo;
    int32_t baro;

    uint8_t checksum;
    uint8_t tail;
};
#pragma pack(pop)

// ============================
// 时间同步器（核心）
// ============================
class TimeSync
{
public:
    static constexpr int WINDOW_SIZE = 1000;
    
    TimeSync()
    {
        a = 1.0;
        b = 0.0;
        sx = sy = sxx = sxy = 0;
        N = 0;
    }

    void update(double t_imu, double t_ros)
    {
        // outlier rejection
        double pred = a * t_imu + b;
        if (N >= WINDOW_SIZE && std::fabs(t_ros - pred) > 0.05)
        {
            printf("Outlier detected: %f %f %f\n", t_imu, t_ros, pred);
            return;
        }
            
        // 如果窗口已满，移除最旧的样本并更新累加和
        if (N >= WINDOW_SIZE)
        {
            auto &oldest = window.front();
            sx -= oldest.imu;
            sy -= oldest.ros;
            sxx -= oldest.imu * oldest.imu;
            sxy -= oldest.imu * oldest.ros;
            window.pop_front();
        }
        else
        {
            N++;
        }

        // 添加新样本并更新累加和
        window.push_back({t_imu, t_ros});
        sx += t_imu;
        sy += t_ros;
        sxx += t_imu * t_imu;
        sxy += t_imu * t_ros;

        fit();
    }

    double convert(double t_imu)
    {
        return a * t_imu + b;
    }

private:
    struct Pair { double imu, ros; };
    std::deque<Pair> window;

    double a, b;
    
    // 增量式最小二乘法的累加和
    double sx, sy, sxx, sxy;
    int N;

    void fit()
    {
        if (N < WINDOW_SIZE) return;

        double denom = WINDOW_SIZE * sxx - sx * sx;
        if (fabs(denom) < 1e-12) return;

        double a_new = (WINDOW_SIZE * sxy - sx * sy) / denom;
        double b_new = (sy - a_new * sx) / WINDOW_SIZE;

        a = 0.9 * a + 0.1 * a_new;
        b = 0.9 * b + 0.1 * b_new;
    }
};

// ============================
// 全局变量
// ============================
std::vector<uint8_t> buffer;
TimeSync time_sync;

// ============================
// Frame校验
// ============================
bool checkFrame(uint8_t* d)
{
    if (d[0] != 0x55 || d[1] != 0x55) return false;
    if (d[133] != 0xBF) return false;

    uint8_t sum = 0;
    for (int i = 2; i <= 131; i++)
        sum += d[i];

    return sum == d[132];
}

// ============================
// cnt1ms溢出处理
// ============================
/*
double unwrap_cnt(uint32_t cnt, uint32_t& last_cnt, uint64_t& offset)
{
    if (cnt < last_cnt)
        offset += (uint64_t)1 << 32;

    last_cnt = cnt;
    return (double)(cnt + offset);
}
*/
// ============================
// scale
// ============================
double gyro_scale(int32_t v)
{
    return v * 400.0 / (double)(1LL<<31) * M_PI / 180.0;
}

double acc_scale(int32_t v)
{
    return v * 10.0 / (double)(1LL<<31) * 9.81;
}
double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
// ============================
// batch处理核心
// ============================
void processBuffer(ros::Publisher& pub_imu, ros::Publisher& pub_gnss)
{
    std::vector<ImuFrame> frames;
    frames.reserve(5);

    while (buffer.size() >= FRAME_LEN)
    {
        if (!(buffer[0] == 0x55 && buffer[1] == 0x55))
        {
            buffer.erase(buffer.begin());
            continue;
        }

        if (buffer.size() < FRAME_LEN)
            break;

        if (!checkFrame(buffer.data()))
        {
            buffer.erase(buffer.begin());
            continue;
        }

        ImuFrame f;
        memcpy(&f, buffer.data(), sizeof(ImuFrame));
        frames.push_back(f);

        buffer.erase(buffer.begin(), buffer.begin() + FRAME_LEN);
    }

    if (frames.empty()) return;

    // ============================
    // 1. 只用最后一帧做时间同步
    // ============================
    ImuFrame &last = frames.back();

    //static uint32_t last_cnt = 0;
    //static uint64_t offset = 0;

    double t_imu_last =   last.cnt1ms;
        //超过2^32ms的处理  unwrap_cnt(last.cnt1ms, last_cnt, offset) * 1e-3;

    double t_ros_now = ros::Time::now().toSec();

    time_sync.update(t_imu_last, t_ros_now);

    // ============================
    // 2. batch内所有帧回推时间
    // ============================
    for (auto &f : frames)
    {
        double t_imu = f.cnt1ms;
        //超过2^32ms的处理  unwrap_cnt(f.cnt1ms, last_cnt, offset) * 1e-3;

        double t_ros = time_sync.convert(t_imu);

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time(t_ros);
        msg.header.frame_id = "imu_link";


        tf2::Quaternion q;
        q.setRPY(
            deg2rad(f.roll / 1000.0),
            deg2rad(f.pitch / 1000.0),
            deg2rad(f.yaw / 1000.0)
        );

        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();


        msg.angular_velocity.x = gyro_scale(f.gyro[0]);
        msg.angular_velocity.y = gyro_scale(f.gyro[1]);
        msg.angular_velocity.z = gyro_scale(f.gyro[2]);

        msg.linear_acceleration.x = acc_scale(f.acc[0]);
        msg.linear_acceleration.y = acc_scale(f.acc[1]);
        msg.linear_acceleration.z = acc_scale(f.acc[2]);

        pub_imu.publish(msg);



        // 发布 GNSS 数据
        sensor_msgs::NavSatFix gnss_msg;
        gnss_msg.header.stamp = ros::Time(t_ros);
        gnss_msg.header.frame_id = "gnss_link";

        gnss_msg.latitude = f.gps_lat;
        gnss_msg.longitude = f.gps_lon;
        gnss_msg.altitude = f.gps_h / 1000.0;

        gnss_msg.position_covariance[0] = 1.0;
        gnss_msg.position_covariance[4] = 1.0;
        gnss_msg.position_covariance[8] = 1.0;
        gnss_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gnss_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        gnss_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        pub_gnss.publish(gnss_msg);
    }
}

// ============================
// main
// ============================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh;

    ros::Publisher pub_imu =
        nh.advertise<sensor_msgs::Imu>("/imu/data", 200);

    ros::Publisher pub_gnss =
        nh.advertise<sensor_msgs::NavSatFix>("/gnss/fix", 100);

    serial::Serial ser("/dev/ttyUSB0", 115200,
        serial::Timeout::simpleTimeout(10));

    ROS_INFO("IMU driver started.");

    while (ros::ok())
    {
        size_t n = ser.available();

        if (n > 0)
        {
            std::vector<uint8_t> tmp;
            ser.read(tmp, n);

            buffer.insert(buffer.end(), tmp.begin(), tmp.end());

            processBuffer(pub_imu, pub_gnss);
        }

        ros::spinOnce();
    }

    return 0;
}