#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE {
    AVIA = 1, VELO16, OUST64, HESAIxt32, UNILIDAR
}; //{1, 2, 3, 4}
enum TIME_UNIT {
    SEC = 0, MS = 1, US = 2, NS = 3
};
enum Feature {
    Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
};
enum Surround {
    Prev, Next
};
enum E_jump {
    Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind
};

const bool time_list_cut_frame(PointType &x, PointType &y);

struct orgtype {
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;

    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, time, time)
                                          (std::uint16_t, ring, ring)
)

/**
 * @brief Unilidar Point Type
 */
namespace unilidar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace unilidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(unilidar_ros::Point,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (std::uint16_t, ring, ring)
                                    (float, time, time)
                                    )

namespace hesai_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (std::uint16_t, ring, ring)
)

namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint32_t, t, t)
                                          (std::uint16_t, reflectivity, reflectivity)
                                          (std::uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (std::uint32_t, range, range)
)

class Preprocess {
public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();

    ~Preprocess();

    void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    /**
     * @brief Enable the static self-filter: drop returns from the robot's own
     *        chassis (and any arm pose that stays inside the box).
     *
     * The box is expressed in base_link; `lidar_to_base` is the rigid
     * lidar -> base_link extrinsic (x, y, z in metres, roll, pitch, yaw in
     * degrees).  The rotation is applied per point, which is the same work as
     * transforming the points, but it keeps the output cloud in the lidar
     * frame -- point_lio, the loop closure and everything downstream expect
     * that.  Only points that already passed `blind` are tested, and a
     * bounding-radius early-out skips almost all of them, so the cost is a
     * handful of multiply-adds per frame.
     */
    void set_self_filter(bool en, const std::vector<double> &box_min,
                         const std::vector<double> &box_max,
                         const std::vector<double> &lidar_to_base);

    /**
     * @brief Configure the manipulation envelope: a second, larger box that is
     *        only active while the arm is deployed.
     *
     * A static box big enough to cover a grasping reach would also swallow the
     * real ground / low obstacles up to 1 m in front of the dog, so it must not
     * be on by default.  While grasping the dog is stationary and not mapping
     * anything it needs, so trading the near field for clean arm returns is
     * fine -- and because the removal happens before point_lio, the keyframes
     * written during the grasp are clean too (no ghosts survive in the map).
     *
     * `timeout_sec > 0` makes the envelope self-releasing: if no update
     * arrives for that long it switches itself off, so a stuck "grasping"
     * signal cannot leave the robot blind to near-field obstacles.
     */
    void set_manip_box(bool en, const std::vector<double> &box_min,
                       const std::vector<double> &box_max, double timeout_sec);

    /// Called from the ROS callback for /manipulation_mode.
    void set_manip_active(bool active);

    bool self_filter_en = false;
    int self_removed = 0;        // points dropped in the last processed frame
    int self_removed_total = 0;  // since startup
    bool manip_active = false;   // manipulation envelope currently enabled
    int manip_removed = 0;       // points dropped by it in the last frame
    int manip_removed_total = 0;

    // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    vector<orgtype> typess[128]; //maximum 128 line lidar
    float time_unit_scale;
    int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
    double blind;
    double z_max;        // upper bound on z (lidar frame); points with z > z_max are dropped
    bool given_offset_time;
    //ros::Publisher pub_full, pub_surf, pub_corn;


private:
    /// True when the lidar-frame point falls inside the given base_link box.
    inline bool in_box(const float x, const float y, const float z,
                       const float *box_min, const float *box_max,
                       const float r2_max) const {
        if (x * x + y * y + z * z > r2_max) return false;  // cannot be inside
        const float bx = self_R[0] * x + self_R[1] * y + self_R[2] * z + self_t[0];
        const float by = self_R[3] * x + self_R[4] * y + self_R[5] * z + self_t[1];
        const float bz = self_R[6] * x + self_R[7] * y + self_R[8] * z + self_t[2];
        return bx >= box_min[0] && bx <= box_max[0]
            && by >= box_min[1] && by <= box_max[1]
            && bz >= box_min[2] && bz <= box_max[2];
    }

    /// Squared distance from the lidar to the farthest corner of a box.
    static float box_radius2(const float *box_min, const float *box_max,
                             const float *lidar_in_base);

    // Self-filter box (base_link) and the lidar -> base_link extrinsic.
    float self_box_min[3] = {-1.f, -1.f, -1.f};
    float self_box_max[3] = {1.f, 1.f, 1.f};
    float self_R[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
    float self_t[3] = {0.f, 0.f, 0.f};
    float self_r2_max_ = 0.f;  // squared distance lidar -> farthest box corner
    bool self_extrinsic_ok = false;  // lidar -> base_link was parsed

    // Manipulation envelope (same extrinsic, larger box, on demand).
    bool manip_box_en = false;
    float manip_box_min[3] = {-1.f, -1.f, -1.f};
    float manip_box_max[3] = {1.f, 1.f, 1.f};
    float manip_r2_max_ = 0.f;
    double manip_timeout_sec = 0.0;
    std::chrono::steady_clock::time_point manip_last_active{};
    /// Disables the envelope when it has not been refreshed for too long.
    void check_manip_timeout();

    void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    void oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    void velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    void unilidar_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    void hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

    void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);

    int
    plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

    bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);

    bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};
