#include "free_space_motion_tube/core/motion_tube.h"
#include "free_space_motion_tube/core/motion_primitive.h"

#include <vector>

double compute_min_curvature(double forward_velocity, double max_angular_rate);
double compute_curvature_radix(double r0, double r_min, size_t number_of_samples);
void compute_vector_of_curvatures(double r0, double rmin, size_t number_of_curvatures, std::vector<double> &curvatures);
void print_cartesian_points(motion_tube_cartesian_t *motion_tube_cartesian);
void print_beams(motion_tube_sensor_space_t *motion_tube_sensor_space);

typedef struct
{
    double forward_velocity;
    double time_horizon;
}motion_primitive_params_t;

class BarnMotionTube
{
    public:
        bool **availability_;
        size_t *curvature_offset_;
        point2d_t **final_position_;

        struct{
            double r0, rmin;
            size_t number_of_curvatures;
            double max_angular_rate;
            pose2d_t pose_sensor;
            point2d_t footprint[4];
            // Motion primitives
            std::vector<motion_primitive_params_t> vec_mp_params;
            // Sampling parameters
            double sampling_interval;
            size_t max_number_of_samples;
        }params_;

        std::vector<std::vector<motion_primitive_t>> motion_primitive_;
        std::vector<std::vector<motion_tube_t>> motion_tube_;

    public:
        BarnMotionTube(void);
        void Configure(range_sensor_t *range_sensor);   
        void Compute(range_sensor_t *range_sensor, range_scan_t *range_scan);
        void Deallocate(void);
        
        void PrintAvailability();
};
