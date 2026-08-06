#include <vlcal/calib/visual_camera_calibration.hpp>

#include <boost/format.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ceres/autodiff_first_order_function.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>

#include <sophus/ceres_manifold.hpp>
#include <sophus/se3.hpp>

#include <dfo/nelder_mead.hpp>
#include <gtsam/geometry/Pose3.h>

#include <vlcal/calib/cost_calculator_nid.hpp>
#include <vlcal/calib/view_culling.hpp>
#include <vlcal/costs/nid_cost.hpp>

// #include <glk/pointcloud_buffer.hpp>
// #include <guik/viewer/light_viewer.hpp>

namespace vlcal
{

namespace
{

/**
 * Manifold over the Sophus SE3 parameter block ([qx qy qz qw tx ty tz]) whose
 * tangent space is only the 3 rotation DoF: Plus() applies an SO3 increment and
 * copies the translation through unchanged, so the optimizer cannot move the
 * lever arm. Used when VisualCameraCalibrationParams::rotation_only is set.
 */
class RotationOnlySE3Manifold : public ceres::Manifold
{
  public:
    int AmbientSize() const override { return Sophus::SE3d::num_parameters; }
    int TangentSize() const override { return Sophus::SO3d::DoF; }

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override
    {
        const Eigen::Map<const Sophus::SE3d> T(x);
        const Eigen::Map<const Eigen::Vector3d> d(delta);
        Eigen::Map<Sophus::SE3d> T_out(x_plus_delta);
        T_out = Sophus::SE3d(T.so3() * Sophus::SO3d::exp(d), T.translation());
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override
    {
        const Eigen::Map<const Sophus::SE3d> T(x);
        Eigen::Map<Eigen::Matrix<double, Sophus::SE3d::num_parameters, Sophus::SO3d::DoF, Eigen::RowMajor>> J(
            jacobian);
        J.setZero();
        J.template topRows<Sophus::SO3d::num_parameters>() = T.so3().Dx_this_mul_exp_x_at_0();
        return true;
    }

    bool Minus(const double *y, const double *x, double *y_minus_x) const override
    {
        const Eigen::Map<const Sophus::SE3d> T_y(y);
        const Eigen::Map<const Sophus::SE3d> T_x(x);
        Eigen::Map<Eigen::Vector3d> d(y_minus_x);
        d = (T_x.so3().inverse() * T_y.so3()).log();
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override
    {
        const Eigen::Map<const Sophus::SE3d> T(x);
        Eigen::Map<Eigen::Matrix<double, Sophus::SO3d::DoF, Sophus::SE3d::num_parameters, Eigen::RowMajor>> J(
            jacobian);
        J.setZero();
        J.template leftCols<Sophus::SO3d::num_parameters>() = T.so3().Dx_log_this_inv_by_x_at_this();
        return true;
    }
};

} // namespace

VisualCameraCalibration::VisualCameraCalibration(const camera::GenericCameraBase::ConstPtr &proj,
                                                 const std::vector<VisualLiDARData::ConstPtr> &dataset,
                                                 const VisualCameraCalibrationParams &params)
    : params(params), proj(proj), dataset(dataset)
{
}

Eigen::Isometry3d VisualCameraCalibration::calibrate(const Eigen::Isometry3d &init_T_camera_lidar)
{
    Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;

    // Outer loop
    std::cout << "[0%], [Running Auto-Extrinsic Calibration...]" << std::endl;
    for (int outer_idx = 0; outer_idx < params.max_outer_iterations; outer_idx++)
    {
        Eigen::Isometry3d new_T_camera_lidar;
        switch (params.registration_type)
        {
        case RegistrationType::NID_BFGS:
            new_T_camera_lidar = estimate_pose_bfgs(T_camera_lidar);
            break;
        case RegistrationType::NID_NELDER_MEAD:
            new_T_camera_lidar = estimate_pose_nelder_mead(T_camera_lidar);
            break;
        }

        const Eigen::Isometry3d delta = new_T_camera_lidar.inverse() * T_camera_lidar;
        T_camera_lidar = new_T_camera_lidar;

        const double delta_t = delta.translation().norm();
        const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
        const bool converged = delta_t < params.delta_trans_thresh && delta_r < params.delta_rot_thresh;

        std::stringstream sst;
        auto fmt = boost::format("delta_t: %.5f [m]  delta_r: %.5f [rad]") % delta_t % delta_r;
        sst << fmt << std::endl;
        sst << (converged ? "Outer loop converged" : "Re-run inner optimization with the new viewpoint");
        std::cout << sst.str() << std::endl;

        int progress_percent = static_cast<int>(100 * (outer_idx + 1) / params.max_outer_iterations);
        std::cout << "[" << progress_percent << "%], [Running Auto-Extrinsic Calibration: " << fmt << "]" << std::endl;
        if (converged)
            break;
        else
            std::cout << "Outer loop: " << outer_idx << std::endl;
    }
    std::cout << "[100%], [Auto-Extrinsic Calibration Finished]" << std::endl;

    return T_camera_lidar;
}

Eigen::Isometry3d VisualCameraCalibration::estimate_pose_nelder_mead(const Eigen::Isometry3d &init_T_camera_lidar)
{
    ViewCullingParams view_culling_params;
    view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
    ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);

    std::vector<CostCalculator::Ptr> costs;
    for (const auto &data : dataset)
    {
        // Remove hidden points
        auto culled_points = view_culling.cull(data->points, init_T_camera_lidar);
        auto new_data = std::make_shared<VisualLiDARData>(data->image, culled_points);

        // Create NID cost
        NIDCostParams nid_params;
        nid_params.bins = params.nid_bins;
        costs.emplace_back(std::make_shared<CostCalculatorNID>(proj, new_data, nid_params));

        /*
        auto viewer = guik::LightViewer::instance();
        viewer->invoke([=] {
          auto culled_cloud_buffer = std::make_shared<glk::PointCloudBuffer>(culled_points->points,
        culled_points->size()); culled_cloud_buffer->add_intensity(glk::COLORMAP::TURBO, culled_points->intensities,
        culled_points->size()); auto raw_cloud_buffer = std::make_shared<glk::PointCloudBuffer>(data->points->points,
        data->points->size()); auto sub = viewer->sub_viewer("view_culling"); sub->set_draw_xy_grid(false);
          sub->set_camera_control(viewer->get_camera_control());
          sub->update_drawable("raw", raw_cloud_buffer, guik::FlatColor(0.2f, 0.2f, 0.2f, 1.0f));
          sub->update_drawable("culled", culled_cloud_buffer, guik::VertexColor().add("point_scale", 1.5f));
        });
        */
    }

    double best_cost = std::numeric_limits<double>::max();

    const auto f = [&](const gtsam::Vector6 &x)
    {
        gtsam::Vector6 x_masked = x;
        if (params.rotation_only)
        {
            // Pose3 tangent is [omega; v]; zeroing v keeps the translation at init
            x_masked.tail<3>().setZero();
        }
        const Eigen::Isometry3d T_camera_lidar =
            init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(x_masked).matrix());
        double sum_costs = 0.0;

#pragma omp parallel for reduction(+ : sum_costs)
        for (int i = 0; i < costs.size(); i++)
        {
            sum_costs += costs[i]->calculate(T_camera_lidar);
        }

        if (sum_costs < best_cost)
        {
            best_cost = sum_costs;
            // params.callback(T_camera_lidar);
            std::cout << "cost:" << best_cost << std::endl;
        }

        return sum_costs;
    };

    // Optimize
    dfo::NelderMead<6>::Params nelder_mead_params;
    nelder_mead_params.init_step = params.nelder_mead_init_step;
    nelder_mead_params.convergence_var_thresh = params.nelder_mead_convergence_criteria;
    nelder_mead_params.max_iterations = params.max_inner_iterations;
    dfo::NelderMead<6> optimizer(nelder_mead_params);
    auto result = optimizer.optimize(f, gtsam::Vector6::Zero());

    const Eigen::Isometry3d T_camera_lidar =
        init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(result.x).matrix());

    std::stringstream sst;
    sst << boost::format("Inner optimization (Nelder-Mead) terminated after %d iterations") % result.num_iterations
        << std::endl;
    sst << boost::format("Final cost: %.3f") % result.y << std::endl;
    sst << "--- T_camera_lidar ---" << std::endl << T_camera_lidar.matrix();
    std::cout << sst.str() << std::endl;

    return T_camera_lidar;
}

struct MultiNIDCost
{
  public:
    MultiNIDCost(const Sophus::SE3d &init_T_camera_lidar) : init_T_camera_lidar(init_T_camera_lidar) {}

    void add(const std::shared_ptr<NIDCost> &cost) { costs.emplace_back(cost); }

    template <typename T> bool operator()(const T *params, T *residual) const
    {
        std::vector<double> values(Sophus::SE3d::num_parameters);
        std::transform(params, params + Sophus::SE3d::num_parameters, values.begin(),
                       [](const auto &x) { return get_real(x); });
        const Eigen::Map<const Sophus::SE3d> T_camera_lidar(values.data());
        const Sophus::SE3d delta = init_T_camera_lidar.inverse() * T_camera_lidar;

        if (delta.translation().norm() > 0.2 || Eigen::AngleAxisd(delta.rotationMatrix()).angle() > 2.0 * M_PI / 180.0)
        {
            return false;
        }

        std::vector<bool> results(costs.size());
        std::vector<T> residuals(costs.size());

#pragma omp parallel for
        for (int i = 0; i < costs.size(); i++)
        {
            results[i] = (*costs[i])(params, &residuals[i]);
        }

        for (int i = 1; i < costs.size(); i++)
        {
            residuals[0] += residuals[i];
        }

        *residual = residuals[0];

        return std::count(results.begin(), results.end(), false) == 0;
    }

  private:
    Sophus::SE3d init_T_camera_lidar;
    std::vector<std::shared_ptr<NIDCost>> costs;
};

struct IterationCallbackWrapper : public ceres::IterationCallback
{
  public:
    IterationCallbackWrapper(const std::function<ceres::CallbackReturnType(const ceres::IterationSummary &)> &callback)
        : callback(callback)
    {
    }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) { return callback(summary); }

  private:
    std::function<ceres::CallbackReturnType(const ceres::IterationSummary &)> callback;
};

Eigen::Isometry3d VisualCameraCalibration::estimate_pose_bfgs(const Eigen::Isometry3d &init_T_camera_lidar)
{
    ViewCullingParams view_culling_params;
    view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
    ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);

    Sophus::SE3d T_camera_lidar(init_T_camera_lidar.matrix());

    std::vector<std::shared_ptr<NIDCost>> nid_costs;

    for (int i = 0; i < dataset.size(); i++)
    {
        // Remove hidden points
        auto culled_points = view_culling.cull(dataset[i]->points, init_T_camera_lidar);

        cv::Mat normalized_image;
        dataset[i]->image.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);

        std::shared_ptr<NIDCost> nid_cost(new NIDCost(proj, normalized_image, culled_points, params.nid_bins));
        nid_costs.emplace_back(nid_cost);
    }

    auto sum_nid = new MultiNIDCost(T_camera_lidar);
    for (const auto &nid_cost : nid_costs)
    {
        sum_nid->add(nid_cost);
    }

    auto cost = new ceres::AutoDiffFirstOrderFunction<MultiNIDCost, Sophus::SE3d::num_parameters>(sum_nid);
    ceres::Manifold *manifold = params.rotation_only
                                    ? static_cast<ceres::Manifold *>(new RotationOnlySE3Manifold())
                                    : new Sophus::Manifold<Sophus::SE3>();
    ceres::GradientProblem problem(cost, manifold);

    ceres::GradientProblemSolver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;
    options.line_search_direction_type = ceres::BFGS;

    // options.callbacks.emplace_back(new IterationCallbackWrapper([&](const ceres::IterationSummary &summary) {
    //     params.callback(Eigen::Isometry3d(T_camera_lidar.matrix()));
    //     return ceres::CallbackReturnType::SOLVER_CONTINUE;
    // }));

    ceres::GradientProblemSolver::Summary summary;
    ceres::Solve(options, problem, T_camera_lidar.data(), &summary);

    std::stringstream sst;
    sst << boost::format("Inner optimization (BFGS) terminated after %d iterations") % summary.iterations.size()
        << std::endl;
    sst << boost::format("Initial cost: %.8f") % summary.initial_cost << std::endl;
    sst << boost::format("Final cost: %.8f") % summary.final_cost << std::endl;
    sst << "--- T_camera_lidar ---" << std::endl << T_camera_lidar.matrix();
    std::cout << sst.str() << std::endl;

    return Eigen::Isometry3d(T_camera_lidar.matrix());
}

} // namespace vlcal
