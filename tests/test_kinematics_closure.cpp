/*
    Tests kinematic closure of the Stewart platform by verifying that the
    forward kinematics (FK) recovers the same pose that the inverse kinematics
    (IK) was asked to reach. For each test pose, IK computes servo angles,
    those angles are fed to FK, and the FK result is compared to the IK target.

    Two test modes are run:
      Warm start: poses evaluated sequentially so the FK initial condition is
                  the previous IK target, which mirrors real runtime usage.
      Cold start: each pose evaluated from the home position as the FK initial
                  condition, to detect sensitivity to the starting estimate.

    A pose that fails IK (outside the workspace) is skipped and does not count
    as a failure. A pose that fails FK convergence is counted as a failure.
*/

#include "stewartPlatform.hpp"

#include <iostream>
#include <cmath>
#include <vector>

// Closure error tolerances
#define POSITION_TOLERANCE_MM    0.5f
#define ORIENTATION_TOLERANCE_DEG 0.5f

// Internal z offset applied by moveTo() — must match BASE_Z_OFFSET + PLATFORM_Z_OFFSET
// in stewartPlatform.cpp
#define INTERNAL_Z_OFFSET 13.0248f


// Subclass to expose protected members needed for testing only
class PlatformTestFixture : public StewartPlatform
{
    public:
        // Copies computed servo targets into servo state to simulate motor feedback
        void applyServoTargets()
        {
            for (int i = 0; i < NUM_SERVOS; i++)
            {
                servos[i].setAngle(servo_targets[i]);
            }
        }

        // Resets the FK initial condition to the home position
        void resetToHome()
        {
            platform_pose_target = { Vector3f(0.0f, 0.0f, INTERNAL_Z_OFFSET), Quaternionf::Identity() };
        }

        // Returns the internal pose target used as the FK initial condition
        const PlatformPose& getPoseTarget() const
        {
            return platform_pose_target;
        }
};


// Holds the outcome of a single round-trip test
struct TestResult
{
    float pos_error_mm;
    float ori_error_deg;
    bool ik_failed;
    bool fk_failed;
    bool pass;
};


// Computes the Euclidean position error between two poses in mm
float positionError(const PlatformPose& a, const PlatformPose& b)
{
    return (a.position - b.position).norm();
}

// Computes the angular orientation error between two poses in degrees
float orientationError(const PlatformPose& a, const PlatformPose& b)
{
    // Angular distance = 2 * acos(|q1 . q2|), full rotation angle between orientations
    float dot = std::abs(a.orientation.dot(b.orientation));
    dot = std::min(1.0f, dot);
    return 2.0f * std::acos(dot) * 180.0f / (float)M_PI;
}

// Runs one round-trip IK->FK test using the provided platform instance.
// If cold_start is true, the FK initial condition is reset to home before running FK.
TestResult runSingleTest(PlatformTestFixture& platform, const PlatformPose& test_pose, bool cold_start)
{
    TestResult result = { 0.0f, 0.0f, false, false, false };
    PlatformPose pose = test_pose;

    // Run IK to compute servo targets for this pose
    if (!platform.moveTo(&pose))
    {
        result.ik_failed = true;
        return result;
    }

    // Simulate motor feedback by copying servo targets into servo state
    platform.applyServoTargets();

    // For cold start, reset FK initial condition to home before solving
    if (cold_start)
    {
        platform.resetToHome();
    }

    // Run FK to recover the platform pose from the current servo angles
    if (!platform.computePlatformPose())
    {
        result.fk_failed = true;
        return result;
    }

    // Compare the FK result against the IK target
    result.pos_error_mm  = positionError(platform.getPlatformPose(), platform.getPoseTarget());
    result.ori_error_deg = orientationError(platform.getPlatformPose(), platform.getPoseTarget());
    result.pass = (result.pos_error_mm  <= POSITION_TOLERANCE_MM) &&
                  (result.ori_error_deg <= ORIENTATION_TOLERANCE_DEG);

    return result;
}

// Prints per-pose results for a test run
void printResults(const std::vector<TestResult>& results)
{
    for (size_t i = 0; i < results.size(); i++)
    {
        const TestResult& r = results[i];

        if (r.ik_failed)
        {
            std::cout << "  Pose " << i << ": IK failed — outside workspace, skipped" << std::endl;
        }
        else if (r.fk_failed)
        {
            std::cout << "  Pose " << i << ": FK failed to converge" << std::endl;
        }
        else
        {
            std::cout << "  Pose " << i
                      << ": pos=" << r.pos_error_mm  << " mm"
                      << "  ori=" << r.ori_error_deg << " deg"
                      << "  " << (r.pass ? "PASS" : "FAIL") << std::endl;
        }
    }
}

// Prints summary statistics and returns true if all measured poses passed
bool printSummary(const std::vector<TestResult>& results)
{
    int pass_count  = 0;
    int fail_count  = 0;
    int ik_skipped  = 0;
    int fk_failures = 0;
    float max_pos_err  = 0.0f;
    float max_ori_err  = 0.0f;
    float sum_pos_err  = 0.0f;
    float sum_ori_err  = 0.0f;
    int measured = 0;

    // Accumulate statistics across all results
    for (const auto& r : results)
    {
        if (r.ik_failed)
        {
            ik_skipped++;
            continue;
        }

        if (r.fk_failed)
        {
            fk_failures++;
            fail_count++;
            continue;
        }

        measured++;
        sum_pos_err += r.pos_error_mm;
        sum_ori_err += r.ori_error_deg;
        max_pos_err = std::max(max_pos_err, r.pos_error_mm);
        max_ori_err = std::max(max_ori_err, r.ori_error_deg);

        if (r.pass)
        {
            pass_count++;
        }
        else
        {
            fail_count++;
        }
    }

    // Print the aggregated statistics
    std::cout << "  Measured: "     << measured
              << " | Passed: "      << pass_count
              << " | Failed: "      << fail_count
              << " | IK skipped: "  << ik_skipped
              << " | FK failures: " << fk_failures << std::endl;

    if (measured > 0)
    {
        std::cout << "  Max pos error:  " << max_pos_err          << " mm"  << std::endl;
        std::cout << "  Mean pos error: " << sum_pos_err / measured << " mm" << std::endl;
        std::cout << "  Max ori error:  " << max_ori_err           << " deg" << std::endl;
        std::cout << "  Mean ori error: " << sum_ori_err / measured << " deg" << std::endl;
    }

    return fail_count == 0;
}


int main()
{
    // Generate structured test poses covering the main degrees of freedom
    std::vector<PlatformPose> test_poses;

    // Pure z translations from 0 to 20 mm
    for (float z = 0.0f; z <= 20.0f; z += 2.0f)
    {
        test_poses.push_back({ Vector3f(0.0f, 0.0f, z), Quaternionf::Identity() });
    }

    // Pure x translations from -10 to +10 mm at a mid-range z
    for (float x = -10.0f; x <= 10.0f; x += 2.0f)
    {
        test_poses.push_back({ Vector3f(x, 0.0f, 10.0f), Quaternionf::Identity() });
    }

    // Pure y translations from -10 to +10 mm at a mid-range z
    for (float y = -10.0f; y <= 10.0f; y += 2.0f)
    {
        test_poses.push_back({ Vector3f(0.0f, y, 10.0f), Quaternionf::Identity() });
    }

    // Pure rotations about each axis from -15 to +15 degrees, at a mid-range z
    for (float deg = -15.0f; deg <= 15.0f; deg += 3.0f)
    {
        float rad = deg * (float)M_PI / 180.0f;

        // Roll
        test_poses.push_back({ Vector3f(0.0f, 0.0f, 10.0f),
            Quaternionf(Eigen::AngleAxisf(rad, Vector3f::UnitX())) });

        // Pitch
        test_poses.push_back({ Vector3f(0.0f, 0.0f, 10.0f),
            Quaternionf(Eigen::AngleAxisf(rad, Vector3f::UnitY())) });

        // Yaw
        test_poses.push_back({ Vector3f(0.0f, 0.0f, 10.0f),
            Quaternionf(Eigen::AngleAxisf(rad, Vector3f::UnitZ())) });
    }

    // Combined z translation and roll to test coupled motion
    for (float z = 5.0f; z <= 15.0f; z += 5.0f)
    {
        for (float deg = -10.0f; deg <= 10.0f; deg += 5.0f)
        {
            float rad = deg * (float)M_PI / 180.0f;
            test_poses.push_back({ Vector3f(0.0f, 0.0f, z),
                Quaternionf(Eigen::AngleAxisf(rad, Vector3f::UnitX())) });
        }
    }

    std::cout << "Kinematics closure test" << std::endl;
    std::cout << "  " << test_poses.size() << " test poses" << std::endl;
    std::cout << "  Tolerance: position=" << POSITION_TOLERANCE_MM   << " mm"
              << "  orientation=" << ORIENTATION_TOLERANCE_DEG << " deg" << std::endl;

    // --- Warm start tests ------------------------------------------------
    // Poses are evaluated sequentially on one platform instance.
    // FK initial condition for each pose is the previous IK target, which
    // mirrors how the platform is used at runtime.
    std::cout << "\n--- Warm start tests ---" << std::endl;

    PlatformTestFixture warm_platform;
    std::vector<TestResult> warm_results;

    for (const auto& pose : test_poses)
    {
        warm_results.push_back(runSingleTest(warm_platform, pose, false));
    }

    printResults(warm_results);
    bool warm_passed = printSummary(warm_results);

    // --- Cold start tests ------------------------------------------------
    // Each pose is evaluated on a fresh platform instance with the FK initial
    // condition reset to home, to expose sensitivity to the starting estimate.
    std::cout << "\n--- Cold start tests ---" << std::endl;

    std::vector<TestResult> cold_results;

    for (const auto& pose : test_poses)
    {
        // Construct a fresh platform so servo state does not carry over between poses
        PlatformTestFixture cold_platform;
        cold_results.push_back(runSingleTest(cold_platform, pose, true));
    }

    printResults(cold_results);
    bool cold_passed = printSummary(cold_results);

    // --- Overall result --------------------------------------------------
    bool overall_passed = warm_passed && cold_passed;
    std::cout << "\n=== RESULT: " << (overall_passed ? "PASS" : "FAIL") << " ===" << std::endl;

    return overall_passed ? 0 : 1;
}
