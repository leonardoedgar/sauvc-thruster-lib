# include "gtest/gtest.h"
# define private public
# include "thruster_aggregator.h"
# include <vector>
# include <string>
# include <map>
# include <algorithm>
# include "../config/config.h"
class ThrusterAggregatorTest:public::testing::Test {
protected:
    ThrusterAggregator sample_thruster_aggregator;
    std::map <int, int> sample_thruster_speed = {
            {1, 1500},
            {2, 1600},
            {3, 1550},
            {4, 1450},
            {5, 1675},
            {6, 1700},
            {7, 1600},
            {8, 1500}
    };
    std::map<int, int> thruster_id_to_arduino_pin_map = {THRUSTER_ID_TO_ARDUINO_PIN};
    virtual void SetUp() {
        sample_thruster_aggregator.setup();
    }
};

/**
 * Test that thruster aggregator adds thrusters with correctly
 */
TEST_F(ThrusterAggregatorTest, thrusters_addition) {
    ASSERT_EQ(int(sample_thruster_aggregator.thruster_id_to_instance_map.size()),
            thruster_id_to_arduino_pin_map.size());
    for (auto &added_thruster: sample_thruster_aggregator.thruster_id_to_instance_map) {
        ASSERT_EQ(added_thruster.first, added_thruster.second.get_id());
    }
}

/**
 * Test that thruster aggregator creates a correct map based on the pre-defined motion.
 */
TEST_F(ThrusterAggregatorTest, predefined_motion) {
    std::vector<int> thrusters_id_to_run_forward {1, 2, 3, 4}, thrusters_id_to_stop {5, 6, 7, 8};
    for (const auto &[thruster_id_to_run, esc_input]:
        sample_thruster_aggregator.motion_to_thruster_id_to_esc_input_map["forward"]) {
            if (esc_input > ESC_INPUT_FOR_STOP_SIGNAL) {
                ASSERT_TRUE(std::find(thrusters_id_to_run_forward.begin(), thrusters_id_to_run_forward.end(),
                        thruster_id_to_run) != thrusters_id_to_run_forward.end());
            }
            else if (esc_input == ESC_INPUT_FOR_STOP_SIGNAL) {
                ASSERT_TRUE(std::find(thrusters_id_to_stop.begin(), thrusters_id_to_stop.end(), thruster_id_to_run)
                        != thrusters_id_to_stop.end());
            }
    }
}

/**
 * Test that thruster aggregator is able to drive required thrusters with the specified ESC input in forward motion.
 */
TEST_F(ThrusterAggregatorTest, move_forward_with_desired_motors_and_speed) {
    std::map<int, int> thrusters_id_for_motion_forward =
            sample_thruster_aggregator.motion_to_thruster_id_to_esc_input_map["forward"];
    std::string desired_output = "";
    for (const auto &[thruster_id_to_run, esc_input]: thrusters_id_for_motion_forward) {
        if (esc_input > ESC_INPUT_FOR_STOP_SIGNAL) {
            desired_output += "Thruster with id: " + std::to_string(thruster_id_to_run) + " with pin: " +
                    std::to_string(thruster_id_to_arduino_pin_map[thruster_id_to_run]) +
                    " is running with ESC input: " + std::to_string(esc_input) + ".\n";
            }
        else if (esc_input == ESC_INPUT_FOR_STOP_SIGNAL) {
            desired_output += "Thruster with id: " + std::to_string(thruster_id_to_run) + " with pin: " +
                    std::to_string(thruster_id_to_arduino_pin_map[thruster_id_to_run]) +
                    " is running with ESC input: " + std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";;
            }
    }
    testing::internal::CaptureStdout();
    sample_thruster_aggregator.move("forward");
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that thruster aggregator should not drive thrusters when prompted with unrecognised motion.
 */
 TEST_F(ThrusterAggregatorTest, unknown_motion) {
     std::string desired_output = "";
     testing::internal::CaptureStdout();
     ASSERT_FALSE(sample_thruster_aggregator.move("randomly"));
     std::string actual_output = testing::internal::GetCapturedStdout();
     ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that thruster aggregator is able to load all predefined motion configuration.
 */
TEST(ThrusterAggregator, load_predefined_motions) {
    ASSERT_EQ(typeid(ThrusterAggregator::load_predefined_motion()),
            typeid(std::map<std::string, std::map<int, int>>));
}

/**
 * Test that thruster aggregator is able to stop all thrusters.
 */
TEST_F(ThrusterAggregatorTest, stop_all_motors) {
    std::string desired_output = "";
    for (const auto&[thruster_id_to_stop, thruster_pin_to_stop]: thruster_id_to_arduino_pin_map) {
        desired_output += "Thruster with id: " + std::to_string(thruster_id_to_stop) + " with pin: " +
                std::to_string(thruster_pin_to_stop) + " is running with ESC input: " +
                std::to_string(ESC_INPUT_FOR_STOP_SIGNAL) + ".\n";
    }
    testing::internal::CaptureStdout();
    sample_thruster_aggregator.stop();
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that thruster aggregator is able to stabilise all thrusters
 */
TEST_F(ThrusterAggregatorTest, stabilise) {
    sample_thruster_aggregator.thruster_id_to_stabilised_speed_map = sample_thruster_speed;
    std::string desired_output = "";
    for (const auto&[thruster_id_to_run, thruster_pin_to_run]: thruster_id_to_arduino_pin_map) {

        desired_output += "Thruster with id: " + std::to_string(thruster_id_to_run) + " with pin: " +
                          std::to_string(thruster_pin_to_run) + " is running with ESC input: " +
                          std::to_string(sample_thruster_speed[thruster_id_to_run]) + ".\n";
    }
    testing::internal::CaptureStdout();
    sample_thruster_aggregator.stabilise();
    std::string actual_output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(actual_output, desired_output);
}

/**
 * Test that thruster aggregator is able its stabilised speed.
 */
TEST_F(ThrusterAggregatorTest, update_stabilised_speed) {
    sample_thruster_aggregator.update_stabilised_speed(sample_thruster_speed);
    ASSERT_EQ(sample_thruster_aggregator.thruster_id_to_stabilised_speed_map, sample_thruster_speed);
}

/**
 * Test that thruster aggregator is able to get its actual thrusters' speed.
 */
TEST_F(ThrusterAggregatorTest, get_actual_thrusters_speed) {
    sample_thruster_aggregator.thruster_id_to_actual_speed_map = sample_thruster_speed;
    ASSERT_EQ(sample_thruster_aggregator.get_actual_thrusters_speed(), sample_thruster_speed);
}