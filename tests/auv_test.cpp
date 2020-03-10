# include "gtest/gtest.h"
# define private public
# include "auv.h"
# include <string>
class AUVTest:public::testing::Test {
protected:
    AUV sample_auv;
    std::string initial_motion = "stop";
    virtual void SetUp() {
        sample_auv.setup();
    }
};

/**
 * Test that AUV is able to move according to pre-defined motion.
 */
TEST_F(AUVTest, move) {
    ASSERT_EQ(sample_auv.motion, initial_motion);
    std::string motion = "forward";
    sample_auv.move(motion);
}

/**
 * Test that AUV is able to stop.
 */
 TEST_F(AUVTest, stop) {
     sample_auv.move("forward");
     sample_auv.stop();
     ASSERT_EQ(sample_auv.motion, "stop");
 }
