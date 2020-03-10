# include "auv.h"
# include "thruster_aggregator.h"

/**
 * The implementation of the function to setup the AUV.
 * @return {bool} whether the setup was successful or not.
 */
bool AUV::setup() {
    this->thruster_aggregator = ThrusterAggregator();
    return this->thruster_aggregator.setup();
}

/**
 * A function to move the AUV according to a pre-defined motion.
 * @param motion {std::string} movement to perform
 * @return {bool} whether the execution of the movement was successful or not
 */
bool AUV::move(std::string motion) {
    if (this->motion == motion) {
        return this->thruster_aggregator.stabilise();
    }
    else {
        this->motion = motion;
        return this->thruster_aggregator.move(motion);
    }
}

/**
 * A function to stop the AUV.
 * @return {bool} indicates whether the AUV stops successfully or not
 */
bool AUV::stop() {
    this->motion = "stop";
    return this->thruster_aggregator.stop();
}
