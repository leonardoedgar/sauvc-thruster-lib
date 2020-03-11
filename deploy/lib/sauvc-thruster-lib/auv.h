# ifndef SAUVC_THRUSTER_LIB_AUV_H
# define SAUVC_THRUSTER_LIB_AUV_H
# include <Arduino.h>
# include <thruster_aggregator.h>
# include <string>

class AUV {
private:
    ThrusterAggregator thruster_aggregator;
    std::string motion {"stop"};
public:
    /**
     * A function to setup the AUV.
     * @return {bool} whether the setup was successful or not.
     */
    bool setup();
    /**
     * A function to move the AUV according to a pre-defined motion.
     * @param motion {std::string} movement to perform
     * @return {bool} whether the execution of the movement was successful or not
     */
    bool move(std::string motion);

    /**
     * A function to stop the AUV.
     * @return {bool} indicates whether the AUV stops successfully or not
     */
    bool stop();

    /**
     * A function to get AUV motion.
     * @return {std::string} AUV motion
     */
    std::string get_motion() const;
};


#endif //SAUVC_THRUSTER_LIB_AUV_H
