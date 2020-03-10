# ifndef SAUVC_THRUSTER_LIB_AUV_H
# define SAUVC_THRUSTER_LIB_AUV_H
# include <thruster_aggregator.h>
# include <string>

class AUV {
private:
    ThrusterAggregator thruster_aggregator;
    std::string motion {"stop"};
    /**
     * A function to setup the AUV.
     * @return {bool} whether the setup was successful or not.
     */
    bool setup();
public:
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
};


#endif //SAUVC_THRUSTER_LIB_AUV_H
