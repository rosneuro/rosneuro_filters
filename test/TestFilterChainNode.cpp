#include <gtest/gtest.h>
#include "FilterChainNode.h"

namespace rosneuro {
    class FilterChainNodeTest : public ::testing::Test {
        public:
            void SetUp() override { node = new FilterChainNode(); }
            FilterChainNode* node;
    };

    TEST_F(FilterChainNodeTest, ConstructorTest) {
        ros::Subscriber sub = node->sub_;
        ros::Publisher pub = node->pub_;

        EXPECT_EQ(sub.getNumPublishers(), 0);
        EXPECT_EQ(pub.getNumSubscribers(), 0);
    }

    TEST_F(FilterChainNodeTest, ConfigureSuccessTest) {
        node->p_nh_.setParam("configname", "filterchain");
        EXPECT_FALSE(node->configure());
    }

    TEST_F(FilterChainNodeTest, ConfigureFailureTest) {
        EXPECT_FALSE(node->configure());
    }

    TEST_F(FilterChainNodeTest, ReceivedNeurodataTest) {
        rosneuro_msgs::NeuroFrame sample_msg;

        sample_msg.exg = {};
        sample_msg.tri = {};
        sample_msg.eeg.info.nsamples = 10;
        sample_msg.eeg.info.nchannels = 1;
        sample_msg.sr = 512;
        sample_msg.eeg.data = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

        node->on_received_neurodata(sample_msg);

        EXPECT_EQ(node->neuromsg_.exg, sample_msg.exg);
        EXPECT_EQ(node->neuromsg_.tri, sample_msg.tri);
        EXPECT_EQ(node->neuromsg_.eeg.info.nsamples, 10);
        EXPECT_EQ(node->neuromsg_.eeg.info.nchannels, 1);
        EXPECT_EQ(node->neuromsg_.sr, 512);
        EXPECT_EQ(node->neuromsg_.eeg.data, sample_msg.eeg.data);
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "FilterChainNodeTest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
