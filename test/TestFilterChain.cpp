#include <gtest/gtest.h>
#include "FilterChain.hpp"

namespace rosneuro {

class FilterChainTestSuite : public ::testing::Test {
    public:
        FilterChainTestSuite() {};
        ~FilterChainTestSuite() {};
        virtual void SetUp() {
            filterChain = new FilterChain<double>();
        };
        virtual void TearDown() {
            delete filterChain;
        };
        FilterChain<double>* filterChain;
};

TEST_F(FilterChainTestSuite, ConstructorTest) {
    EXPECT_FALSE(filterChain->configured_);
}

TEST_F(FilterChainTestSuite, DestructorTest) {
    FilterChain<double>* drop = new FilterChain<double>();
    drop->~FilterChain();
    EXPECT_FALSE(drop->configured_);
}

TEST_F(FilterChainTestSuite, GetBaseClassName) {
    FilterChain<double>* drop_double = new FilterChain<double>();
    FilterChain<int>* drop_int = new FilterChain<int>();
    FilterChain<float>* drop_float = new FilterChain<float>();

    EXPECT_EQ(drop_double->get_baseclass_name(), "rosneuro::Filter<double>");
    EXPECT_EQ(drop_int->get_baseclass_name(), "rosneuro::Filter<int>");
    EXPECT_EQ(drop_float->get_baseclass_name(), "rosneuro::Filter<float>");

    delete drop_double;
    delete drop_int;
    delete drop_float;
}

TEST_F(FilterChainTestSuite, ApplyTest) {
    DynamicMatrix<double> input(3, 3);
    DynamicMatrix<double> result = filterChain->apply(input);

    EXPECT_EQ(result.rows(), input.rows());
    EXPECT_EQ(result.cols(), input.cols());
}

TEST_F(FilterChainTestSuite, DumpTest) {
    EXPECT_NO_THROW(filterChain->dump());
}

TEST_F(FilterChainTestSuite, ConfigureTest) {
    ros::NodeHandle nh("~");
    EXPECT_FALSE(filterChain->configure("test_param", nh));

    XmlRpc::XmlRpcValue validConfig;
    validConfig[0]["name"] = "rosneuro::Car<double>";
    validConfig[0]["type"] = "rosneuro_filters/CarFilterDouble";
    nh.setParam("test/filters_chain", validConfig);

    EXPECT_TRUE(filterChain->configure("test", nh));
    EXPECT_TRUE(filterChain->configured_);
}

TEST_F(FilterChainTestSuite, EmptyConfiguration) {
    ros::NodeHandle nh("~");
    EXPECT_FALSE(filterChain->configure("test_param", nh));
    EXPECT_FALSE(filterChain->configured_);
}

TEST_F(FilterChainTestSuite, InvalidTypeInConfiguration) {
    ros::NodeHandle nh("~");
    XmlRpc::XmlRpcValue invalidTypeConfig;
    invalidTypeConfig.setSize(1);
    invalidTypeConfig[0]["type"] = 123;

    bool result = filterChain->configure("test_param", nh);

    EXPECT_FALSE(result);
    EXPECT_FALSE(filterChain->configured_);
}

}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_filter_chain");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}