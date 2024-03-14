#include <gtest/gtest.h>
#include "Filter.hpp"

namespace rosneuro {

template<typename T>
class FilterTest : public Filter<T> {
    public:
        FilterTest() : Filter<T>() {};
        ~FilterTest() {};
        virtual void dump(void) { Filter<T>::dump(); };
        virtual bool configure(void) { return true; };
        virtual DynamicMatrix<T> apply(const DynamicMatrix<T>& in) { return in; };
};

class FilterTestSuite : public ::testing::Test {
    public:
        FilterTestSuite() {};
        ~FilterTestSuite() {};
        virtual void SetUp() { filter = new FilterTest<double>(); };
        virtual void TearDown() { delete filter; };
        FilterTest<double>* filter;
};

TEST_F(FilterTestSuite, Type) {
    EXPECT_EQ(filter->type(), "");
}

TEST_F(FilterTestSuite, Name) {
    EXPECT_EQ(filter->name(), "");
}

TEST_F(FilterTestSuite, Dump) {
    filter->dump();
    SUCCEED();
}

TEST_F(FilterTestSuite, SetNameAndType) {
    XmlRpc::XmlRpcValue config;
    EXPECT_EQ(filter->setNameAndType(config), false);

    config["type"] = "type";
    config["name"] = "name";
    EXPECT_EQ(filter->setNameAndType(config), true);
    EXPECT_EQ(filter->type(), "type");
    EXPECT_EQ(filter->name(), "name");
}

TEST_F(FilterTestSuite, Configure) {
    EXPECT_EQ(filter->configure(), true);
}

}  // namespace rosneuro

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_filter");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}