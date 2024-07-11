#include <gtest/gtest.h>
#include <gtest/gtest_prod.h>
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
            virtual bool add(const DynamicMatrix<T>& in) { return true; };
    };

    class FilterTestSuite : public ::testing::Test {
        public:
            FilterTestSuite() {};
            ~FilterTestSuite() {};
            virtual void SetUp() { filter = new FilterTest<double>(); };
            virtual void TearDown() { delete filter; };
            FilterTest<double>* filter;
    };

    template<typename T>
    void fromVectorToXmlRpcValue(const std::vector<T>& vec, XmlRpc::XmlRpcValue& value) {
        value.setSize(vec.size());
        for (size_t i = 0; i < vec.size(); ++i)
            value[i] = vec[i];
    }

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

    TEST_F(FilterTestSuite, GetParamString) {
        filter->params_["name"] = "buffer_name";
        std::string value;
        EXPECT_TRUE(filter->getParam("name", value));
        EXPECT_EQ(value, "buffer_name");
    }

    TEST_F(FilterTestSuite, GetParamBool) {
        filter->params_["flag"] = true;
        bool value;
        EXPECT_TRUE(filter->getParam("flag", value));
        EXPECT_TRUE(value);
    }

    TEST_F(FilterTestSuite, GetParamDouble) {
        filter->params_["double"] = 1.1;
        double value;
        EXPECT_TRUE(filter->getParam("double", value));
        EXPECT_DOUBLE_EQ(value, 1.1);
    }

    TEST_F(FilterTestSuite, GetParamInt) {
        filter->params_["count"] = 42;
        int value;
        EXPECT_TRUE(filter->getParam("count", value));
        EXPECT_EQ(value, 42);
    }

    TEST_F(FilterTestSuite, GetParamUnsignedInt) {
        filter->params_["size"] = 100;
        unsigned int value;
        EXPECT_TRUE(filter->getParam("size", value));
        EXPECT_EQ(value, 100);
    }

    TEST_F(FilterTestSuite, GetParamVectorDouble) {
        std::vector<double> expected_values = {1.1, 2.2, 3.3};

        XmlRpc::XmlRpcValue xml_rpc_value;
        fromVectorToXmlRpcValue(expected_values, xml_rpc_value);

        filter->params_["values"] = xml_rpc_value;
        std::vector<double> value;
        EXPECT_TRUE(filter->getParam("values", value));
        EXPECT_EQ(value, expected_values);
    }

    TEST_F(FilterTestSuite, GetParamVectorString) {
        std::vector<std::string> expected_values = {"abc", "def", "ghi"};

        XmlRpc::XmlRpcValue xml_rpc_value;
        fromVectorToXmlRpcValue(expected_values, xml_rpc_value);

        filter->params_["strings"] = xml_rpc_value;
        std::vector<std::string> value;
        EXPECT_TRUE(filter->getParam("strings", value));
        EXPECT_EQ(value, expected_values);
    }

    TEST_F(FilterTestSuite, GetParamXmlRpcValue) {
        XmlRpc::XmlRpcValue expected_value = 42;
        filter->params_["param"] = expected_value;
        XmlRpc::XmlRpcValue value;
        EXPECT_TRUE(filter->getParam("param", value));
        EXPECT_EQ(value, expected_value);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_filter");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}