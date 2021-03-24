#include <expressiongraph_tf/context.hpp>
#include <gtest/gtest.h>


// Declare a test
TEST(Tags, tags_from_and_to_string)
{
    using namespace KDL;
    Context ctx;
    ctx.add_tagtype("natural");
    ctx.add_tagtype("artificial");
    TagValue v1;
    bool ok;
    boost::tie(v1,ok) = ctx.tags_from_string("natural , artificial, somethingelse");
    EXPECT_EQ( ok, false);
    boost::tie(v1,ok) = ctx.tags_from_string("natural , artificial");
    EXPECT_EQ( ok, true);
    std::string result = ctx.to_string(v1);
    EXPECT_EQ( result, " artificial natural"); 
    EXPECT_EQ( 0 , ctx.get_tagtype_number("natural")); 
    EXPECT_EQ( 1 , ctx.get_tagtype_number("artificial")); 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}




