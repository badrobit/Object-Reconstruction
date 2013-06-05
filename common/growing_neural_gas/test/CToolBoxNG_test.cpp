#include "gtest/gtest.h"
#include "CToolBoxNG.h"


TEST( angle_tests, positive )
{
	EXPECT_EQ( 1, 1 );
}

int main(int argc, char **argv )
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
