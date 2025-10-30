#include <cstdio>

#include "mock_vex.h"
#include "../include/subsystems/odometry.h"

/** Command in order to run:
 * g++ --std=c++11 odom_tests.cpp ../src/subsystems/odometry.cpp -D__TESTING__ && ./a.out
 */

// If parameter is not true, test fails
// P.S. I don't really use this right now, i should
#define IS_TRUE(x)                                                    \
    {                                                                 \
        if (!(x))                                                     \
            printf("%s failed on line %d\n", __FUNCTION__, __LINE__); \
    }

class OdometryTest
{
private:
    vex::rotation leftEncoder{0}, rightEncoder{0}, backEncoder{0};
    Odometry odomTest{leftEncoder, rightEncoder, backEncoder};

    void setMockEncoderDistInches(vex::rotation &leftEncoder, vex::rotation &rightEncoder, vex::rotation &backEncoder,
                                  double leftDist, double rightDist, double backDist)
    {
        leftEncoder.setMockPosition(encoderDistToRevs(leftDist));
        rightEncoder.setMockPosition(encoderDistToRevs(rightDist));
        backEncoder.setMockPosition(encoderDistToRevs(backDist));
    }

    double encoderDistToRevs(double encoderDistInches)
    {
        return encoderDistInches / (2 * M_PI * Odometry::WHEEL_RADIUS_INCHES);
    }

public:
    /* Updates odometry object with mock encoder values and prints the resulting pose */
    void testWithEncoderDistance(double leftDist, double rightDist, double backDist)
    {
        setMockEncoderDistInches(leftEncoder, rightEncoder, backEncoder,
                                 leftDist, rightDist, backDist);
        printf("  ");
        vex::this_thread::sleep_for(20);
        odomTest.getPose().print();
    }

    /* Repeats updating odometry object and print pose for n times */
    void repeatTestWithEncoderDistance(int n, double leftDist, double rightDist, double backDist)
    {
        for (int i = 0; i < n; i++)
        {
            testWithEncoderDistance(leftDist, rightDist, backDist);
            leftDist += leftDist / double(i + 1);
            rightDist += rightDist / double(i + 1);
            backDist += backDist / double(i + 1);
        }
    }

    void reset()
    {
        odomTest.reset(0, 0, 0);
    }
};

void simpleTests(OdometryTest &odomTest)
{
    printf("\nGOING STRAIGHT:\n");
    odomTest.testWithEncoderDistance(1.0, 1.0, 0.0);
    odomTest.reset();

    printf("\nDRIFTING PURE SIDEWAYS\n");
    odomTest.testWithEncoderDistance(
        0, 0, 10.0);
    odomTest.reset();

    printf("\nDRIFTING DIAGONALLY\n");
    odomTest.testWithEncoderDistance(
        10, 10, 7.07);
    odomTest.reset();

    printf("\nROTATING IN PLACE:\n");
    odomTest.testWithEncoderDistance(
        5, -5, -5.34483);
    odomTest.reset();

    printf("\nGOING RIGHT ANGLED:\n");
    odomTest.testWithEncoderDistance(17.606, 13.810, -2.03);
    odomTest.reset();

    printf("\nGOING LEFT ANGLED:\n");
    odomTest.testWithEncoderDistance(13.810, 17.606, 2.03);
    odomTest.reset();
}

void repeatedTests(OdometryTest &odomTest)
{
    printf("\nROTATING IN PLACE CW 4 TIMES:\n");
    odomTest.repeatTestWithEncoderDistance(4, 15.0, -15.0, -16.034);
    odomTest.reset();

    printf("\nROTATING IN PLACE CCW 4 TIMES:\n");
    odomTest.repeatTestWithEncoderDistance(4, -15.0, 15.0, 16.034);
    odomTest.reset();

    printf("\nGOING RIGHT ANGLED 4 TIMES:\n");
    odomTest.repeatTestWithEncoderDistance(4, 17.606, 13.810, -2.029);
    odomTest.reset();

    printf("\nGOING HARD RIGHT ANGLED 4 TIMES:\n");
    odomTest.repeatTestWithEncoderDistance(4, 16.0, -12.0, -14.97);
    odomTest.reset();
}

int main()
{
    OdometryTest odomTest{};

    simpleTests(odomTest);
    repeatedTests(odomTest);

    return 0;
}