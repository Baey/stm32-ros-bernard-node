#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>
#include <bernard_sensors.hpp>

// Mock classes and functions for testing
// Mock classes and functions for testing
class MockAdafruit_BNO055 : public Adafruit_BNO055 {
    public:
        bool begin() { return true; }
        imu::Quaternion getQuat() { return imu::Quaternion(1, 0, 0, 0); }
        imu::Vector<3> getVector(adafruit_vector_type_t type) {
            if (type == VECTOR_LINEARACCEL) {
                return imu::Vector<3>(1.0, 2.0, 3.0);
            } else if (type == VECTOR_ACCELEROMETER) {
                return imu::Vector<3>(4.0, 5.0, 6.0);
            } else if (type == VECTOR_GYROSCOPE) {
                return imu::Vector<3>(7.0, 8.0, 9.0);
            }
            return imu::Vector<3>();
        }
        void setExtCrystalUse(bool usextal) {}
    };

uint32_t mockAnalogRead(uint32_t pin) {
    if (pin == 1) return 512; // Mock value for left foot sensor
    if (pin == 2) return 256; // Mock value for right foot sensor
    return 0;
}

#define analogRead mockAnalogRead

TEST_CASE("BernardSensors Initialization") {
    MockAdafruit_BNO055 mockIMU;
    BernardSensors sensors(&mockIMU, 1, 2);

    CHECK(sensors.initIMU() == true);
}

TEST_CASE("BernardSensors IMU Data Reading") {
    MockAdafruit_BNO055 mockIMU;
    BernardSensors sensors(&mockIMU, 1, 2);

    sensors.initIMU();

    SUBCASE("Read Quaternion") {
        auto quat = sensors.readQuaternion();
        CHECK(quat.w() == 1);
        CHECK(quat.x() == 0);
        CHECK(quat.y() == 0);
        CHECK(quat.z() == 0);
    }

    SUBCASE("Read Linear Acceleration") {
        auto linearAcc = sensors.readLinearAcceleration();
        CHECK(linearAcc.x() == 1.0);
        CHECK(linearAcc.y() == 2.0);
        CHECK(linearAcc.z() == 3.0);
    }

    SUBCASE("Read Accelerometer Data") {
        auto accel = sensors.readAccelerometer();
        CHECK(accel.x() == 4.0);
        CHECK(accel.y() == 5.0);
        CHECK(accel.z() == 6.0);
    }

    SUBCASE("Read Gyroscope Data") {
        auto gyro = sensors.readGyroscope();
        CHECK(gyro.x() == 7.0);
        CHECK(gyro.y() == 8.0);
        CHECK(gyro.z() == 9.0);
    }
}

TEST_CASE("BernardSensors Foot Pressure Reading") {
    MockAdafruit_BNO055 mockIMU;
    BernardSensors sensors(&mockIMU, 1, 2);

    auto footPressure = sensors.readFootPressure();
    CHECK(footPressure.size() == 2);
    CHECK(footPressure[0] == 512); // Mock value for left foot sensor
    CHECK(footPressure[1] == 256); // Mock value for right foot sensor
}