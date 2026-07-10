// f4flight unit tests - vector/matrix/quaternion types
#include "f4flight/core/constants.h"
#include "f4flight/core/types.h"
#include <gtest/gtest.h>

using namespace f4flight;

TEST(TypesTest, Vec3BasicOps) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 c = a + b;
    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);

    Vec3 d = a - b;
    EXPECT_DOUBLE_EQ(d.x, -3.0);
    EXPECT_DOUBLE_EQ(d.y, -3.0);
    EXPECT_DOUBLE_EQ(d.z, -3.0);

    Vec3 e = a * 2.0;
    EXPECT_DOUBLE_EQ(e.x, 2.0);
    EXPECT_DOUBLE_EQ(e.y, 4.0);
    EXPECT_DOUBLE_EQ(e.z, 6.0);
}

TEST(TypesTest, Vec3DotCross) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{0.0, 1.0, 0.0};
    EXPECT_DOUBLE_EQ(dot(a, b), 0.0);
    EXPECT_DOUBLE_EQ(dot(a, a), 1.0);

    Vec3 c = cross(a, b);
    EXPECT_DOUBLE_EQ(c.x, 0.0);
    EXPECT_DOUBLE_EQ(c.y, 0.0);
    EXPECT_DOUBLE_EQ(c.z, 1.0);
}

TEST(TypesTest, Vec3Norm) {
    Vec3 a{3.0, 4.0, 0.0};
    EXPECT_NEAR(a.norm(), 5.0, 1e-12);
    EXPECT_NEAR(a.normSquared(), 25.0, 1e-12);

    Vec3 n = a.normalized();
    EXPECT_NEAR(n.norm(), 1.0, 1e-12);
}

TEST(TypesTest, Matrix3Identity) {
    Matrix3 I = Matrix3::identity();
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 r = I * v;
    EXPECT_DOUBLE_EQ(r.x, 1.0);
    EXPECT_DOUBLE_EQ(r.y, 2.0);
    EXPECT_DOUBLE_EQ(r.z, 3.0);
}

TEST(TypesTest, Matrix3Multiply) {
    // 90-degree rotation about Z
    Matrix3 R;
    R.m[0][0] = 0.0; R.m[0][1] = -1.0; R.m[0][2] = 0.0;
    R.m[1][0] = 1.0; R.m[1][1] =  0.0; R.m[1][2] = 0.0;
    R.m[2][0] = 0.0; R.m[2][1] =  0.0; R.m[2][2] = 1.0;

    Vec3 v{1.0, 0.0, 0.0};
    Vec3 r = R * v;
    EXPECT_NEAR(r.x, 0.0, 1e-12);
    EXPECT_NEAR(r.y, 1.0, 1e-12);
    EXPECT_NEAR(r.z, 0.0, 1e-12);

    // R * R^T should be identity
    Matrix3 I = R * R.transposed();
    EXPECT_NEAR(I.m[0][0], 1.0, 1e-12);
    EXPECT_NEAR(I.m[1][1], 1.0, 1e-12);
    EXPECT_NEAR(I.m[2][2], 1.0, 1e-12);
    EXPECT_NEAR(I.m[0][1], 0.0, 1e-12);
}

TEST(TypesTest, QuaternionIdentity) {
    Quaternion q; // default = identity
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 r = q.rotate(v);
    EXPECT_NEAR(r.x, 1.0, 1e-12);
    EXPECT_NEAR(r.y, 2.0, 1e-12);
    EXPECT_NEAR(r.z, 3.0, 1e-12);
}

TEST(TypesTest, QuaternionRotation) {
    // 90-degree rotation about Z
    const double a = PI / 2.0;
    Quaternion q{std::cos(a / 2.0), 0.0, 0.0, std::sin(a / 2.0)};
    Vec3 v{1.0, 0.0, 0.0};
    Vec3 r = q.rotate(v);
    EXPECT_NEAR(r.x, 0.0, 1e-12);
    EXPECT_NEAR(r.y, 1.0, 1e-12);
    EXPECT_NEAR(r.z, 0.0, 1e-12);
}

TEST(TypesTest, QuaternionEulerRoundtrip) {
    double psi = 0.5, theta = 0.3, phi = 0.2;
    Quaternion q = quatFromEuler(psi, theta, phi);
    double p2, t2, f2;
    eulerFromQuat(q, p2, t2, f2);
    EXPECT_NEAR(p2, psi, 1e-10);
    EXPECT_NEAR(t2, theta, 1e-10);
    EXPECT_NEAR(f2, phi, 1e-10);
}

TEST(TypesTest, QuaternionNorm) {
    Quaternion q{1.0, 2.0, 3.0, 4.0};
    EXPECT_NEAR(q.norm(), std::sqrt(30.0), 1e-12);

    Quaternion n = q.normalized();
    EXPECT_NEAR(n.norm(), 1.0, 1e-12);
}

TEST(TypesTest, DcmFromEuler) {
    // Zero rotation -> identity
    Matrix3 I = dcmFromEuler(0.0, 0.0, 0.0);
    EXPECT_NEAR(I.m[0][0], 1.0, 1e-12);
    EXPECT_NEAR(I.m[1][1], 1.0, 1e-12);
    EXPECT_NEAR(I.m[2][2], 1.0, 1e-12);

    // 90-degree yaw -> rotates X into Y
    Matrix3 R = dcmFromEuler(PI / 2.0, 0.0, 0.0);
    Vec3 v{1.0, 0.0, 0.0};
    Vec3 r = R * v;
    EXPECT_NEAR(r.x, 0.0, 1e-12);
    EXPECT_NEAR(r.y, 1.0, 1e-12);
    EXPECT_NEAR(r.z, 0.0, 1e-12);
}
